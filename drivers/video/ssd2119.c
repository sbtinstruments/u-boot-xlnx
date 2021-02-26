// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2020 SBT Instruments - All Rights Reserved
 * Author(s): Frederik Aalund <fpa@sbtinstruments.com> for SBT Instruments.
 *
 */

#include <common.h>
#include <dm.h>
#include <backlight.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <video.h>
#include <spi.h>
#include <asm/gpio.h>

#include <linux/io.h>

#define SSD2119_CMD_WRITE_GRAM			0x22
#define SSD2119_CMD_SET_GRAM_ADDR_X     0x4e
#define SSD2119_CMD_SET_GRAM_ADDR_Y     0x4f

#define SSD2119_REG_OUTPUT_CONTROL		0x01
#define SSD2119_REG_ENTRY_MODE			0x11

#define SSD2119_ENTRY_MODE_UPPER_BITS	0x6E40
#define SSD2119_ROT_0					0x30
#define SSD2119_ROT_90					0x18
#define SSD2119_ROT_180					0x00
#define SSD2119_ROT_270					0x28

struct buf16 {
	uint16_t *data;
	size_t len;
};

struct ssd2119_priv {
	struct gpio_desc reset, dc;
	struct udevice *backlight;
	struct buf16 tx_buf;
};

enum ssd2119_spi_mode {
	SSD2119_SPI_COMMAND = 0,
	SSD2119_SPI_DATA = 1,
};

struct ssd2119_cmd16 {
	uint8_t cmd;
	uint16_t data;
};

#define ssd2119_command(mipi, cmd, seq...) \
({ \
	uint8_t d[] = { seq }; \
	ssd2119_command_buf(mipi, cmd, d, ARRAY_SIZE(d)); \
})

static int ssd2119_set_spi_mode(struct udevice *dev, enum ssd2119_spi_mode mode)
{
	struct ssd2119_priv *priv = dev_get_priv(dev);
	return dm_gpio_set_value(&priv->dc, mode);
}

/* This function assumes that the caller has claimed the SPI bus */
static int ssd2119_command_buf(struct udevice *dev, uint8_t cmd, uint8_t *data, size_t len)
{
	int ret;
	/* Enter command mode */
	ret = ssd2119_set_spi_mode(dev, SSD2119_SPI_COMMAND);
	if (ret)
		return ret;
	/* Send the command */
	ret = dm_spi_xfer(dev, 8, &cmd, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
	if (ret)
		return ret;
	/* Early out if there is no data to send */
	if (len == 0)
		return 0;
	/* Enter data mode */
	ret = ssd2119_set_spi_mode(dev, SSD2119_SPI_DATA);
	if (ret)
		return ret;
	/* Send the data */
	while (0 < len) {
		/*
		 * I know that it seems strange to use (SPI_XFER_BEGIN | SPI_XFER_END)
		 * since this will assert and deassert the chip-select (CS) pin for
		 * each iteration of this loop. One would think that we should assert
		 * the CS pin for the transfer as a whole (outside the for loop).
		 * The SSD2119, however, does not work like that.
		 *
		 * See section "7.1.3: 4-wire Serial Peripheral Interface (8 bits)",
		 * page 30 out of 95 in SSD2119 "Advance Information" Rev 1.4 (Jun 2009)
		 */
		ret = dm_spi_xfer(dev, 8, data, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret)
			return ret;
		++data;
		--len;
	}
	return 0;
}

static int ssd2119_command_array(struct udevice *dev, uint8_t cmds[][3], size_t len)
{
	int ret;
	int i;
	for (i = 0; i < len; ++i) {
		ret = ssd2119_command_buf(dev, cmds[i][0], cmds[i] + 1, 2);
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * ssd2119_write_gram() - Copy data to the device's GRAM
 *
 * This function assumes that the caller has claimed the SPI bus
 *
 * @dev: Device to copy to.
 * @data: Data buffer.
 * @len: Data buffer length in units of uint16_t.
 */
static int ssd2119_write_gram(struct udevice *dev, uint16_t *data, size_t len)
{
	int ret;
	struct ssd2119_priv *priv = dev_get_priv(dev);
	struct buf16 *tx_buf = &priv->tx_buf;
	size_t copy_len;
	size_t remain_len = len;
	/* Issue "write GRAM" command */
	ret = ssd2119_command(dev, SSD2119_CMD_WRITE_GRAM);
	if (ret)
		return ret;
	/* Enter data mode */
	ret = ssd2119_set_spi_mode(dev, SSD2119_SPI_DATA);
	if (ret)
		return ret;
	/* Write data in chunks */
	while (0 < remain_len) {
		copy_len = min(tx_buf->len, remain_len);
		/* Copy from framebuffer to transfer buffer */
		int j;
		for (j = 0; j < copy_len; ++j) {
			tx_buf->data[j] = cpu_to_be16(*data++);
		}
		/* Copy from transfer buffer to hardware frame buffer (via SPI) */
		ret = dm_spi_xfer(dev, 8 * copy_len * sizeof(uint16_t), tx_buf->data, NULL, SPI_XFER_BEGIN | SPI_XFER_END);
		if (ret)
			return ret;
		remain_len -= copy_len;
	}
	return 0;
}

static int ssd2119_video_sync(struct udevice *dev)
{
	int ret;
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	/* Claim bus before SPI transfers */
	ret = dm_spi_claim_bus(dev);
	if (ret)
		goto out;
	/* Reset GRAM address counter. The address counter increments
	 * itself automatically with each write to the GRAM. We reset
	 * the address counter to be sure that we start the write
	 * process from the beginning of the GRAM.
	 */
	ret = ssd2119_command(dev, SSD2119_CMD_SET_GRAM_ADDR_X, 0, 0);
	if (ret)
		goto release_bus;
	ret = ssd2119_command(dev, SSD2119_CMD_SET_GRAM_ADDR_Y, 0, 0);
	if (ret)
		goto release_bus;
	/* Copy framebuffer to graphics memory (GRAM) */
	ret = ssd2119_write_gram(dev, uc_priv->fb, uc_priv->fb_size / sizeof(uint16_t));
	if (ret)
		goto release_bus;
release_bus:
	dm_spi_release_bus(dev);
out:
	return ret;
}

static int ssd2119_reset(struct udevice *dev)
{
	int ret;
	struct ssd2119_priv *priv = dev_get_priv(dev);
	ret = dm_gpio_set_value(&priv->reset, 0);
	if (ret)
		return ret;
	udelay(40);
	ret = dm_gpio_set_value(&priv->reset, 1);
	if (ret)
		return ret;
	mdelay(120);
	return 0;
}

static int ssd2119_init(struct udevice *dev)
{
	int ret;

	ret = ssd2119_reset(dev);
	if (ret)
		goto out;

	ret = dm_spi_claim_bus(dev);
	if (ret)
		goto out;

	uint8_t init_cmds[][3] = {
		{0x28, 0x00, 0x06},
		{0x00, 0x00, 0x01},
		{0x10, 0x00, 0x00},
		{SSD2119_REG_OUTPUT_CONTROL, 0x30, 0xEF},
		{0x02, 0x06, 0x00},
		{0x03, 0x6A, 0x38},
		{SSD2119_REG_ENTRY_MODE, 0x6e, 0x70},
		{0X0F, 0x00, 0x00},
		{0X0B, 0x53, 0x08},
		{0x0C, 0x00, 0x03},
		{0x0D, 0x00, 0x0A},
		{0x0E, 0x2E, 0x00},
		{0x1E, 0x00, 0xBE},
		{0x25, 0xA0, 0x00},
		{0x26, 0x78, 0x00},
		{0x12, 0x08, 0xD9},
		{0x30, 0x00, 0x00},
		{0x31, 0x01, 0x04},
		{0x32, 0x01, 0x00},
		{0x33, 0x03, 0x05},
		{0x34, 0x05, 0x05},
		{0x35, 0x03, 0x05},
		{0x36, 0x07, 0x07},
		{0x37, 0x03, 0x00},
		{0x3A, 0x12, 0x00},
		{0x3B, 0x08, 0x00},
		{0x07, 0x00, 0x33},
		{0x22, 0x00, 0x00},
	};
	ret = ssd2119_command_array(dev, init_cmds, sizeof(init_cmds) / 3);
	if (ret)
		goto release_bus;

release_bus:
	dm_spi_release_bus(dev);
out:
	return ret;
}


static int ssd2119_probe(struct udevice *dev)
{
	int ret;
	struct video_uc_platdata *plat = dev_get_uclass_platdata(dev);
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ssd2119_priv *priv = dev_get_priv(dev);
	const void *blob = gd->fdt_blob;
	const int node = dev_of_offset(dev);
	fdt_addr_t base;
	fdt_size_t size;

	base = fdtdec_get_addr_size_auto_parent(blob, dev_of_offset(dev->parent),
			node, "reg", 0, &size, false);
	if (base == FDT_ADDR_T_NONE) {
		debug("%s: Failed to decode memory region\n", __func__);
		return -EINVAL;
	}

	plat->base = base;
	plat->size = size;

	uc_priv->xsize = 320;
	uc_priv->ysize = 240;
	uc_priv->rot = 0;
	uc_priv->bpix = VIDEO_BPP16;

	/* Reset GPIO */
	ret = gpio_request_by_name(dev, "reset-gpios", 0, &priv->reset, GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "Warning: cannot get reset GPIO\n");
		return ret;
	}

	/* Data/command (DC) GPIO */
	ret = gpio_request_by_name(dev, "dc-gpios", 0, &priv->dc, GPIOD_IS_OUT);
	if (ret) {
		dev_err(dev, "Warning: cannot get dc (data/command) GPIO\n");
		return ret;
	}

	/* SPI transfer buffer */
	priv->tx_buf.len = PAGE_SIZE;
	priv->tx_buf.data = malloc(priv->tx_buf.len * sizeof(uint16_t));
	if (!priv->tx_buf.data) {
		dev_err(dev, "Warning: cannot allocate memory for transfer buffer\n");
		return -ENOMEM;
	}

	/* Backlight */
	ret = uclass_get_device_by_phandle(UCLASS_PANEL_BACKLIGHT, dev,
					   "backlight", &priv->backlight);
	if (ret) {
		dev_err(dev, "Cannot get backlight: %d\n", ret);
		return ret;
	}
	ret = backlight_enable(priv->backlight);
	if (ret) {
		dev_err(dev, "Cannot enable backlight: %d\n", ret);
		return ret;
	}

	/* Init sequence */
	ret = ssd2119_init(dev);
	if (ret) {
		dev_err(dev, "Failed to initialize SSD2119: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ssd2119_remove(struct udevice *dev)
{
	int ret;
	struct ssd2119_priv *priv = dev_get_priv(dev);
	if (priv->tx_buf.data)
		free(priv->tx_buf.data);
	if (priv->dc.dev) {
		ret = dm_gpio_free(dev, &priv->dc);
		if (ret)
			return ret;
	}
	if (priv->reset.dev) {
		ret = dm_gpio_free(dev, &priv->reset);
		if (ret)
			return ret;
	}
	return ret;
}

static const struct video_ops ssd2119_video_ops = {
	.video_sync = ssd2119_video_sync,
};

static const struct udevice_id ssd2119_ids[] = {
	{ .compatible = "solomon,ssd2119" },
	{ }
};

U_BOOT_DRIVER(ssd2119) = {
	.name = "ssd2119",
	.id = UCLASS_VIDEO,
	.of_match = ssd2119_ids,
	.ops = &ssd2119_video_ops,
	.probe = ssd2119_probe,
	.remove = ssd2119_remove,
	.priv_auto_alloc_size = sizeof(struct ssd2119_priv),
};
