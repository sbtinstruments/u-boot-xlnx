// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2021 SBT Instruments - All Rights Reserved
 * Author(s): Frederik Aalund <fpa@sbtinstruments.com> for SBT Instruments.
 *
 * Based on drivers/gpu/drm/tiny/ili9488.c from the Linux repo
 */

#include <asm/gpio.h>
#include <backlight.h>
#include <common.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <fdtdec.h>
#include <fdt_support.h>
#include <mipi_display.h>
#include <spi.h>
#include <video.h>

#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>


#define MIPI_DBI_B_BASE (volatile void __iomem *)0x43C10000

#define MIPI_DBI_B_REG_VERSION      0x0
#define MIPI_DBI_B_REG_CONTROL      0x4
#define MIPI_DBI_B_REG_COMMAND      0x10
#define MIPI_DBI_B_REG_DATA         0x20

#define MIPI_DBI_B_CONTROL_RESET    BIT(0)
#define MIPI_DBI_B_CONTROL_CS       BIT(1)

#define ILI9488_CMD_DISPLAY_INVERSION_CONTROL   0xb4
#define ILI9488_CMD_FRAME_RATE_CONTROL          0xb1

#define ILI9488_DINV_2_DOT_INVERSION            0x02
#define ILI9488_DPI_16_BPP                      0x5
#define ILI9488_DBI_16_BPP                      0x5


struct clip_rect {
	int x1, x2, y1, y2;
};

struct ili9488_priv {
	void __iomem *mmio_base;
	struct udevice *backlight;
};

static void mipi_dbi_type_b_hw_reset(struct ili9488_priv *priv)
{
	iowrite32(MIPI_DBI_B_CONTROL_RESET, priv->mmio_base + MIPI_DBI_B_REG_CONTROL);
	mdelay(10);
	iowrite32(0, priv->mmio_base + MIPI_DBI_B_REG_CONTROL);
	mdelay(120);
}

static int mipi_dbi_type_b_command(struct ili9488_priv *priv, u8 cmd, u8 *param, size_t num)
{
	/* Assert CS */
	iowrite32(MIPI_DBI_B_CONTROL_CS, priv->mmio_base + MIPI_DBI_B_REG_CONTROL);
	/* Write command */
	iowrite8(cmd, priv->mmio_base + MIPI_DBI_B_REG_COMMAND);
	/* Some special commands may send the parameters in an optimized way */
	switch (cmd) {
	/* 8 bits at a time is the default */
	default:
		while (0 < num) {
			iowrite8(*param, priv->mmio_base + MIPI_DBI_B_REG_DATA);
			++param;
			--num;
		}
		break;
	/* Memory writes are optimized in hardware */
	case MIPI_DCS_WRITE_MEMORY_START:
		while (0 < num) {
			iowrite32(*(u32*)param, priv->mmio_base + MIPI_DBI_B_REG_DATA);
			param += 4;
			num -= 4;
		}
		break;
	}
	/* Deassert CS */
	iowrite32(0, priv->mmio_base + MIPI_DBI_B_REG_CONTROL);
	return 0;
}

static int mipi_dbi_command_buf(struct ili9488_priv *priv, u8 cmd, u8 *data, size_t len)
{
	return mipi_dbi_type_b_command(priv, cmd, data, len);
}

#define mipi_dbi_command(priv, cmd, seq...) \
({ \
	u8 d[] = { seq }; \
	mipi_dbi_command_buf(priv, cmd, d, ARRAY_SIZE(d)); \
})

static int ili9488_video_sync(struct udevice *dev)
{
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ili9488_priv *priv = dev_get_priv(dev);
	/* For now, we always update the entire screen. If U-boot supports
	 * partial framebuffer updates in the future, we can easily support
	 * that as well. */
	struct clip_rect clip = {0, uc_priv->xsize, 0, uc_priv->ysize};
	mipi_dbi_command(priv, MIPI_DCS_SET_COLUMN_ADDRESS,
	                 (clip.x1 >> 8) & 0xFF, clip.x1 & 0xFF,
	                 (clip.x2 >> 8) & 0xFF, (clip.x2 - 1) & 0xFF);
	mipi_dbi_command(priv, MIPI_DCS_SET_PAGE_ADDRESS,
	                 (clip.y1 >> 8) & 0xFF, clip.y1 & 0xFF,
	                 (clip.y2 >> 8) & 0xFF, (clip.y2 - 1) & 0xFF);
	mipi_dbi_command_buf(priv, MIPI_DCS_WRITE_MEMORY_START, uc_priv->fb,
	                 (clip.x2 - clip.x1) * (clip.y2 - clip.y1) * sizeof(u16));
	return 0;
}

static int ili9488_init(struct ili9488_priv *priv)
{
	/* reset (hardware) */
	mipi_dbi_type_b_hw_reset(priv);

	/* reset (software) */
	mipi_dbi_command(priv, MIPI_DCS_SOFT_RESET);
	mdelay(240);

	/* display off */
	mipi_dbi_command(priv, MIPI_DCS_SET_DISPLAY_OFF);
	/* positive gamma control */
	mipi_dbi_command(priv, 0xE0, 0x00, 0x03, 0x09, 0x08, 0x16, \
			0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, \
			0x08, 0x16, 0x1A, 0x0F);
	/* negative gamma control */
	mipi_dbi_command(priv, 0xE1, 0x00, 0x16, 0x19, 0x03, 0x0F, \
			0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, \
			0x0D, 0x35, 0x37, 0x0F);
	/* power control 1 */
	mipi_dbi_command(priv, 0xC0, 0x17, 0x15);
	/* power control 2 */
	mipi_dbi_command(priv, 0xC1, 0x41);
	/* VCOM control 1 */
	mipi_dbi_command(priv, 0xC5, 0x00, 0x12, 0x80);
	/* memory access control: MX and BGR */
	mipi_dbi_command(priv, MIPI_DCS_SET_ADDRESS_MODE, 0x48);
	/* pixel interchange format: RGB565 over MIPI 16 bit */
	mipi_dbi_command(priv, MIPI_DCS_SET_PIXEL_FORMAT,
	                 ILI9488_DBI_16_BPP | (ILI9488_DPI_16_BPP << 4));
	/* interface mode control */
	mipi_dbi_command(priv, 0xB0, 0x00);
	/* frame rate control (0x01 is 30.38 Hz; 0xA0 is 60.76 Hz) */
	mipi_dbi_command(priv, ILI9488_CMD_FRAME_RATE_CONTROL, 0xA0);
	/* display inversion ON */
	mipi_dbi_command(priv, 0x21);
	/* display inversion control */
	mipi_dbi_command(priv, ILI9488_CMD_DISPLAY_INVERSION_CONTROL,
                     ILI9488_DINV_2_DOT_INVERSION);
	/* write CTRL display value (brightness, dimming, backlight) */
	mipi_dbi_command(priv, MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x28);
	/* write display brightness value */
	mipi_dbi_command(priv, MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x7F);
	/* exit sleep */
	mipi_dbi_command(priv, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);
	/* display on */
	mipi_dbi_command(priv, MIPI_DCS_SET_DISPLAY_ON);
	mdelay(50);

	return 0;
}


static int ili9488_probe(struct udevice *dev)
{
	int ret;
	struct video_priv *uc_priv = dev_get_uclass_priv(dev);
	struct ili9488_priv *priv = dev_get_priv(dev);
    struct resource res;

	/* io memory */
	ret = dev_read_resource(dev, 0, &res);
    if (ret) {
        dev_err(dev, "Could not read resource: %d\n", ret);
        return ret;
    }
	priv->mmio_base = devm_ioremap(dev, res.start, resource_size(&res));
	if (IS_ERR(priv->mmio_base)) {
        dev_err(dev, "Could not remap IO: %d\n", ret);
		return PTR_ERR(priv->mmio_base);
	}

	/* video properties */
	uc_priv->xsize = 320;
	uc_priv->ysize = 480;
	uc_priv->rot = 0;
	uc_priv->bpix = VIDEO_BPP16;

	/* init sequence */
	ret = ili9488_init(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize ILI9488: %d\n", ret);
		return ret;
	}

	/* backlight (after init to avoid screen flicker) */
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

	return 0;
}

static int ili9488_remove(struct udevice *dev)
{
	return 0;
}

static const struct video_ops ili9488_video_ops = {
	.video_sync = ili9488_video_sync,
};

static const struct udevice_id ili9488_ids[] = {
	{ .compatible = "urt,p220md-t" },
	{ }
};

U_BOOT_DRIVER(ili9488) = {
	.name = "ili9488",
	.id = UCLASS_VIDEO,
	.of_match = ili9488_ids,
	.ops = &ili9488_video_ops,
	.probe = ili9488_probe,
	.remove = ili9488_remove,
	.priv_auto_alloc_size = sizeof(struct ili9488_priv),
};
