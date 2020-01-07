// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Commands to control Ilitek ILI9488 panels.
 *
 * Supported panels:
 *   * URT P220MD-T
 *
 * Copyright 2019 Frederik Peter Aalund <fpa@sbtinstruments.com
 */
#include <common.h>
#include <errno.h>
#include <asm/gpio.h>
#include <linux/io.h>

#include "mipi_display.h"

/*
 * Backlight
 *
 * Based on drivers/pwm/pwm-xlnx.c from the Linux repo
 */

#define BACKLIGHT_BASE (volatile void __iomem *)0x42800000
#define TCSR0		0x00	/* Timer 0 Control and Status Register */
#define TLR0		0x04	/* Timer 0 Load Register */
#define TCSR1		0x10	/* Timer 1 Control and Status Register */
#define TLR1		0x14	/* Timer 1 Load Register */

#define PERIOD		TLR0
#define DUTY		TLR1

#define UDT_BIT		BIT(1)	/* Up/Down Count Timer */
#define GENT_BIT	BIT(2)	/* Enable External Generate Signal Timer */
#define ENT_BIT		BIT(7)	/* Enable Timer */
#define PWMA_BIT	BIT(9)	/* Enable Pulse Width Modulation for Timer */
#define PWM_CONF	(UDT_BIT | GENT_BIT | ENT_BIT | PWMA_BIT)

static void backlight_enable(void)
{
	int tlrx_duty = 0x3E5;
	int tlrx_period = 0x3E6;
	iowrite32(tlrx_duty, BACKLIGHT_BASE + DUTY);
	iowrite32(tlrx_period, BACKLIGHT_BASE + PERIOD);
	iowrite32(PWM_CONF, BACKLIGHT_BASE + TCSR0);
	iowrite32(PWM_CONF, BACKLIGHT_BASE + TCSR1);
}

/*
 * Display
 *
 * Based on drivers/gpu/drm/tinydrm/ili9488.c from the Linux repo
 */

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

static void mipi_dbi_type_b_hw_reset(void)
{
	iowrite32(MIPI_DBI_B_CONTROL_RESET, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
	mdelay(10);
	iowrite32(0, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
	mdelay(120);
}

static int mipi_dbi_type_b_command(u8 cmd, u8 *param, size_t num)
{
	/* Assert CS */
	iowrite32(MIPI_DBI_B_CONTROL_CS, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
	/* Write command */
	iowrite8(cmd, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_COMMAND);
	/* Some special commands may send the parameters in an optimized way */
	switch (cmd) {
	/* 8 bits at a time is the default */
	default:
		while (0 < num) {
			iowrite8(*param, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_DATA);
			++param;
			--num;
		}
		break;
	/* Memory writes are optimized in hardware */
	case MIPI_DCS_WRITE_MEMORY_START:
		while (0 < num) {
			iowrite32(*(u32*)param, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_DATA);
			param += 4;
			num -= 4;
		}
		break;
	}
	/* Deassert CS */
	iowrite32(0, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
	return 0;
}

static int mipi_dbi_command_buf(u8 cmd, u8 *data, size_t len)
{
	return mipi_dbi_type_b_command(cmd, data, len);
}

#define mipi_dbi_command(cmd, seq...) \
({ \
	u8 d[] = { seq }; \
	mipi_dbi_command_buf(cmd, d, ARRAY_SIZE(d)); \
})

static void ili9488_enable(void)
{
	/* reset */
	mipi_dbi_type_b_hw_reset();
	mipi_dbi_command(MIPI_DCS_SOFT_RESET);
	mdelay(240);

	/* display off */
	mipi_dbi_command(MIPI_DCS_SET_DISPLAY_OFF);
	/* positive gamma control */
	mipi_dbi_command(0xE0, 0x00, 0x03, 0x09, 0x08, 0x16, \
			0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, \
			0x08, 0x16, 0x1A, 0x0F);
	/* negative gamma control */
	mipi_dbi_command(0xE1, 0x00, 0x16, 0x19, 0x03, 0x0F, \
			0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, \
			0x0D, 0x35, 0x37, 0x0F);
	/* power control 1 */
	mipi_dbi_command(0xC0, 0x17, 0x15);
	/* power control 2 */
	mipi_dbi_command(0xC1, 0x41);
	/* VCOM control 1 */
	mipi_dbi_command(0xC5, 0x00, 0x12, 0x80);
	/* memory access control: MX and BGR */
	mipi_dbi_command(MIPI_DCS_SET_ADDRESS_MODE, 0x48);
	/* pixel interchange format: RGB565 over MIPI 16 bit */
	mipi_dbi_command(MIPI_DCS_SET_PIXEL_FORMAT,
	                 ILI9488_DBI_16_BPP | (ILI9488_DPI_16_BPP << 4));
	/* interface mode control */
	mipi_dbi_command(0xB0, 0x00);
	/* frame rate control (0x01 is 30.38 Hz; 0xA0 is 60.76 Hz) */
	mipi_dbi_command(ILI9488_CMD_FRAME_RATE_CONTROL, 0xA0);
	/* display inversion ON */
	mipi_dbi_command(0x21);
	/* display inversion control */
	mipi_dbi_command(ILI9488_CMD_DISPLAY_INVERSION_CONTROL,
                     ILI9488_DINV_2_DOT_INVERSION);
	/* write CTRL display value (brightness, dimming, backlight) */
	mipi_dbi_command(MIPI_DCS_WRITE_CONTROL_DISPLAY, 0x28);
	/* write display brightness value */
	mipi_dbi_command(MIPI_DCS_SET_DISPLAY_BRIGHTNESS, 0x7F);
	/* exit sleep */
	mipi_dbi_command(MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);
	/* display on */
	mipi_dbi_command(MIPI_DCS_SET_DISPLAY_ON);
	mdelay(50);
}

struct clip_rect {
	int x1, x2, y1, y2;
};

static void ili9488_write(void *buf, struct clip_rect *clip)
{
	mipi_dbi_command(MIPI_DCS_SET_COLUMN_ADDRESS,
	                 (clip->x1 >> 8) & 0xFF, clip->x1 & 0xFF,
	                 (clip->x2 >> 8) & 0xFF, (clip->x2 - 1) & 0xFF);
	mipi_dbi_command(MIPI_DCS_SET_PAGE_ADDRESS,
	                 (clip->y1 >> 8) & 0xFF, clip->y1 & 0xFF,
	                 (clip->y2 >> 8) & 0xFF, (clip->y2 - 1) & 0xFF);

	mipi_dbi_command_buf(MIPI_DCS_WRITE_MEMORY_START, buf,
	                     (clip->x2 - clip->x1) * (clip->y2 - clip->y1) * 2);
}

static void ili9488_clear(u16 color, struct clip_rect *clip)
{
	mipi_dbi_command(MIPI_DCS_SET_COLUMN_ADDRESS,
	                 (clip->x1 >> 8) & 0xFF, clip->x1 & 0xFF,
	                 (clip->x2 >> 8) & 0xFF, (clip->x2 - 1) & 0xFF);
	mipi_dbi_command(MIPI_DCS_SET_PAGE_ADDRESS,
	                 (clip->y1 >> 8) & 0xFF, clip->y1 & 0xFF,
	                 (clip->y2 >> 8) & 0xFF, (clip->y2 - 1) & 0xFF);

	size_t num = (clip->x2 - clip->x1) * (clip->y2 - clip->y1) * 2;

	/* Assert CS */
	iowrite32(MIPI_DBI_B_CONTROL_CS, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
	/* Write command */
	iowrite8(MIPI_DCS_WRITE_MEMORY_START, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_COMMAND);
	/* Write data */
	while (0 < num) {
		u32 data = (color << 16) | color;
		iowrite32(data, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_DATA);
		num -= 4;
	}
	/* Deassert CS */
	iowrite32(0, MIPI_DBI_B_BASE + MIPI_DBI_B_REG_CONTROL);
}

static int do_zeus_display(cmd_tbl_t *cmdtp, int flag, int argc,
                           char * const argv[])
{
	/* Drop the 'display' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	if (0 == strcmp(argv[0], "init")) {
		backlight_enable();
		ili9488_enable();
	} else if (0 == strcmp(argv[0], "write.rgb565")) {
		if (6 != argc) {
			return CMD_RET_USAGE;
		}
		void *buf = (void *)simple_strtoul(argv[1], NULL, 16);
		struct clip_rect clip = {
			.x1 = simple_strtoul(argv[2], NULL, 10),
			.x2 = simple_strtoul(argv[3], NULL, 10),
			.y1 = simple_strtoul(argv[4], NULL, 10),
			.y2 = simple_strtoul(argv[5], NULL, 10),
		};
		ili9488_write(buf, &clip);
	} else if (0 == strcmp(argv[0], "clear.rgb565")) {
		if (6 != argc) {
			return CMD_RET_USAGE;
		}
		s16 color = simple_strtoul(argv[1], NULL, 16);
		struct clip_rect clip = {
			.x1 = simple_strtoul(argv[2], NULL, 10),
			.x2 = simple_strtoul(argv[3], NULL, 10),
			.y1 = simple_strtoul(argv[4], NULL, 10),
			.y2 = simple_strtoul(argv[5], NULL, 10),
		};
		ili9488_clear(color, &clip);
	}
	return CMD_RET_SUCCESS;
}

/*
 * Lock-in Amplifier
 *
 * Based on drivers/char/sbt_lockamp from the Linux repo
 */

/* GPIO-based regulators */
#define REG_OSC_125MHZ_GPIO		60
#define REG_12V_AND_5W_GPIO		61
/* lia_gpio (AXI GPIO v2.0) */
#define LIA_RESET_GPIO_BASE		(volatile void __iomem *)0x41220000
/* custom lock-in amplifier module */
#define LOCKAMP_BASE			(volatile void __iomem *)0x43c00000
#define LOCKAMP_REG_VERSION		0x00
#define LOCKAMP_REG_BUF_FILL	0x04
#define LOCKAMP_REG_BUF_DATA	0x08

struct site_sample {
	u32 hf_re, hf_im, lf_re, lf_im;
};

struct sample {
	struct site_sample sites[2];
};

static u32 lockamp_fifo_fill(void)
{
	return ioread32(LOCKAMP_BASE + LOCKAMP_REG_BUF_FILL) / 8;
}

static u32 lockamp_fifo_pop(void)
{
	return ioread32(LOCKAMP_BASE + LOCKAMP_REG_BUF_DATA);
}

static void lockamp_fifo_pop_sample(struct sample *s)
{
	int i;
	for (i = 0; 2 > i; ++i) {
		s->sites[i].hf_re = lockamp_fifo_pop();
		s->sites[i].hf_im = lockamp_fifo_pop();
		s->sites[i].lf_re = lockamp_fifo_pop();
		s->sites[i].lf_im = lockamp_fifo_pop();
	}
}

static void print_site_sample(struct site_sample *si)
{
	printf("%08x %08x %08x %08x\n", si->hf_re, si->hf_im, si->lf_re, si->lf_im);
}

static void print_sample(struct sample *s)
{
	struct site_sample *si;
	print_site_sample(&s->sites[0]);
	print_site_sample(&s->sites[1]);
}

static int do_zeus_lockamp(cmd_tbl_t *cmdtp, int flag, int argc,
                           char * const argv[])
{
	/* Drop the 'lockamp' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	if (0 == strcmp(argv[0], "init")) {
		/* Enable 125 MHz oscillator */
		//gpio_direction_output(REG_OSC_125MHZ_GPIO, 1);
		/* Enable analogue eletronics */
		//gpio_direction_output(REG_12V_AND_5W_GPIO, 1);
		/* Deassert reset GPIO (by setting GPIO_DATA reg to 0) */
		iowrite32(0, LIA_RESET_GPIO_BASE);
		/* Display version */
		u32 version = ioread32(LOCKAMP_BASE + LOCKAMP_REG_VERSION);
		printf("lockamp HW version: %x\n", version);
	} else if (0 == strcmp(argv[0], "read")) {
		struct sample s;
		lockamp_fifo_pop_sample(&s);
		print_sample(&s);
	} else if (0 == strcmp(argv[0], "discard")) {
		if (2 != argc) {
			return CMD_RET_USAGE;
		}
		u32 count = simple_strtoul(argv[1], NULL, 16);
		printf("iterations: %d\n", count);
		struct sample s;
		for (int i = 0; count > i; ++i) {
			u32 fill = lockamp_fifo_fill();
			printf("fill level: %d\n", fill);
			for (int j = 0; fill > j; ++j) {
				lockamp_fifo_pop_sample(&s);
			}
			udelay(89043);
		}
	}
	return CMD_RET_SUCCESS;
}


static cmd_tbl_t cmd_zeus[] = {
	U_BOOT_CMD_MKENT(display, 7, 1, do_zeus_display, "", ""),
	U_BOOT_CMD_MKENT(lockamp, 3, 1, do_zeus_lockamp, "", ""),
};

static int do_zeus(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *cp;

	cp = find_cmd_tbl(argv[1], cmd_zeus, ARRAY_SIZE(cmd_zeus));

	/* Drop the 'zeus' param */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs) {
		return CMD_RET_USAGE;
	}
	if (flag == CMD_FLAG_REPEAT && !cp->repeatable) {
		return CMD_RET_SUCCESS;
	}

	return cp->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(
	zeus, 8, 1, do_zeus,
	"Board-specific commands",
	"display init\n"
	"zeus display write.rgb565 address x1 x2 y1 y2\n"
	"zeus display clear.rgb565 color x1 x2 y1 y2\n"
	"zeus lockamp ...\n"
);

