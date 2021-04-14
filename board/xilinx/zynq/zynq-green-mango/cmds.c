// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Commands to control Ilitek ILI9488 panels.
 *
 * Supported panels:
 *   * URT P220MD-T
 *
 * Copyright 2019 Frederik Peter Aalund <fpa@sbtinstruments.com>
 */
#include <asm/gpio.h>
#include <common.h>
#include <command.h>
#include <dm/device.h>
#include <dm/uclass.h>
#include <env.h>
#include <errno.h>
#include <fs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <power/da9063_pmic.h>
#include <power/pmic.h>
#include <video.h>
#include <video_console.h>
#include "files.h"

// For register EVENT_A
#define DA9063_E_NONKEY BIT(0)
// For register EVENT_B
#define DA9063_E_WAKE BIT(0)
// For register CONTROL_F
#define DA9063_SHUTDOWN BIT(1)


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
	print_site_sample(&s->sites[0]);
	print_site_sample(&s->sites[1]);
}

static int do_mango_lockamp(struct cmd_tbl *cmdtp, int flag, int argc,
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

#ifndef CONFIG_SPL_BUILD

static int do_mango_video(struct cmd_tbl *cmdtp, int flag, int argc,
                           char * const argv[])
{
	struct udevice *dev, *console_dev;
	struct video_priv *priv;
	struct vidconsole_priv *console_priv;
	int ret, x, y;

	/* drop the 'video' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	/* get video device */
	if (uclass_first_device_err(UCLASS_VIDEO, &dev)) {
		return CMD_RET_FAILURE;
	}
	priv = dev_get_uclass_priv(dev);

	/* fg.hex: set foreground color */
	if (0 == strcmp(argv[0], "fg.hex")) {
		if (2 > argc) {
			return CMD_RET_USAGE;
		}
		priv->colour_fg = simple_strtoul(argv[1], NULL, 16);
		return CMD_RET_SUCCESS;
	}
	/* bg.hex: set background color */
	if (0 == strcmp(argv[0], "bg.hex")) {
		if (2 > argc) {
			return CMD_RET_USAGE;
		}
		priv->colour_bg = simple_strtoul(argv[1], NULL, 16);
		return CMD_RET_SUCCESS;
	}
	/* clear: fill display with background color */
	if (0 == strcmp(argv[0], "clear")) {
		if (video_clear(dev)) {
			return CMD_RET_FAILURE;
		}
		return CMD_RET_SUCCESS;
	}
	/* cursor.px: set the video console cursor in pixel coordinates
	 *
	 * Alternatively, use the "setcurs" command that uses column/row coordinates.
	 */
	if (0 == strcmp(argv[0], "cursor.px")) {
		if (3 > argc) {
			return CMD_RET_USAGE;
		}
		x = simple_strtoul(argv[1], NULL, 10);
		y = simple_strtoul(argv[2], NULL, 10);
		/* get video console device */
		ret = uclass_get_device(UCLASS_VIDEO_CONSOLE, 0, &console_dev);
		if (ret) {
			log_err("Could not get video console device: %d.\n", ret);
			return CMD_RET_FAILURE;
		}
		console_priv = dev_get_uclass_priv(console_dev);
		/* set cursor */
		console_priv->xcur_frac = VID_TO_POS(min_t(short, x, priv->xsize - 1));
		console_priv->xstart_frac = console_priv->xcur_frac;
		console_priv->ycur = min_t(short, y, priv->ysize - 1);
		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

static int da9063_shutdown(struct udevice *dev) {
	int ret;
	printf("PMIC shuts down NOW. If this succeeds, this is the last message for now.\n");
	ret = pmic_reg_write(dev, DA9063_REG_CONTROL_F, DA9063_SHUTDOWN);
	if (ret) {
		return ret;
	}
	return 0;
}

static int do_mango_pmic(struct cmd_tbl *cmdtp, int flag, int argc,
                           char * const argv[])
{
	struct udevice *dev;
	int ret;

	/* drop the 'pmic' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	/* get the PMIC device (there is only one on the Green Mango platform) */
	ret = pmic_get("da9063@58", &dev);
	if (ret) {
		log_err("Could not get PMIC device: %d.\n", ret);
		return CMD_RET_FAILURE;
	}

	/* wakereason: what caused the PMIC to wake up? */
	if (0 == strcmp(argv[0], "wakereason")) {
		int event_a = pmic_reg_read(dev, DA9063_REG_EVENT_A);
		if (event_a < 0) {
			log_err("Could not read EVENT_A register: %d.\n", event_a);
			return CMD_RET_FAILURE;
		}
		int event_b = pmic_reg_read(dev, DA9063_REG_EVENT_B);
		if (event_b < 0) {
			log_err("Could not read EVENT_B register: %d.\n", event_b);
			return CMD_RET_FAILURE;
		}
		char *reason;
		/* user pressed the on/off key */
		if (event_a & DA9063_E_NONKEY) {
			reason = "on-off-key";
		}
		/* the E_WAKE bit is set when the PMIC detecs a rising edge
		 * on CHG_WAKE. */
		else if (event_b & DA9063_E_WAKE) {
			reason = "charging";
		}
		else {
			reason = "unknown";
		}
		if (2 > argc) {
			/* simply print the result */
			printf("%s\n", reason);
			return CMD_RET_SUCCESS;
		}
		/* use the second arg to store the result */
		ret = env_set(argv[1], reason);
		if (ret) {
			log_err("Could not set the '%s' environment variable: %d.\n", argv[1], ret);
			return CMD_RET_FAILURE;
		}
		return CMD_RET_SUCCESS;
	}
	/* shutdown: power down the PMIC immediately */
	if (0 == strcmp(argv[0], "shutdown")) {
		if (da9063_shutdown(dev)) {
			log_err("Could not issue shutdown: %d.\n", ret);
			return CMD_RET_FAILURE;
		}
		/* We'll probably not even get to this point since the CPU
		 * just lost it's power source. */
		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

static int do_mango_resource(struct cmd_tbl *cmdtp, int flag, int argc,
                           char * const argv[])
{
	int ret;
	uintptr_t resource_ptr;
	char buf[32];

	/* drop the 'resource' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	/* getaddr: get address of resource */
	if (0 == strcmp(argv[0], "getaddr")) {
		if (2 > argc) {
			return CMD_RET_USAGE;
		}
		if (0 == strcmp(argv[1], "battery-charging.bmp")) {
			resource_ptr = (uintptr_t) mango_files_battery_charging_bmp;
		}
		else {
			printf("No such resource\n");
			return CMD_RET_FAILURE;
		}
		snprintf(buf, sizeof(buf), "0x%lx", resource_ptr);
		if (2 > argc) {
			/* simply print the result */
			printf("%s\n", buf);
			return CMD_RET_SUCCESS;
		}
		/* use the third arg to store the result */
		ret = env_set(argv[2], buf);
		if (ret) {
			log_err("Could not set the '%s' environment variable: %d.\n", argv[2], ret);
			return CMD_RET_FAILURE;
		}
		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

static int do_mango_config(struct cmd_tbl *cmdtp, int flag, int argc,
                           char * const argv[])
{
	int ret;

	/* drop the 'config' param */
	argc--;
	argv++;

	if (1 > argc) {
		return CMD_RET_USAGE;
	}

	/* load: open and read a file into memory
	 *
	 * Inspired by "do_load" in "fs/fs.c".
	 *
	 * TODO: Prefer files from the "override" folder over
	 * the "individual" folder. Like a light version of
	 * OverlayFS.
	 */
	if (0 == strcmp(argv[0], "load")) {
		if (3 > argc || 6 < argc) {
			return CMD_RET_USAGE;
		}
		/* hard-coded to use the "CONFIG" partition */
		ret = fs_set_blk_dev("mmc", "0:3", FS_TYPE_ANY);
		if (ret) {
			log_err("Could not set block device: %d\n", ret);
			return CMD_RET_FAILURE;
		}
		/* arg 1: address */
		char *ep;
		unsigned long addr = simple_strtoul(argv[1], &ep, 16);
		if (ep == argv[1] || *ep != '\0') {
			return CMD_RET_USAGE;
		}
		/* arg 2: file name */
		const char *file_name = argv[2];
		/* arg 3 (optional): max bytes to read */
		loff_t bytes;
		if (argc >= 4)
			bytes = simple_strtoul(argv[3], NULL, 16);
		else
			bytes = 0;
		/* arg 5 (optional): offset into file */
		loff_t pos;
		if (argc >= 5)
			pos = simple_strtoul(argv[4], NULL, 16);
		else
			pos = 0;

		loff_t len_read;
		unsigned long time;
		time = get_timer(0);
		ret = fs_read(file_name, addr, pos, bytes, &len_read);
		time = get_timer(time);
		if (ret < 0) {
			log_err("Failed to load '%s'\n", file_name);
			return CMD_RET_FAILURE;
		}

		printf("%llu bytes read in %lu ms", len_read, time);
		if (time > 0) {
			puts(" (");
			print_size(div_u64(len_read, time) * 1000, "/s");
			puts(")");
		}
		puts("\n");

		return CMD_RET_SUCCESS;
	}
	return CMD_RET_USAGE;
}

#endif

static struct cmd_tbl cmd_mango[] = {
	U_BOOT_CMD_MKENT(lockamp, 3, 1, do_mango_lockamp, "", ""),
#ifndef CONFIG_SPL_BUILD
	U_BOOT_CMD_MKENT(video, 5, 1, do_mango_video, "", ""),
	U_BOOT_CMD_MKENT(pmic, 3, 1, do_mango_pmic, "", ""),
	U_BOOT_CMD_MKENT(config, 7, 1, do_mango_config, "", ""),
	U_BOOT_CMD_MKENT(resource, 4, 1, do_mango_resource, "", ""),
#endif
};

static int do_mango(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	struct cmd_tbl *cp;

	cp = find_cmd_tbl(argv[1], cmd_mango, ARRAY_SIZE(cmd_mango));

	/* drop the 'mango' param */
	argc--;
	argv++;

	if (cp == NULL || argc > cp->maxargs) {
		return CMD_RET_USAGE;
	}
	if (flag == CMD_FLAG_REPEAT && !cmd_is_repeatable(cp)) {
		return CMD_RET_SUCCESS;
	}

	return cp->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(
	mango, 8, 1, do_mango,
	"Board-specific commands",
	"mango lockamp ...\n"
#ifndef CONFIG_SPL_BUILD
	"mango video ...\n"
	"mango pmic ...\n"
	"mango config ...\n"
	"mango resource ...\n"
#endif
);
