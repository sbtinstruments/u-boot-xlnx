/* SPDX-License-Identifier:	GPL-2.0-or-later */
/*
 * (C) Copyright 2019 Frederik Peter Aalund <fpa@sbtinstruments.com>
 *
 * Configuration settings for the SBT Instruments Green Mango board.
 * See zynq-common.h for Zynq common configs.
 *
 */

#ifndef __CONFIG_ZYNQ_GREEN_MANGO_H
#define __CONFIG_ZYNQ_GREEN_MANGO_H

#define CONFIG_ZYNQ_I2C0

#include <configs/zynq-common.h>

/* Override default environment */
#undef  CONFIG_EXTRA_ENV_SETTINGS
#define CONFIG_EXTRA_ENV_SETTINGS	\
	"fpa_test_from_uboot=123\0"	\
	"ethaddr=00:0a:35:00:01:22\0"	\
	"kernel_image=uImage\0"	\
	"kernel_load_address=0x2080000\0" \
	"ramdisk_image=uramdisk.image.gz\0"	\
	"ramdisk_load_address=0x4000000\0"	\
	"devicetree_image=devicetree.dtb\0"	\
	"devicetree_load_address=0x2000000\0"	\
	"bitstream_image=system.bit.bin\0"	\
	"boot_image=BOOT.bin\0"	\
	"loadbit_addr=0x100000\0"	\
	"loadbootenv_addr=0x2000000\0" \
	"kernel_size=0x500000\0"	\
	"devicetree_size=0x20000\0"	\
	"ramdisk_size=0x5E0000\0"	\
	"boot_size=0xF00000\0"	\
	"fdt_high=0x20000000\0"	\
	"initrd_high=0x20000000\0"	\
	"bootenv=uEnv.txt\0" \
	"loadbootenv=load mmc 0 ${loadbootenv_addr} ${bootenv}\0" \
	"importbootenv=echo Importing environment from SD ...; " \
		"env import -t ${loadbootenv_addr} $filesize\0" \
	"sd_uEnvtxt_existence_test=test -e mmc 0 /uEnv.txt\0" \
	"preboot=if test $modeboot = sdboot && env run sd_uEnvtxt_existence_test; " \
			"then if env run loadbootenv; " \
				"then env run importbootenv; " \
			"fi; " \
		"fi; \0" \
	"mmc_loadbit=echo Loading bitstream from SD/MMC/eMMC to RAM.. && " \
		"mmcinfo && " \
		"load mmc 0 ${loadbit_addr} ${bitstream_image} && " \
		"fpga load 0 ${loadbit_addr} ${filesize}\0" \
	"norboot=echo Copying Linux from NOR flash to RAM... && " \
		"cp.b 0xE2100000 ${kernel_load_address} ${kernel_size} && " \
		"cp.b 0xE2600000 ${devicetree_load_address} ${devicetree_size} && " \
		"echo Copying ramdisk... && " \
		"cp.b 0xE2620000 ${ramdisk_load_address} ${ramdisk_size} && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"qspiboot=echo Copying Linux from QSPI flash to RAM... && " \
		"sf probe 0 0 0 && " \
		"sf read ${kernel_load_address} 0x100000 ${kernel_size} && " \
		"sf read ${devicetree_load_address} 0x600000 ${devicetree_size} && " \
		"echo Copying ramdisk... && " \
		"sf read ${ramdisk_load_address} 0x620000 ${ramdisk_size} && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"uenvboot=" \
		"if run loadbootenv; then " \
			"echo Loaded environment from ${bootenv}; " \
			"run importbootenv; " \
		"fi; " \
		"if test -n $uenvcmd; then " \
			"echo Running uenvcmd ...; " \
			"run uenvcmd; " \
		"fi\0" \
	"sdboot=if mmcinfo; then " \
			"run uenvboot; " \
			"echo Copying Linux from SD to RAM... && " \
			"load mmc 0 ${kernel_load_address} ${kernel_image} && " \
			"load mmc 0 ${devicetree_load_address} ${devicetree_image} && " \
			"load mmc 0 ${ramdisk_load_address} ${ramdisk_image} && " \
			"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}; " \
		"fi\0" \
	"usbboot=if usb start; then " \
			"run uenvboot; " \
			"echo Copying Linux from USB to RAM... && " \
			"load usb 0 ${kernel_load_address} ${kernel_image} && " \
			"load usb 0 ${devicetree_load_address} ${devicetree_image} && " \
			"load usb 0 ${ramdisk_load_address} ${ramdisk_image} && " \
			"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}; " \
		"fi\0" \
	"nandboot=echo Copying Linux from NAND flash to RAM... && " \
		"nand read ${kernel_load_address} 0x100000 ${kernel_size} && " \
		"nand read ${devicetree_load_address} 0x600000 ${devicetree_size} && " \
		"echo Copying ramdisk... && " \
		"nand read ${ramdisk_load_address} 0x620000 ${ramdisk_size} && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"jtagboot=echo TFTPing Linux to RAM... && " \
		"tftpboot ${kernel_load_address} ${kernel_image} && " \
		"tftpboot ${devicetree_load_address} ${devicetree_image} && " \
		"tftpboot ${ramdisk_load_address} ${ramdisk_image} && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_norboot=echo Copying Image from NOR flash to RAM... && " \
		"cp.b 0xE2100000 0x100000 ${boot_size} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_nandboot=echo Copying Image from NAND flash to RAM... && " \
		"nand read 0x100000 0x0 ${boot_size} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_qspiboot=echo Copying Image from QSPI flash to RAM... && " \
		"sf probe 0 0 0 && " \
		"sf read 0x100000 0x0 ${boot_size} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_sdboot=echo Copying Image from SD to RAM... && " \
		"load mmc 0 0x100000 ${boot_image} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"rsa_jtagboot=echo TFTPing Image to RAM... && " \
		"tftpboot 0x100000 ${boot_image} && " \
		"zynqrsa 0x100000 && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
	"load_system_ver_address=0x2000000\0" \
	"load_system_ver_path=boot/version.txt\0" \
	"load_system0_ver=load mmc 0:1 ${load_system_ver_address} ${load_system_ver_path}\0" \
	"load_system1_ver=load mmc 0:2 ${load_system_ver_address} ${load_system_ver_path}\0" \
	"import_system_ver=env import -t $fileaddr $filesize\0" \
	"set_system0_version=if run load_system0_ver; then " \
		"run import_system_ver; " \
	"fi\0" \
	"set_system1_version=if run load_system1_ver; then " \
		"run import_system_ver; " \
	"fi\0" \
	"versioned_sdboot=echo Determining active system partition && " \
		"run set_system0_version; " \
		"run set_system1_version; " \
		"if test \"${system0_version}\" -gt \"${system1_version}\"; then " /* The active partition is the one with the greatest version number */ \
			"setenv active_partition 1; " /* System 0 */ \
		"else; " \
			"setenv active_partition 2; " /* System 1 */ \
		"fi; " \
		"echo Active system partition: ${active_partition} && " \
		"echo Copying Linux from SD to RAM... && " \
		"load mmc 0:${active_partition} ${kernel_load_address} boot/${kernel_image} && " \
		"load mmc 0:${active_partition} ${devicetree_load_address} boot/${devicetree_image} && " \
		"load mmc 0:${active_partition} ${ramdisk_load_address} boot/${ramdisk_image} && " \
		"bootm ${kernel_load_address} ${ramdisk_load_address} ${devicetree_load_address}\0" \
		DFU_ALT_INFO \
		BOOTENV
/*
echo "Loading from SD card to memory"
fatload mmc 0:$active_partition 0x2001000 boot/uImage
fatload mmc 0:$active_partition 0x3000000 boot/uramdisk.image.gz
fatload mmc 0:$active_partition 0x6000000 boot/devicetree.dtb" \
*/

#endif /* __CONFIG_ZYNQ_GREEN_MANGO_H */