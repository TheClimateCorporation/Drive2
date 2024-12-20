/*
 * am335x_evm.h
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __CONFIG_AM335X_EVM_H
#define __CONFIG_AM335X_EVM_H

#include <configs/ti_am335x_common.h>
#include <linux/sizes.h>

#ifndef CONFIG_SPL_BUILD
# define CONFIG_TIMESTAMP
#endif

#define CONFIG_SYS_BOOTM_LEN		SZ_16M

#define CONFIG_MACH_TYPE		MACH_TYPE_AM335XEVM

/* Clock Defines */
#define V_OSCK				24000000  /* Clock output from T2 */
#define V_SCLK				(V_OSCK)

#ifdef CONFIG_MTD_RAW_NAND
#define NANDARGS \
	"mtdids=" CONFIG_MTDIDS_DEFAULT "\0" \
	"mtdparts=" CONFIG_MTDPARTS_DEFAULT "\0" \
	"nandargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${nandroot} " \
		"rootfstype=${nandrootfstype}\0" \
	"nandroot=ubi0:rootfs rw ubi.mtd=NAND.file-system,2048\0" \
	"nandrootfstype=ubifs rootwait=1\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		"nand read ${fdtaddr} NAND.u-boot-spl-os; " \
		"nand read ${loadaddr} NAND.kernel; " \
		"bootz ${loadaddr} - ${fdtaddr}\0"
#else
#define NANDARGS ""
#endif

#define RAWMMCARGS \
       "rawboot=echo Booting from raw emmc partition ${bootpartname}-${slot}...; " \
               "run rawargs; "\
               "mmc dev ${mmcdev}; " \
               "mmc read ${loadaddr} ${raw_start_addr} ${raw_size}; "\
               "if iminfo ${loadaddr}; "\
               "then " \
               "  if test -n \"${configname}\"; "\
               "  then " \
               "    bootm ${loadaddr}${configname}; "\
               "  else "\
               "    fitfind ${loadaddr}; " \
               "    if test -n \"${fitkernel}\" && test -n \"${fitramdisk}\" && test -n \"${fitbestfdt}\"; "\
               "    then " \
               "      bootm ${loadaddr}:${fitkernel} ${loadaddr}:${fitramdisk} ${loadaddr}:${fitbestfdt};"\
               "    fi; "\
               "  fi; "\
               "fi; "\
               "run bootcmd_fastboot;\0"
  
#define BOOTENV_DEV_RAW_MMC(devtypeu, devtypel, instance) \
       "bootcmd_" #devtypel #instance "=" \
       "part start mmc " #instance " ${bootpartname}-${slot} raw_start_addr;" \
       "part size mmc " #instance " ${bootpartname}-${slot} raw_size;" \
       "part number mmc " #instance " ${rootpartname}-${slot} root_partnum_h;" \
       "h2d $root_partnum_h root_partnum_d;" \
       "setenv rawroot /dev/mmcblk" #instance "p${root_partnum_d};" \
       "setenv mmcdev " #instance"; "\
       "run rawboot\0"

#define BOOTENV_DEV_NAME_RAW_MMC(devtypeu, devtypel, instance) \
       #devtypel #instance " "

#define BOOTENV_DEV_LEGACY_MMC(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel #instance "=" \
	"setenv mmcdev " #instance"; "\
	"setenv bootpart " #instance":2 ; "\
	"run mmcboot\0"

#define BOOTENV_DEV_NAME_LEGACY_MMC(devtypeu, devtypel, instance) \
	#devtypel #instance " "

#define BOOTENV_DEV_NAND(devtypeu, devtypel, instance) \
	"bootcmd_" #devtypel "=" \
	"run nandboot\0"

#define BOOTENV_DEV_NAME_NAND(devtypeu, devtypel, instance) \
	#devtypel #instance " "

#if CONFIG_IS_ENABLED(CMD_PXE)
# define BOOT_TARGET_PXE(func) func(PXE, pxe, na)
#else
# define BOOT_TARGET_PXE(func)
#endif

#if CONFIG_IS_ENABLED(CMD_DHCP)
# define BOOT_TARGET_DHCP(func) func(DHCP, dhcp, na)
#else
# define BOOT_TARGET_DHCP(func)
#endif

#ifdef CONFIG_RECOVERY_MODE
#define BOOT_TARGET_DEVICES(func) \
	func(FASTBOOT, usb, 0)
#else
#define BOOT_TARGET_DEVICES(func) \
	func(RAW_MMC, raw_mmc, 1) \
	func(FASTBOOT, usb, 0)
#endif

#include <config_distro_bootcmd.h>

#ifndef CONFIG_SPL_BUILD
#include <environment/ti/dfu.h>
#include <environment/ti/mmc.h>

#ifdef PARTITIONS_LARGE_ROOTFS
/* Make rootfs 2GB for debug builds; requires blankflash to
 * move between debug and other builds types */
#define PARTITION_TABLE "partitions=" \
		"name=spl,start=128KiB,size=128KiB;" \
		"name=spl-bak,start=256KiB,size=128KiB;" \
		"name=keys,start=384KiB,size=128Kib;" \
		"name=factory,start=512KiB,size=32256KiB;" \
		"name=u-boot-a,start=32MiB,size=2MiB;" \
		"name=u-boot-b,start=34MiB,size=2MiB;" \
		"name=kernel-a,start=36MiB,size=14MiB,type=linux,bootable;" \
		"name=kernel-b,start=50MiB,size=14MiB,type=linux,bootable;" \
		"name=root-a,start=64MiB,size=2048MiB,type=system;" \
		"name=root-b,start=2112MiB,size=2048MiB,type=system;" \
		"name=data,start=4160Mb,type=data,size=-;\0"
#else
#define PARTITION_TABLE "partitions=" \
		"name=spl,start=128KiB,size=128KiB;" \
		"name=spl-bak,start=256KiB,size=128KiB;" \
		"name=keys,start=384KiB,size=128Kib;" \
		"name=factory,start=512KiB,size=32256KiB;" \
		"name=u-boot-a,start=32MiB,size=2MiB;" \
		"name=u-boot-b,start=34MiB,size=2MiB;" \
		"name=kernel-a,start=36MiB,size=14MiB,type=linux,bootable;" \
		"name=kernel-b,start=50MiB,size=14MiB,type=linux,bootable;" \
		"name=root-a,start=64MiB,size=256MiB,type=system;" \
		"name=root-b,start=320MiB,size=256MiB,type=system;" \
		"name=data,start=576Mb,type=data,size=-;\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
	DEFAULT_LINUX_BOOT_ENV \
	DEFAULT_MMC_TI_ARGS \
	DEFAULT_FIT_TI_ARGS \
	"bootpart=1:6\0" \
	"bootpartname=kernel\0" \
	"rootpartname=root\0" \
	"bootdir=/boot\0" \
	"bootfile=zImage\0" \
	"bootmenu_delay=0\0" \
	"fdtfile=undefined\0" \
	"console=ttyS0,115200n8\0" \
         PARTITION_TABLE \
        "fastboot.slot-count=2\0" \
        "fastboot.partition-type:factory=ext4\0" \
        "fastboot.partition-type:root=ext4\0" \
        "fastboot.partition-type:root-bak=ext4\0" \
        "fastboot.partition-type:cache=ext4\0" \
        "fastboot.partition-type:data=ext4\0" \
        "fastboot_partition_alias_u-boot=u-boot-a\0" \
        "fastboot_partition_alias_kernel=kernel-a\0" \
        "fastboot_partition_alias_root=root-a\0" \
        "fastboot_led=i2c:green pwm:green i2c:blue pwm:blue\0" \
        "fastboot_led_off=i2c:red pwm:red\0" \
        "upgrade_led_on=i2c:green pwm:green\0" \
        "upgrade_led_flashing=i2c:blue pwm:blue\0" \
        "upgrade_led_off=i2c:red pwm:red\0" \
	"bootmenu_0=Fastboot mode=run bootcmd_fastboot\0" \
	"bootmenu_1=Boot (auto detect hardware)=run bootcmd_raw_mmc1\0" \
	"bootmenu_2=Reboot=reset\0" \
	"ethsetup=env set optargs ${optargs} dev_addr=${ethaddr} host_addr=${eth1addr}\0" \
	"optargs=\0" \
	"secureargs=\0" \
	"rawrootfstype=ext4\0" \
	"ramroot=/dev/ram0 rw\0" \
	"ramrootfstype=ext2\0" \
	"spiroot=/dev/mtdblock4 rw\0" \
	"spirootfstype=jffs2\0" \
	"spisrcaddr=0xe0000\0" \
	"spiimgsize=0x362000\0" \
	"spibusno=0\0" \
	"spiargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${spiroot} " \
		"rootfstype=${spirootfstype}\0" \
	"ramargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"root=${ramroot} " \
		"rootfstype=${ramrootfstype}\0" \
	"rawargs=setenv bootargs console=${console} " \
		"${optargs} " \
		"${secureargs} " \
		"root=${rawroot} " \
		"rootfstype=${rawrootfstype}\0" \
	"loadramdisk=load mmc ${mmcdev} ${rdaddr} ramdisk.gz\0" \
	"spiboot=echo Booting from spi ...; " \
		"run spiargs; " \
		"sf probe ${spibusno}:0; " \
		"sf read ${loadaddr} ${spisrcaddr} ${spiimgsize}; " \
		"bootz ${loadaddr}\0" \
	"ramboot=echo Booting from ramdisk ...; " \
		"run ramargs; " \
		"bootz ${loadaddr} ${rdaddr} ${fdtaddr}\0" \
	"findfdt="\
		"if test $board_name = A335BONE; then " \
			"setenv fdtfile am335x-bone.dtb; fi; " \
		"if test $board_name = A335BNLT; then " \
			"setenv fdtfile am335x-boneblack.dtb; fi; " \
		"if test $board_name = A335PBGL; then " \
			"setenv fdtfile am335x-pocketbeagle.dtb; fi; " \
		"if test $board_name = BBBW; then " \
			"setenv fdtfile am335x-boneblack-wireless.dtb; fi; " \
		"if test $board_name = BBG1; then " \
			"setenv fdtfile am335x-bonegreen.dtb; fi; " \
		"if test $board_name = BBGW; then " \
			"setenv fdtfile am335x-bonegreen-wireless.dtb; fi; " \
		"if test $board_name = BBBL; then " \
			"setenv fdtfile am335x-boneblue.dtb; fi; " \
		"if test $board_name = BBEN; then " \
			"setenv fdtfile am335x-sancloud-bbe.dtb; fi; " \
		"if test $board_name = A33515BB; then " \
			"setenv fdtfile am335x-evm.dtb; fi; " \
		"if test $board_name = A335X_SK; then " \
			"setenv fdtfile am335x-evmsk.dtb; fi; " \
		"if test $board_name = A335_ICE; then " \
			"setenv fdtfile am335x-icev2.dtb; fi; " \
		"if test $fdtfile = undefined; then " \
			"echo WARNING: Could not determine device tree to use; fi; \0" \
	"init_console=" \
		"if test $board_name = A335_ICE; then "\
			"setenv console ttyO3,115200n8;" \
		"else " \
			"setenv console ttyO0,115200n8;" \
		"fi;\0" \
	NANDARGS \
        RAWMMCARGS \
	NETARGS \
	DFUARGS \
	BOOTENV
#endif

/* Preserved Memory */
#define CONFIG_PRAM			512            /* Reserve top 512KB for pstore */

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550_COM1		0x44e09000	/* Base EVM has UART0 */
#define CONFIG_SYS_NS16550_COM2		0x48022000	/* UART1 */
#define CONFIG_SYS_NS16550_COM3		0x48024000	/* UART2 */
#define CONFIG_SYS_NS16550_COM4		0x481a6000	/* UART3 */
#define CONFIG_SYS_NS16550_COM5		0x481a8000	/* UART4 */
#define CONFIG_SYS_NS16550_COM6		0x481aa000	/* UART5 */

#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50	/* Main EEPROM */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	2

/* MMC1 Configuration */
#define CONFIG_HSMMC2_8BIT              /* 8-bit MMC support */

/* PMIC support */
#define CONFIG_POWER_TPS65217
#define CONFIG_POWER_TPS65910

/* SPL */
#ifndef CONFIG_NOR_BOOT
/* Bootcount using the RTC block */
//#define CONFIG_SYS_BOOTCOUNT_BE

/* USB gadget RNDIS */
#endif

#ifdef CONFIG_MTD_RAW_NAND
/* NAND: device related configs */
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_PAGE_COUNT	(CONFIG_SYS_NAND_BLOCK_SIZE / \
					 CONFIG_SYS_NAND_PAGE_SIZE)
#define CONFIG_SYS_NAND_PAGE_SIZE	2048
#define CONFIG_SYS_NAND_OOBSIZE		64
#define CONFIG_SYS_NAND_BLOCK_SIZE	(128*1024)
/* NAND: driver related configs */
#define CONFIG_SYS_NAND_BAD_BLOCK_POS	NAND_LARGE_BADBLOCK_POS
#define CONFIG_SYS_NAND_ECCPOS		{ 2, 3, 4, 5, 6, 7, 8, 9, \
					 10, 11, 12, 13, 14, 15, 16, 17, \
					 18, 19, 20, 21, 22, 23, 24, 25, \
					 26, 27, 28, 29, 30, 31, 32, 33, \
					 34, 35, 36, 37, 38, 39, 40, 41, \
					 42, 43, 44, 45, 46, 47, 48, 49, \
					 50, 51, 52, 53, 54, 55, 56, 57, }

#define CONFIG_SYS_NAND_ECCSIZE		512
#define CONFIG_SYS_NAND_ECCBYTES	14
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_NAND_OMAP_ECCSCHEME	OMAP_ECC_BCH8_CODE_HW
#define CONFIG_SYS_NAND_U_BOOT_OFFS	0x000c0000
/* NAND: SPL related configs */
#ifdef CONFIG_SPL_OS_BOOT
#define CONFIG_SYS_NAND_SPL_KERNEL_OFFS	0x00200000 /* kernel offset */
#endif
#endif /* !CONFIG_MTD_RAW_NAND */

/*
 * For NOR boot, we must set this to the start of where NOR is mapped
 * in memory.
 */

/*
 * USB configuration.  We enable MUSB support, both for host and for
 * gadget.  We set USB0 as peripheral and USB1 as host, based on the
 * board schematic and physical port wired to each.  Then for host we
 * add mass storage support and for gadget we add both RNDIS ethernet
 * and DFU.
 */
#define CONFIG_AM335X_USB0
#define CONFIG_AM335X_USB0_MODE	MUSB_PERIPHERAL
#define CONFIG_AM335X_USB1
#define CONFIG_AM335X_USB1_MODE MUSB_HOST

/*
 * Disable MMC DM for SPL build and can be re-enabled after adding
 * DM support in SPL
 */
#ifdef CONFIG_SPL_BUILD
#undef CONFIG_DM_MMC
#undef CONFIG_TIMER
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_USB_ETHER)
/* Remove other SPL modes. */
/* disable host part of MUSB in SPL */
/* disable EFI partitions and partition UUID support */
#endif

/* USB Device Firmware Update support */
#ifndef CONFIG_SPL_BUILD
#define DFUARGS \
	DFU_ALT_INFO_EMMC \
	DFU_ALT_INFO_MMC \
	DFU_ALT_INFO_RAM \
	DFU_ALT_INFO_NAND
#endif

/*
 * Default to using SPI for environment, etc.
 * 0x000000 - 0x020000 : SPL (128KiB)
 * 0x020000 - 0x0A0000 : U-Boot (512KiB)
 * 0x0A0000 - 0x0BFFFF : First copy of U-Boot Environment (128KiB)
 * 0x0C0000 - 0x0DFFFF : Second copy of U-Boot Environment (128KiB)
 * 0x0E0000 - 0x442000 : Linux Kernel
 * 0x442000 - 0x800000 : Userland
 */
#if defined(CONFIG_SPI_BOOT)
/* SPL related */
#elif defined(CONFIG_EMMC_BOOT)
#define CONFIG_SYS_MMC_ENV_DEV		1
#define CONFIG_SYS_MMC_ENV_PART		2
#define CONFIG_SYS_MMC_MAX_DEVICE	2
#elif defined(CONFIG_ENV_IS_IN_NAND)
#define CONFIG_SYS_ENV_SECT_SIZE	CONFIG_SYS_NAND_BLOCK_SIZE
#endif

/* SPI flash. */

/* Network. */
/* Enable Atheros phy driver */

/*
 * NOR Size = 16 MiB
 * Number of Sectors/Blocks = 128
 * Sector Size = 128 KiB
 * Word length = 16 bits
 * Default layout:
 * 0x000000 - 0x07FFFF : U-Boot (512 KiB)
 * 0x080000 - 0x09FFFF : First copy of U-Boot Environment (128 KiB)
 * 0x0A0000 - 0x0BFFFF : Second copy of U-Boot Environment (128 KiB)
 * 0x0C0000 - 0x4BFFFF : Linux Kernel (4 MiB)
 * 0x4C0000 - 0xFFFFFF : Userland (11 MiB + 256 KiB)
 */
#if defined(CONFIG_NOR)
#define CONFIG_SYS_MAX_FLASH_SECT	128
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_FLASH_BASE		(0x08000000)
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#define CONFIG_SYS_FLASH_SIZE		0x01000000
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_FLASH_BASE
#endif  /* NOR support */

#ifdef CONFIG_DRIVER_TI_CPSW
#define CONFIG_CLOCK_SYNTHESIZER
#define CLK_SYNTHESIZER_I2C_ADDR 0x65
#endif

#endif	/* ! __CONFIG_AM335X_EVM_H */
