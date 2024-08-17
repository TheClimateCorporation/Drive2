// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2019 Climate Corporation
 *
 */
#include <common.h>
#include <spl.h>
#include <linux/delay.h>
#include <asm/u-boot.h>
#include <usb.h>
#include <g_dnl.h>
#include <fastboot.h>

__weak void spl_set_fastboot_mode(void)
{
	/* Does nothing */
}

static int do_fastboot_usb(void)
{
	int ret;


        usb_gadget_initialize(CONFIG_FASTBOOT_USB_DEV);

	g_dnl_clear_detach();
	ret = g_dnl_register("usb_dnl_fastboot");
	if (ret)
		return ret;

	if (!g_dnl_board_usb_cable_connected()) {
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
		printf("USB cable not detected.\n");
#endif
		ret = -1;
		goto exit;
	}
#ifdef CONFIG_SPL_LIBCOMMON_SUPPORT
	printf("USB cable detected.\n");
#endif

	while (1) {
		if (g_dnl_detach())
			break;
		usb_gadget_handle_interrupts(CONFIG_FASTBOOT_USB_DEV);
	}

	ret = 0;

exit:
	g_dnl_unregister();
	g_dnl_clear_detach();
	board_usb_cleanup(CONFIG_FASTBOOT_USB_DEV, USB_INIT_DEVICE);

	return ret;
}

static int spl_fastboot_load_image(struct spl_image_info *spl_image,
				   struct spl_boot_device *bootdev)
{
        fastboot_init(NULL, 0);
	spl_set_fastboot_mode();	// board-specific fastboot/SPL setup

        return do_fastboot_usb();
}
SPL_LOAD_IMAGE_METHOD("FASTBOOT USB", 9, BOOT_DEVICE_FASTBOOT, spl_fastboot_load_image);
