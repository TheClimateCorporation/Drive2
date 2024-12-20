/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Common header file for U-Boot
 *
 * This file still includes quite a few headers that should be included
 * individually as needed. Patches to remove things are welcome.
 *
 * (C) Copyright 2000-2009
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

#ifndef __COMMON_H_
#define __COMMON_H_	1

#ifndef __ASSEMBLY__		/* put C only stuff in this section */
#include <config.h>
#include <errno.h>
#include <time.h>
#include <linux/types.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <stdarg.h>
#include <stdio.h>
#include <linux/kernel.h>
#ifdef CONFIG_CMD_FITFIND
#include <linux/libfdt.h>
#endif
#include <asm/u-boot.h> /* boot information for Linux kernel */
#include <asm/global_data.h>	/* global data used for startup functions */
#include <display_options.h>

#ifdef CONFIG_CMD_FITFIND
/**
 * Check if fdt is better in a board-specific way.
 *
 * If potential fdt is better than current best, update best pointer
 * @param potential_fdt  FDT pointer under consideration
 * @param best_fdt Best FDT found so far (may be null)
 *
 * @return
 *  - negative on error
 *  - distance metric (0 is perfect match)
 */
__weak int board_is_fdt_better(struct fdt_header *potential_fdt, struct fdt_header **best_fdt);
__weak int board_is_fdt_secure(struct fdt_header *potential_fdt);
#endif
#include <vsprintf.h>
#endif	/* __ASSEMBLY__ */

/* Pull in stuff for the build system */
#ifdef DO_DEPS_ONLY
# include <env_internal.h>
#endif

#endif	/* __COMMON_H_ */
