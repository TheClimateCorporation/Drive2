/*
 * (C) Copyright 2017
 * The Climate Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <command.h>
#include <linux/libfdt.h>
#include <fdt_support.h>
#include <mapmem.h>

#define MAX_RD_NAME_LEN 32

static int fdt_valid(struct fdt_header **blobp);

/*
 * Global data (for the gd->bd)
 */
DECLARE_GLOBAL_DATA_PTR;

/*
 * FIT find, see the help for parameter definitions.
 */

static int do_fitfind(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	unsigned long addr;
	struct fdt_header *fit_fdt, *best_fdt = NULL;
	int  images, nodeoffset, ramdiskfound = 0, kernelfound = 0;
	const char *name;
	const char *subname;
	const char *best_fdtsubname = NULL;
	const struct fdt_property *fdt_prop;

	if (argc < 2)
		return CMD_RET_USAGE;

	/*
	 * Set the address of the FIT image
	 */

	addr = simple_strtoul(argv[1], NULL, 16);
	fit_fdt = map_sysmem(addr, 0);
	if (!fdt_valid(&fit_fdt)) {
		return CMD_RET_FAILURE;
	}

	images = fdt_path_offset (fit_fdt, "/images");
	if (images < 0) {
		printf ("No /images record found\n");
		return CMD_RET_FAILURE;
	}
	
	fdt_for_each_subnode(nodeoffset, fit_fdt, images) {
		fdt_prop = fdt_get_property(fit_fdt, nodeoffset, "type", NULL);
		if (!fdt_prop)
			break;
		name = fdt_prop->data;
		if (!kernelfound && !strcmp(name, "kernel")) {
			subname = fdt_get_name(fit_fdt, nodeoffset, NULL);
                        if (subname) {
				env_set("fitkernel", subname);
				kernelfound = 1;
			}
		} else if (!ramdiskfound && !strcmp(name, "ramdisk")) {
			subname = fdt_get_name(fit_fdt, nodeoffset, NULL);
                        if (subname) {
				env_set("fitramdisk", subname);
				ramdiskfound = 1;
			}
			else {
				env_set("fitramdisk", "-");
			}
		} else if (!strcmp(name, "flat_dt")) {
			fdt_prop = fdt_get_property(fit_fdt, nodeoffset, "data", NULL);
			if (fdt_prop) {
				if (board_is_fdt_better((struct fdt_header *)fdt_prop->data, &best_fdt) >= 0) {
					best_fdtsubname = fdt_get_name(fit_fdt, nodeoffset, NULL);
				}
			}
		}
	}

	if (best_fdtsubname) {
		env_set("fitbestfdt", best_fdtsubname);

		/* If board is secure, make sure FDT is authenticated */
		
		if (board_is_fdt_secure(best_fdt) != 0)	{
			return CMD_RET_FAILURE;
		}

	} else
		printf ("No valid FDT found\n");

	if (!kernelfound)
		printf ("No valid kernel image found\n");

	return 0;
}

/****************************************************************************/

/**
 * fdt_valid() - Check if an FDT is valid. If not, change it to NULL
 *
 * @blobp: Pointer to FDT pointer
 * @return 1 if OK, 0 if bad (in which case *blobp is set to NULL)
 */
static int fdt_valid(struct fdt_header **blobp)
{
	const void *blob = *blobp;
	int err;

	if (blob == NULL) {
		printf ("The address of the fdt is invalid (NULL).\n");
		return 0;
	}

	err = fdt_check_header(blob);
	if (err == 0)
		return 1;	/* valid */

	if (err < 0) {
		printf("libfdt fdt_check_header(): %s", fdt_strerror(err));
		/*
		 * Be more informative on bad version.
		 */
		if (err == -FDT_ERR_BADVERSION) {
			if (fdt_version(blob) <
			    FDT_FIRST_SUPPORTED_VERSION) {
				printf (" - too old, fdt %d < %d",
					fdt_version(blob),
					FDT_FIRST_SUPPORTED_VERSION);
			}
			if (fdt_last_comp_version(blob) >
			    FDT_LAST_SUPPORTED_VERSION) {
				printf (" - too new, fdt %d > %d",
					fdt_version(blob),
					FDT_LAST_SUPPORTED_VERSION);
			}
		}
		printf("\n");
		*blobp = NULL;
		return 0;
	}
	return 1;
}

/********************************************************************/
#ifdef CONFIG_SYS_LONGHELP
static char fitfind_help_text[] =
	"fitfind <fit_addr>    - Parse the FIT image at <fit_addr>\n"
	"                        - If a valid kernel image is found, the \n"
        "                          subimage name will be set in '$fitkernel'\n"
	"                        - If a valid initramfs image is found, the \n"
        "                          subimage name will be set in '$fitramdisk'\n"
	"                        - The subimage name of the best matched DTS \n"
        "                          will be set in '$fitbestfdt'\n";
#endif

U_BOOT_CMD(
	fitfind,	3,	1,	do_fitfind,
	"flattened device tree utility commands", fitfind_help_text
);
