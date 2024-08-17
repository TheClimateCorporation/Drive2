// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020
 */

#include <common.h>
#include <command.h>

static int do_h2d(struct cmd_tbl *cmdtp, int flag, int argc,
		  char *const argv[])
{
	long value;
        char buf[512] = { 0 };

	if ((argc < 2) || (argc > 3))
                return CMD_RET_USAGE;

	char *p = argv[1];
	/* consume leading 0x */
	if (strstr(p, "0x"))
		p += 2;

	value = simple_strtol(p, NULL, 16);
        snprintf(buf, sizeof(buf), "%ld", value);

        if (argc > 2)
                env_set(argv[2], buf);
	else
		printf("%s\n", buf);

	return 0;
}

static int do_d2h(struct cmd_tbl *cmdtp, int flag, int argc,
		  char *const argv[])
{
	long value;
        char buf[512] = { 0 };

	if ((argc < 2) || (argc > 3))
                return CMD_RET_USAGE;

	char *p = argv[1];

	value = simple_strtol(p, NULL, 10);
        snprintf(buf, sizeof(buf), "%lx", value);

        if (argc > 2)
                env_set(argv[2], buf);
	else
		printf("%s\n", buf);

	return 0;
}

U_BOOT_CMD(
	d2h,	CONFIG_SYS_MAXARGS,	1,	do_d2h,
	"convert decimal value to hex",
	"<decimal value> <variable>\n"
	"    - convert inputs to hex.  Optionally store in variable."
	"      assumes long values for negative numbers"
);
U_BOOT_CMD(
	h2d,	CONFIG_SYS_MAXARGS,	1,	do_h2d,
	"convert hex value to decimal",
	"<hex value> <variable>\n"
	"    - convert inputs to decimal.  Optionally store in variable"
	"      assumes long values for negative numbers"
);
