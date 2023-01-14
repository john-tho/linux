// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Machine declaration for Alpine platforms.
 *
 * Copyright (C) 2015 Annapurna Labs Ltd.
 */

#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>

#include <asm/mach/arch.h>

extern const struct smp_operations alpine_smp_ops;

extern void get_pcie_info(void);

static void __init al_dt_fixup(void)
{
	extern unsigned long __dtb_rb1100ahx4_begin;
	extern unsigned long __dtb_rb4011_begin;
	unsigned long *dtc = NULL;

	of_scan_flat_dt(early_init_dt_scan_chosen, boot_command_line);

	if (strstr(boot_command_line, "board=1100Dx4")) {
		dtc = &__dtb_rb1100ahx4_begin;
	} else if (strstr(boot_command_line, "board=4011")) {
		dtc = &__dtb_rb4011_begin;
	}
	if (dtc) {
		int offset;
		const struct fdt_property *prop;
		unsigned char hwaddr[6];

		offset = fdt_path_offset(initial_boot_params, "/");
		prop = fdt_get_property(initial_boot_params, offset,
					"mac-address", NULL);
		memcpy(hwaddr, prop->data, 6);

		get_pcie_info();

		early_init_dt_verify(dtc);
		
		offset = fdt_path_offset(initial_boot_params, "/");
		fdt_setprop_inplace(initial_boot_params, offset, "mac-address",
				    hwaddr, 6);
	}
}

static const char * const al_match[] __initconst = {
	"al,alpine",
	"annapurna-labs,alpine",
	NULL,
};

DT_MACHINE_START(AL_DT, "Annapurna Labs Alpine")
	.dt_fixup	= al_dt_fixup,
	.dt_compat	= al_match,
MACHINE_END
