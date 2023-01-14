#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/io.h>

static int __init set_eth_mac(char *s) {
	static unsigned char hwaddr[6];
	static struct property prop = {
	    .name = "mac-address",
	    .value = hwaddr,
	    .length = 6,
	};
	struct device_node *root;
	int m[6];
	int i;
	
	sscanf(s, "=%02x:%02x:%02x:%02x:%02x:%02x",
	       m + 0, m + 1, m + 2, m + 3, m + 4, m + 5);
	for (i = 0; i < 6; ++i) hwaddr[i] = m[i];

	root = of_find_node_by_path("/");
	of_add_property(root, &prop);
	of_node_put(root);
	return 1;
}

__setup("eth_mac", set_eth_mac);

static void toggle_watchdog(int enable) {
	void __iomem *wdt_base;
	struct resource res;
	int akronite = 0;
	struct device_node *np = of_find_compatible_node(NULL, NULL, "qcom,kpss-wdt");
	if (!np) {
	    np = of_find_compatible_node(NULL, NULL, "qcom,kpss-wdt-ipq8064");
	    if (np) akronite = 1;
	}
	if (!np) {
		pr_err("%s:can't find node\n", __func__);
		return -ENODEV;
	}
	if (of_address_to_resource(np, 0, &res)) {
		of_node_put(np);
		return -ENOMEM;
	}
	request_mem_region(res.start, resource_size(&res), "wdt");
	wdt_base = (unsigned)of_iomap(np, 0);
	writel(1, wdt_base + (akronite ? 0x38 : 0x04)); // wdt reset
	writel(0, wdt_base + (akronite ? 0x40 : 0x08)); // wdt disable
	if (enable) {
	    writel(0x1000, wdt_base + (akronite ? 0x4c : 0x10));
	    writel(0x1000, wdt_base + (akronite ? 0x5c : 0x14));
	    writel(enable, wdt_base + (akronite ? 0x40 : 0x08));
	    wmb();
	    msleep(200);
	}
	release_mem_region(res.start, resource_size(&res));
}

static int __init disable_early_watchdog(void) {
	toggle_watchdog(0);
	return 0;
}
early_initcall(disable_early_watchdog);

static int restart_by_watchdog(
    struct notifier_block *nb, unsigned long action, void *data) {
	toggle_watchdog(1);
	return NOTIFY_DONE;
}

static struct notifier_block restart_nb = {
	.notifier_call = restart_by_watchdog,
};

static void __init ipq8064_fixup(struct tag *tag, char **cmdline_p)
{
	extern unsigned long __dtb_rb3011_begin;
	const char *cmdline = 0;

	for (; tag->hdr.size; tag = tag_next(tag)) {
		if (tag->hdr.tag == ATAG_CMDLINE) {
			cmdline = tag->u.cmdline.cmdline;
			break;
		}
	}
	if (cmdline && strstr(cmdline, "board=3011")) {
		early_init_dt_scan(&__dtb_rb3011_begin);
		register_restart_handler(&restart_nb);
	}
}

static void __init ipq40xx_dt_fixup(void)
{
	extern unsigned long __dtb_lhg60_begin;
	extern unsigned long __dtb_450g_begin;
	extern unsigned long __dtb_d52g_begin;
	extern unsigned long __dtb_lhgg60_begin;
	extern unsigned long __dtb_lhg5ac_begin;
	extern unsigned long __dtb_wap2gr_begin;
	extern unsigned long __dtb_wapgr_begin;
	extern unsigned long __dtb_capg5ac_begin;
	extern unsigned long __dtb_wap_pcie_begin;
	extern unsigned long __dtb_plc5ac_begin;
	extern unsigned long __dtb_lhgb5hpa_begin;
	extern unsigned long __dtb_d23ugs_begin;
	extern unsigned long __dtb_d53g_begin;
	extern unsigned long __dtb_d53pk_begin;
	extern unsigned long __dtb_d53igr_begin;
	extern unsigned long __dtb_d53ig_begin;
	extern unsigned long __dtb_d24g_begin;
	extern unsigned long __dtb_d24gi_begin;
	extern unsigned long __dtb_d24gir_begin;
	extern unsigned long __dtb_d24gir60_begin;

	unsigned long *dtc = NULL;
	unsigned memory = 0;
	unsigned char *mem_offset;
	uint32_t pcie_bus = ~0;
	int offset, serial = 0, alt_serial = 0;

	of_scan_flat_dt(early_init_dt_scan_chosen, boot_command_line);

	offset = fdt_path_offset(initial_boot_params,
		"/soc/serial@78af000/");
	if (offset >= 0) {
		const struct fdt_property *status =
			fdt_get_property(initial_boot_params, 
				offset, "status", NULL);
		if (status && !strcmp(status->data, "ok"))
			serial = 1;
	}
	offset = fdt_path_offset(initial_boot_params,
		"/soc/serial@78b0000/");
	if (offset >= 0) {
		const struct fdt_property *status =
			fdt_get_property(initial_boot_params, 
				offset, "status", NULL);
		if (status && !strcmp(status->data, "ok"))
			alt_serial = 1;
	}

	if (strstr(boot_command_line, "board=lhg60-dk")) {
		dtc = &__dtb_lhgg60_begin;
	}
	if (strstr(boot_command_line, "board=450g-dk")) {
		dtc = &__dtb_450g_begin;
	}
	if (strstr(boot_command_line, "board=hap-dk")) {
		dtc = &__dtb_d52g_begin;
	}
	if (strstr(boot_command_line, "board=lhg-dk")) {
		dtc = &__dtb_lhg5ac_begin;
	}
	if (strstr(boot_command_line, "board=wapr-dk")) {
		dtc = &__dtb_wapgr_begin;
	}
	if (strstr(boot_command_line, "board=wap2r-dk")) {
		dtc = &__dtb_wap2gr_begin;
	}
	if (strstr(boot_command_line, "board=cap-dk")) {
		dtc = &__dtb_capg5ac_begin;
	}
	if (strstr(boot_command_line, "board=wap-dk")) {
		dtc = &__dtb_wap_pcie_begin;
	}
	if (strstr(boot_command_line, "board=lhgb52-dk")) {
		dtc = &__dtb_lhgb5hpa_begin;
	}
	if (strstr(boot_command_line, "board=pl002-dk")) {
		dtc = &__dtb_plc5ac_begin;
	}
	if (strstr(boot_command_line, "board=d23-dk")) {
		dtc = &__dtb_d23ugs_begin;
	}
	if (strstr(boot_command_line, "board=d53g-dk")) {
		dtc = &__dtb_d53g_begin;
	}
	if (strstr(boot_command_line, "board=d53pk-dk")) {
		dtc = &__dtb_d53pk_begin;
	}
	if (strstr(boot_command_line, "board=d53i-dk")) {
		dtc = &__dtb_d53igr_begin;
	}
	if (strstr(boot_command_line, "board=d53ig-dk")) {
		dtc = &__dtb_d53ig_begin;
	}
	if (strstr(boot_command_line, "board=d24-dk")) {
		int cascade, wigig, mpcie;
		/* safe backup */
		dtc = &__dtb_d24g_begin;

		offset = fdt_path_offset(initial_boot_params, "/wlan-data/wlan1/");
		if (offset >= 0) {
			const struct fdt_property *cal_inst = fdt_get_property(initial_boot_params,
				offset, "pci-bus-nr", NULL);
			memcpy(&pcie_bus, cal_inst->data, sizeof(uint32_t));
		}

		cascade = fdt_path_offset(initial_boot_params, "/soc/qcom,pcie@80000/switch/cascade/");
		wigig = fdt_path_offset(initial_boot_params, "/soc/qcom,pcie@80000/switch/wigig/");
		mpcie = fdt_path_offset(initial_boot_params, "/soc/qcom,pcie@80000/switch/mpcie_card/");
		if ((cascade >= 0) && (wigig < 0) && (mpcie < 0)) {
			printk("detected d24gi\n");
			dtc = &__dtb_d24gi_begin;
		} else if ((cascade >= 0) && (mpcie >= 0) &&
				((wigig < 0) || (ntohl(pcie_bus) == 1))) {
			printk("detected d24gir\n");
			dtc = &__dtb_d24gir_begin;
		} else if ((cascade >= 0) && (wigig >= 0) && (mpcie >= 0)
				&& (ntohl(pcie_bus) == 3)) {
			printk("detected d24gir60\n");
			dtc = &__dtb_d24gir60_begin;
		}

	}

	mem_offset = strstr(boot_command_line, "memory=");
	if (mem_offset)
		sscanf(mem_offset, "memory=%08x", &memory);

	if (dtc) {
		const struct fdt_property *mac_prop, *mem_prop;
		unsigned char hwaddr[6];
		unsigned mem_range[2];
		int len = 0;

		offset = fdt_path_offset(initial_boot_params, "/");
		mac_prop = fdt_get_property(initial_boot_params, offset,
					"mac-address", NULL);
		memcpy(hwaddr, mac_prop->data, 6);

		if (!memory) {
			offset = fdt_path_offset(initial_boot_params,
						 "/memory@80000000/");
			mem_prop = fdt_get_property(initial_boot_params,
						    offset, "reg", NULL);
			if (offset >= 0 && mem_prop != NULL) {
				memcpy(mem_range, mem_prop->data,
				       sizeof(mem_range));
				memory = ntohl(mem_range[1]);
			}
		}

		early_init_dt_verify(dtc);

		if (pcie_bus != ~0) {
			offset = fdt_path_offset(initial_boot_params, "/wlan-data/wlan1/");
			if (offset >= 0)
				fdt_setprop_inplace(initial_boot_params, offset, "pci-bus-nr",
				    &pcie_bus, sizeof(uint32_t));
		}
		
		if (!memory)
			goto patch_mac;
		offset = fdt_path_offset(initial_boot_params, "/memory@80000000/");
		if (offset < 0)
			goto patch_mac;
		mem_prop = fdt_get_property(initial_boot_params, offset, "reg", NULL);
		if (!mem_prop)
			goto patch_mac;
		memcpy(mem_range, mem_prop->data, sizeof(mem_range));
		mem_range[1] = htonl(memory);
		fdt_setprop_inplace(initial_boot_params, offset, "reg",
				    mem_range, sizeof(mem_range));
patch_mac:
		offset = fdt_path_offset(initial_boot_params, "/");
		fdt_setprop_inplace(initial_boot_params, offset, "mac-address",
				    hwaddr, 6);

		if (alt_serial) {
			offset = fdt_path_offset(initial_boot_params,
					"/soc/serial@78b0000/");
			if (offset)
				fdt_nop_property(initial_boot_params, offset, "status");
		}
		if (serial || strstr(boot_command_line, "debug")) {
			offset = fdt_path_offset(initial_boot_params,
					"/soc/serial@78af000/");
			if (offset)
				fdt_nop_property(initial_boot_params, offset, "status");
		}
	}
}

MACHINE_START(IPQ806X, "RB3011")
	.fixup		= ipq8064_fixup,
MACHINE_END

static const char * const ipq40xx_match[] __initconst = {
	"qcom,ipq40xx",
	"qcom,ipq4019",
	NULL,
};

MACHINE_START(IPQ4019, "IPQ4019")
	.dt_compat	= ipq40xx_match,
	.dt_fixup	= ipq40xx_dt_fixup,
MACHINE_END
    
