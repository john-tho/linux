#include <linux/printk.h>
#include <linux/of_fdt.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/libfdt_env.h>

#include <asm/io.h>

static struct property *add_u32_property(struct device_node *dn,
					 const char *prop_name,
					 u32 value) {
	struct property *pp;
	u32 *pp_value = kzalloc(sizeof(u32), GFP_KERNEL);
	*pp_value = cpu_to_fdt32(value);
	pp = kzalloc(sizeof(struct property), GFP_KERNEL);
	pp->name = kstrdup(prop_name, GFP_KERNEL);
	pp->length = sizeof(u32);
	pp->value = pp_value;
	of_add_property(dn, pp);
	return pp;
}

static void update_u32_property(struct device_node *dn,
				struct property *pp,
				u32 value) {
	struct property *upd;
	upd = kzalloc(sizeof(struct property), GFP_KERNEL);
	upd->name = kstrdup(pp->name, GFP_KERNEL);
	upd->length = pp->length;
	upd->value = kzalloc(pp->length, GFP_KERNEL);
	memcpy(upd->value, pp->value, pp->length);
	writel(value, upd->value);
	of_update_property(dn, upd);
}

static void __init armada3700_dt_fixup(void) {
	int lenp, i, j;
	struct property *pp;
	struct device_node *dn;
	u32 phandle_map[2][2];
	u32 next_phandle = 0;

	for_each_node_with_property(dn, "phandle") {
		pp = of_find_property(dn, "phandle", &lenp);
		if (pp) {
			u32 index = fdt32_to_cpu(readl(pp->value));
			next_phandle = max(next_phandle, index);
		}
	}
	for_each_node_with_property(dn, "gpio-controller") {
		struct property *reg;
		pp = of_find_property(dn, "phandle", &lenp);
		if (!pp) {
			dn->phandle = ++next_phandle;
			pp = add_u32_property(dn, "phandle", dn->phandle);
		}
		i = j = 0;
		reg = of_find_property(dn, "reg", &lenp);
		if (reg == NULL) {
			reg = of_find_property(dn->parent, "reg", &lenp);
			j = 1;
		}
		if (fdt32_to_cpu(readl(reg->value)) == 0x00018800) {
			i = 1;
		}
		if (j == 1) {
			add_u32_property(dn, "gpiobase", i == 0 ? 0 : 36);
		}
		phandle_map[i][j] = readl(pp->value);
	}
	for_each_node_with_property(dn, "gpios") {
		pp = of_find_property(dn, "gpios", &lenp);
		for (i = 0; i < 2; i++) {
			if (pp && phandle_map[i][0] == readl(pp->value)) {
				u32 val = phandle_map[i][1];
				update_u32_property(dn, pp, val);
				break;
			}
		}
	}
	dn = of_find_node_by_name(of_find_node_by_path("/"), "all-leds");
	if (dn) {
		pp = kzalloc(sizeof(struct property), GFP_KERNEL);
		pp->name = kstrdup("default-state", GFP_KERNEL);
		pp->value = kstrdup("on", GFP_KERNEL);
		pp->length = sizeof("on");
		of_update_property(dn, pp);
	}
}

static void __init ccr2004_1g_dt_fixup(void) {
	int lenp;
	struct property *pp;
	struct device_node *dn;

	for_each_compatible_node(dn, NULL, "snps,designware-i2c") {
		pp = of_find_property(dn, "i2c-sda-hold-time-ns", &lenp);
		if (!pp) {
			add_u32_property(dn, "i2c-sda-hold-time-ns", 300);
		}
		pp = of_find_property(dn, "i2c-sda-falling-time-ns", &lenp);
		if (!pp) {
			add_u32_property(dn, "i2c-sda-falling-time-ns", 251);
		}
		pp = of_find_property(dn, "i2c-scl-falling-time-ns", &lenp);
		if (!pp) {
			add_u32_property(dn, "i2c-scl-falling-time-ns", 628);
		}
	}
}

static int __init fixup_rb_dt(void) {
	if (of_machine_is_compatible("marvell,armada3720")) {
		printk("armada3700 DT fixup\n");
		armada3700_dt_fixup();
	}
	if (of_machine_is_compatible("annapurna-labs,alpine2")) {
		if (strstr(boot_command_line, "board=ccr2004-1g-12s+2xs")) {
			printk("ccr2004-1g-12s+2xs DT fixup\n");
			ccr2004_1g_dt_fixup();
		}
	}
	return 0;
}

arch_initcall(fixup_rb_dt);
