#include <linux/of.h>
#include <linux/slab.h>

#define cpu_to_fdt32(x)		cpu_to_be32(x)

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
	prom_add_property(dn, pp);
	return pp;
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
	}
}

static int __init fixup_rb_dt(void) {
	if (of_machine_is_compatible("annapurna-labs,alpine2")) {
		if (strstr(boot_command_line, "board=ccr2004-1g-12s+2xs")) {
			printk("ccr2004-1g-12s+2xs DT fixup\n");
			ccr2004_1g_dt_fixup();
		}
	}
	return 0;
}

arch_initcall(fixup_rb_dt);
