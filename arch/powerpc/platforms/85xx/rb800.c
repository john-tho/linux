#include <linux/of_platform.h>
#include <linux/of_fdt.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/mpic.h>
#include <asm/mtvic.h>
#include <sysdev/fsl_pci.h>
#include <asm/rb_aux.h>

void rb_power_off(void);

static void __init rb800_setup_arch(void)
{
	mtspr(SPRN_HID0, 1 << 14); /* set TBEN */
	mb();

	if ((mfspr(SPRN_SVR) >> 16) == 0x803c)
		add_crypto_of_node(0x2d);

	ppc_md.power_save = rb_idle;

	fsl_pci_assign_primary();

	pm_power_off = rb_power_off;
}

static void __init rb800_init_early(void) {
	add_second_serial_of_node();
}

static int __init rb800_probe(void)
{
	const char *model;

	model = of_get_flat_dt_prop(of_get_flat_dt_root(), "model", NULL);

	if (!model)
		return 0;

	if (strcmp(model, "RB800") != 0)
		return 0;

	rb800_init_early();
	return 1;
}

define_machine(rb800) {
	.name			= "RB800",
	.probe			= rb800_probe,
	.setup_arch		= rb800_setup_arch,
	.init_IRQ		= rb_pic_init,
	.get_irq		= rb_get_irq,
	.show_cpuinfo		= rb_show_cpuinfo,
	.restart		= rb_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
};
