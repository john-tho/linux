#include <linux/spinlock.h>
#include <linux/memblock.h>
#include <linux/of_address.h>
#include <linux/of_fdt.h>
#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/mpic.h>
#include <asm/mtvic.h>
#include <sysdev/fsl_pci.h>
#include <asm/rb_aux.h>

#ifdef CONFIG_SMP
extern void __init mpc85xx_smp_init(void);
#endif

extern struct smp_ops_t smp_85xx_ops;
extern int rb_big_boot_partition;

static unsigned *soc_regs;

spinlock_t localbus_lock = __SPIN_LOCK_UNLOCKED(localbus_lock);
EXPORT_SYMBOL(localbus_lock);

void rb_power_off(void);

static void __init rb1120_setup_arch(void)
{
	struct device_node *soc;
    
	rb_big_boot_partition = 1;

	mtspr(SPRN_HID0, 1 << 14); /* set TBEN */
	mb();

	ppc_md.power_save = rb_idle;

	fsl_pci_assign_primary();

#ifdef CONFIG_SMP
	mpc85xx_smp_init();
	smp_85xx_ops.give_timebase = smp_generic_give_timebase;
	smp_85xx_ops.take_timebase = smp_generic_take_timebase;
#endif

	pm_power_off = rb_power_off;

	/* setup for lockless reboot (no iomaps at reboot) */
	soc = of_find_node_by_type(NULL, "soc");
	if (soc) {
	    phys_addr_t immrbase;
	    int size;
	    const void *prop = of_get_property(soc, "reg", &size);
	    immrbase = of_translate_address(soc, prop);
	    of_node_put(soc);

	    soc_regs = (unsigned *) ioremap(immrbase + 0xe0000, 0x100);
	}
}

static int __init is_board(const char *str) {
	const char *model;
	model = of_get_flat_dt_prop(of_get_flat_dt_root(), "model", NULL);
	return model ? (strcmp(model, str) == 0) : 0;    
}

#ifdef CONFIG_FSL_QMAN_CONFIG
void qman_init_early(void);
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
void bman_init_early(void);
#endif
static void __init rb1120_init_early(void) {
	if (!is_board("RB850G")) {
		add_second_serial_of_node();
	}
#ifdef CONFIG_FSL_QMAN_CONFIG
	qman_init_early();
#endif
#ifdef CONFIG_FSL_BMAN_CONFIG
	bman_init_early();
#endif
}

static int __init rb1120_probe(void)
{
	int is_rb_p_type = is_board("RB1120");
	if (is_board("RB850G")) is_rb_p_type = true;

	if (is_rb_p_type) {
	    unsigned top = memblock_end_of_DRAM();
	    memblock_add(top, 0x1000);
	    memblock_reserve(top, 0x1000);
	}

	if (is_rb_p_type)
		rb1120_init_early();
	return is_rb_p_type;
}

void __attribute__((noreturn)) rb1120_restart(char *cmd)
{
	if (soc_regs) {
	    local_irq_disable();
	    out_be32(soc_regs + 44, 2); 
	}
	else {
	    printk("This is emergency!\n");
	    local_irq_disable();
	    rb_restart(NULL);
	}
	while (1) ;
}

define_machine(rb1120) {
	.name			= "RB1120",
	.probe			= rb1120_probe,
	.setup_arch		= rb1120_setup_arch,
	.init_IRQ		= rb_pic_init,
	.get_irq		= rb_get_irq,
	.show_cpuinfo		= rb_show_cpuinfo,
	.restart		= rb1120_restart,
	.calibrate_decr		= generic_calibrate_decr,
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
};
