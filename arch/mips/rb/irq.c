#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/bitops.h>
#include <linux/irqchip.h>
#include <linux/module.h>
#include <asm/signal.h>
#include <asm/mipsregs.h>
#include <asm/irq_cpu.h>
#include <asm/bootinfo.h>
#include <asm/vm.h>
#include <asm/rb/boards.h>

extern void rb400_init_irq(void);
extern void mmips_init_irq(void);
extern void __init music_arch_init_irq(void);

extern unsigned int gic_present;

int gpio_to_irq(unsigned gpio) {
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(gpio_to_irq);

unsigned int get_c0_compare_int(void)
{
	extern int gic_get_c0_compare_int(void);
#ifdef CONFIG_MIPS_GIC    
	if (gic_present)
		return gic_get_c0_compare_int();
#endif	
	return MIPS_CPU_IRQ_BASE + cp0_compare_irq;
}

int get_c0_perfcount_int(void)
{
	extern int gic_get_c0_perfcount_int(void);
#ifdef CONFIG_MIPS_GIC    
	if (gic_present)
		return gic_get_c0_perfcount_int();
#endif	
	return MIPS_CPU_IRQ_BASE + cp0_perfcount_irq;
}

volatile unsigned long virqs;
EXPORT_SYMBOL(virqs);

static void ack_virq(struct irq_data *d)
{
	clear_bit(d->irq - VIRQ_BASE, &virqs);
}

static inline void unmask_virq(struct irq_data *d)
{
}

static inline void mask_virq(struct irq_data *d)
{
}

static struct irq_chip virq_controller = {
	.name	= "virq",
	.irq_ack	= ack_virq,
	.irq_unmask     = unmask_virq,
	.irq_mask	= mask_virq,
};

static irqreturn_t virq_cascade_irq(int irq, void *dev_id)
{
	unsigned i;
	unsigned irqs = virqs;

	for (i = 0; irqs; ++i) {
		if (irqs & (1 << i)) {
			generic_handle_irq(i + VIRQ_BASE);
			irqs ^= (1 << i);
		}
	}
	return IRQ_HANDLED;
}

static struct irqaction virq_cascade  = {
	.handler = virq_cascade_irq,
	.name = "virq-cascade",
};

static void soft_irq_ack(struct irq_data *d)
{
	clear_c0_cause(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
}

static inline void unmask_soft_irq(struct irq_data *d)
 {
	set_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
	irq_enable_hazard();
 }
 
static inline void mask_soft_irq(struct irq_data *d)
{
	clear_c0_status(0x100 << (d->irq - MIPS_CPU_IRQ_BASE));
	irq_disable_hazard();
}

static struct irq_chip soft_irq_controller = {
	.name	= "MIPS",
	.irq_ack	= soft_irq_ack,
	.irq_unmask     = unmask_soft_irq,
	.irq_mask	= mask_soft_irq,
};

extern void __init ont_arch_init_irq(void);

void __init arch_init_irq(void)
{
	unsigned i;

	switch (mips_machgroup) {
	case MACH_GROUP_MT_RB400:
	case MACH_GROUP_MT_RB700:
		rb400_init_irq();
		break;
	case MACH_GROUP_MT_MUSIC:
		music_arch_init_irq();
		break;
	case MACH_GROUP_MT_MMIPS:
		irqchip_init();
		break;
	case MACH_GROUP_MT_VM:
		mips_cpu_irq_init();
		break;
	}

	if (mips_machgroup != MACH_GROUP_MT_MMIPS) {
		irq_set_chip_and_handler(1, &soft_irq_controller, handle_percpu_irq);
		setup_irq(1, &virq_cascade);
		
		for (i = VIRQ_BASE;  i < VIRQ_BASE + 32; ++i)
			irq_set_chip_and_handler(i, &virq_controller,
						 handle_edge_irq);
	}
}
