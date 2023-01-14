#include <linux/types.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/export.h>
#include <asm/rb/rb400.h>
#include <asm/bootinfo.h>
#include <asm/rb/boards.h>

#define PCI_BASE 0x18080000
#define PCIB_REG(reg) (*(volatile u32 *) KSEG1ADDR(PCI_BASE + (reg)))

#define PCIB_CONTROL		0x0
#define PCIB_STATUS		0x4
#define PCIB_STATUS_MASK	0x8
#define PCIB_CONF_ADDR		0xc
#define PCIB_CONF_DATA		0x10
#define PCIB_LBA0		0x14
#define PCIB_LBA0_CONTROL	0x18
#define PCIB_LBA0_MAPPING	0x1c
#define PCIB_LBA1		0x20
#define PCIB_LBA1_CONTROL	0x24
#define PCIB_LBA1_MAPPING	0x28
#define PCIB_LBA2		0x2c
#define PCIB_LBA2_CONTROL	0x30
#define PCIB_LBA2_MAPPING	0x34
#define PCIB_LBA3		0x38
#define PCIB_LBA3_CONTROL	0x3c
#define PCIB_LBA3_MAPPING	0x40
#define PCIB_DA_CONTROL		0x44
#define PCIB_DA_STATUS		0x48
#define PCIB_DA_STATUS_MASK	0x4c
#define PCIB_DA_DATA		0x50
#define PCIB_TARGET_CONTROL	0x5c

#define PCIC_EN  0x00000001
#define PCIC_EAP 0x00000020
#define PCIC_IGM 0x00000200

#define PCIS_RIP 0x00020000

#define PCILBA_SIZE_1MB  (20 << 2)
#define PCILBA_SIZE_16MB (24 << 2)
#define PCILBA_SIZE_64MB (26 << 2)
#define PCILBA_MSI 1

#define PCITC_RTIMER_DEF 16
#define PCITC_DTIMER_DEF 8

#define PCIDAS_DONE		0x1
#define PCIDAS_BUSY		0x2
#define PCIDAS_ERROR		0x4
#define PCIDAS_OUT_FIFO_EMPTY	0x8

#define RB400_RESET_BASE	0x18060000
#define RB400_PCI_WINDOW_BASE	0x18000000
#define RB400_RESET_REG(x)	((unsigned long) reset_base + (x))
#define RB400_PCI_WINDOW_REG(x) ((unsigned long) window_base + (x))
#define RB400_RESET		RB400_RESET_REG(0x24)
#define RB400_PCI_WINDOW_OFFSET	RB400_PCI_WINDOW_REG(0x7c)

#define RB400_PCI_BASE		0x17010000
#define RB400_PCI_REG(x)	((unsigned long) rb400_pci_base + (x))
#define RB400_PCI_CRP_AD_CBE	RB400_PCI_REG(0x00)
#define RB400_PCI_CRP_WRDATA	RB400_PCI_REG(0x04)
#define RB400_PCI_ERROR		RB400_PCI_REG(0x1c)
#define RB400_PCI_AHB_ERROR	RB400_PCI_REG(0x24)

extern struct pci_ops rb400_pci_ops;
extern struct pci_ops rb700_pci_ops;

int pci_decoupled_access = 0;

static struct resource rb400_res_pci_io = {
	.name	= "PCI IO space",
	.start	= 0x100,
	.end	= 0xffff,
	.flags	= IORESOURCE_IO
};

static struct resource rb400_res_pci_mem = {
	.name	= "PCI memory space",
	.start	= 0x10000000,
	.end	= 0x17ffffff,
	.flags	= IORESOURCE_MEM
};

static struct pci_controller rb400_controller = {
	.pci_ops = &rb400_pci_ops,
	.io_resource = &rb400_res_pci_io,
	.mem_resource = &rb400_res_pci_mem,
};

static struct resource rb700_io_resource = {
	.name = "PCI IO space",
	.start = 0,
	.end = 0,
	.flags = IORESOURCE_IO
};

static struct resource rb700_mem_resource = {
	.name = "PCI memory space",
	.start = 0x10000000,
	.end = 0x12000000 - 1,
	.flags = IORESOURCE_MEM
};

static struct pci_controller rb700_controller = {
	.pci_ops	= &rb700_pci_ops,
	.mem_resource	= &rb700_mem_resource,
	.io_resource	= &rb700_io_resource,
	.index		= 0,
};

static struct resource rb900_io_resource = {
	.name = "PCI IO space",
	.start = 1,
	.end = 1,
	.flags = IORESOURCE_IO
};

static struct resource rb900_mem_resource = {
	.name = "PCI memory space",
	.start = 0x12000000,
	.end = 0x14000000 - 1,
	.flags = IORESOURCE_MEM
};

static struct pci_controller rb900_controller = {
	.pci_ops	= &rb700_pci_ops,
	.mem_resource	= &rb900_mem_resource,
	.io_resource	= &rb900_io_resource,
	.index		= 1,
};

int pcibios_plat_dev_init(struct pci_dev *dev)
{
	return 0;
}

static int rb400_irq_map[7][4] = {
	{ 50, -1, -1, -1 },
	{ 48, 49, -1, -1 },
	{ 49, 50, -1, -1 },
	{ 50, 48, -1, -1 },
	{ 48, -1, -1, -1 },
	{ 49, 50, -1, -1 },
	{ 50, 48, -1, -1 },
};

extern int mt7621_map_irq(const struct pci_dev *dev, u8 slot, u8 pin);
extern int ar_has_second_pcie_bus(void);

int pcibios_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	switch (mips_machgroup) {
	case MACH_GROUP_MT_RB400:
		if (slot < 17 || slot > 23) return -1;
		return rb400_irq_map[slot - 17][pin - 1];
	case MACH_GROUP_MT_RB700:
		if (ar_has_second_pcie_bus()) {
		    return pci_domain_nr(dev->bus) == 0 ? 2 : 3;
		}
		return 2;
	case MACH_GROUP_MT_MMIPS:
		return mt7621_map_irq(dev, slot, pin);
	}
        return -1;
}

static int __init rb400_pci_init(void)
{
	void __iomem *reset_base;
	void __iomem *window_base;
	extern void __iomem *rb400_pci_base;
	unsigned val;
	unsigned i;

	reset_base = ioremap(RB400_RESET_BASE, PAGE_SIZE);
	window_base = ioremap(RB400_PCI_WINDOW_BASE, PAGE_SIZE);
	rb400_pci_base = ioremap(RB400_PCI_BASE, PAGE_SIZE);

	val = rb400_readl(RB400_RESET);
	rb400_writel(val | 3, RB400_RESET);

	if (mips_machtype == MACH_MT_RB450 ||
	    mips_machtype == MACH_MT_RB450G)
		return 0;

	mdelay(100);
	rb400_writel(val & ~3, RB400_RESET);
	mdelay(100);

	for (i = 0; i < 7; ++i)
		rb400_writel(0x10000000 + 0x01000000 * i,
			     RB400_PCI_WINDOW_OFFSET + i * 4);
	rb400_writel(0x07000000, RB400_PCI_WINDOW_OFFSET + 7 * 4);

	mdelay(100);

	rb400_writel(0x00010000 | PCI_COMMAND, RB400_PCI_CRP_AD_CBE);
	rb400_writel(0x356, RB400_PCI_CRP_WRDATA);

        rb400_writel(3, RB400_PCI_ERROR);
        rb400_writel(1, RB400_PCI_AHB_ERROR);

	iounmap(reset_base);
	iounmap(window_base);

	register_pci_controller(&rb400_controller);

	return 0;
}

int rb700_local_write_config(int i, int where, int size, uint32_t value);

static int __init rb700_pci_init_advanced(struct pci_controller *ctrl) {
	unsigned host_base = (ctrl->index == 0) ? 0xb80c0000 : 0xb8250000;
	if (!(rb400_readl(0xb8050010) & 0x02000000)
		&& rb400_readl(host_base + 0x30018) == 7) {
	    struct pci_bus dummy_bus;
	    uint32_t cmd = PCI_COMMAND_MEMORY
		| PCI_COMMAND_MASTER
		| PCI_COMMAND_INVALIDATE
		| PCI_COMMAND_PARITY
		| PCI_COMMAND_SERR
		| PCI_COMMAND_FAST_BACK;

	    dummy_bus.sysdata = ctrl;
	    rb700_local_write_config(ctrl->index, PCI_COMMAND, 4, cmd);
	    rb700_pci_ops.write(&dummy_bus, 0, PCI_COMMAND, 4, cmd);

	    rb400_writel(rb400_readl(host_base + 0x30050) | (1 << 14),
			 host_base + 0x30050);
	    if ((rb400_readl(0xb8060090) & 0xffffff00) == 0)
		    ctrl->mem_offset = 0x10000000;
	    register_pci_controller(ctrl);
	}

	return 0;
}

static int __init rb700_pci_init(void) {
    printk("rb700_pci_init\n");
    return rb700_pci_init_advanced(&rb700_controller);
}

static int __init rb900_pci_init(void) {
    printk("rb900_pci_init\n");
    return rb700_pci_init_advanced(&rb900_controller);
}

static int __init rb_pci_init(void)
{
	switch (mips_machgroup) {
	case MACH_GROUP_MT_RB400:
		return rb400_pci_init();
	case MACH_GROUP_MT_RB700:
		if (mips_machtype == MACH_MT_RB951) break;
		if (mips_machtype == MACH_MT_CM2N) break;
		if (mips_machtype == MACH_MT_mAP) break;
		if (ar_has_second_pcie_bus()) {
		    rb900_pci_init();
		}
		rb700_pci_init();
		return 0;
	}
	return 0;
}

arch_initcall(rb_pci_init);
