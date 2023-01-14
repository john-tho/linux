#include <asm/io.h>
#include <linux/libfdt.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <asm/io.h>

#define AL_CCU_SNOOP_CONTROL_IOFAB_0_OFFSET	0x4000
#define AL_CCU_SNOOP_CONTROL_IOFAB_1_OFFSET	0x5000
#define AL_CCU_SPECULATION_CONTROL_OFFSET	0x4


static void al_pci_fixup(struct pci_dev *dev)
{
	extern int pcie_bus_configure_set(struct pci_dev *dev, void *data);
	u8 smpss = 0;
	pcie_bus_configure_set(dev, &smpss);
}
DECLARE_PCI_FIXUP_HEADER(PCI_ANY_ID, PCI_ANY_ID, al_pci_fixup);

static void al_msi_intx_disable_bug(struct pci_dev *dev)
{
	dev->dev_flags |= PCI_DEV_FLAGS_MSI_INTX_DISABLE_BUG;
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_ANNAPURNA_LABS, 0x0001,
			al_msi_intx_disable_bug);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_ANNAPURNA_LABS, 0x0011,
			al_msi_intx_disable_bug);
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_ANNAPURNA_LABS, 0x0021,
			al_msi_intx_disable_bug);

static void al_ccu_init(void __iomem *ccu_address)
{
	/* enable snoop */
	writel(1, ccu_address + AL_CCU_SNOOP_CONTROL_IOFAB_0_OFFSET);
	writel(1, ccu_address + AL_CCU_SNOOP_CONTROL_IOFAB_1_OFFSET);

	/* disable speculative fetches from masters */
	writel(7, ccu_address + AL_CCU_SPECULATION_CONTROL_OFFSET);
}

static int al_fabric_pci_device_notifier(struct notifier_block *nb,
					 unsigned long event, void *__dev)
{
	struct device *dev = __dev;
	struct pci_dev *pdev = to_pci_dev(dev);
	u32 temp;

	if (event != BUS_NOTIFY_BIND_DRIVER)
		return NOTIFY_DONE;


	/* Force the PCIE adapter to set AXI attr to match CC*/
	pci_read_config_dword(pdev, 0x110 ,&temp);
	temp |= 0x3;
	pci_write_config_dword(pdev, 0x110 ,temp);
	/* Enable cache coherency for VF's (except USB and SATA) */
	if (PCI_SLOT(pdev->devfn) < 6) {
		pci_write_config_dword(pdev, 0x130 ,temp);
		pci_write_config_dword(pdev, 0x150 ,temp);
		pci_write_config_dword(pdev, 0x170 ,temp);
	}

	pci_read_config_dword(pdev, 0x220 ,&temp);
	temp &= ~0xffff;
	temp |= 0x3ff;
	pci_write_config_dword(pdev, 0x220 ,temp);

	return NOTIFY_OK;
}

static struct notifier_block al_fabric_pci_device_nb = {
	.notifier_call = al_fabric_pci_device_notifier,
};

static int __init coherency_pci_init(void)
{
	struct device_node *np = of_find_compatible_node(NULL, NULL, "al,alpine-ccu");

	if (np) {
		void __iomem *ccu_address = of_iomap(np, 0);
		al_ccu_init(ccu_address);
	}
	
	bus_register_notifier(&pci_bus_type,
			      &al_fabric_pci_device_nb);
	return 0;
}
arch_initcall(coherency_pci_init);

#define TOTAL_PCIE_DEVICES 3

static int pcie_link_up[TOTAL_PCIE_DEVICES];

static void update_pcie_info(int i, int offset) {
	const struct fdt_property *prop;

	prop = fdt_get_property(initial_boot_params, offset, "status", NULL);
	pcie_link_up[i] = (strcmp((char *) prop->data, "okay") == 0);
	printk("PCIE%d: status=%s\n", i, (char *) prop->data);
}

void get_pcie_info(void) {
	int i, offset;
	char *pcie_node_name = "/soc/pcie-external0/";

	for (i = 0; i < TOTAL_PCIE_DEVICES; i++) {
		pcie_node_name[18] = '0' + i;
		offset = fdt_path_offset(initial_boot_params, pcie_node_name);

		if (offset) {
			update_pcie_info(i, offset);
		}
	}
}

static int is_link_up(int nr) {
	return (nr >= 0 && nr < TOTAL_PCIE_DEVICES) ? pcie_link_up[nr] : 0;
}

static void __iomem *get_local_bridge(struct pci_bus *bus, int where) {
	int index = bus->domain_nr - 1;

	static phys_addr_t paddr[TOTAL_PCIE_DEVICES] = {
	    0xfd802000, 0xfd822000, 0xfd842000
	};

	static void __iomem *vaddr[TOTAL_PCIE_DEVICES] = {
	    NULL, NULL, NULL
	};

	BUG_ON(index < 0 || index >= TOTAL_PCIE_DEVICES);

	if (vaddr[index] == NULL) {
		vaddr[index] = ioremap(paddr[index], 0x1000);
	}

	return vaddr[index] + where;
}

static void __iomem *pci_ecam_map_alpine_external_bus(struct pci_bus *bus,
						      unsigned int devfn,
						      int where)
{
	unsigned int busnr = bus->number;
	struct pci_config_window *cfg = bus->sysdata;
	if (!is_link_up(bus->domain_nr - 1)) {
		return NULL;
	}
	if (busnr == 0) {
		return devfn ? NULL : get_local_bridge(bus, where);
	}
	else {
		return cfg->winp[busnr - 1] + (devfn << 12) + where;
	}
}

struct pci_ecam_ops pci_alpine_external_ecam_ops = {
	.bus_shift	= 16,
	.pci_ops	= {
		.map_bus	= pci_ecam_map_alpine_external_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write,
	}
};
