#include <linux/mtd/rawnand.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/crc32.h>

#define BOOTER_PART_SIZE	256 * 1024
#define KERNEL_SMALL_PART_SIZE	4 * 1024 * 1024
#define KERNEL_PART_SIZE	6 * 1024 * 1024

static unsigned booter_size = 0;

int rb_big_boot_partition = 0;

static unsigned boot_part_size = 0;

static int __init set_boot_part_size(char *s) {
    boot_part_size = simple_strtoul(s + 1, NULL, 0);
    return 1;
}
__setup("boot_part_size", set_boot_part_size);

static unsigned parts = 1;
static int __init set_parts(char *s) {
    parts = simple_strtoul(s + 1, NULL, 0);
    if (parts < 1) parts = 1;
    if (parts > 8) parts = 8;
    return 0;
}
__setup("parts", set_parts);

static unsigned get_boot_part_size(void)
{
#ifndef __tile__
	if (boot_part_size) {
	    return boot_part_size;
	} else if (rb_big_boot_partition) {
	    return KERNEL_PART_SIZE;
	} else {
	    return KERNEL_SMALL_PART_SIZE;
	}
#else
	return 16 * 1024 * 1024;
#endif
}

static struct mtd_info *main_mtd;

static int register_of_partitions(struct mtd_info *mtd) {
	char name[80];
	int ret;
	struct device_node *mtd_node;
	struct device_node *ofpart_node;
	struct of_phandle_iterator it;

	// see ofpart.c
	mtd_node = mtd_get_of_node(mtd);
	if (!mtd_node) {
		return -1;
	}
	ofpart_node = of_get_child_by_name(mtd_node, "partitions");
	if (!ofpart_node) {
		return -1;
	}
	return mtd_device_parse_register(mtd, NULL, NULL, NULL, 0);
}

static int register_partitions(struct mtd_info *mtd)
{
	unsigned offset = 0;
	unsigned bootsize = get_boot_part_size();
	unsigned nandsize = mtd->size;
	unsigned eraseshift = ffs(mtd->erasesize) - 1;
	unsigned partsize;
	unsigned i;
	main_mtd = mtd;
	partsize = ((nandsize >> eraseshift) / parts) << eraseshift;
	mtd->orig_flags = mtd->flags;   // fix MTD_WRITEABLE on partitions

	for (i = 0; i < parts; ++i) {
	    char name[80];
	    unsigned bsize = bootsize;
	    int ret;

	    if (i == 0) {
		offset += booter_size;
		bsize -= booter_size;
	    }

	    snprintf(name, sizeof(name), "RouterBoard NAND %u Boot", i + 1);
	    ret = mtd_add_partition(mtd, name, offset, bsize);
	    if (ret != 0) return ret;
	    offset += bsize;

	    snprintf(name, sizeof(name), "RouterBoard NAND %u Main", i + 1);
	    ret = mtd_add_partition(mtd, name,
				    offset, partsize - bootsize);
	    offset += partsize - bootsize;
	    if (ret != 0) return ret;
	}

	if (booter_size) {
	    return mtd_add_partition(mtd, "RouterBoot NAND Booter",
				     0, booter_size);
	}

	return 0;
}

static ssize_t partitions_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d", parts);
}

static ssize_t partitions_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	extern int del_mtd_partitions(struct mtd_info *);
	unsigned long p;
	int ret;

	if (kstrtoul(buf, 0, &p) || p < 1 || p > 8)
		return -EINVAL;
	if (!main_mtd)
		return -EINVAL;

	parts = p;

	ret = del_mtd_partitions(main_mtd);
	register_partitions(main_mtd);

	return count;
}

static struct kobj_attribute partitions_attr =
    __ATTR(partitions, 0644, partitions_show, partitions_store);

int rb_nand_probe(struct nand_chip *nand, int booter)
{
	nand->ecc.mode = NAND_ECC_SOFT;
	nand->ecc.algo = NAND_ECC_HAMMING;
//	nand->ecc.options = NAND_ECC_GENERIC_ERASED_CHECK;
	nand->legacy.chip_delay = 25;

//	FIXME:	chip->bbt_options &= ~NAND_BBM_LASTPAGE;
	if (nand_scan(nand, 1) && nand_scan(nand, 1) &&
	    nand_scan(nand, 1) && nand_scan(nand, 1)) {
		printk("RBxxx nand device not found\n");
		return -ENXIO;
	}

	if (sysfs_create_file(kernel_kobj, &partitions_attr.attr)) {
	    printk("ERROR: could not create sys/kernel/partitions");
	}

	booter_size = booter ? max(BOOTER_PART_SIZE,
				   (int)nand_to_mtd(nand)->erasesize) : 0;
	return register_partitions(nand_to_mtd(nand));
}

int rb_nand_register_partitions(struct mtd_info *mtd) {
	int ret;

	rb_big_boot_partition = 1;
	if (sysfs_create_file(kernel_kobj, &partitions_attr.attr)) {
	    printk("ERROR: could not create sys/kernel/partitions");
	}
	mtd->orig_flags = mtd->flags;   // fix MTD_WRITEABLE on partitions

	ret = register_of_partitions(mtd);
	if (ret == -1) ret = register_partitions(mtd);
	return ret;
}
