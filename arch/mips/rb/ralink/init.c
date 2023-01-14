#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/pm.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/reset-controller.h>
#include <linux/dma-direct.h>
#include <asm/prom.h>
#include <asm/reboot.h>

#define RSTCTL       *((volatile unsigned *) KSEG1ADDR(0x1e000034))

#ifdef CONFIG_USE_OF
static int ralink_assert_device(struct reset_controller_dev *rcdev,
				unsigned long id) {
	if (id < 8)
		return -1;

	RSTCTL |= BIT(id);
	return 0;
}

static int ralink_deassert_device(struct reset_controller_dev *rcdev,
				  unsigned long id) {
	if (id < 8)
		return -1;

	RSTCTL &= ~BIT(id);
	return 0;
}

static int ralink_reset_device(struct reset_controller_dev *rcdev,
			       unsigned long id) {
	ralink_assert_device(rcdev, id);
	return ralink_deassert_device(rcdev, id);
}

static const struct reset_control_ops reset_ops = {
	.reset = ralink_reset_device,
	.assert = ralink_assert_device,
	.deassert = ralink_deassert_device,
};

static struct reset_controller_dev reset_dev = {
	.ops			= &reset_ops,
	.owner			= THIS_MODULE,
	.nr_resets		= 32,
	.of_reset_n_cells	= 1,
};

static int __init plat_of_setup(void)
{
	__dt_register_buses("mtk,mt7621-soc", "palmbus");

	reset_dev.of_node = of_find_compatible_node(NULL, NULL,
						"ralink,rt2880-reset");
	reset_controller_register(&reset_dev);

	/* force all dma operations to use ZONE_DMA */
	zone_dma_bits = 32;
	return 0;
}
arch_initcall(plat_of_setup);
#endif

static void ralink_machine_restart(char *command) {
	RSTCTL = 1;
	RSTCTL = 0;
}

void __init ralink_setup(void)
{
	_machine_restart = ralink_machine_restart;
}

