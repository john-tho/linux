// SPDX-License-Identifier: GPL-2.0
/*
 * Qualcomm PCIe root complex driver
 *
 * Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
 * Copyright 2015 Linaro Limited.
 *
 * Author: Stanimir Varbanov <svarbanov@mm-sol.com>
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/leds-rb.h>

#include "pcie-designware.h"

#define PCIE20_PARF_SYS_CTRL			0x00
#define ECAM_BLOCKER_EN_RANGE2			BIT(30)
#define MAC_PHY_POWERDOWN_IN_P2_D_MUX_EN	BIT(29)
#define ECAM_REMOVE_OFFSET_EN			BIT(27)
#define ECAM_BLOCKER_EN				BIT(26)
#define MST_WAKEUP_EN				BIT(13)
#define SLV_WAKEUP_EN				BIT(12)
#define MSTR_ACLK_CGC_DIS			BIT(10)
#define SLV_ACLK_CGC_DIS			BIT(9)
#define CORE_CLK_CGC_DIS			BIT(6)
#define AUX_PWR_DET				BIT(4)
#define CORE_CLK_2AUX_CLK_MUX_DIS		BIT(3)
#define L23_CLK_RMV_DIS				BIT(2)
#define L1_CLK_RMV_DIS				BIT(1)

#define PCIE20_PARF_PHY_CTRL			0x40
#define CMD_BME_VAL				0x4
#define PCIE20_DEVICE_CONTROL2_STATUS2		0x98
#define PCIE_CAP_CPL_TIMEOUT_DISABLE		0x10

#define PCIE20_PARF_PHY_REFCLK			0x4C
#define PHY_REFCLK_SSP_EN			BIT(16)
#define PHY_REFCLK_USE_PAD			BIT(12)

#define PCIE20_PARF_DBI_BASE_ADDR		0x168
#define PCIE20_PARF_SLV_ADDR_SPACE_SIZE		0x16C
#define PCIE20_PARF_MHI_CLOCK_RESET_CTRL	0x174
#define PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT	0x178
#define PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT_V2	0x1A8
#define PCIE20_PARF_LTSSM			0x1B0
#define PCIE20_PARF_SID_OFFSET			0x234
#define PCIE20_PARF_BDF_TRANSLATE_CFG		0x24C
#define PCIE20_PARF_DEVICE_TYPE			0x1000
#define PCIE20_PARF_BDF_TO_SID_TABLE_N		0x2000

#define PCIE20_ELBI_SYS_CTRL			0x04
#define PCIE20_ELBI_SYS_CTRL_LT_ENABLE		BIT(0)

#define PCIE20_AXI_MSTR_RESP_COMP_CTRL0		0x818
#define CFG_REMOTE_RD_REQ_BRIDGE_SIZE_2K	0x4
#define CFG_REMOTE_RD_REQ_BRIDGE_SIZE_4K	0x5
#define PCIE20_AXI_MSTR_RESP_COMP_CTRL1		0x81c
#define CFG_BRIDGE_SB_INIT			BIT(0)

#define PCIE20_CAP				0x70
#define PCIE20_CAP_LINK_CAPABILITIES		(PCIE20_CAP + 0xC)
#define PCIE20_CAP_ACTIVE_STATE_LINK_PM_SUPPORT	(BIT(10) | BIT(11))
#define PCIE20_CAP_LINK_1			(PCIE20_CAP + 0x14)
#define PCIE_CAP_LINK1_VAL			0x2FD7F

#define PCIE20_PARF_Q2A_FLUSH			0x1AC

#define PCIE20_MISC_CONTROL_1_REG		0x8BC
#define DBI_RO_WR_EN				1

#define PERST_DELAY_US				1000


#define LTSSM_EN				(1 << 8)

#define PHY_CTRL_PHY_TX0_TERM_OFFSET_MASK	(0x1f << 16)
#define PHY_CTRL_PHY_TX0_TERM_OFFSET(x)		(x << 16)

#define PCIE20_PARF_PHY_REFCLK			0x4C
#define REF_SSP_EN				BIT(16)
#define REF_USE_PAD				BIT(12)

#define BYPASS					BIT(4)
#define MSTR_AXI_CLK_EN				BIT(1)
#define AHB_CLK_EN				BIT(0)

#define PARF_BLOCK_SLV_AXI_WR_BASE		0x360
#define PARF_BLOCK_SLV_AXI_WR_LIMIT		0x368
#define PARF_BLOCK_SLV_AXI_RD_BASE		0x370
#define PARF_BLOCK_SLV_AXI_RD_LIMIT 		0x378
#define PARF_ECAM_BASE				0x380
#define PARF_ECAM_OFFSET_REMOVAL_BASE		0x388
#define PARF_ECAM_OFFSET_REMOVAL_LIMIT		0x390
#define PARF_BLOCK_SLV_AXI_WR_BASE_2		0x398
#define PARF_BLOCK_SLV_AXI_WR_LIMIT_2		0x3A0
#define PARF_BLOCK_SLV_AXI_RD_BASE_2		0x3A8
#define PARF_BLOCK_SLV_AXI_RD_LIMIT_2		0x3B0
#define PARF_BDF_TO_SID_TABLE                   0x2000

#define PCIE_PARF_DEVICE_TYPE			0x1000

#define PCIE20_ELBI_SYS_CTRL			0x04
#define PCIE20_ELBI_SYS_CTRL_LT_ENABLE		BIT(0)

#define PCIE20_CAP				0x70
#define PCIE20_CAP_LINKCTRLSTATUS		(PCIE20_CAP + 0x10)

#define PCIE20_PLR_IATU_VIEWPORT		0x900
#define PCIE20_PLR_IATU_REGION_OUTBOUND		(0x0 << 31)
#define PCIE20_PLR_IATU_REGION_INDEX(x)		(x << 0)

#define PCIE20_PLR_IATU_CTRL1			0x904
#define PCIE20_PLR_IATU_TYPE_CFG0		(0x4 << 0)
#define PCIE20_PLR_IATU_TYPE_MEM		(0x0 << 0)

#define PCIE20_PLR_IATU_CTRL2			0x908
#define PCIE20_PLR_IATU_ENABLE			BIT(31)

#define PCIE20_PLR_IATU_LBAR			0x90C
#define PCIE20_PLR_IATU_UBAR			0x910
#define PCIE20_PLR_IATU_LAR			0x914
#define PCIE20_PLR_IATU_LTAR			0x918
#define PCIE20_PLR_IATU_UTAR			0x91c

#define MSM_PCIE_DEV_CFG_ADDR			0x01000000
#define PCIE20_CAP_LINK_CAPABILITIES		(PCIE20_CAP + 0xC)
#define PCIE20_CAP_LINK_1			(PCIE20_CAP + 0x14)

#define PCIE20_COMMAND_STATUS			0x04
#define CMD_BME_VAL				0x4
#define BUS_MASTER_EN				0x7

#define PCIE30_GEN3_RELATED_OFF			0x890
#define GEN3_EQUALIZATION_DISABLE		BIT(16)
#define RXEQ_RGRDLESS_RXTS			BIT(13)
#define GEN3_ZRXDC_NONCOMPL			BIT(0)

/* PARF registers */
#define PCIE20_PARF_PCS_DEEMPH			0x34
#define PCS_DEEMPH_TX_DEEMPH_GEN1(x)		((x) << 16)
#define PCS_DEEMPH_TX_DEEMPH_GEN2_3_5DB(x)	((x) << 8)
#define PCS_DEEMPH_TX_DEEMPH_GEN2_6DB(x)	((x) << 0)

#define PCIE20_PARF_PCS_SWING			0x38
#define PCS_SWING_TX_SWING_FULL(x)		((x) << 8)
#define PCS_SWING_TX_SWING_LOW(x)		((x) << 0)

#define PCIE20_PARF_CONFIG_BITS			0x50
#define PHY_RX0_EQ(x)				((x) << 24)

#define PCIE20_v3_PARF_SLV_ADDR_SPACE_SIZE	0x358
#define SLV_ADDR_SPACE_SZ			0x10000000

#define PCIE_CAP_CURR_DEEMPHASIS		BIT(16)
#define SPEED_GEN3				0x3

#define __set(v, a, b)	(((v) << (b)) & GENMASK(a, b))
#define __mask(a, b)	(((1 << ((a) + 1)) - 1) & ~((1 << (b)) - 1))
#define PCIE20_DEV_CAS			0x78
#define PCIE20_MRRS_MASK		__mask(14, 12)
#define PCIE20_MRRS(x)			__set(x, 14, 12)
#define PCIE20_MPS_MASK			__mask(7, 5)
#define PCIE20_MPS(x)			__set(x, 7, 5)

#define AXI_CLK_RATE		200000000
#define RCHNG_CLK_RATE		100000000

#define PCIE_V2_PARF_SIZE	0x2000


#define MV_PCI_REGS_OFFSET 0x40000
#define MV_SWITCH_REGS_BASE (0x20800000)
#define MV_DFX_REGS_BASE (0x20600000)
#define MV_PCIE_WIN_CTRL_OFF(i) (MV_PCI_REGS_OFFSET + ((i) < 6 ? 0x1820 \
				+ 0x10 * (i) : 0x1c00 + 0x10 * ((i) - 6)))
#define MV_PCIE_WIN_BASE_OFF(i) (MV_PCI_REGS_OFFSET + ((i) < 6 ? 0x1824 \
				+ 0x10 * (i) : 0x1c04 + 0x10 * ((i) - 6)))

#define PCIE20_LNK_CONTROL2_LINK_STATUS2	0xa0

#define DEVICE_TYPE_RC				0x4

#define QCOM_PCIE_2_1_0_MAX_SUPPLY	3
struct qcom_pcie_resources_2_1_0 {
	struct clk *iface_clk;
	struct clk *core_clk;
	struct clk *phy_clk;
	struct reset_control *pci_reset;
	struct reset_control *axi_reset;
	struct reset_control *ahb_reset;
	struct reset_control *por_reset;
	struct reset_control *phy_reset;
	struct regulator_bulk_data supplies[QCOM_PCIE_2_1_0_MAX_SUPPLY];
};

struct qcom_pcie_resources_1_0_0 {
	struct clk *iface;
	struct clk *aux;
	struct clk *master_bus;
	struct clk *slave_bus;
	struct reset_control *core;
	struct regulator *vdda;
};

#define QCOM_PCIE_2_3_2_MAX_SUPPLY	2
struct qcom_pcie_resources_2_3_2 {
	struct clk *aux_clk;
	struct clk *master_clk;
	struct clk *slave_clk;
	struct clk *cfg_clk;
	struct clk *pipe_clk;
	struct regulator_bulk_data supplies[QCOM_PCIE_2_3_2_MAX_SUPPLY];
};

#define QCOM_PCIE_2_4_0_MAX_CLOCKS	4
struct qcom_pcie_resources_2_4_0 {
	struct clk_bulk_data clks[QCOM_PCIE_2_4_0_MAX_CLOCKS];
	int num_clks;
	struct reset_control *axi_m_reset;
	struct reset_control *axi_s_reset;
	struct reset_control *pipe_reset;
	struct reset_control *axi_m_vmid_reset;
	struct reset_control *axi_s_xpu_reset;
	struct reset_control *parf_reset;
	struct reset_control *phy_reset;
	struct reset_control *axi_m_sticky_reset;
	struct reset_control *pipe_sticky_reset;
	struct reset_control *pwr_reset;
	struct reset_control *ahb_reset;
	struct reset_control *phy_ahb_reset;
};

struct qcom_pcie_resources_2_3_3 {
	struct clk *iface;
	struct clk *axi_m_clk;
	struct clk *axi_s_clk;
	struct clk *ahb_clk;
	struct clk *aux_clk;
	struct reset_control *rst[7];
};

struct qcom_pcie_resources_2_7_0 {
	struct clk_bulk_data clks[6];
	struct regulator_bulk_data supplies[2];
	struct reset_control *pci_reset;
	struct clk *pipe_clk;
};

struct qcom_pcie_resources_v3 {
	struct clk *sys_noc_clk;
	struct clk *axi_m_clk;
	struct clk *axi_s_clk;
	struct clk *ahb_clk;
	struct clk *aux_clk;
	struct clk *axi_bridge_clk;
	struct clk *rchng_clk;
	struct reset_control *axi_m_reset;
	struct reset_control *axi_s_reset;
	struct reset_control *pipe_reset;
	struct reset_control *axi_m_sticky_reset;
	struct reset_control *axi_s_sticky_reset;
	struct reset_control *ahb_reset;
	struct reset_control *sticky_reset;
	struct reset_control *sleep_reset;

	struct regulator *vdda;
	struct regulator *vdda_phy;
	struct regulator *vdda_refclk;
};

union qcom_pcie_resources {
	struct qcom_pcie_resources_1_0_0 v1_0_0;
	struct qcom_pcie_resources_2_1_0 v2_1_0;
	struct qcom_pcie_resources_2_3_2 v2_3_2;
	struct qcom_pcie_resources_2_3_3 v2_3_3;
	struct qcom_pcie_resources_2_4_0 v2_4_0;
	struct qcom_pcie_resources_2_7_0 v2_7_0;
	struct qcom_pcie_resources_v3 v3;
};

struct qcom_pcie;

struct qcom_pcie_ops {
	int (*get_resources)(struct qcom_pcie *pcie);
	int (*init)(struct qcom_pcie *pcie);
	int (*post_init)(struct qcom_pcie *pcie);
	void (*deinit)(struct qcom_pcie *pcie);
	void (*post_deinit)(struct qcom_pcie *pcie);
	void (*ltssm_enable)(struct qcom_pcie *pcie);
	int (*config_sid)(struct qcom_pcie *pcie);
};

struct qcom_pcie {
	struct dw_pcie *pci;
	void __iomem *parf;			/* DT parf */
	void __iomem *elbi;			/* DT elbi */
	void __iomem *dm_iatu;
	union qcom_pcie_resources res;
	struct phy *phy;
	struct gpio_desc *reset;
	struct gpio_descs *resets;
	struct gpio_desc *booster;
	struct gpio_desc *power;
	struct led_classdev *pwr_rb;
	struct led_classdev *rst_rb;
	const struct qcom_pcie_ops *ops;
	u32 is_emulation;
	u32 is_gen3;
};

#define to_qcom_pcie(x)		dev_get_drvdata((x)->dev)

static void qcom_ep_reset_assert(struct qcom_pcie *pcie)
{
	struct device_node *np = pcie->pci->dev->of_node;
	if (!pcie->rst_rb && !pcie->reset) {
		pcie->reset = devm_gpiod_get_optional(pcie->pci->dev, "perst", GPIOD_OUT_LOW);
		if (IS_ERR(pcie->reset)) {
			pcie->reset = NULL;
			pcie->rst_rb = leds_rb_request_gpio("lte-reset");
			if (IS_ERR(pcie->rst_rb))
				pcie->rst_rb = NULL;
		}
	}
	if (pcie->rst_rb)
		leds_rb_set_gpio(pcie->rst_rb, 0);
	if (pcie->reset)
	gpiod_set_value_cansleep(pcie->reset, 1);

	if (pcie->resets) {
		int i;
		for (i = 0; i < pcie->resets->ndescs; i++)
			gpiod_set_value_cansleep(pcie->resets->desc[i], 1);
	}
	if (of_device_is_compatible(np, "qcom,pcie-ipq6018"))
		usleep_range(PERST_DELAY_US, PERST_DELAY_US + 100);
	else
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static void qcom_ep_reset_deassert(struct qcom_pcie *pcie)
{
	struct device_node *np = pcie->pci->dev->of_node;
	/* Ensure that PERST has been asserted for at least 100 ms */
	msleep(100);
	if (pcie->reset) {
	gpiod_set_value_cansleep(pcie->reset, 0);
		devm_gpiod_put(pcie->pci->dev, pcie->reset);
		pcie->reset = NULL;
	}

	if (pcie->rst_rb)
		leds_rb_set_gpio(pcie->rst_rb, 1);

	if (pcie->resets) {
		int i;
		for (i = 0; i < pcie->resets->ndescs; i++)
			gpiod_set_value_cansleep(pcie->resets->desc[i], 0);
	}
	if (of_device_is_compatible(np, "qcom,pcie-ipq6018"))
		usleep_range(PERST_DELAY_US, PERST_DELAY_US + 100);
	else
	usleep_range(PERST_DELAY_US, PERST_DELAY_US + 500);
}

static int qcom_pcie_establish_link(struct qcom_pcie *pcie)
{
	struct dw_pcie *pci = pcie->pci;

	if (dw_pcie_link_up(pci))
		return 0;

	/* Enable Link Training state machine */
	if (pcie->ops->ltssm_enable)
		pcie->ops->ltssm_enable(pcie);

	return dw_pcie_wait_for_link(pci);
}

static void qcom_pcie_2_1_0_ltssm_enable(struct qcom_pcie *pcie)
{
	u32 val;

	/* enable link training */
	val = readl(pcie->elbi + PCIE20_ELBI_SYS_CTRL);
	val |= PCIE20_ELBI_SYS_CTRL_LT_ENABLE;
	writel(val, pcie->elbi + PCIE20_ELBI_SYS_CTRL);
}

static int qcom_pcie_get_resources_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vdda_phy";
	res->supplies[2].supply = "vdda_refclk";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->iface_clk = devm_clk_get(dev, "iface");
	if (IS_ERR(res->iface_clk))
		return PTR_ERR(res->iface_clk);

	res->core_clk = devm_clk_get(dev, "core");
	if (IS_ERR(res->core_clk))
		return PTR_ERR(res->core_clk);

	res->phy_clk = devm_clk_get(dev, "phy");
	if (IS_ERR(res->phy_clk))
		return PTR_ERR(res->phy_clk);

	res->pci_reset = devm_reset_control_get_exclusive(dev, "pci");
	if (IS_ERR(res->pci_reset))
		return PTR_ERR(res->pci_reset);

	res->axi_reset = devm_reset_control_get_exclusive(dev, "axi");
	if (IS_ERR(res->axi_reset))
		return PTR_ERR(res->axi_reset);

	res->ahb_reset = devm_reset_control_get_exclusive(dev, "ahb");
	if (IS_ERR(res->ahb_reset))
		return PTR_ERR(res->ahb_reset);

	res->por_reset = devm_reset_control_get_exclusive(dev, "por");
	if (IS_ERR(res->por_reset))
		return PTR_ERR(res->por_reset);

	res->phy_reset = devm_reset_control_get_exclusive(dev, "phy");
	return PTR_ERR_OR_ZERO(res->phy_reset);
}

static void qcom_pcie_deinit_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;

	reset_control_assert(res->pci_reset);
	reset_control_assert(res->axi_reset);
	reset_control_assert(res->ahb_reset);
	reset_control_assert(res->por_reset);
	reset_control_assert(res->pci_reset);
	clk_disable_unprepare(res->iface_clk);
	clk_disable_unprepare(res->core_clk);
	clk_disable_unprepare(res->phy_clk);
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static int qcom_pcie_init_2_1_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_1_0 *res = &pcie->res.v2_1_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = reset_control_assert(res->ahb_reset);
	if (ret) {
		dev_err(dev, "cannot assert ahb reset\n");
		goto err_assert_ahb;
	}

	ret = clk_prepare_enable(res->iface_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable iface clock\n");
		goto err_assert_ahb;
	}

	ret = clk_prepare_enable(res->phy_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable phy clock\n");
		goto err_clk_phy;
	}

	ret = clk_prepare_enable(res->core_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable core clock\n");
		goto err_clk_core;
	}

	ret = reset_control_deassert(res->ahb_reset);
	if (ret) {
		dev_err(dev, "cannot deassert ahb reset\n");
		goto err_deassert_ahb;
	}


	ret = reset_control_deassert(res->phy_reset);
	if (ret) {
		dev_err(dev, "cannot deassert phy reset\n");
		return ret;
	}

	ret = reset_control_deassert(res->pci_reset);
	if (ret) {
		dev_err(dev, "cannot deassert pci reset\n");
		return ret;
	}

	ret = reset_control_deassert(res->por_reset);
	if (ret) {
		dev_err(dev, "cannot deassert por reset\n");
		return ret;
	}

	ret = reset_control_deassert(res->axi_reset);
	if (ret) {
		dev_err(dev, "cannot deassert axi reset\n");
		return ret;
	}

	/* enable PCIe clocks and resets */
	val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	val &= ~BIT(0);
	writel(val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	/* enable external reference clock */
	val = readl(pcie->parf + PCIE20_PARF_PHY_REFCLK);
	val &= ~PHY_REFCLK_USE_PAD;
	val |= PHY_REFCLK_SSP_EN;
	writel(val, pcie->parf + PCIE20_PARF_PHY_REFCLK);

	/* wait for clock acquisition */
	usleep_range(1000, 1500);

	/* Set the Max TLP size to 2K, instead of using default of 4K */
	writel(CFG_REMOTE_RD_REQ_BRIDGE_SIZE_2K,
	       pci->dbi_base + PCIE20_AXI_MSTR_RESP_COMP_CTRL0);
	writel(CFG_BRIDGE_SB_INIT,
	       pci->dbi_base + PCIE20_AXI_MSTR_RESP_COMP_CTRL1);

	return 0;

err_deassert_ahb:
	clk_disable_unprepare(res->core_clk);
err_clk_core:
	clk_disable_unprepare(res->phy_clk);
err_clk_phy:
	clk_disable_unprepare(res->iface_clk);
err_assert_ahb:
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);

	return ret;
}

static int qcom_pcie_get_resources_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;

	res->vdda = devm_regulator_get(dev, "vdda");
	if (IS_ERR(res->vdda))
		return PTR_ERR(res->vdda);

	res->iface = devm_clk_get(dev, "iface");
	if (IS_ERR(res->iface))
		return PTR_ERR(res->iface);

	res->aux = devm_clk_get(dev, "aux");
	if (IS_ERR(res->aux))
		return PTR_ERR(res->aux);

	res->master_bus = devm_clk_get(dev, "master_bus");
	if (IS_ERR(res->master_bus))
		return PTR_ERR(res->master_bus);

	res->slave_bus = devm_clk_get(dev, "slave_bus");
	if (IS_ERR(res->slave_bus))
		return PTR_ERR(res->slave_bus);

	res->core = devm_reset_control_get_exclusive(dev, "core");
	return PTR_ERR_OR_ZERO(res->core);
}

static void qcom_pcie_deinit_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;

	reset_control_assert(res->core);
	clk_disable_unprepare(res->slave_bus);
	clk_disable_unprepare(res->master_bus);
	clk_disable_unprepare(res->iface);
	clk_disable_unprepare(res->aux);
	regulator_disable(res->vdda);
}

static int qcom_pcie_init_1_0_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_1_0_0 *res = &pcie->res.v1_0_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = reset_control_deassert(res->core);
	if (ret) {
		dev_err(dev, "cannot deassert core reset\n");
		return ret;
	}

	ret = clk_prepare_enable(res->aux);
	if (ret) {
		dev_err(dev, "cannot prepare/enable aux clock\n");
		goto err_res;
	}

	ret = clk_prepare_enable(res->iface);
	if (ret) {
		dev_err(dev, "cannot prepare/enable iface clock\n");
		goto err_aux;
	}

	ret = clk_prepare_enable(res->master_bus);
	if (ret) {
		dev_err(dev, "cannot prepare/enable master_bus clock\n");
		goto err_iface;
	}

	ret = clk_prepare_enable(res->slave_bus);
	if (ret) {
		dev_err(dev, "cannot prepare/enable slave_bus clock\n");
		goto err_master;
	}

	ret = regulator_enable(res->vdda);
	if (ret) {
		dev_err(dev, "cannot enable vdda regulator\n");
		goto err_slave;
	}

	/* change DBI base address */
	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		u32 val = readl(pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT);

		val |= BIT(31);
		writel(val, pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT);
	}

	return 0;
err_slave:
	clk_disable_unprepare(res->slave_bus);
err_master:
	clk_disable_unprepare(res->master_bus);
err_iface:
	clk_disable_unprepare(res->iface);
err_aux:
	clk_disable_unprepare(res->aux);
err_res:
	reset_control_assert(res->core);

	return ret;
}

static void qcom_pcie_2_3_2_ltssm_enable(struct qcom_pcie *pcie)
{
	u32 val;

	/* enable link training */
	val = readl(pcie->parf + PCIE20_PARF_LTSSM);
	val |= BIT(8);
	writel(val, pcie->parf + PCIE20_PARF_LTSSM);
}

static int qcom_pcie_get_resources_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vddpe-3v3";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->aux_clk = devm_clk_get(dev, "aux");
	if (IS_ERR(res->aux_clk))
		return PTR_ERR(res->aux_clk);

	res->cfg_clk = devm_clk_get(dev, "cfg");
	if (IS_ERR(res->cfg_clk))
		return PTR_ERR(res->cfg_clk);

	res->master_clk = devm_clk_get(dev, "bus_master");
	if (IS_ERR(res->master_clk))
		return PTR_ERR(res->master_clk);

	res->slave_clk = devm_clk_get(dev, "bus_slave");
	if (IS_ERR(res->slave_clk))
		return PTR_ERR(res->slave_clk);

	res->pipe_clk = devm_clk_get(dev, "pipe");
	return PTR_ERR_OR_ZERO(res->pipe_clk);
}

static void qcom_pcie_deinit_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;

	clk_disable_unprepare(res->slave_clk);
	clk_disable_unprepare(res->master_clk);
	clk_disable_unprepare(res->cfg_clk);
	clk_disable_unprepare(res->aux_clk);

	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static void qcom_pcie_post_deinit_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;

	clk_disable_unprepare(res->pipe_clk);
}

static int qcom_pcie_init_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = clk_prepare_enable(res->aux_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable aux clock\n");
		goto err_aux_clk;
	}

	ret = clk_prepare_enable(res->cfg_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable cfg clock\n");
		goto err_cfg_clk;
	}

	ret = clk_prepare_enable(res->master_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable master clock\n");
		goto err_master_clk;
	}

	ret = clk_prepare_enable(res->slave_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable slave clock\n");
		goto err_slave_clk;
	}

	/* enable PCIe clocks and resets */
	val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	val &= ~BIT(0);
	writel(val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	/* change DBI base address */
	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	/* MAC PHY_POWERDOWN MUX DISABLE  */
	val = readl(pcie->parf + PCIE20_PARF_SYS_CTRL);
	val &= ~BIT(29);
	writel(val, pcie->parf + PCIE20_PARF_SYS_CTRL);

	val = readl(pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);
	val |= BIT(4);
	writel(val, pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);

	val = readl(pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT_V2);
	val |= BIT(31);
	writel(val, pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT_V2);

	return 0;

err_slave_clk:
	clk_disable_unprepare(res->master_clk);
err_master_clk:
	clk_disable_unprepare(res->cfg_clk);
err_cfg_clk:
	clk_disable_unprepare(res->aux_clk);

err_aux_clk:
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);

	return ret;
}

static int qcom_pcie_post_init_2_3_2(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_2 *res = &pcie->res.v2_3_2;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = clk_prepare_enable(res->pipe_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable pipe clock\n");
		return ret;
	}

	return 0;
}

static int qcom_pcie_get_resources_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	bool is_ipq = of_device_is_compatible(dev->of_node, "qcom,pcie-ipq4019");
	int ret;

	res->clks[0].id = "aux";
	res->clks[1].id = "master_bus";
	res->clks[2].id = "slave_bus";
	res->clks[3].id = "iface";

	/* qcom,pcie-ipq4019 is defined without "iface" */
	res->num_clks = is_ipq ? 3 : 4;

	ret = devm_clk_bulk_get(dev, res->num_clks, res->clks);
	if (ret < 0)
		return ret;

	res->axi_m_reset = devm_reset_control_get_exclusive(dev, "axi_m");
	if (IS_ERR(res->axi_m_reset))
		return PTR_ERR(res->axi_m_reset);

	res->axi_s_reset = devm_reset_control_get_exclusive(dev, "axi_s");
	if (IS_ERR(res->axi_s_reset))
		return PTR_ERR(res->axi_s_reset);

	if (is_ipq) {
		/*
		 * These resources relates to the PHY or are secure clocks, but
		 * are controlled here for IPQ4019
		 */
		res->pipe_reset = devm_reset_control_get_exclusive(dev, "pipe");
		if (IS_ERR(res->pipe_reset))
			return PTR_ERR(res->pipe_reset);

		res->axi_m_vmid_reset = devm_reset_control_get_exclusive(dev,
									 "axi_m_vmid");
		if (IS_ERR(res->axi_m_vmid_reset))
			return PTR_ERR(res->axi_m_vmid_reset);

		res->axi_s_xpu_reset = devm_reset_control_get_exclusive(dev,
									"axi_s_xpu");
		if (IS_ERR(res->axi_s_xpu_reset))
			return PTR_ERR(res->axi_s_xpu_reset);

		res->parf_reset = devm_reset_control_get_exclusive(dev, "parf");
		if (IS_ERR(res->parf_reset))
			return PTR_ERR(res->parf_reset);

		res->phy_reset = devm_reset_control_get_exclusive(dev, "phy");
		if (IS_ERR(res->phy_reset))
			return PTR_ERR(res->phy_reset);
	}

	res->axi_m_sticky_reset = devm_reset_control_get_exclusive(dev,
								   "axi_m_sticky");
	if (IS_ERR(res->axi_m_sticky_reset))
		return PTR_ERR(res->axi_m_sticky_reset);

	res->pipe_sticky_reset = devm_reset_control_get_exclusive(dev,
								  "pipe_sticky");
	if (IS_ERR(res->pipe_sticky_reset))
		return PTR_ERR(res->pipe_sticky_reset);

	res->pwr_reset = devm_reset_control_get_exclusive(dev, "pwr");
	if (IS_ERR(res->pwr_reset))
		return PTR_ERR(res->pwr_reset);

	res->ahb_reset = devm_reset_control_get_exclusive(dev, "ahb");
	if (IS_ERR(res->ahb_reset))
		return PTR_ERR(res->ahb_reset);

	if (is_ipq) {
		res->phy_ahb_reset = devm_reset_control_get_exclusive(dev, "phy_ahb");
		if (IS_ERR(res->phy_ahb_reset))
			return PTR_ERR(res->phy_ahb_reset);
	}

	return 0;
}

static void qcom_pcie_deinit_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;

	reset_control_assert(res->axi_m_reset);
	reset_control_assert(res->axi_s_reset);
	reset_control_assert(res->pipe_reset);
	reset_control_assert(res->pipe_sticky_reset);
	reset_control_assert(res->phy_reset);
	reset_control_assert(res->phy_ahb_reset);
	reset_control_assert(res->axi_m_sticky_reset);
	reset_control_assert(res->pwr_reset);
	reset_control_assert(res->ahb_reset);
	clk_bulk_disable_unprepare(res->num_clks, res->clks);
}

static int qcom_pcie_init_2_4_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_4_0 *res = &pcie->res.v2_4_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	ret = reset_control_assert(res->axi_m_reset);
	if (ret) {
		dev_err(dev, "cannot assert axi master reset\n");
		return ret;
	}

	ret = reset_control_assert(res->axi_s_reset);
	if (ret) {
		dev_err(dev, "cannot assert axi slave reset\n");
		return ret;
	}

	usleep_range(10000, 12000);

	ret = reset_control_assert(res->pipe_reset);
	if (ret) {
		dev_err(dev, "cannot assert pipe reset\n");
		return ret;
	}

	ret = reset_control_assert(res->pipe_sticky_reset);
	if (ret) {
		dev_err(dev, "cannot assert pipe sticky reset\n");
		return ret;
	}

	ret = reset_control_assert(res->phy_reset);
	if (ret) {
		dev_err(dev, "cannot assert phy reset\n");
		return ret;
	}

	ret = reset_control_assert(res->phy_ahb_reset);
	if (ret) {
		dev_err(dev, "cannot assert phy ahb reset\n");
		return ret;
	}

	usleep_range(10000, 12000);

	ret = reset_control_assert(res->axi_m_sticky_reset);
	if (ret) {
		dev_err(dev, "cannot assert axi master sticky reset\n");
		return ret;
	}

	ret = reset_control_assert(res->pwr_reset);
	if (ret) {
		dev_err(dev, "cannot assert power reset\n");
		return ret;
	}

	ret = reset_control_assert(res->ahb_reset);
	if (ret) {
		dev_err(dev, "cannot assert ahb reset\n");
		return ret;
	}

	usleep_range(10000, 12000);

	ret = reset_control_deassert(res->phy_ahb_reset);
	if (ret) {
		dev_err(dev, "cannot deassert phy ahb reset\n");
		return ret;
	}

	ret = reset_control_deassert(res->phy_reset);
	if (ret) {
		dev_err(dev, "cannot deassert phy reset\n");
		goto err_rst_phy;
	}

	ret = reset_control_deassert(res->pipe_reset);
	if (ret) {
		dev_err(dev, "cannot deassert pipe reset\n");
		goto err_rst_pipe;
	}

	ret = reset_control_deassert(res->pipe_sticky_reset);
	if (ret) {
		dev_err(dev, "cannot deassert pipe sticky reset\n");
		goto err_rst_pipe_sticky;
	}

	usleep_range(10000, 12000);

	ret = reset_control_deassert(res->axi_m_reset);
	if (ret) {
		dev_err(dev, "cannot deassert axi master reset\n");
		goto err_rst_axi_m;
	}

	ret = reset_control_deassert(res->axi_m_sticky_reset);
	if (ret) {
		dev_err(dev, "cannot deassert axi master sticky reset\n");
		goto err_rst_axi_m_sticky;
	}

	ret = reset_control_deassert(res->axi_s_reset);
	if (ret) {
		dev_err(dev, "cannot deassert axi slave reset\n");
		goto err_rst_axi_s;
	}

	ret = reset_control_deassert(res->pwr_reset);
	if (ret) {
		dev_err(dev, "cannot deassert power reset\n");
		goto err_rst_pwr;
	}

	ret = reset_control_deassert(res->ahb_reset);
	if (ret) {
		dev_err(dev, "cannot deassert ahb reset\n");
		goto err_rst_ahb;
	}

	usleep_range(10000, 12000);

	ret = clk_bulk_prepare_enable(res->num_clks, res->clks);
	if (ret)
		goto err_clks;

	/* enable PCIe clocks and resets */
	val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	val &= ~BIT(0);
	writel(val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	/* change DBI base address */
	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	/* MAC PHY_POWERDOWN MUX DISABLE  */
	val = readl(pcie->parf + PCIE20_PARF_SYS_CTRL);
	val &= ~BIT(29);
	writel(val, pcie->parf + PCIE20_PARF_SYS_CTRL);

	val = readl(pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);
	val |= BIT(4);
	writel(val, pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);

	val = readl(pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT_V2);
	val |= BIT(31);
	writel(val, pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT_V2);

	return 0;

err_clks:
	reset_control_assert(res->ahb_reset);
err_rst_ahb:
	reset_control_assert(res->pwr_reset);
err_rst_pwr:
	reset_control_assert(res->axi_s_reset);
err_rst_axi_s:
	reset_control_assert(res->axi_m_sticky_reset);
err_rst_axi_m_sticky:
	reset_control_assert(res->axi_m_reset);
err_rst_axi_m:
	reset_control_assert(res->pipe_sticky_reset);
err_rst_pipe_sticky:
	reset_control_assert(res->pipe_reset);
err_rst_pipe:
	reset_control_assert(res->phy_reset);
err_rst_phy:
	reset_control_assert(res->phy_ahb_reset);
	return ret;
}

static int qcom_pcie_get_resources_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int i;
	const char *rst_names[] = { "axi_m", "axi_s", "pipe",
				    "axi_m_sticky", "sticky",
				    "ahb", "sleep", };

	res->iface = devm_clk_get(dev, "iface");
	if (IS_ERR(res->iface))
		return PTR_ERR(res->iface);

	res->axi_m_clk = devm_clk_get(dev, "axi_m");
	if (IS_ERR(res->axi_m_clk))
		return PTR_ERR(res->axi_m_clk);

	res->axi_s_clk = devm_clk_get(dev, "axi_s");
	if (IS_ERR(res->axi_s_clk))
		return PTR_ERR(res->axi_s_clk);

	res->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(res->ahb_clk))
		return PTR_ERR(res->ahb_clk);

	res->aux_clk = devm_clk_get(dev, "aux");
	if (IS_ERR(res->aux_clk))
		return PTR_ERR(res->aux_clk);

	for (i = 0; i < ARRAY_SIZE(rst_names); i++) {
		res->rst[i] = devm_reset_control_get(dev, rst_names[i]);
		if (IS_ERR(res->rst[i]))
			return PTR_ERR(res->rst[i]);
	}

	return 0;
}

static void qcom_pcie_v3_ltssm_enable(struct qcom_pcie *pcie)
{
	int i;
	u32 val;

	writel(LTSSM_EN, pcie->parf + PCIE20_PARF_LTSSM);
	if (pcie->is_gen3) {
		writel(0x20001000, pcie->parf + PARF_BLOCK_SLV_AXI_WR_BASE);
		writel(0x20100000, pcie->parf + PARF_BLOCK_SLV_AXI_WR_LIMIT);
		writel(0x20001000, pcie->parf + PARF_BLOCK_SLV_AXI_RD_BASE);
		writel(0x20100000, pcie->parf + PARF_BLOCK_SLV_AXI_RD_LIMIT);
		writel(0x20000000, pcie->parf + PARF_ECAM_BASE);
		writel(0x20001000, pcie->parf + PARF_ECAM_OFFSET_REMOVAL_BASE);
		writel(0x20200000, pcie->parf + PARF_ECAM_OFFSET_REMOVAL_LIMIT);
		writel(0x20108000, pcie->parf + PARF_BLOCK_SLV_AXI_WR_BASE_2);
		writel(0x20200000, pcie->parf + PARF_BLOCK_SLV_AXI_WR_LIMIT_2);
		writel(0x20108000, pcie->parf + PARF_BLOCK_SLV_AXI_RD_BASE_2);
		writel(0x20200000, pcie->parf + PARF_BLOCK_SLV_AXI_RD_LIMIT_2);
		for (i = 0; i < 255; i++)
			writel(0x0, pcie->parf + PARF_BDF_TO_SID_TABLE + (4 * i));
	}

	val = readl(pcie->elbi + PCIE20_ELBI_SYS_CTRL);
	val |= PCIE20_ELBI_SYS_CTRL_LT_ENABLE;
	writel(val, pcie->elbi + PCIE20_ELBI_SYS_CTRL);
}

static int qcom_pcie_get_resources_v3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_v3 *res = &pcie->res.v3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;

	res->vdda = devm_regulator_get(dev, "vdda");
	if (IS_ERR(res->vdda))
		return PTR_ERR(res->vdda);

	res->vdda_phy = devm_regulator_get(dev, "vdda_phy");
	if (IS_ERR(res->vdda_phy))
		return PTR_ERR(res->vdda_phy);

	res->vdda_refclk = devm_regulator_get(dev, "vdda_refclk");
	if (IS_ERR(res->vdda_refclk))
		return PTR_ERR(res->vdda_refclk);

	res->sys_noc_clk = devm_clk_get(dev, "sys_noc");
	if (IS_ERR(res->sys_noc_clk))
		return PTR_ERR(res->sys_noc_clk);

	res->axi_m_clk = devm_clk_get(dev, "axi_m");
	if (IS_ERR(res->axi_m_clk))
		return PTR_ERR(res->axi_m_clk);

	res->axi_s_clk = devm_clk_get(dev, "axi_s");
	if (IS_ERR(res->axi_s_clk))
		return PTR_ERR(res->axi_s_clk);

	res->ahb_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(res->ahb_clk))
		return PTR_ERR(res->ahb_clk);

	res->aux_clk = devm_clk_get(dev, "aux");
	if (IS_ERR(res->aux_clk))
		return PTR_ERR(res->aux_clk);

	if (pcie->is_gen3) {
		res->axi_bridge_clk = devm_clk_get(dev, "axi_bridge");
		if (IS_ERR(res->axi_bridge_clk))
			return PTR_ERR(res->axi_bridge_clk);

		res->rchng_clk = devm_clk_get(dev, "rchng");
		if (IS_ERR(res->rchng_clk))
			return PTR_ERR(res->rchng_clk);
	}

	res->axi_m_reset = devm_reset_control_get(dev, "axi_m");
	if (IS_ERR(res->axi_m_reset))
		return PTR_ERR(res->axi_m_reset);

	res->axi_s_reset = devm_reset_control_get(dev, "axi_s");
	if (IS_ERR(res->axi_s_reset))
		return PTR_ERR(res->axi_s_reset);

	res->pipe_reset = devm_reset_control_get(dev, "pipe");
	if (IS_ERR(res->pipe_reset))
		return PTR_ERR(res->pipe_reset);

	res->axi_m_sticky_reset = devm_reset_control_get(dev, "axi_m_sticky");
	if (IS_ERR(res->axi_m_sticky_reset))
		return PTR_ERR(res->axi_m_sticky_reset);

	if (pcie->is_gen3) {
		res->axi_s_sticky_reset = devm_reset_control_get(dev, "axi_s_sticky");
		if (IS_ERR(res->axi_s_sticky_reset))
			return PTR_ERR(res->axi_s_sticky_reset);
	}

	res->sticky_reset = devm_reset_control_get(dev, "sticky");
	if (IS_ERR(res->sticky_reset))
		return PTR_ERR(res->sticky_reset);

	res->ahb_reset = devm_reset_control_get(dev, "ahb");
	if (IS_ERR(res->ahb_reset))
		return PTR_ERR(res->ahb_reset);

	res->sleep_reset = devm_reset_control_get(dev, "sleep");
	if (IS_ERR(res->sleep_reset))
		return PTR_ERR(res->sleep_reset);

	return 0;
}

static void qcom_pcie_deinit_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;

	clk_disable_unprepare(res->iface);
	clk_disable_unprepare(res->axi_m_clk);
	clk_disable_unprepare(res->axi_s_clk);
	clk_disable_unprepare(res->ahb_clk);
	clk_disable_unprepare(res->aux_clk);
}

static void qcom_pcie_deinit_v3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_v3 *res = &pcie->res.v3;

	clk_disable_unprepare(res->axi_m_clk);
	clk_disable_unprepare(res->axi_s_clk);
	clk_disable_unprepare(res->ahb_clk);
	clk_disable_unprepare(res->aux_clk);
	clk_disable_unprepare(res->sys_noc_clk);
	regulator_disable(res->vdda);
	regulator_disable(res->vdda_phy);
	regulator_disable(res->vdda_refclk);
}

static int qcom_pcie_init_2_3_3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_3_3 *res = &pcie->res.v2_3_3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int i, ret;
	u32 val;

	for (i = 0; i < ARRAY_SIZE(res->rst); i++) {
		ret = reset_control_assert(res->rst[i]);
		if (ret) {
			dev_err(dev, "reset #%d assert failed (%d)\n", i, ret);
			return ret;
		}
	}

	usleep_range(2000, 2500);

	for (i = 0; i < ARRAY_SIZE(res->rst); i++) {
		ret = reset_control_deassert(res->rst[i]);
		if (ret) {
			dev_err(dev, "reset #%d deassert failed (%d)\n", i,
				ret);
			return ret;
		}
	}

	/*
	 * Don't have a way to see if the reset has completed.
	 * Wait for some time.
	 */
	usleep_range(2000, 2500);

	ret = clk_prepare_enable(res->iface);
	if (ret) {
		dev_err(dev, "cannot prepare/enable core clock\n");
		goto err_clk_iface;
	}

	ret = clk_prepare_enable(res->axi_m_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable core clock\n");
		goto err_clk_axi_m;
	}

	ret = clk_prepare_enable(res->axi_s_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable axi slave clock\n");
		goto err_clk_axi_s;
	}

	ret = clk_prepare_enable(res->ahb_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable ahb clock\n");
		goto err_clk_ahb;
	}

	ret = clk_prepare_enable(res->aux_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable aux clock\n");
		goto err_clk_aux;
	}

	writel(SLV_ADDR_SPACE_SZ,
		pcie->parf + PCIE20_v3_PARF_SLV_ADDR_SPACE_SIZE);

	val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	val &= ~BIT(0);
	writel(val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	writel(MST_WAKEUP_EN | SLV_WAKEUP_EN | MSTR_ACLK_CGC_DIS
		| SLV_ACLK_CGC_DIS | CORE_CLK_CGC_DIS |
		AUX_PWR_DET | L23_CLK_RMV_DIS | L1_CLK_RMV_DIS,
		pcie->parf + PCIE20_PARF_SYS_CTRL);
	writel(0, pcie->parf + PCIE20_PARF_Q2A_FLUSH);

	writel(CMD_BME_VAL, pci->dbi_base + PCIE20_COMMAND_STATUS);
	writel(DBI_RO_WR_EN, pci->dbi_base + PCIE20_MISC_CONTROL_1_REG);
	writel(PCIE_CAP_LINK1_VAL, pci->dbi_base + PCIE20_CAP_LINK_1);

	val = readl(pci->dbi_base + PCIE20_CAP_LINK_CAPABILITIES);
	val &= ~PCIE20_CAP_ACTIVE_STATE_LINK_PM_SUPPORT;
	writel(val, pci->dbi_base + PCIE20_CAP_LINK_CAPABILITIES);

	writel(PCIE_CAP_CPL_TIMEOUT_DISABLE, pci->dbi_base +
		PCIE20_DEVICE_CONTROL2_STATUS2);

	return 0;

err_clk_aux:
	clk_disable_unprepare(res->ahb_clk);
err_clk_ahb:
	clk_disable_unprepare(res->axi_s_clk);
err_clk_axi_s:
	clk_disable_unprepare(res->axi_m_clk);
err_clk_axi_m:
	clk_disable_unprepare(res->iface);
err_clk_iface:
	/*
	 * Not checking for failure, will anyway return
	 * the original failure in 'ret'.
	 */
	for (i = 0; i < ARRAY_SIZE(res->rst); i++)
		reset_control_assert(res->rst[i]);

	return ret;
}

static int qcom_pcie_get_resources_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	res->pci_reset = devm_reset_control_get_exclusive(dev, "pci");
	if (IS_ERR(res->pci_reset))
		return PTR_ERR(res->pci_reset);

	res->supplies[0].supply = "vdda";
	res->supplies[1].supply = "vddpe-3v3";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(res->supplies),
				      res->supplies);
	if (ret)
		return ret;

	res->clks[0].id = "aux";
	res->clks[1].id = "cfg";
	res->clks[2].id = "bus_master";
	res->clks[3].id = "bus_slave";
	res->clks[4].id = "slave_q2a";
	res->clks[5].id = "tbu";

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(res->clks), res->clks);
	if (ret < 0)
		return ret;

	res->pipe_clk = devm_clk_get(dev, "pipe");
	return PTR_ERR_OR_ZERO(res->pipe_clk);
}

static int qcom_pcie_init_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	u32 val;
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(res->supplies), res->supplies);
	if (ret < 0) {
		dev_err(dev, "cannot enable regulators\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(res->clks), res->clks);
	if (ret < 0)
		goto err_disable_regulators;

	ret = reset_control_assert(res->pci_reset);
	if (ret < 0) {
		dev_err(dev, "cannot deassert pci reset\n");
		goto err_disable_clocks;
	}

	usleep_range(1000, 1500);

	ret = reset_control_deassert(res->pci_reset);
	if (ret < 0) {
		dev_err(dev, "cannot deassert pci reset\n");
		goto err_disable_clocks;
	}

	ret = clk_prepare_enable(res->pipe_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable pipe clock\n");
		goto err_disable_clocks;
	}

	/* configure PCIe to RC mode */
	writel(DEVICE_TYPE_RC, pcie->parf + PCIE20_PARF_DEVICE_TYPE);

	/* enable PCIe clocks and resets */
	val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	val &= ~BIT(0);
	writel(val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	/* change DBI base address */
	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	/* MAC PHY_POWERDOWN MUX DISABLE  */
	val = readl(pcie->parf + PCIE20_PARF_SYS_CTRL);
	val &= ~BIT(29);
	writel(val, pcie->parf + PCIE20_PARF_SYS_CTRL);

	val = readl(pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);
	val |= BIT(4);
	writel(val, pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		val = readl(pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT);
		val |= BIT(31);
		writel(val, pcie->parf + PCIE20_PARF_AXI_MSTR_WR_ADDR_HALT);
	}

	return 0;
err_disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(res->clks), res->clks);
err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);

	return ret;
}

static void qcom_pcie_deinit_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;

	clk_bulk_disable_unprepare(ARRAY_SIZE(res->clks), res->clks);
	regulator_bulk_disable(ARRAY_SIZE(res->supplies), res->supplies);
}

static int qcom_pcie_post_init_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;

	return clk_prepare_enable(res->pipe_clk);
}

static void qcom_pcie_post_deinit_2_7_0(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_2_7_0 *res = &pcie->res.v2_7_0;

	clk_disable_unprepare(res->pipe_clk);
}

static int qcom_pcie_link_up(struct dw_pcie *pci)
{
	u16 val = readw(pci->dbi_base + PCIE20_CAP + PCI_EXP_LNKSTA);

	return !!(val & PCI_EXP_LNKSTA_DLLLA);
}

static void qcom_pcie_v3_reset(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_v3 *res = &pcie->res.v3;
	/* Assert pcie_pipe_ares */
	reset_control_assert(res->pipe_reset);
	reset_control_assert(res->sleep_reset);
	reset_control_assert(res->sticky_reset);
	reset_control_assert(res->axi_m_reset);
	reset_control_assert(res->axi_s_reset);
	reset_control_assert(res->ahb_reset);
	reset_control_assert(res->axi_m_sticky_reset);
	if (pcie->is_gen3)
		reset_control_assert(res->axi_s_sticky_reset);
	usleep_range(10000, 12000); /* wait 12ms */

	reset_control_deassert(res->pipe_reset);
	reset_control_deassert(res->sleep_reset);
	reset_control_deassert(res->sticky_reset);
	reset_control_deassert(res->axi_m_reset);
	reset_control_deassert(res->axi_s_reset);
	reset_control_deassert(res->ahb_reset);
	reset_control_deassert(res->axi_m_sticky_reset);
	if (pcie->is_gen3)
		reset_control_deassert(res->axi_s_sticky_reset);
	usleep_range(10000, 12000); /* wait 12ms */
	wmb(); /* ensure data is written to hw register */
}

void map_write(unsigned long reg_val, unsigned long addr) {
	void __iomem *reg = ioremap(addr, 0x4);
	writel(reg_val, reg);
	iounmap(reg);
}

void map_write_masked(unsigned long addr, unsigned long clear, unsigned long set) {
	unsigned long reg_val;
	void __iomem *reg = ioremap(addr, 0x4);
	reg_val = readl(reg);
	reg_val &= ~clear;
	reg_val |= set;
	writel(reg_val, reg);
	iounmap(reg);
}

#include <linux/crc8.h>

static int qcom_pcie_config_sid_sm8250(struct qcom_pcie *pcie)
{
	/* iommu map structure */
	struct {
		u32 bdf;
		u32 phandle;
		u32 smmu_sid;
		u32 smmu_sid_len;
	} *map;
	void __iomem *bdf_to_sid_base = pcie->parf + PCIE20_PARF_BDF_TO_SID_TABLE_N;
	struct device *dev = pcie->pci->dev;
	u8 qcom_pcie_crc8_table[CRC8_TABLE_SIZE];
	int i, nr_map, size = 0;
	u32 smmu_sid_base;

	memset_io(bdf_to_sid_base, 0, CRC8_TABLE_SIZE * sizeof(u32));
#if 0
	of_get_property(dev->of_node, "iommu-map", &size);
	if (!size)
		return 0;

	map = kzalloc(size, GFP_KERNEL);
	if (!map)
		return -ENOMEM;

	of_property_read_u32_array(dev->of_node,
		"iommu-map", (u32 *)map, size / sizeof(u32));

	nr_map = size / (sizeof(*map));

#define QCOM_PCIE_CRC8_POLYNOMIAL (BIT(2) | BIT(1) | BIT(0))

	crc8_populate_msb(qcom_pcie_crc8_table, QCOM_PCIE_CRC8_POLYNOMIAL);

	/* Registers need to be zero out first */
	memset_io(bdf_to_sid_base, 0, CRC8_TABLE_SIZE * sizeof(u32));

	/* Extract the SMMU SID base from the first entry of iommu-map */
	smmu_sid_base = map[0].smmu_sid;

	/* Look for an available entry to hold the mapping */
	for (i = 0; i < nr_map; i++) {
		u16 bdf_be = cpu_to_be16(map[i].bdf);
		u32 val;
		u8 hash;

		hash = crc8(qcom_pcie_crc8_table, (u8 *)&bdf_be, sizeof(bdf_be),
			0);

		val = readl(bdf_to_sid_base + hash * sizeof(u32));

		/* If the register is already populated, look for next available entry */
		while (val) {
			u8 current_hash = hash++;
			u8 next_mask = 0xff;

			/* If NEXT field is NULL then update it with next hash */
			if (!(val & next_mask)) {
				val |= (u32)hash;
				writel(val, bdf_to_sid_base + current_hash * sizeof(u32));
			}

			val = readl(bdf_to_sid_base + hash * sizeof(u32));
		}

		/* BDF [31:16] | SID [15:8] | NEXT [7:0] */
		val = map[i].bdf << 16 | (map[i].smmu_sid - smmu_sid_base) << 8 | 0;
		writel(val, bdf_to_sid_base + hash * sizeof(u32));
	}

	kfree(map);
#endif
	return 0;
}

static int qcom_pcie_enable_resources_v3(struct qcom_pcie *pcie)
{
	struct qcom_pcie_resources_v3 *res = &pcie->res.v3;
	struct dw_pcie *pci = pcie->pci;
	struct device *dev = pci->dev;
	int ret;

	ret = regulator_enable(res->vdda);
	if (ret) {
		dev_err(dev, "cannot enable vdda regulator\n");
		return ret;
	}

	ret = regulator_enable(res->vdda_refclk);
	if (ret) {
		dev_err(dev, "cannot enable vdda_refclk regulator\n");
		goto err_refclk;
	}

	ret = regulator_enable(res->vdda_phy);
	if (ret) {
		dev_err(dev, "cannot enable vdda_phy regulator\n");
		goto err_vdda_phy;
	}

	ret = clk_prepare_enable(res->sys_noc_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable core clock\n");
		goto err_clk_sys_noc;
	}

	ret = clk_prepare_enable(res->axi_m_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable core clock\n");
		goto err_clk_axi_m;
	}

	ret = clk_set_rate(res->axi_m_clk, AXI_CLK_RATE);
	if (ret) {
		dev_err(dev, "MClk rate set failed (%d)\n", ret);
		goto err_clk_axi_m;
	}

	ret = clk_prepare_enable(res->axi_s_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable axi slave clock\n");
		goto err_clk_axi_s;
	}

	ret = clk_set_rate(res->axi_s_clk, AXI_CLK_RATE);
	if (ret) {
		dev_err(dev, "MClk rate set failed (%d)\n", ret);
		goto err_clk_axi_s;
	}

	ret = clk_prepare_enable(res->ahb_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable ahb clock\n");
		goto err_clk_ahb;
	}

	ret = clk_prepare_enable(res->aux_clk);
	if (ret) {
		dev_err(dev, "cannot prepare/enable aux clock\n");
		goto err_clk_aux;
	}

	if (pcie->is_gen3) {
		ret = clk_prepare_enable(res->axi_bridge_clk);
		if (ret) {
			dev_err(dev, "cannot prepare/enable axi_bridge clock\n");
			goto err_clk_axi_bridge;
		}

		ret = clk_prepare_enable(res->rchng_clk);
		if (ret) {
			dev_err(dev, "cannot prepare/enable rchng_clk clock\n");
			goto err_clk_rchng;
		}

		ret = clk_set_rate(res->rchng_clk, RCHNG_CLK_RATE);
		if (ret) {
			dev_err(dev, "rchng_clk rate set failed (%d)\n", ret);
			goto err_clk_rchng;
		}
	}

	udelay(1);

	return 0;

err_clk_rchng:
	clk_disable_unprepare(res->axi_bridge_clk);
err_clk_axi_bridge:
	clk_disable_unprepare(res->aux_clk);
err_clk_aux:
	clk_disable_unprepare(res->ahb_clk);
err_clk_ahb:
	clk_disable_unprepare(res->axi_s_clk);
err_clk_axi_s:
	clk_disable_unprepare(res->axi_m_clk);
err_clk_axi_m:
	clk_disable_unprepare(res->sys_noc_clk);
err_clk_sys_noc:
	regulator_disable(res->vdda_phy);
err_vdda_phy:
	regulator_disable(res->vdda_refclk);
err_refclk:
	regulator_disable(res->vdda);
	return ret;
}

static int qcom_pcie_init_v3(struct qcom_pcie *pcie)
{
	int ret;
	struct dw_pcie *pci = pcie->pci;
	unsigned long reg_val;

	qcom_pcie_v3_reset(pcie);
	if (!pcie->is_emulation)
	qcom_ep_reset_assert(pcie);

	ret = qcom_pcie_enable_resources_v3(pcie);
	if (ret)
		return ret;

	writel(SLV_ADDR_SPACE_SZ, pcie->parf + PCIE20_v3_PARF_SLV_ADDR_SPACE_SIZE);

	ret = phy_power_on(pcie->phy);
	if (ret)
		return ret;

	reg_val = readl(pcie->parf + PCIE20_PARF_PHY_CTRL);
	reg_val &= ~BIT(0);
	writel(reg_val, pcie->parf + PCIE20_PARF_PHY_CTRL);

	writel(0, pcie->parf + PCIE20_PARF_DBI_BASE_ADDR);

	if (pcie->is_gen3) {
		writel(DEVICE_TYPE_RC, pcie->parf + PCIE_PARF_DEVICE_TYPE);
		writel(BYPASS | MSTR_AXI_CLK_EN | AHB_CLK_EN,
			pcie->parf + PCIE20_PARF_MHI_CLOCK_RESET_CTRL);
		writel(GEN3_EQUALIZATION_DISABLE | RXEQ_RGRDLESS_RXTS |
			GEN3_ZRXDC_NONCOMPL, pci->dbi_base + PCIE30_GEN3_RELATED_OFF);
	}

	if (pcie->is_gen3)
		writel(ECAM_BLOCKER_EN_RANGE2 | MAC_PHY_POWERDOWN_IN_P2_D_MUX_EN
		| ECAM_REMOVE_OFFSET_EN | ECAM_BLOCKER_EN |
		MST_WAKEUP_EN | SLV_WAKEUP_EN | MSTR_ACLK_CGC_DIS
		| SLV_ACLK_CGC_DIS | AUX_PWR_DET |
		CORE_CLK_2AUX_CLK_MUX_DIS | L23_CLK_RMV_DIS,
		pcie->parf + PCIE20_PARF_SYS_CTRL);
	else
		writel(MST_WAKEUP_EN | SLV_WAKEUP_EN | MSTR_ACLK_CGC_DIS
		| SLV_ACLK_CGC_DIS | CORE_CLK_CGC_DIS |
		AUX_PWR_DET | L23_CLK_RMV_DIS | L1_CLK_RMV_DIS,
		pcie->parf + PCIE20_PARF_SYS_CTRL);

	writel(0, pcie->parf + PCIE20_PARF_Q2A_FLUSH);
	if (pcie->is_gen3)
		writel(BUS_MASTER_EN, pci->dbi_base + PCIE20_COMMAND_STATUS);
	else
		writel(CMD_BME_VAL, pci->dbi_base + PCIE20_COMMAND_STATUS);
	writel(DBI_RO_WR_EN, pci->dbi_base + PCIE20_MISC_CONTROL_1_REG);
	writel(PCIE_CAP_LINK1_VAL, pci->dbi_base + PCIE20_CAP_LINK_1);

	reg_val = readl(pci->dbi_base + PCIE20_CAP_LINK_CAPABILITIES);
	reg_val &= ~(BIT(10) | BIT(11));
	writel(reg_val, pci->dbi_base + PCIE20_CAP_LINK_CAPABILITIES);

	writel(PCIE_CAP_CPL_TIMEOUT_DISABLE, pci->dbi_base +
		PCIE20_DEVICE_CONTROL2_STATUS2);
//	if (pcie->force_gen1) {
//		writel_relaxed((readl_relaxed(
//			pci->dbi_base + PCIE20_LNK_CONTROL2_LINK_STATUS2) | 1),
//			pci->dbi_base + PCIE20_LNK_CONTROL2_LINK_STATUS2);
//	}

	if (pcie->is_gen3)
		writel_relaxed(PCIE_CAP_CURR_DEEMPHASIS | SPEED_GEN3,
			pci->dbi_base + PCIE20_LNK_CONTROL2_LINK_STATUS2);

	return 0;
}

static int qcom_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct qcom_pcie *pcie = to_qcom_pcie(pci);
	int ret, nr_retries = 0;
	unsigned long jiffies_end = 0;

	ret = pcie->ops->init(pcie);
	if (ret)
		return ret;

	ret = phy_power_on(pcie->phy);
	if (ret)
		goto err_deinit;

	if (pcie->ops->post_init) {
		ret = pcie->ops->post_init(pcie);
		if (ret)
			goto err_disable_phy;
	}

	dw_pcie_setup_rc(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI) && !pp->msi_gicm_addr)
		dw_pcie_msi_init(pp);

	while(nr_retries < 10) {
		if (!pcie->is_emulation) {
			qcom_ep_reset_assert(pcie);
	qcom_ep_reset_deassert(pcie);
		}
		ret = qcom_pcie_establish_link(pcie);

		if (ret && pcie->booster) {
			if (!jiffies_end) {
				pr_err("Failed to establish PCIE link, enabling booster\n");
				jiffies_end = jiffies + 150 * HZ;
				gpiod_set_value_cansleep(pcie->booster, 1);

			} else if (time_after(jiffies, jiffies_end)) {
				break;
			}
			nr_retries--;
		} else {
			break;
		}
		nr_retries++;

	}

	if (jiffies_end) {
		if (!time_after(jiffies, jiffies_end)) {
			int secsBoosted = (jiffies + 150*HZ - jiffies_end) / HZ;
			msleep(secsBoosted * 1000 / 3);
		}
		printk("Disabling PCIE booster after %lds\n",
			(jiffies + 150 * HZ - jiffies_end) / HZ);
		gpiod_set_value_cansleep(pcie->booster, 0);
		if (!pcie->is_emulation) {
			qcom_ep_reset_assert(pcie);
			qcom_ep_reset_deassert(pcie);
		}
	ret = qcom_pcie_establish_link(pcie);
	}

	if (ret)
		goto err;
	if (of_device_is_compatible(pci->dev->of_node, "qcom,pcie-ipq807x")) {
		dw_pcie_prog_outbound_atu(pci, 1, 4, 0x20100000, 0x1000000, 0x800);
		if ((readl(pp->va_cfg0_base) & 0xffff) == 0x11ab) {
			unsigned i;
			unsigned long old_val;
			void __iomem *mem;

			printk("Adjusting BAR ranges for Marvell PCIE device");
			writel(0x00011042, pp->va_cfg0_base + 0x4);
			old_val = readl(pp->va_cfg0_base + 0x10);
			writel(pp->mem_base | 0xc, pp->va_cfg0_base + 0x10);
			mem = ioremap(pp->mem_base, 0xd00000);
			// change BAR1 size to 4MB and BAR2 size to 2MB
			writel(0x3f0001, mem + MV_PCI_REGS_OFFSET + 0x1804);
			writel(0x1f0001, mem + MV_PCI_REGS_OFFSET + 0x1808);

			writel(MV_SWITCH_REGS_BASE & 0xffff0000, mem + MV_PCIE_WIN_BASE_OFF(0));
			writel(((0x400000 - 1) & 0xffff0000) | 0x31, mem + MV_PCIE_WIN_CTRL_OFF(0));
			writel(MV_DFX_REGS_BASE & 0xffff0000, mem + MV_PCIE_WIN_BASE_OFF(1));
			writel(((0x100000 - 1) & 0xffff0000) | 0x83, mem + MV_PCIE_WIN_CTRL_OFF(1));
			for (i = 2; i < 20; ++i)
				writel(0, mem + MV_PCIE_WIN_CTRL_OFF(i));

			iounmap(mem);
			writel(old_val, pp->va_cfg0_base + 0x10);
		}
	}
	if (pcie->ops->config_sid) {
		printk("config-sid:");
		ret = pcie->ops->config_sid(pcie);
		if (ret)
			goto err;
	}

	return 0;

err:
	if (!pcie->is_emulation)
		qcom_ep_reset_deassert(pcie);
	if (pcie->ops->post_deinit)
		pcie->ops->post_deinit(pcie);
err_disable_phy:
	phy_power_off(pcie->phy);
err_deinit:
	pcie->ops->deinit(pcie);

	return ret;
}

static const struct dw_pcie_host_ops qcom_pcie_dw_ops = {
	.host_init = qcom_pcie_host_init,
};

/* Qcom IP rev.: 2.1.0	Synopsys IP rev.: 4.01a */
static const struct qcom_pcie_ops ops_2_1_0 = {
	.get_resources = qcom_pcie_get_resources_2_1_0,
	.init = qcom_pcie_init_2_1_0,
	.deinit = qcom_pcie_deinit_2_1_0,
	.ltssm_enable = qcom_pcie_2_1_0_ltssm_enable,
};

/* Qcom IP rev.: 1.0.0	Synopsys IP rev.: 4.11a */
static const struct qcom_pcie_ops ops_1_0_0 = {
	.get_resources = qcom_pcie_get_resources_1_0_0,
	.init = qcom_pcie_init_1_0_0,
	.deinit = qcom_pcie_deinit_1_0_0,
	.ltssm_enable = qcom_pcie_2_1_0_ltssm_enable,
};

/* Qcom IP rev.: 2.3.2	Synopsys IP rev.: 4.21a */
static const struct qcom_pcie_ops ops_2_3_2 = {
	.get_resources = qcom_pcie_get_resources_2_3_2,
	.init = qcom_pcie_init_2_3_2,
	.post_init = qcom_pcie_post_init_2_3_2,
	.deinit = qcom_pcie_deinit_2_3_2,
	.post_deinit = qcom_pcie_post_deinit_2_3_2,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.4.0	Synopsys IP rev.: 4.20a */
static const struct qcom_pcie_ops ops_2_4_0 = {
	.get_resources = qcom_pcie_get_resources_2_4_0,
	.init = qcom_pcie_init_2_4_0,
	.deinit = qcom_pcie_deinit_2_4_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.3.3	Synopsys IP rev.: 4.30a */
static const struct qcom_pcie_ops ops_2_3_3 = {
	.get_resources = qcom_pcie_get_resources_2_3_3,
	.init = qcom_pcie_init_2_3_3,
	.deinit = qcom_pcie_deinit_2_3_3,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
};

/* Qcom IP rev.: 2.7.0	Synopsys IP rev.: 4.30a */
static const struct qcom_pcie_ops ops_2_7_0 = {
	.get_resources = qcom_pcie_get_resources_2_7_0,
	.init = qcom_pcie_init_2_7_0,
	.deinit = qcom_pcie_deinit_2_7_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
	.post_init = qcom_pcie_post_init_2_7_0,
	.post_deinit = qcom_pcie_post_deinit_2_7_0,
};

/* Qcom IP rev.: 1.9.0 */
static const struct qcom_pcie_ops ops_1_9_0 = {
	.get_resources = qcom_pcie_get_resources_2_7_0,
	.init = qcom_pcie_init_2_7_0,
	.deinit = qcom_pcie_deinit_2_7_0,
	.ltssm_enable = qcom_pcie_2_3_2_ltssm_enable,
	.post_init = qcom_pcie_post_init_2_7_0,
	.post_deinit = qcom_pcie_post_deinit_2_7_0,
	.config_sid = qcom_pcie_config_sid_sm8250,
};

static const struct qcom_pcie_ops ops_v3 = {
	.get_resources = qcom_pcie_get_resources_v3,
	.init = qcom_pcie_init_v3,
	.deinit = qcom_pcie_deinit_v3,
	.ltssm_enable = qcom_pcie_v3_ltssm_enable,
};

static const struct dw_pcie_ops dw_pcie_ops = {
	.link_up = qcom_pcie_link_up,
};

static int qcom_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct pcie_port *pp;
	struct dw_pcie *pci;
	struct qcom_pcie *pcie;
	struct device_node *np = pdev->dev.of_node;
	u32 is_emulation = 0;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_disable(dev);
		return ret;
	}

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;
	pp = &pci->pp;

	pcie->pci = pci;

	pcie->ops = of_device_get_match_data(dev);

	pcie->booster = devm_gpiod_get_optional(dev, "booster", GPIOD_OUT_LOW);
	if (IS_ERR(pcie->booster)) {
		ret = PTR_ERR(pcie->booster);
		goto err_pm_runtime_put;
	}

	pcie->power = devm_gpiod_get_optional(dev, "pwr", GPIOD_OUT_LOW);
	if (IS_ERR(pcie->power)) {
		pcie->power = NULL;
		ret = PTR_ERR(pcie->power);
		pcie->pwr_rb = leds_rb_request_gpio("mpcie-power-off");
		if (IS_ERR(pcie->pwr_rb)) 
		goto err_pm_runtime_put;
		if (pcie->pwr_rb)
			leds_rb_set_gpio(pcie->pwr_rb, 0);
	} else if (pcie->power) {
		gpiod_set_value_cansleep(pcie->power, 1);
	}

	of_property_read_u32(np, "is_emulation", &is_emulation);
	pcie->is_emulation = is_emulation;

	pcie->resets = devm_gpiod_get_array_optional(dev, "persts", GPIOD_OUT_LOW);
	if (IS_ERR(pcie->resets))
		return PTR_ERR(pcie->resets);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pci->dbi_base = devm_pci_remap_cfg_resource(dev, res);
	if (IS_ERR(pci->dbi_base)) {
		ret = PTR_ERR(pci->dbi_base);
		goto err_pm_runtime_put;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "elbi");
	pcie->elbi = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->elbi)) {
		ret = PTR_ERR(pcie->elbi);
		goto err_pm_runtime_put;
	}

	pcie->is_gen3 = 0;
	if (of_device_is_compatible(pdev->dev.of_node, "qcom,pcie-ipq807x")) {
		int index;
		const int *soc_version_major = of_get_property(
				of_find_node_by_path("/"), "soc_version_major", NULL);
		BUG_ON(!soc_version_major);

		index = of_property_match_string(dev->of_node,  "phy-names",
				"pciephy");

		if (index < 0) {
			if (*soc_version_major == 1) {
				pcie->phy = devm_phy_optional_get(dev, "pciephy-gen2");
				if (IS_ERR(pcie->phy)) {
					ret = PTR_ERR(pcie->phy);
					goto err_pm_runtime_put;
				}
				pcie->is_gen3 = 0;
			} else if (*soc_version_major == 2){
				pcie->phy = devm_phy_optional_get(dev, "pciephy-gen3");
				if (IS_ERR(pcie->phy)) {
					ret = PTR_ERR(pcie->phy);	
					goto err_pm_runtime_put;
				}
				pcie->is_gen3 = 1;
			} else {
				dev_err(dev, "missing phy-names\n");
				ret = index;
				goto err_pm_runtime_put;
			}
		} else {
	pcie->phy = devm_phy_optional_get(dev, "pciephy");
	if (IS_ERR(pcie->phy)) {
		ret = PTR_ERR(pcie->phy);
		goto err_pm_runtime_put;
	}
			pcie->is_gen3 = 0;
		}
	} else {
		pcie->phy = devm_phy_optional_get(dev, "pciephy");
		if (IS_ERR(pcie->phy)) {
			printk("pcie phy is missing\n");
			ret = PTR_ERR(pcie->phy);
			goto err_pm_runtime_put;
		}
		if (of_device_is_compatible(pdev->dev.of_node, "qcom,pcie-ipq6018"))
			pcie->is_gen3 = 1;
	}

	if (pcie->is_gen3) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dm_iatu");
		pcie->dm_iatu = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->dm_iatu)) {
			ret = PTR_ERR(pcie->dm_iatu);
			goto err_pm_runtime_put;
		}
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "parf");
		res->end += PCIE_V2_PARF_SIZE;
		pcie->parf = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->parf)) {
			ret = PTR_ERR(pcie->parf);
			goto err_pm_runtime_put;
		}
	} else {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "parf");
		pcie->parf = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->parf)) {
			ret = PTR_ERR(pcie->parf);
			goto err_pm_runtime_put;
		}
	}
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "iatu");
	if (res) {
		pci->atu_base = devm_pci_remap_cfg_resource(dev, res);
	}

	ret = pcie->ops->get_resources(pcie);
	if (ret)
		goto err_pm_runtime_put;

	pp->ops = &qcom_pcie_dw_ops;

	pp->dm_iatu = pcie->dm_iatu;
	pp->is_gen3 = pcie->is_gen3;

	pp->msi_gicm_addr = 0;
	pp->msi_gicm_base = 0;
	of_property_read_u32(np, "qcom,msi-gicm-addr", &pp->msi_gicm_addr);
	of_property_read_u32(np, "qcom,msi-gicm-base", &pp->msi_gicm_base);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq_byname(pdev, "msi");
		if (pp->msi_irq < 0) {
			ret = pp->msi_irq;
			goto err_pm_runtime_put;
		}
	}

	ret = phy_init(pcie->phy);
	if (ret) {
		pm_runtime_disable(&pdev->dev);
		goto err_pm_runtime_put;
	}

	platform_set_drvdata(pdev, pcie);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "cannot initialize host\n");
		pm_runtime_disable(&pdev->dev);
		goto err_pm_runtime_put;
	}

	return 0;

err_pm_runtime_put:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return ret;
}

static int qcom_pcie_remove(struct platform_device *pdev) {
	struct qcom_pcie *pcie = platform_get_drvdata(pdev);
	dw_pcie_host_deinit(&pcie->pci->pp);
	return 0;
}

static const struct of_device_id qcom_pcie_match[] = {
	{ .compatible = "qcom,pcie-apq8084", .data = &ops_1_0_0 },
	{ .compatible = "qcom,pcie-ipq8064", .data = &ops_2_1_0 },
	{ .compatible = "qcom,pcie-apq8064", .data = &ops_2_1_0 },
	{ .compatible = "qcom,pcie-msm8996", .data = &ops_2_3_2 },
	{ .compatible = "qcom,pcie-ipq8074", .data = &ops_2_3_3 },
	{ .compatible = "qcom,pcie-ipq4019", .data = &ops_2_4_0 },
	{ .compatible = "qcom,pcie-ipq807x", .data = &ops_v3 },
	{ .compatible = "qcom,pcie-ipq6018", .data = &ops_v3 },
	{ .compatible = "qcom,pcie-qcs404", .data = &ops_2_4_0 },
	{ .compatible = "qcom,pcie-sdm845", .data = &ops_2_7_0 },
	{ .compatible = "qcom,pcie-sm8250", .data = &ops_1_9_0 },
	{ }
};

static void qcom_fixup_class(struct pci_dev *dev)
{
	dev->class = PCI_CLASS_BRIDGE_PCI << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0101, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0104, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0106, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0107, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x0302, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x1000, qcom_fixup_class);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_QCOM, 0x1001, qcom_fixup_class);

static struct platform_driver qcom_pcie_driver = {
	.probe = qcom_pcie_probe,
	.remove = qcom_pcie_remove,
	.driver = {
		.name = "qcom-pcie",
		.suppress_bind_attrs = true,
		.of_match_table = qcom_pcie_match,
	},
};

static int __init pcie_qcom_init(void) {
    return platform_driver_register(&qcom_pcie_driver);
}

static void __exit pcie_qcom_exit(void) {
    platform_driver_unregister(&qcom_pcie_driver);
}

module_init(pcie_qcom_init);
module_exit(pcie_qcom_exit);
