// uses drivers/clk/qcom/clk-smd-rpm.c
// adapted from: /media/fast/extsdk/eg12/kernel/drivers/clk/msm/clock-gcc-9650.c 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <dt-bindings/clock/qcom,gcc-sdx20.h>
#include <linux/regmap.h>

#include "common.h" // used
#include "clk-regmap.h"
#include "clk-alpha-pll.h"
#include "clk-pll.h"
#include "clk-rcg.h"
#include "clk-branch.h"
#include "reset.h"
#include "gdsc.h"

enum {
	P_CORE_BI_PLL_TEST_SE,
	P_DSI0_PHY_PLL_OUT_BYTECLK,
	P_DSI0_PHY_PLL_OUT_DSICLK,
	P_GPLL0_OUT_MAIN,
	P_GPLL0_OUT_MAIN_DIV2,
	P_PCIE_0_PIPE_CLK,
	P_SLEEP_CLK,
	P_XO, // "xo" is rpm controlled clock
};
static const struct parent_map gcc_parent_map_0[] = {
	{ P_XO, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
	{ P_GPLL0_OUT_MAIN_DIV2, 2 },
};

static void __iomem *virt_apcsbase;

static struct clk_pll gpll0 = {
	.l_reg = 0x0004,
	.m_reg = 0x0008,
	.n_reg = 0x000c,
	.config_reg = 0x0014,
	.mode_reg = 0x0000, 

	.status_reg = 0x21000,
	.status_bit = 30,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gpll0",
		.parent_names = (const char *[]){ "xo" },
		.num_parents = 1,
		.ops = &clk_pll_ops,
	},
};

static struct clk_regmap gpll0_vote = {
	.enable_reg = 0x45000,
	.enable_mask = BIT(0),
	.hw.init = &(struct clk_init_data){
		.name = "gpll0_vote",
		.parent_names = (const char *[]){ "gpll0" },
		.num_parents = 1,
		.ops = &clk_pll_vote_ops,
	},
};

static struct clk_fixed_factor gpll0_div2 = {
	.mult = 1,
	.div = 2,
	.hw.init = &(struct clk_init_data){
		.name = "gpll0_div2",
		.parent_names = (const char *[]){ "gpll0" },
		.num_parents = 1,
		.ops = &clk_fixed_factor_ops,
	},
};

static struct clk_branch gcc_blsp1_ahb_clk = {
	.halt_reg = 0x1008,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(10),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct freq_tbl ftbl_blsp_uart_apps_clk_src[] = {
	// freq_out, clk_enum, m_val, n_val, d_val
	F(   3686400, P_GPLL0_OUT_MAIN_DIV2,    1,  192, 15625),
	F(   7372800, P_GPLL0_OUT_MAIN_DIV2,    1,  384, 15625),
	F(  14745600, P_GPLL0_OUT_MAIN_DIV2,    1,  768, 15625),
	F(  16000000, P_GPLL0_OUT_MAIN_DIV2,    1,    4,    75),
	F(  19200000, P_XO, 1, 0, 0),
	F(24000000, P_GPLL0_OUT_MAIN, 5, 1, 5),
	F(32000000, P_GPLL0_OUT_MAIN, 1, 4, 75),
	F(40000000, P_GPLL0_OUT_MAIN, 15, 0, 0),
	F(46400000, P_GPLL0_OUT_MAIN, 1, 29, 375),
	F(48000000, P_GPLL0_OUT_MAIN, 12.5, 0, 0),
	F(51200000, P_GPLL0_OUT_MAIN, 1, 32, 375),
	F(56000000, P_GPLL0_OUT_MAIN, 1, 7, 75),
	F(58982400, P_GPLL0_OUT_MAIN, 1, 1536, 15625),
	F(60000000, P_GPLL0_OUT_MAIN, 10, 0, 0),
	F(63160000, P_GPLL0_OUT_MAIN, 9.5, 0, 0),
	{ }
};

static const char * const gcc_parent_names_0[] = {
	"xo",
	"gpll0_vote",
	"gpll0_div2",
};

#define BLSP1_UART3_APPS_CMD_RCGR                        (0x4044)
static struct clk_rcg2 blsp1_uart3_apps_clk_src = {
	.cmd_rcgr = BLSP1_UART3_APPS_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_uart3_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = ARRAY_SIZE(gcc_parent_names_0),
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_UART3_APPS_CBCR                            (0x403C)
static struct clk_branch gcc_blsp1_uart3_apps_clk = {
	.halt_reg = BLSP1_UART3_APPS_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = BLSP1_UART3_APPS_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_uart3_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_uart3_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct freq_tbl ftbl_blsp_spi_apps_clk_src[] = {
	F(    960000,             P_XO,   10,    1,     2),
	F(   4800000,             P_XO,    4,    0,     0),
	F(   9600000,             P_XO,    2,    0,     0),
	F(  15000000, P_GPLL0_OUT_MAIN,   10,    1,     4),
	F(  19200000,             P_XO,    1,    0,     0),
	F(  24000000, P_GPLL0_OUT_MAIN, 12.5,    1,     2),
	F(  25000000, P_GPLL0_OUT_MAIN,   12,    1,     2),
	F(  50000000, P_GPLL0_OUT_MAIN,   12,    0,     0),
	{}
};
#define BLSP1_SPI2_APPS_CMD_RCGR                        (0x3014)
static struct clk_rcg2 blsp1_spi2_apps_clk_src = {
	.cmd_rcgr = BLSP1_SPI2_APPS_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_spi2_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = 3,
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_SPI2_APPS_CBCR                            (0x300C)
static struct clk_branch gcc_blsp1_spi2_apps_clk = {
	.halt_reg = BLSP1_SPI2_APPS_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = BLSP1_SPI2_APPS_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_spi2_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_spi2_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

#define BLSP1_SPI3_APPS_CMD_RCGR                        (0x4024)
static struct clk_rcg2 blsp1_spi3_apps_clk_src = {
	.cmd_rcgr = BLSP1_SPI3_APPS_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_blsp_spi_apps_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_spi3_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = 3,
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_SPI3_APPS_CBCR                            (0x401C)
static struct clk_branch gcc_blsp1_spi3_apps_clk = {
	.halt_reg = BLSP1_SPI3_APPS_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = BLSP1_SPI3_APPS_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_spi3_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_spi3_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

/* PCIE */
#define PCIE_CFG_AHB_CBCR				(0x5D008)
static struct clk_branch gcc_pcie_cfg_ahb_clk = {
	.halt_reg = PCIE_CFG_AHB_CBCR,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = PCIE_CFG_AHB_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_cfg_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

#define PCIE_AXI_MSTR_CBCR				(0x5D018) // not switch on
static struct clk_branch gcc_pcie_mstr_axi_clk = {
	.halt_reg = PCIE_AXI_MSTR_CBCR,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = PCIE_AXI_MSTR_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_mstr_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

#define PCIE_AXI_SLV_CBCR				(0x5D010)
static struct clk_branch gcc_pcie_slv_axi_clk = {
	.halt_reg = PCIE_AXI_SLV_CBCR,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = PCIE_AXI_SLV_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_slv_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

#define PCIE_PIPE_CBCR					(0x5D00c)
static struct clk_branch gcc_pcie_pipe_clk = {
	.halt_reg = PCIE_PIPE_CBCR,
	.halt_check = BRANCH_HALT_DELAY,
	.clkr = {
		.enable_reg = PCIE_PIPE_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_pipe_clk",
			.parent_names = (const char *[]){
				"pcie_0_pipe_clk",
			},
			.num_parents = 1,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct freq_tbl ftbl_pcie_aux_clk_src[] = {
	F(   1000000,         P_XO,    1,    5,    96),
	F(  19200000,         P_XO,    1,    0,     0),
	{}
};

#define PCIE_AUX_PHY_CMD_RCGR				(0x5D030)
static struct clk_rcg2 gcc_pcie_aux_clk_src = {
	.cmd_rcgr = PCIE_AUX_PHY_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_pcie_aux_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "gcc_pcie_aux_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = ARRAY_SIZE(gcc_parent_names_0),
		.ops = &clk_rcg2_ops,
	},
};

#define PCIE_SLEEP_CBCR					(0x5D014)
#define PCIE_AUX_CBCR					(0x5D024)
static struct clk_branch gcc_pcie_aux_clk = {
	.halt_reg = PCIE_AUX_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = PCIE_SLEEP_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_aux_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

#define PCIE_REFCLK_EN					(0x5D04c)
static struct clk_regmap gcc_pcie_refclk_en = {
	.enable_reg = PCIE_REFCLK_EN,
	.enable_mask = BIT(0),
	.hw.init = &(struct clk_init_data){
		.name = "gcc_pcie_refclk_en",
		.ops = &clk_branch_simple_ops,
	},
};

#define PCIE_AXI_TBU_CBCR				(0x12064)
#define APCS_SMMU_CLOCK_BRANCH_ENA_VOTE			(0x4500C)
static struct clk_branch gcc_pcie_axi_tbu_clk = {
	.halt_reg = PCIE_AXI_TBU_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = APCS_SMMU_CLOCK_BRANCH_ENA_VOTE,
		.enable_mask = BIT(16),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_pcie_axi_tbu_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

/*  I2C */
static const struct freq_tbl ftbl_blsp1_qup_i2c_apps_clk_src[] = {
	F(19200000, P_XO, 1, 0, 0),
	F(25000000, P_GPLL0_OUT_MAIN_DIV2, 16, 0, 0),
	F(50000000, P_GPLL0_OUT_MAIN, 16, 0, 0),
	{ }
};
#define BLSP1_QUP3_I2C_APPS_CMD_RCGR                     (0x4000)
static struct clk_rcg2 blsp1_qup3_i2c_apps_clk_src = {
	.cmd_rcgr = BLSP1_QUP3_I2C_APPS_CMD_RCGR,
	.freq_tbl = ftbl_blsp1_qup_i2c_apps_clk_src,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup3_i2c_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = ARRAY_SIZE(gcc_parent_names_0),
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_QUP3_I2C_APPS_CBCR                         (0x4020)
static struct clk_branch gcc_blsp1_qup3_i2c_apps_clk = {
	.halt_reg = BLSP1_QUP3_I2C_APPS_CMD_RCGR,
	.clkr = {
		.enable_reg = BLSP1_QUP3_I2C_APPS_CMD_RCGR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup3_i2c_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup3_i2c_apps_clk_src"
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *gcc_sdx20_clocks[] = {
	[GCC_GPLL0] = &gpll0.clkr,
	[GCC_GPLL0_VOTE] = &gpll0_vote,
	[GCC_BLSP1_AHB_CLK] = &gcc_blsp1_ahb_clk.clkr,

#if 1	// disable this if printk use "printascii"
	[GCC_BLSP1_UART3_APPS_CLK_SRC] = &blsp1_uart3_apps_clk_src.clkr,
	[GCC_BLSP1_UART3_APPS_CLK] = &gcc_blsp1_uart3_apps_clk.clkr,
#endif
	[GCC_BLSP1_SPI2_APPS_CLK_SRC] = &blsp1_spi2_apps_clk_src.clkr,
	[GCC_BLSP1_SPI2_APPS_CLK] = &gcc_blsp1_spi2_apps_clk.clkr,

	[GCC_BLSP1_SPI3_APPS_CLK_SRC] = &blsp1_spi3_apps_clk_src.clkr,
	[GCC_BLSP1_SPI3_APPS_CLK] = &gcc_blsp1_spi3_apps_clk.clkr,

	[GCC_BLSP1_QUP3_I2C_APPS_CLK_SRC] = &blsp1_qup3_i2c_apps_clk_src.clkr,
	[GCC_BLSP1_QUP3_I2C_APPS_CLK] = &gcc_blsp1_qup3_i2c_apps_clk.clkr,

	/* PCIE parent clocks */
	[GCC_PCIE_AUX_CLK_SRC] = &gcc_pcie_aux_clk_src.clkr,

	/* PCIE */
	[GCC_PCIE_AUX_CLK] = &gcc_pcie_aux_clk.clkr,
	[GCC_PCIE_AHB_CFG_CLK] = &gcc_pcie_cfg_ahb_clk.clkr,
	[GCC_PCIE_MSTR_AXI_CLK] = &gcc_pcie_mstr_axi_clk.clkr,
	[GCC_PCIE_SLV_AXI_CLK] = &gcc_pcie_slv_axi_clk.clkr,

	[GCC_PCIE_PIPE_CLK] = &gcc_pcie_pipe_clk.clkr,
	[GCC_PCIE_REFCLK_EN] = &gcc_pcie_refclk_en,
	[GCC_PCIE_AXI_TBU_CLK] = &gcc_pcie_axi_tbu_clk.clkr,
};

static struct clk_hw *gcc_sdx20_hws[] = {
	&gpll0_div2.hw,
};

#define PCIE_PHY_BCR					(0x5D048)
static const struct qcom_reset_map gcc_sdx20_resets[] = {
	[GCC_PCIEPHY_PHY_RESET] = { PCIE_PHY_BCR },
	[GCC_PCIE_PHY_RESET] = { 0x5D050 },
	[GCC_PCIE_0_PHY_BCR] = { PCIE_PHY_BCR }, // FIXME:
};

static const struct regmap_config gcc_sdx20_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x7f000,
	.fast_io	= true,
};

static const struct qcom_cc_desc gcc_sdx20_desc = {
	.config = &gcc_sdx20_regmap_config,
	.clks = gcc_sdx20_clocks,
	.num_clks = ARRAY_SIZE(gcc_sdx20_clocks),
	.resets = gcc_sdx20_resets,
	.num_resets = ARRAY_SIZE(gcc_sdx20_resets),
	.clk_hws = gcc_sdx20_hws,
	.num_clk_hws = ARRAY_SIZE(gcc_sdx20_hws),
};

static int gcc_sdx20_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &gcc_sdx20_desc); // get IORESOURCE_MEM 0
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = qcom_cc_really_probe(pdev, &gcc_sdx20_desc, regmap);
	return ret;
}

static struct of_device_id msm_clock_gcc_match_table[] = {
	{ .compatible = "qcom,gcc-sdx20" },
	{}
};

static struct platform_driver msm_clock_gcc_driver = {
	.probe = gcc_sdx20_probe,
	.driver = {
		.name = "qcom,gcc-sdx20",
		.of_match_table = msm_clock_gcc_match_table,
		.owner = THIS_MODULE,
	},
};

int __init msm_gcc_sdx20_init(void)
{
	return platform_driver_register(&msm_clock_gcc_driver);
}
arch_initcall(msm_gcc_sdx20_init);
