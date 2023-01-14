// uses drivers/clk/qcom/clk-smd-rpm.c
// adapted from: gcc-sdx20.c 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <dt-bindings/clock/qcom,gcc-mdm9607.h>
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
	P_GPLL0_OUT_MAIN,
	P_XO, // "xo" is rpm controlled clock
};
static const struct parent_map gcc_parent_map_0[] = {
	{ P_XO, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
};

static struct clk_alpha_pll gpll0 = {
	.offset = 0x21000,
	.regs = clk_alpha_pll_regs[CLK_ALPHA_PLL_TYPE_DEFAULT],
	.flags = SUPPORTS_FSM_MODE,
	.clkr = {
		.enable_reg = 0x45000,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gpll0",
			.parent_names = (const char *[]){ "xo" },
			.num_parents = 1,
			.flags = CLK_IS_CRITICAL,
			.ops = &clk_alpha_pll_ops,
		},
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

static struct clk_branch gcc_xo_pil_mss_clk = {
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "gcc_xo_pil_mss_clk",
			.ops = &clk_branch2_ops,
			.parent_names = (const char *[]){ "xo" },
			.num_parents = 1,
		},
	},
};

static struct clk_branch gcc_mss_cfg_ahb_clk = {
	.halt_reg = 0x49000,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mss_cfg_ahb_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_mss_q6_bimc_axi_clk = {
	.halt_reg = 0x49004,
	.clkr = {
		.hw.init = &(struct clk_init_data){
			.name = "gcc_mss_q6_bimc_axi_clk",
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_boot_rom_ahb_clk = {
	.halt_reg = 0x1300C,
	.halt_check = BRANCH_HALT_VOTED,
	.clkr = {
		.enable_reg = 0x45004,
		.enable_mask = BIT(7),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_boot_rom_ahb_clk",
			.parent_names = (const char *[]){ "xo" },
			.num_parents = 1,
			.ops = &clk_branch2_ops,
			.flags = CLK_SET_RATE_PARENT,
		},
	},
};

static struct freq_tbl ftbl_blsp_uart_apps_clk_src[] = {
	// freq_out, clk_enum, m_val, n_val, d_val
	F(   3686400, P_GPLL0_OUT_MAIN,    1,   72, 15625),
	F(   7372800, P_GPLL0_OUT_MAIN,    1,  144, 15625),
	F(  14745600, P_GPLL0_OUT_MAIN,    1,  288, 15625),
	F(  16000000, P_GPLL0_OUT_MAIN,   10,    1,     5),
	F(  19200000, P_XO            ,    1,    0,     0),
	F(  24000000, P_GPLL0_OUT_MAIN,    1,    3,   100),
	F(  25000000, P_GPLL0_OUT_MAIN,   16,    1,     2),
	F(  32000000, P_GPLL0_OUT_MAIN,    1,    1,    25),
	F(  40000000, P_GPLL0_OUT_MAIN,    1,    1,    20),
	F(  46400000, P_GPLL0_OUT_MAIN,    1,   29,   500),
	F(  48000000, P_GPLL0_OUT_MAIN,    1,    3,    50),
	F(  51200000, P_GPLL0_OUT_MAIN,    1,    8,   125),
	F(  56000000, P_GPLL0_OUT_MAIN,    1,    7,   100),
	F(  58982400, P_GPLL0_OUT_MAIN,    1, 1152, 15625),
	F(  60000000, P_GPLL0_OUT_MAIN,    1,    3,    40),
	{ }
};

static const char * const gcc_parent_names_0[] = {
	"xo",
	"gpll0",
};

#define BLSP1_UART5_APPS_CMD_RCGR		 0x6044
static struct clk_rcg2 blsp1_uart5_apps_clk_src = {
	.cmd_rcgr = BLSP1_UART5_APPS_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_blsp_uart_apps_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_uart5_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = ARRAY_SIZE(gcc_parent_names_0),
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_UART5_APPS_CBCR			 0x603C
static struct clk_branch gcc_blsp1_uart5_apps_clk = {
	.halt_reg = BLSP1_UART5_APPS_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = BLSP1_UART5_APPS_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_uart5_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_uart5_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct freq_tbl ftbl_blsp1_qup6_spi_apps_clk_src[] = {
	F(    960000,             P_XO,   10,    1,     2),
	F(   4800000,             P_XO,    4,    0,     0),
	F(   9600000,             P_XO,    2,    0,     0),
	F(  16000000, P_GPLL0_OUT_MAIN,   10,    1,     5),
	F(  19200000,             P_XO,    1,    0,     0),
	F(  25000000, P_GPLL0_OUT_MAIN,   16,    1,     2),
	F(  50000000, P_GPLL0_OUT_MAIN,   16,    0,     0),
	{}
};

#define BLSP1_QUP6_SPI_APPS_CMD_RCGR		 0x7024
static struct clk_rcg2 blsp1_qup6_spi_apps_clk_src = {
	.cmd_rcgr = BLSP1_QUP6_SPI_APPS_CMD_RCGR,
	.mnd_width = 16,
	.hid_width = 5,
	.parent_map = gcc_parent_map_0,
	.freq_tbl = ftbl_blsp1_qup6_spi_apps_clk_src,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "blsp1_qup6_spi_apps_clk_src",
		.parent_names = gcc_parent_names_0,
		.num_parents = ARRAY_SIZE(gcc_parent_names_0),
		.ops = &clk_rcg2_ops,
	},
};

#define BLSP1_QUP6_SPI_APPS_CBCR		 0x701C
static struct clk_branch gcc_blsp1_qup6_spi_apps_clk = {
	.halt_reg = BLSP1_QUP6_SPI_APPS_CBCR,
	.halt_check = BRANCH_HALT,
	.clkr = {
		.enable_reg = BLSP1_QUP6_SPI_APPS_CBCR,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_blsp1_qup6_spi_apps_clk",
			.parent_names = (const char *[]){
				"blsp1_qup6_spi_apps_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_branch gcc_usb2a_phy_sleep_clk = {
	.halt_reg = 0x4102c,
	.clkr = {
		.enable_reg = 0x4102c,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb2a_phy_sleep_clk",
/*
			.parent_names = (const char *[]){
				"sleep_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
*/
			.ops = &clk_branch2_ops,
		},
	},
};

static const struct parent_map gcc_xo_gpll0_bimc_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
//	{ P_BIMC, 2 },
};

static const char * const gcc_xo_gpll0_bimc[] = {
	"xo",
	"gpll0_vote",
//	"bimc_pll_vote",
};

static struct clk_rcg2 pcnoc_bfdcd_clk_src = {
	.cmd_rcgr = 0x27000,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_bimc_map,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "pcnoc_bfdcd_clk_src",
		.parent_names = gcc_xo_gpll0_bimc,
		.num_parents = ARRAY_SIZE(gcc_xo_gpll0_bimc),
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch gcc_usb_hs_ahb_clk = {
	.halt_reg = 0x41008,
	.clkr = {
		.enable_reg = 0x41008,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_ahb_clk",
			.parent_names = (const char *[]){
				"pcnoc_bfdcd_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static const struct parent_map gcc_xo_gpll0_map[] = {
	{ P_XO, 0 },
	{ P_GPLL0_OUT_MAIN, 1 },
};

static const char * const gcc_xo_gpll0[] = {
	"xo",
	"gpll0_vote",
};

static const struct freq_tbl ftbl_gcc_usb_hs_system_clk[] = {
	F(  19200000,             P_XO,    1,    0,     0),
	F(  57140000, P_GPLL0_OUT_MAIN,   14,    0,     0),
	F(  69565000, P_GPLL0_OUT_MAIN, 11.5,    0,     0),
	F( 133330000, P_GPLL0_OUT_MAIN,    6,    0,     0),
	F( 177778000, P_GPLL0_OUT_MAIN,  4.5,    0,     0),
	{}
};

static struct clk_rcg2 usb_hs_system_clk_src = {
	.cmd_rcgr = 0x41010,
	.hid_width = 5,
	.parent_map = gcc_xo_gpll0_map,
	.freq_tbl = ftbl_gcc_usb_hs_system_clk,
	.clkr.hw.init = &(struct clk_init_data){
		.name = "usb_hs_system_clk_src",
		.parent_names = gcc_xo_gpll0,
		.num_parents = 2,
		.ops = &clk_rcg2_ops,
	},
};

static struct clk_branch gcc_usb_hs_system_clk = {
	.halt_reg = 0x41004,
	.clkr = {
		.enable_reg = 0x41004,
		.enable_mask = BIT(0),
		.hw.init = &(struct clk_init_data){
			.name = "gcc_usb_hs_system_clk",
			.parent_names = (const char *[]){
				"usb_hs_system_clk_src",
			},
			.num_parents = 1,
			.flags = CLK_SET_RATE_PARENT,
			.ops = &clk_branch2_ops,
		},
	},
};

static struct clk_regmap *gcc_mdm9607_clocks[] = {
	[GCC_GPLL0] = &gpll0.clkr,
	[GCC_BLSP1_AHB_CLK] = &gcc_blsp1_ahb_clk.clkr,

#if 0
	[GCC_MSS_XO_PIL_CLK] = &gcc_xo_pil_mss_clk.clkr,
	[GCC_MSS_CFG_AHB_CLK] = &gcc_mss_cfg_ahb_clk.clkr,
	[GCC_MSS_Q6_BIMC_AXI_CLK] = &gcc_mss_q6_bimc_axi_clk.clkr,
	[GCC_BOOT_ROM_AHB_CLK] = &gcc_boot_rom_ahb_clk.clkr,
#endif
	[GCC_BLSP1_UART5_APPS_CLK_SRC] = &blsp1_uart5_apps_clk_src.clkr,
	[GCC_BLSP1_UART5_APPS_CLK] = &gcc_blsp1_uart5_apps_clk.clkr,
	[GCC_BLSP1_SPI6_APPS_CLK_SRC] = &blsp1_qup6_spi_apps_clk_src.clkr,
	[GCC_BLSP1_SPI6_APPS_CLK] = &gcc_blsp1_qup6_spi_apps_clk.clkr,

	[GCC_USB2A_PHY_SLEEP_CLK] = &gcc_usb2a_phy_sleep_clk.clkr,
	[GCC_USB_HS_AHB_CLK] = &gcc_usb_hs_ahb_clk.clkr,
	[GCC_USB_HS_SYSTEM_CLK] = &gcc_usb_hs_system_clk.clkr,
};

static struct clk_hw *gcc_mdm9607_hws[] = {
};


#define USB_HS_BCR				0x41000
#define USB_HS_HSIC_BCR				0x3D05C
#define USB2_HS_PHY_ONLY_BCR			0x41034
#define QUSB2_PHY_BCR				0x4103C


static const struct qcom_reset_map gcc_mdm9607_resets[] = {
	[GCC_USB_HS_BCR] = USB_HS_BCR,
	[GCC_USB2A_PHY_BCR] = USB2_HS_PHY_ONLY_BCR,
};

static const struct regmap_config gcc_mdm9607_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x7f000,
	.fast_io	= true,
};

static const struct qcom_cc_desc gcc_mdm9607_desc = {
	.config = &gcc_mdm9607_regmap_config,
	.clks = gcc_mdm9607_clocks,
	.num_clks = ARRAY_SIZE(gcc_mdm9607_clocks),
	.resets = gcc_mdm9607_resets,
	.num_resets = ARRAY_SIZE(gcc_mdm9607_resets),
	.clk_hws = gcc_mdm9607_hws,
	.num_clk_hws = ARRAY_SIZE(gcc_mdm9607_hws),
};

static int gcc_mdm9607_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	int ret;

	regmap = qcom_cc_map(pdev, &gcc_mdm9607_desc); // get IORESOURCE_MEM 0
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

//	clk_alpha_pll_configure(&gpll3_out_main, regmap, &gpll3_config);

	ret = qcom_cc_really_probe(pdev, &gcc_mdm9607_desc, regmap);
	return ret;
}

static struct of_device_id msm_clock_gcc_match_table[] = {
	{ .compatible = "qcom,gcc-mdm9607" },
	{}
};

static struct platform_driver msm_clock_gcc_driver = {
	.probe = gcc_mdm9607_probe,
	.driver = {
		.name = "qcom,gcc-mdm9607",
		.of_match_table = msm_clock_gcc_match_table,
		.owner = THIS_MODULE,
	},
};

int __init msm_gcc_mdm9607_init(void)
{
	return platform_driver_register(&msm_clock_gcc_driver);
}
arch_initcall(msm_gcc_mdm9607_init);
