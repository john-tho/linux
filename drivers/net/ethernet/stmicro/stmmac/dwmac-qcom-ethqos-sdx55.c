#include <linux/regulator/consumer.h>

static int ethqos_init_reqgulators(struct qcom_ethqos *ethqos)
{
	int ret = 0;

	printk("init regulators\n");

#define EMAC_GDSC_EMAC_NAME "gdsc_emac"
	if (of_property_read_bool(ethqos->pdev->dev.of_node,
				  "gdsc_emac-supply")) {
		ethqos->gdsc_emac =
		devm_regulator_get(&ethqos->pdev->dev, EMAC_GDSC_EMAC_NAME);
		if (IS_ERR(ethqos->gdsc_emac)) {
			pr_err("Can not get <%s>\n", EMAC_GDSC_EMAC_NAME);
			return PTR_ERR(ethqos->gdsc_emac);
		}

		ret = regulator_enable(ethqos->gdsc_emac);
		if (ret) {
			pr_err("Can not enable <%s>\n", EMAC_GDSC_EMAC_NAME);
			goto reg_error;
		}

		pr_debug("Enabled <%s>\n", EMAC_GDSC_EMAC_NAME);
	}
#if 0 // optional
#define EMAC_VREG_RGMII_NAME "vreg_rgmii"
	if (of_property_read_bool(ethqos->pdev->dev.of_node, "vreg_rgmii-supply") // this is missing in device-tree(opintless to enable)
//		&& (2500000 == regulator_get_voltage(ethqos->reg_rgmii_io_pads)) // regulator_get_voltage segfaults
			) {
		ethqos->reg_rgmii =
		devm_regulator_get(&ethqos->pdev->dev, EMAC_VREG_RGMII_NAME);
		if (IS_ERR(ethqos->reg_rgmii)) {
			pr_err("Can not get <%s>\n", EMAC_VREG_RGMII_NAME);
			return PTR_ERR(ethqos->reg_rgmii);
		}

		ret = regulator_enable(ethqos->reg_rgmii);
		if (ret) {
			pr_err("Can not enable <%s>\n",
				  EMAC_VREG_RGMII_NAME);
			goto reg_error;
		}

		pr_debug("Enabled <%s>\n", EMAC_VREG_RGMII_NAME);
	}
#endif
#define EMAC_VREG_EMAC_PHY_NAME "vreg_emac_phy"
	if (of_property_read_bool(ethqos->pdev->dev.of_node,
				  "vreg_emac_phy-supply")) {
		ethqos->reg_emac_phy =
		devm_regulator_get(&ethqos->pdev->dev, EMAC_VREG_EMAC_PHY_NAME);
		if (IS_ERR(ethqos->reg_emac_phy)) {
			pr_err("Can not get <%s>\n",
				  EMAC_VREG_EMAC_PHY_NAME);
			return PTR_ERR(ethqos->reg_emac_phy);
		}

		ret = regulator_enable(ethqos->reg_emac_phy);
		if (ret) {
			pr_err("Can not enable <%s>\n",
				  EMAC_VREG_EMAC_PHY_NAME);
			goto reg_error;
		}

		pr_debug("Enabled <%s>\n", EMAC_VREG_EMAC_PHY_NAME);
	}
#define EMAC_VREG_RGMII_IO_PADS_NAME "vreg_rgmii_io_pads"
	if (of_property_read_bool(ethqos->pdev->dev.of_node,
				  "vreg_rgmii_io_pads-supply")) {

		ethqos->reg_rgmii_io_pads = devm_regulator_get
		(&ethqos->pdev->dev, EMAC_VREG_RGMII_IO_PADS_NAME);
		if (IS_ERR(ethqos->reg_rgmii_io_pads)) {
			pr_err("Can not get <%s>\n",
				  EMAC_VREG_RGMII_IO_PADS_NAME);
			return PTR_ERR(ethqos->reg_rgmii_io_pads);
		}

		ret = regulator_enable(ethqos->reg_rgmii_io_pads);
		if (ret) {
			pr_err("Can not enable <%s>\n",
				  EMAC_VREG_RGMII_IO_PADS_NAME);
			goto reg_error;
		}

		pr_debug("Enabled <%s>\n", EMAC_VREG_RGMII_IO_PADS_NAME);
	}
	return ret;

reg_error:
	pr_err("%s failed\n", __func__);
	return ret;
}

#include <linux/pinctrl/consumer.h>

static int ethqos_init_pinctrl(struct device *dev)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state;
	int i = 0;
	int num_names;
	const char *name;
	int ret = 0;
	
	printk("\n init pinctl\n");


	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		pr_err("Failed to get pinctrl, err = %d\n", ret);
		return ret;
	}

	num_names = of_property_count_strings(dev->of_node, "pinctrl-names");
	if (num_names < 0) {
		dev_err(dev, "Cannot parse pinctrl-names: %d\n", num_names);
		return num_names;
	}

	for (i = 0; i < num_names; i++) {
		ret = of_property_read_string_index(
			dev->of_node, "pinctrl-names", i, &name);

		if (!strcmp(name, PINCTRL_STATE_DEFAULT))
			continue;

		pinctrl_state = pinctrl_lookup_state(pinctrl, name);

		if (IS_ERR_OR_NULL(pinctrl_state)) {
			ret = PTR_ERR(pinctrl_state);
			pr_err("lookup_state %s failed %d\n", name, ret);
			return ret;
		}

		pr_info("pinctrl_lookup_state %s succeded\n", name);

		ret = pinctrl_select_state(pinctrl, pinctrl_state);

		if (ret) {
			pr_err("select_state %s failed %d\n", name, ret);
			return ret;
		}

		pr_info("pinctrl_select_state %s succeded\n", name);
	}

	printk("pinctl done OK\n");

	return ret;
}
#if 0
static int setup_gpio_input_common
	(struct device *dev, const char *name, int *gpio)
{
	int ret = 0;

	if (of_find_property(dev->of_node, name, NULL)) {
		*gpio = ret = of_get_named_gpio(dev->of_node, name, 0);
		if (ret >= 0) {
			ret = gpio_request(*gpio, name);
			if (ret) {
				pr_err("%s: Can't get GPIO %s, ret = %d\n",
					  name, *gpio);
				*gpio = -1;
				return ret;
			}

			ret = gpio_direction_input(*gpio);
			if (ret) {
				pr_err(
				   "%s: Can't set GPIO %s direction, ret = %d\n",
				   name, ret);
				return ret;
			}
		} else {
			if (ret == -EPROBE_DEFER)
				pr_err("get EMAC_GPIO probe defer\n");
			else
				pr_err("can't get gpio %s ret %d\n", name,
					  ret);
			return ret;
		}
	} else {
		pr_err("can't find gpio %s\n", name);
		ret = -EINVAL;
	}

	return ret;
}
#endif
static int ethqos_init_gpio(struct qcom_ethqos *ethqos)
{
	int ret = 0;
//	ethqos->gpio_phy_intr_redirect = -1;

	ret = ethqos_init_pinctrl(&ethqos->pdev->dev);
	if (ret) {
		pr_err("ethqos_init_pinctrl failed");
		return ret;
	}
#if 0
	ret = setup_gpio_input_common(
			&ethqos->pdev->dev, "qcom,phy-intr-redirect"
			/* ,&ethqos->gpio_phy_intr_redirect*/ ); // FIXME: ???

	if (ret) {
		pr_err("Failed to setup <%s> gpio\n",
			  "qcom,phy-intr-redirect");
		return ret;
//		goto gpio_error;
	}
#endif
	return ret;
}

#define TLMM_BASE_ADDRESS (tlmm_central_base_addr)

#define TLMM_RGMII_HDRV_PULL_CTL1_ADDRESS_OFFSET\
	(((ethqos->emac_ver == EMAC_HW_v2_3_2) ? 0xA7000\
	 : (ethqos->emac_ver == EMAC_HW_v2_0_0) ? 0xA5000\
	 : (ethqos->emac_ver == EMAC_HW_v2_2_0) ? 0xA5000\
	 : 0))

#define TLMM_RGMII_HDRV_PULL_CTL1_ADDRESS\
	(((unsigned long *)\
		(TLMM_BASE_ADDRESS + TLMM_RGMII_HDRV_PULL_CTL1_ADDRESS_OFFSET)))

#define TLMM_RGMII_HDRV_PULL_CTL1_RGWR(data)\
	iowrite32(data,	(void __iomem *)TLMM_RGMII_HDRV_PULL_CTL1_ADDRESS)

#define TLMM_RGMII_HDRV_PULL_CTL1_RGRD(data)\
	((data) = ioread32((void __iomem *)TLMM_RGMII_HDRV_PULL_CTL1_ADDRESS))

#define TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK (unsigned long)(0x7)

#define TLMM_RGMII_HDRV_PULL_CTL1_CK_TX_HDRV_WR_MASK_15\
	(unsigned long)(0xFFFC7FFF)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_3_HDRV_WR_MASK_12\
	(unsigned long)(0xFFFF8FFF)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_2_HDRV_WR_MASK_9\
	(unsigned long)(0xFFFFF1FF)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_1_HDRV_WR_MASK_6\
	(unsigned long)(0xFFFFFE3F)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_0_HDRV_WR_MASK_3\
	(unsigned long)(0xFFFFFFC7)
#define TLMM_RGMII_HDRV_PULL_CTL1_CTL_TX_HDRV_WR_MASK_0\
	(unsigned long)(0xFFFFFFF8)

#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_2MA (unsigned long)(0x0)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_4MA (unsigned long)(0x1)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_6MA (unsigned long)(0x2)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_8MA (unsigned long)(0x3)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_10MA (unsigned long)(0x4)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_12MA (unsigned long)(0x5)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA (unsigned long)(0x6)
#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA (unsigned long)(0x7)

#define TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_WR(clk, data, ctl) do {\
		unsigned long v;\
		unsigned long drv = data;\
		TLMM_RGMII_HDRV_PULL_CTL1_RGRD(v);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_CK_TX_HDRV_WR_MASK_15))\
		 | (((clk) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 15);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_TX_3_HDRV_WR_MASK_12))\
		 | (((drv) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 12);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_TX_2_HDRV_WR_MASK_9))\
		 | (((drv) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 9);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_TX_1_HDRV_WR_MASK_6))\
		 | (((drv) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 6);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_TX_0_HDRV_WR_MASK_3))\
		 | (((drv) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 3);\
		v = (v & (TLMM_RGMII_HDRV_PULL_CTL1_CTL_TX_HDRV_WR_MASK_0))\
		 | (((ctl) & (TLMM_RGMII_HDRV_PULL_CTL1_HDRV_MASK)) << 0);\
		TLMM_RGMII_HDRV_PULL_CTL1_RGWR(v);\
} while (0)

#define TLMM_RGMII_RX_HV_MODE_CTL_ADDRESS_OFFSET \
	(((ethqos->emac_ver == EMAC_HW_v2_3_2) ? 0xA7004\
	  : (ethqos->emac_ver == EMAC_HW_v2_0_0) ? 0xA5004\
	  : (ethqos->emac_ver == EMAC_HW_v2_2_0) ? 0xA5004\
	  : 0))

#define TLMM_RGMII_RX_HV_MODE_CTL_ADDRESS\
	((unsigned long *)\
	 (TLMM_BASE_ADDRESS + TLMM_RGMII_RX_HV_MODE_CTL_ADDRESS_OFFSET))\

#define TLMM_RGMII_RX_HV_MODE_CTL_RGWR(data)\
	(iowrite32(data, (void __iomem *)TLMM_RGMII_RX_HV_MODE_CTL_ADDRESS))

static int ethqos_update_rgmii_tx_drv_strength(struct qcom_ethqos *ethqos)
{
	int ret = 0;
	struct resource *resource = NULL;
	struct platform_device *pdev = ethqos->pdev;
	struct net_device *dev = platform_get_drvdata(pdev);
	unsigned long tlmm_central_base = 0;
	unsigned long tlmm_central_size = 0;
	unsigned long reg_rgmii_io_pads_voltage = 0;
	static unsigned long tlmm_central_base_addr;

	resource =
	 platform_get_resource_byname(
	    ethqos->pdev, IORESOURCE_MEM, "tlmm-central-base");

	if (!resource) {
		pr_info("Resource tlmm-central-base not found\n");
		goto err_out;
	}

	tlmm_central_base = resource->start;
	tlmm_central_size = resource_size(resource);
	pr_debug("tlmm_central_base = 0x%x, size = 0x%x\n",
		  tlmm_central_base, tlmm_central_size);

	tlmm_central_base_addr = (unsigned long)ioremap(
	   tlmm_central_base, tlmm_central_size);
	if ((void __iomem *)!tlmm_central_base_addr) {
		pr_err("cannot map dwc_tlmm_central reg memory, aborting\n");
		ret = -EIO;
		goto err_out;
	}

	pr_debug("dwc_tlmm_central = %#lx\n", tlmm_central_base_addr);

	reg_rgmii_io_pads_voltage =
	regulator_get_voltage(ethqos->reg_rgmii_io_pads);

	pr_info("IOMACRO pads voltage: %u uV\n", reg_rgmii_io_pads_voltage);

	switch (reg_rgmii_io_pads_voltage) {
	case 1500000:
	case 1800000: {
		switch (ethqos->emac_ver) {
		case EMAC_HW_v2_0_0:
		case EMAC_HW_v2_2_0:
		case EMAC_HW_v2_3_2: {
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_WR(
				   TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA,
				   TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA,
				   TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA);
				TLMM_RGMII_RX_HV_MODE_CTL_RGWR(0x0);
		}
		break;
		default:
		break;
		}
	}
	break;
	case 2500000:{ // SDX55 has 2.5V
		switch (ethqos->emac_ver) {
		case EMAC_HW_v2_0_0:
		case EMAC_HW_v2_2_0:{
#define ATH8035_PHY_ID 0x004dd072
			if (ethqos->always_on_phy) {
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_WR(
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_16MA,
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA,
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA);
			} else if ((dev->phydev) &&
					(dev->phydev->phy_id ==
					ATH8035_PHY_ID)) {
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_WR(
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA,
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA,
				TLMM_RGMII_HDRV_PULL_CTL1_TX_HDRV_14MA);
			}
		}
		break;
		default:
		break;
		}
	}
	break;
	default:
	break;
	}

err_out:
	if (tlmm_central_base_addr)
		iounmap((void __iomem *)tlmm_central_base_addr);

	return ret;
}
