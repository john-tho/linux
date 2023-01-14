/*******************************************************************************
Copyright (C) 2016 Annapurna Labs Ltd.

This file may be licensed under the terms of the Annapurna Labs Commercial
License Agreement.

Alternatively, this file can be distributed under the terms of the GNU General
Public License V2 as published by the Free Software Foundation and can be
found at http://www.gnu.org/licenses/gpl-2.0.html

Alternatively, redistribution and use in source and binary forms, with or
without modification, are permitted provided that the following conditions are
met:

    *     Redistributions of source code must retain the above copyright notice,
	  this list of conditions and the following disclaimer.

    *     Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in
	  the documentation and/or other materials provided with the
	  distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "al_hal_common.h"
#include "al_hal_eth.h"
#include "al_hal_eth_common.h"
#include "al_hal_eth_common_regs.h"
#include "al_hal_eth_mac_v4_common_regs.h"
#include "al_hal_eth_shared_resource_regs.h"
#include "al_hal_eth_ec_regs.h"

#define AL_ETH_COMMON_UDMA_NUM		4
#define AL_ETH_COMMON_EC_NUM		4
#define AL_ETH_COMMON_PORT_NUM		4
#define AL_ETH_COMMON_ADAPTER_FUNC_NUM	4

#define AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_25G 0x00000001
#define AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_50G 0x00000003
#define AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_CG_100G 0x0000000F
#define AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_40G 0x0000000F

/** General Storage Params
 *
 * This can be used to save dynamic state information (similar to board_params
 * which exist in in al_hal_eth_main.c and are used on a per ETH unit basis)
 */
/** Total number of scratch regs available for board params */
#define AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM		1
/** Params in scratch_pad1 */
#define AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_MASK	AL_FIELD_MASK(31, 0)
#define AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_SHIFT	0
#define AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_REG_IDX	0

/** Common mode to str */
const char *al_eth_common_mode_to_str(enum al_eth_common_mode mode)
{
	switch (mode) {
	/** 4 x 25G */
	case AL_ETH_COMMON_MODE_4X25G:
		return "4x25G";
	/** 2 x 50G */
	case AL_ETH_COMMON_MODE_2X50G:
		return "2x50G";
	/** 1 x 100G */
	case AL_ETH_COMMON_MODE_1X100G:
		return "1x100G";
	/** 1 x 100G Aggregated */
	case AL_ETH_COMMON_MODE_1X100G_AGG:
		return "4x100G Aggregate";
	/** 1 x 40G */
	case AL_ETH_COMMON_MODE_1X40G:
		return "1x40G";
	/** 1 x 50G, 2 x 25G */
	case AL_ETH_COMMON_MODE_1X50G_2X25G:
		return "1x50G + 2x25G";
	/** 2 x 25G, 2 x 10G */
	case AL_ETH_COMMON_MODE_2X25G_2X10G:
		return "2x25G + 2x10G";
	/** 4 x 10G */
	case AL_ETH_COMMON_MODE_4X10G:
		return "4x10G";
	default:
		break;
	}

	return "N/A";
}

unsigned int al_eth_common_mode_num_adapters_get(enum al_eth_common_mode mode)
{
	switch (mode) {
	case AL_ETH_COMMON_MODE_4X10G:
	case AL_ETH_COMMON_MODE_4X25G:
	case AL_ETH_COMMON_MODE_2X25G_2X10G:
		return 4;
	case AL_ETH_COMMON_MODE_2X50G:
		return 2;
	case AL_ETH_COMMON_MODE_1X100G:
	case AL_ETH_COMMON_MODE_1X100G_AGG:
	case AL_ETH_COMMON_MODE_1X40G:
		return 1;
	case AL_ETH_COMMON_MODE_1X50G_2X25G:
		return 3;
	default:
		break;
	}

	al_err("%s: Unknown common mode (%d), Cannot get num of adapters\n", __func__, mode);

	return 0;
}

/** Handle init */
void al_eth_common_handle_init(struct al_eth_common_handle *handle,
	struct al_eth_common_handle_init_params *handle_init_params)
{
	al_assert(handle);
	al_assert(handle_init_params);
	al_assert(handle_init_params->eth_common_regs_base);
	/* eth_ec_0_regs_base & mode shouldn't be asserted, Mainly for 2 reasons:
	 * - Breaks backward compatibility
	 * - It isn't required for most of the APIs provided (Future APIs which
	 *   will use it must assert)
	 */

	handle->eth_common_regs_base = handle_init_params->eth_common_regs_base;
	handle->eth_ec_0_regs_base = handle_init_params->eth_ec_0_regs_base;
	handle->mode = handle_init_params->mode;
}

void al_eth_common_gen_storage_params_set(const struct al_eth_common_handle *handle,
					  const struct al_eth_common_gen_storage_params *params)
{
	struct al_eth_common_regs __iomem *regs_base;
	uint32_t scratch_pad_regs[AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM] = {0};
	uint32_t *scratch_pad_ptr[AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM];
	int i;

	al_assert(handle);
	al_assert(params);
	al_assert(handle->eth_common_regs_base);

	regs_base = handle->eth_common_regs_base;
	scratch_pad_ptr[0] = &regs_base->gen.scratch_pad_1;

	AL_REG_FIELD_SET(scratch_pad_regs[AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_REG_IDX],
			 AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_MASK,
			 AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_SHIFT,
			 params->eth_func_probe_state);

	/** Write scratch regs to hw */
	for (i = 0; i < AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM; i++)
		al_reg_write32(scratch_pad_ptr[i], scratch_pad_regs[i]);
}

void al_eth_common_gen_storage_params_get(const struct al_eth_common_handle *handle,
					  struct al_eth_common_gen_storage_params *params)
{
	struct al_eth_common_regs __iomem *regs_base;
	uint32_t scratch_pad_regs[AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM] = {0};
	uint32_t *scratch_pad_ptr[AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM];
	int i;

	al_assert(handle);
	al_assert(params);
	al_assert(handle->eth_common_regs_base);

	regs_base = handle->eth_common_regs_base;
	scratch_pad_ptr[0] = &regs_base->gen.scratch_pad_1;

	/** Read scratch regs from hw */
	for (i = 0; i < AL_ETH_COMMON_GEN_STORAGE_PARAMS_SCRATCH_REGS_NUM; i++)
		scratch_pad_regs[i] = al_reg_read32(scratch_pad_ptr[i]);

	params->eth_func_probe_state = AL_REG_FIELD_GET(
		scratch_pad_regs[AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_REG_IDX],
		AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_MASK,
		AL_ETH_COMMON_GEN_STORAGE_PARAMS_ETH_FUNC_PROBE_STATE_SHIFT);
}

/* Helper functions */

void al_eth_common_udma_select_config_get(
	void __iomem *regs_base,
	struct al_eth_common_udma_select_params *params)
{
	unsigned int i = 0;
	struct al_ec_regs *ec_0_regs_base = (struct al_ec_regs *)regs_base;

	al_assert(ec_0_regs_base);
	al_assert(params);

	params->udma_select = al_reg_read32(&ec_0_regs_base->rfw_v4.udma_select);

	for (i = 0; i < AL_ARR_SIZE(params->rfw_v4_udma_select); i++) {
		params->rfw_v4_udma_select[i] =
			al_reg_read32(&ec_0_regs_base->rfw_v4_udma_select[i].port_map);
	}
}

void al_eth_common_udma_select_config_set(
	void __iomem *regs_base,
	struct al_eth_common_udma_select_params *params)
{
	unsigned int i = 0;
	struct al_ec_regs *ec_0_regs_base = (struct al_ec_regs *)regs_base;

	al_assert(ec_0_regs_base);
	al_assert(params);

	al_reg_write32(&ec_0_regs_base->rfw_v4.udma_select, params->udma_select);

	for (i = 0; i < AL_ARR_SIZE(params->rfw_v4_udma_select); i++) {
		al_reg_write32(&ec_0_regs_base->rfw_v4_udma_select[i].port_map,
			params->rfw_v4_udma_select[i]);
	}
}

int al_eth_mac_v4_fec_ctrl(struct al_eth_mac_obj *mac_obj,
	enum al_eth_mac_common_mac_v4_lane lane, enum al_eth_fec_type fec_type, al_bool fec_enable)
{
	struct al_eth_mac_v4_common_regs *mac_v4_common_regs =
			mac_obj->mac_common_regs;
	struct al_eth_mac_v4_common_gen_v4 *mac_v4_common_gen_v4 = &mac_v4_common_regs->gen_v4;

	uint32_t reg = 0;
	uint32_t fec_ena = 0;
	uint32_t fec91_ena_in = 0;

	reg = al_reg_read32(&mac_v4_common_gen_v4->pcs_fec_pin_cfg);
	if (fec_type == AL_ETH_FEC_TYPE_CLAUSE_91) {
		fec91_ena_in = AL_REG_FIELD_GET(reg,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_ENA_IN_MASK,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_ENA_IN_SHIFT);

		switch (mac_obj->mac_mode) {
		case AL_ETH_MAC_MODE_XLG_LL_25G:
			/* Set pin-based fec91_ena_in based on current lane */
			if (fec_enable) {
				fec91_ena_in |=
					AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_25G << lane;

				/* Additionally, set 1lane_in bit as special case if lane 0 / 2 */
				if (lane == AL_ETH_MAC_V4_LANE_0)
					reg |=
					ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_1LANE_IN0;
				else if (lane == AL_ETH_MAC_V4_LANE_2)
					reg |=
					ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_1LANE_IN2;
			} else
				fec91_ena_in &=
					~(AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_25G << lane);
			break;
		case AL_ETH_MAC_MODE_XLG_LL_50G:
			if (fec_enable) {
				fec91_ena_in |=
					AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_50G << lane;
			} else {
				fec91_ena_in &=
					~(AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_50G << lane);

				/* If disabled, also clear fec91_1lane_in field! */
				if (lane == AL_ETH_MAC_V4_LANE_0)
					reg &=
					~ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_1LANE_IN0;
				else if (lane == AL_ETH_MAC_V4_LANE_2)
					reg &=
					~ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_1LANE_IN2;
			}

			break;
		case AL_ETH_MAC_MODE_CG_100G:
			if (fec_enable)
				fec91_ena_in |= AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_CG_100G;
			else
				fec91_ena_in &= ~AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_CG_100G;
			break;
		default:
			/** TODO print out the common mac mode aswell ? */
			al_err("%s: fec type: %d is not supported on this MAC mode %d\n",
				__func__, fec_type, mac_obj->mac_mode);
			return -EPERM;
		}
		AL_REG_FIELD_SET(reg,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_ENA_IN_MASK,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC91_ENA_IN_SHIFT,
			fec91_ena_in);
		al_reg_write32(&mac_v4_common_gen_v4->pcs_fec_pin_cfg, reg);

	} else if (fec_type == AL_ETH_FEC_TYPE_CLAUSE_74) {
		fec_ena = AL_REG_FIELD_GET(reg,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ENA_MASK,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ENA_SHIFT);

		switch (mac_obj->mac_mode) {
		case AL_ETH_MAC_MODE_10GbE_Serial:
		case AL_ETH_MAC_MODE_XLG_LL_25G:
			if (fec_enable)
				fec_ena |= AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_25G << lane;
			else
				fec_ena &= ~(AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_25G << lane);
			break;

		case AL_ETH_MAC_MODE_XLG_LL_40G:
			if (fec_enable)
				fec_ena |= AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_40G;
			else
				fec_ena &= ~AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_40G;
			break;
		case AL_ETH_MAC_MODE_XLG_LL_50G:
			if (fec_enable)
				fec_ena |= AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_50G << lane;
			else
				fec_ena &= ~(AL_ETH_COMMON_FEC_ENABLE_MAC_MODE_XLG_LL_50G << lane);
			break;
		default:
			/** TODO print out the common mac mode aswell ? */
			al_err("%s: fec type: %d is not supported on this MAC mode %d\n",
				__func__, fec_type, mac_obj->mac_mode);
			return -EPERM;
		}
		AL_REG_FIELD_SET(reg,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ENA_MASK,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ENA_SHIFT,
			fec_ena);
		AL_REG_FIELD_SET(reg,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ERR_ENA_MASK,
			ETH_MAC_V4_COMMON_GEN_V4_PCS_FEC_PIN_CFG_FEC_ERR_ENA_SHIFT,
			fec_ena);
		al_reg_write32(&mac_v4_common_gen_v4->pcs_fec_pin_cfg, reg);
	} else {
		al_err("%s: No such fec type as %d\n", __func__, fec_type);
		return -EPERM;
	}

return 0;
}

/* API */
void al_eth_common_perf_mode_config(struct al_eth_common_handle *handle,
	enum al_eth_perf_mode mode)
{
	struct al_eth_common_regs __iomem *regs_base;
	struct al_eth_shared_resource_regs __iomem *shared_regs_base;
	al_bool crdt_neg_en; /* Negative credit enable */
	uint32_t crdt_inc_int; /* Credit increment interval */
	uint32_t crdt_lvl_th; /* Credit level threshold */
	uint32_t crdt_lvl_sat; /* Credit level saturation */
	uint32_t crdt_inc_val; /* Credit increment value */
	uint32_t reg;

	al_assert(handle);

	regs_base = (struct al_eth_common_regs __iomem *)handle->eth_common_regs_base;
	shared_regs_base = (struct al_eth_shared_resource_regs __iomem *)&regs_base->eth_sr[0];

	switch (mode) {
	case AL_ETH_PERF_MODE_HW_RESET_VALUE:
		crdt_neg_en = AL_TRUE;
		crdt_inc_int = 0x4;
		crdt_lvl_th = 0x120;
		crdt_lvl_sat = 0x130;
		crdt_inc_val = 0x1;
		break;
	case AL_ETH_PERF_MODE_A:
		crdt_neg_en = AL_TRUE;
		crdt_inc_int = 0x80;
		crdt_lvl_th = 0x120;
		crdt_lvl_sat = 0x168;
		crdt_inc_val = 0x19;
		break;
	default:
		al_assert_msg(AL_FALSE, "Unknown Ethernet Common performance mode: %d\n", mode);
		return;
	}

	/**
	 * Dirty fix to make Fortify pass.
	 * Fortify claims that this variable is written but never read, which is false claim,
	 * as it's used for a comparison operation.
	 */
	(void)crdt_neg_en;

	reg = al_reg_read32(&shared_regs_base->axs_dwrr_arb_ctrl.arb_config);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_ARB_CONFIG_MODE_NEG_CRED_EN_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_ARB_CONFIG_MODE_NEG_CRED_EN_SHIFT,
		(crdt_neg_en == AL_TRUE ? 0xF : 0));
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_ARB_CONFIG_CREDIT_GRANT_INTERVAL_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_ARB_CONFIG_CREDIT_GRANT_INTERVAL_SHIFT,
		crdt_inc_int);
	al_reg_write32(&shared_regs_base->axs_dwrr_arb_ctrl.arb_config, reg);

	reg = al_reg_read32(&shared_regs_base->axs_dwrr_arb_ctrl.cred_level);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_LEVEL_CREDIT_THRESHOLD_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_LEVEL_CREDIT_THRESHOLD_SHIFT,
		crdt_lvl_th);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_LEVEL_CREDIT_SATURATION_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_LEVEL_CREDIT_SATURATION_SHIFT,
		crdt_lvl_sat);
	al_reg_write32(&shared_regs_base->axs_dwrr_arb_ctrl.cred_level, reg);

	reg = al_reg_read32(&shared_regs_base->axs_dwrr_arb_ctrl.cred_grant);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER0_GRANT_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER0_GRANT_SHIFT,
		crdt_inc_val);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER1_GRANT_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER1_GRANT_SHIFT,
		crdt_inc_val);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER2_GRANT_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER2_GRANT_SHIFT,
		crdt_inc_val);
	AL_REG_FIELD_SET(reg,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER3_GRANT_MASK,
		ETH_SHARED_RESOURCE_AXS_DWRR_ARB_CTRL_CRED_GRANT_MASTER3_GRANT_SHIFT,
		crdt_inc_val);
	al_reg_write32(&shared_regs_base->axs_dwrr_arb_ctrl.cred_grant, reg);
}

void al_eth_common_adp2func_flr_read_raw(struct al_eth_common_handle *handle,
					 struct al_eth_common_adp2func_flr_regs *flr_regs)
{
	struct al_eth_common_regs __iomem *regs_base;

	al_assert(handle);
	al_assert(handle->eth_common_regs_base);
	al_assert(flr_regs);

	regs_base = handle->eth_common_regs_base;

	flr_regs->flr = al_reg_read32(&regs_base->adp2func.flr);
	flr_regs->flr_bar = al_reg_read32(&regs_base->adp2func.flr_bar);
}

void al_eth_common_adp2func_flr_write_raw(struct al_eth_common_handle *handle,
					  struct al_eth_common_adp2func_flr_regs *flr_regs)
{
	struct al_eth_common_regs __iomem *regs_base;

	al_assert(handle);
	al_assert(handle->eth_common_regs_base);
	al_assert(flr_regs);

	regs_base = handle->eth_common_regs_base;

	al_reg_write32(&regs_base->adp2func.flr, flr_regs->flr);
	al_reg_write32(&regs_base->adp2func.flr_bar, flr_regs->flr_bar);
}

void al_eth_common_adp2func_flr_udma_config(struct al_eth_common_handle *handle,
					    struct al_eth_common_adp2func_flr_udma_attr *udma_attr)

{
	struct al_eth_common_regs __iomem *regs_base;
	struct al_eth_common_adp2func_flr_regs flr_regs;
	struct al_eth_common_adp2func_flr_regs flr_mask;
	struct al_eth_common_adp2func_flr_regs flr_shift;

	al_assert(handle);
	al_assert(udma_attr);

	al_assert_msg(udma_attr->adapter_num < AL_ETH_COMMON_ADAPTER_NUM,
		      "Invalid adapter_num (%u)! Must be between 0-%d\n",
		      udma_attr->adapter_num,
		      AL_ETH_COMMON_ADAPTER_NUM);

	al_assert_msg(udma_attr->flr_bit < AL_ETH_COMMON_ADAPTER_FUNC_NUM,
		      "Invalid flr_bit (%u)! Must be between 0-%d\n",
		      udma_attr->flr_bit,
		      AL_ETH_COMMON_ADAPTER_FUNC_NUM);

	regs_base = handle->eth_common_regs_base;

	flr_regs.flr = al_reg_read32(&regs_base->adp2func.flr);
	flr_regs.flr_bar = al_reg_read32(&regs_base->adp2func.flr_bar);

	switch (udma_attr->udma_num) {
	case 0:
		flr_mask.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_0_MASK;
		flr_shift.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_0_SHIFT;
		flr_mask.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_0_MASK;
		flr_shift.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_0_SHIFT;
		break;
	case 1:
		flr_mask.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_1_MASK;
		flr_shift.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_1_SHIFT;
		flr_mask.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_1_MASK;
		flr_shift.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_1_SHIFT;
		break;
	case 2:
		flr_mask.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_2_MASK;
		flr_shift.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_2_SHIFT;
		flr_mask.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_2_MASK;
		flr_shift.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_2_SHIFT;
		break;
	case 3:
		flr_mask.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_3_MASK;
		flr_shift.flr = ETH_COMMON_ADP2FUNC_FLR_SEL_3_SHIFT;
		flr_mask.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_3_MASK;
		flr_shift.flr_bar = ETH_COMMON_ADP2FUNC_FLR_BAR_SEL_3_SHIFT;
		break;
	default:
		al_assert_msg(0,
			      "Invalid udma_num (%u)! Must be between 0-%d\n",
			      udma_attr->udma_num,
			      AL_ETH_COMMON_UDMA_NUM);
		return;
	};

	AL_REG_FIELD_SET(flr_regs.flr,
			 flr_mask.flr,
			 flr_shift.flr,
			 udma_attr->adapter_num);
	AL_REG_FIELD_SET(flr_regs.flr_bar,
			 flr_mask.flr_bar,
			 flr_shift.flr_bar,
			 udma_attr->flr_bit);

	al_reg_write32(&regs_base->adp2func.flr, flr_regs.flr);
	al_reg_write32(&regs_base->adp2func.flr_bar, flr_regs.flr_bar);
}

void al_eth_common_rx_pipe_modules_ctrl(struct al_eth_common_handle *handle,
					struct al_eth_common_rx_pipe_modules_ctrl_params *params)
{
	struct al_ec_regs __iomem *ec_0_regs_base;
	uint32_t gen_en;

	al_assert(handle);
	al_assert(handle->eth_ec_0_regs_base);
	al_assert(params);

	ec_0_regs_base = handle->eth_ec_0_regs_base;

	gen_en = al_reg_read32(&ec_0_regs_base->gen.en);

	al_dbg("%s: ec_0_regs_base:%p, gen_en:0x%08x\n",
	       __func__, handle->eth_ec_0_regs_base, gen_en);
	al_dbg("\t ec_gen_msw_in_valid:%d\tec_gen_msw_in:%d\n",
	       params->ec_gen_msw_in_valid, params->ec_gen_msw_in);

	if (params->ec_gen_msw_in_valid)
		AL_REG_FIELD_BIT_SET(gen_en, EC_GEN_EN_MSW_IN, params->ec_gen_msw_in);

	al_reg_write32(&ec_0_regs_base->gen.en, gen_en);
}
