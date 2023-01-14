/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#include "al_hal_axi_bw_throttling.h"

int al_axi_bw_throt_handle_init(struct al_axi_bw_throt *bw_throt,
	struct al_axi_bw_throt_params *bw_throt_params)
{
	al_assert(bw_throt);
	al_assert(bw_throt_params);

	bw_throt->regs = (struct al_axi_bw_throttling_regs *)bw_throt_params->bw_throt_regs_base;

	return 0;
}

int al_axi_bw_throt_op_mode_set(struct al_axi_bw_throt *bw_throt,
	struct al_axi_bw_throt_token_ctrl *token_ctrl)
{
	uint32_t reg;

	al_assert(bw_throt);

	reg = al_reg_read32(&(bw_throt->regs->token_control));
	AL_REG_FIELD_SET(reg,
		AXI_BW_THROTTLING_TOKEN_CONTROL_MODE_MASK,
		AXI_BW_THROTTLING_TOKEN_CONTROL_MODE_SHIFT,
		token_ctrl->mode);
	AL_REG_FIELD_SET(reg,
		AXI_BW_THROTTLING_TOKEN_CONTROL_CL_MASK,
		AXI_BW_THROTTLING_TOKEN_CONTROL_CL_SHIFT,
		token_ctrl->cl_size);
	al_reg_write32(&(bw_throt->regs->token_control), reg);

	return 0;
}

int al_axi_bw_throt_period_set(struct al_axi_bw_throt *bw_throt,
	unsigned int period)
{
	al_assert(bw_throt);

	al_reg_write32(&(bw_throt->regs->token_period), period);

	return 0;
}

int al_axi_bw_throt_tok_inc_set(struct al_axi_bw_throt *bw_throt,
	unsigned int inc_val)
{
	al_assert(bw_throt);

	al_reg_write32(&(bw_throt->regs->token_val), inc_val);

	return 0;
}

int al_axi_bw_throt_tok_init_val_set(struct al_axi_bw_throt *bw_throt,
	unsigned int init_val)
{
	al_assert(bw_throt);

	al_reg_write32(&(bw_throt->regs->init_token), init_val);

	return 0;
}

int al_axi_bw_throt_tok_sat_val_set(struct al_axi_bw_throt *bw_throt,
	unsigned int sat_val)
{
	al_reg_write32(&(bw_throt->regs->token_saturation), sat_val);

	return 0;
}

unsigned int al_axi_bw_throt_tok_amnt_get(struct al_axi_bw_throt *bw_throt)
{
	uint32_t val;

	al_assert(bw_throt);

	val = al_reg_read32(&(bw_throt->regs->read_token));

	return (unsigned int)val;
}

int al_axi_bw_throt_en(struct al_axi_bw_throt *bw_throt,
	al_bool en)
{
	al_assert(bw_throt);

	al_reg_write32_masked(&(bw_throt->regs->token_control),
		AXI_BW_THROTTLING_TOKEN_CONTROL_RATE_LIMIT_EN_MASK,
		en << AXI_BW_THROTTLING_TOKEN_CONTROL_RATE_LIMIT_EN_SHIFT);

	return 0;
}

int al_axi_bw_throt_tokens_clr(struct al_axi_bw_throt *bw_throt)
{
	al_assert(bw_throt);

	al_reg_write32(&(bw_throt->regs->clr), AXI_BW_THROTTLING_CLR_TOKEN);

	return 0;
}
