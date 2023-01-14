/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 *  @defgroup groupaxi_bw_throttling AXI bandwidth throttling hardrware abstraction layer
 *  @{
 *      @file  al_hal_axi_bw_throttling.h
 *      @brief HAL Driver Header for AXI bandwidth throttling
 */

#ifndef _AL_HAL_AXI_BW_THROT_H_
#define _AL_HAL_AXI_BW_THROT_H_

#include "al_hal_common.h"
#include "al_hal_axi_bw_throttling_regs.h"

/**
 * tokens Increment mode
 */
enum al_axi_bw_throt_inc_mode {
	AXI_BW_THROT_INC_MODE_BYTES = 0,
	AXI_BW_THROT_INC_MODE_TRANSACTIONS = 1
};

/**
 * tokens Increment cache line size
 */
enum al_axi_bw_throt_inc_cl_size {
	AXI_BW_THROT_INC_CL_SIZE_32B = 0,
	AXI_BW_THROT_INC_CL_SIZE_64B = 1
};

struct al_axi_bw_throt_params {
	void __iomem *bw_throt_regs_base;
};

struct al_axi_bw_throt {
	struct al_axi_bw_throttling_regs *regs;
};

struct al_axi_bw_throt_token_ctrl {
	enum al_axi_bw_throt_inc_mode mode;
	enum al_axi_bw_throt_inc_cl_size cl_size;
};

/**
 * Initialize AXI BW throttling handle
 *
 * @param	bw_throt
 *		Pointer to uninitialized AXI BW throttling handle object
 * @param	bw_throt_params
 *		Parameters for initialization
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_handle_init(struct al_axi_bw_throt *bw_throt,
	struct al_axi_bw_throt_params *bw_throt_params);

/**
 * Set BW increment mode of operation
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	token_ctrl
 *		Token control configuration parameters
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_op_mode_set(struct al_axi_bw_throt *bw_throt,
	struct al_axi_bw_throt_token_ctrl *token_ctrl);

/**
 * Set tokens increment period in SB clock units
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	period
 *		Clock cycles amount between two consecutive updates
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_period_set(struct al_axi_bw_throt *bw_throt,
	unsigned int period);

/**
 * Set tokens increment value
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	inc_val
 *		The amount of tokens to add on each increment period
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_tok_inc_set(struct al_axi_bw_throt *bw_throt,
	unsigned int inc_val);

/**
 * Set tokens initial value
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	init_val
 *		The initial value of tokens in bucket to set
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_tok_init_val_set(struct al_axi_bw_throt *bw_throt,
	unsigned int init_val);

/**
 * Set tokens saturation value
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	sat_val
 *		The saturation value of tokens in bucket to set
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_tok_sat_val_set(struct al_axi_bw_throt *bw_throt,
	unsigned int sat_val);

/**
 * Get number of tokens in the bucket
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 *
 * @return	The amount of tokens in the bucket.
 */
unsigned int al_axi_bw_throt_tok_amnt_get(struct al_axi_bw_throt *bw_throt);

/**
 * Enable rate limiting
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 * @param	en
 *		Enable if True, otherwise disable
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_en(struct al_axi_bw_throt *bw_throt,
	al_bool en);

/**
 * Clear token bucket
 *
 * @param	bw_throt
 *		Pointer to the AXI BW throttling object
 *
 * @return	0 on success.
 */
int al_axi_bw_throt_tokens_clr(struct al_axi_bw_throt *bw_throt);

#endif /* _AL_HAL_AXI_BW_THROT_H_ */
/** @} end of groupaxi_bw_throttling group */
