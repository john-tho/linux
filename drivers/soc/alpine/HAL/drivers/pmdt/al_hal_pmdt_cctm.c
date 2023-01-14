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

/**
 *  @{
 * @file   al_hal_pmdt_cctm.c
 *
 * @brief  PMDT Central Cross Triggering Matrix HAL functions
 *
 */

#include "al_hal_pmdt_cctm.h"
#include "al_hal_pmdt_cctm_regs.h"
#include "al_hal_pmdt_common.h"
#include "al_hal_iofic.h"
#include "al_hal_iofic_regs.h"

/* Auxiliary enum for the set_cctm_mask function*/
enum al_pmdt_cctm_mask_op {
	AL_PMDT_CCTM_MASK_OP_RO, /* read only */
	AL_PMDT_CCTM_MASK_OP_WO, /* write only */
	AL_PMDT_CCTM_MASK_OP_RMW, /* read modify write */
};

/**
 * Set cctm mask registers (enable/pause/clear/gpio/sw trigger)
 *
 * @param pmdt_cctm
 *		CCTM handle
 * @param addr_base
 *		mask registers base address
 * @param is_enable
 *		enable/disable operation
 * @param op
 *		mask operation
 * @param mask
 *		mask bits
 * @param is_contiguous
 *		AL_TRUE if registers for group 4-5 are contiguous to groups 0-3.
 *		AL_FALSE if registers for group 4-5 are aligned to groups 0-1 + 0x100 offset.
 */
static void set_cctm_mask(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t *addr_base,
		al_bool is_enable,
		enum al_pmdt_cctm_mask_op op,
		uint32_t *mask,
		al_bool is_contiguous)
{
	unsigned int i;
	uint32_t val = 0;
	uint32_t *reg_addr;

	for (i = 0; i < pmdt_cctm->line_size; i++) {
		/* groups 4-5 placed with offset of 0x100 from groups 0-1 */
		reg_addr = (!is_contiguous && (i >= AL_PMDT_CCTM_MATRIX_LINE_SIZE_V0)) ?
			(uint32_t *)((uintptr_t)addr_base + 0x100) : addr_base;

		if (op != AL_PMDT_CCTM_MASK_OP_WO)
			val = al_reg_read32(reg_addr + i);

		if (op != AL_PMDT_CCTM_MASK_OP_RO) {
			if (is_enable)
				val |= mask[i];
			else
				val &= ~(mask[i]);
			al_reg_write32(reg_addr + i, val);
		}

		/* return current mask */
		mask[i] = val;
		val = 0;
	}
}

static unsigned int al_pmdt_cctm_line_size_get(struct al_pmdt_cctm *pmdt_cctm)
{
	unsigned int line_size = 0;

	if (pmdt_cctm->rev_id == AL_PMDT_CCTM_V0)
		line_size = AL_PMDT_CCTM_MATRIX_LINE_SIZE_V0;
	else if (pmdt_cctm->rev_id == AL_PMDT_CCTM_V1)
		line_size = AL_PMDT_CCTM_MATRIX_LINE_SIZE_V1;
	else
		al_assert_msg(0, "Not supported CCTM revision ID - %u\n", pmdt_cctm->rev_id);

	return line_size;
}

/**
 * Initialize the CCTM handle
 */
int al_pmdt_cctm_handle_init(
		struct al_pmdt_cctm *pmdt_cctm,
		void __iomem *pmdt_cctm_regs_base)
{
	al_assert(pmdt_cctm);
	al_assert(pmdt_cctm_regs_base);

	pmdt_cctm->regs =
			(struct al_pmdt_cctm_regs *)pmdt_cctm_regs_base;

	pmdt_cctm->rev_id = al_pmdt_cctm_rev_get(pmdt_cctm);

	pmdt_cctm->line_size = al_pmdt_cctm_line_size_get(pmdt_cctm);

	al_dbg("pmdt cctm: init\n");

	return 0;
}

/**
 * Retrieve PMDT revision
 */
uint32_t al_pmdt_cctm_rev_get(struct al_pmdt_cctm *pmdt_cctm)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t rev;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	rev = AL_REG_FIELD_GET(al_reg_read32(&regs->cctm_cfg.status),
			PMDT_CCTM_CCTM_CFG_STATUS_REVISION_MASK,
			PMDT_CCTM_CCTM_CFG_STATUS_REVISION_SHIFT);

	return rev;
}

/**
 * Retrieve CCTM PMU configuration info
 */
int al_pmdt_cctm_pmu_conf_get(
		struct al_pmdt_cctm *pmdt_cctm,
		struct al_cctm_config *info)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t val;

	al_assert(pmdt_cctm);
	al_assert(info);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	val = al_reg_read32(&regs->cctm_cfg.status);
	/* check the values are valid */
	if (!AL_REG_MASK_IS_SET(val, PMDT_CCTM_CCTM_CFG_STATUS_EVENTS_VALID))
		return -EBUSY;

	info->imp_pmu = AL_REG_FIELD_GET(val,
			PMDT_CCTM_CCTM_CFG_STATUS_EVENTS_IMPL_MASK,
			PMDT_CCTM_CCTM_CFG_STATUS_EVENTS_IMPL_SHIFT);
	info->active_pmu = AL_REG_FIELD_GET(val,
			PMDT_CCTM_CCTM_CFG_STATUS_EVENT_SUM_MASK,
			PMDT_CCTM_CCTM_CFG_STATUS_EVENT_SUM_SHIFT);

	return 0;
}

/**
 * Retrieve CCTM iofic register base address extend
 *
 * @param pmdt_cctm
 *		CCTM handle
 * @param groups_0_3
 *		In case of AL_TRUE, return ptr for groups 0..3, else, return groups 4..7 ptr.
 *
 * @return	iofic base address
 */
void __iomem *al_pmdt_cctm_iofic_base_get_ext(struct al_pmdt_cctm *pmdt_cctm, al_bool groups_0_3)
{
	struct al_pmdt_cctm_regs *regs;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	if (groups_0_3)
		return (void __iomem *)regs->interrupt_ctl0;
	else
		return (void __iomem *)regs->interrupt_ctl1;
}

/**
 * Retrieve CCTM iofic (groups 0..3) register base address
 *
 * @param pmdt_cctm
 *		CCTM handle
 *
 * @return	iofic base address
 */
void __iomem *al_pmdt_cctm_iofic_base_get(struct al_pmdt_cctm *pmdt_cctm)
{
	al_assert(pmdt_cctm);

	return al_pmdt_cctm_iofic_base_get_ext(pmdt_cctm, AL_TRUE);
}

/**
 * Configure CCTM pause delays
 */
int al_pmdt_cctm_pause_config(
		struct al_pmdt_cctm *pmdt_cctm,
		const struct al_cctm_pause_config *pause_config)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t val = 0;

	al_assert(pmdt_cctm);
	al_assert(pause_config);
	al_assert(AL_REG_FIELD_RANGE_VALID(pause_config->reset_delay_exp,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_PAUSE_EXTEND_MASK,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_PAUSE_EXTEND_SHIFT));

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	al_reg_write32(&regs->cctm_cfg.cfg_pause_delay,
			pause_config->pause_start_delay);
	AL_REG_FIELD_SET(val,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_PAUSE_EXTEND_MASK,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_PAUSE_EXTEND_SHIFT,
			pause_config->reset_delay_exp);
	AL_REG_MASK_SET_VAL(val,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_INFER_DELAY,
			pause_config->trig_delay_en);
	AL_REG_MASK_SET_VAL(val,
			PMDT_CCTM_CCTM_CFG_CFG_PAUSE_CTRL_SW_SET_DELAY,
			pause_config->sw_delay_en);
	al_reg_write32(&regs->cctm_cfg.cfg_pause_ctrl, val);

	return 0;
}

/**
 * Set central timestamp generator ratio (to all PMU units)
 */
int al_pmdt_cctm_timestamp_ratio_set(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t ratio)
{
	struct al_pmdt_cctm_regs *regs;

	al_assert(pmdt_cctm);
	al_assert(AL_REG_FIELD_RANGE_VALID(ratio,
			PMDT_CCTM_CCTM_CFG_TSTAMP_CTL0_TSTAMP_RATIO_MASK,
			PMDT_CCTM_CCTM_CFG_TSTAMP_CTL0_TSTAMP_RATIO_SHIFT));

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	al_reg_write32(&regs->cctm_cfg.tstamp_ctl0, ratio);

	return 0;
}

/**
 * Set timestamp initial value
 */
int al_pmdt_cctm_initial_timestamp_set(
		struct al_pmdt_cctm *pmdt_cctm,
		uint64_t cctm_ts_init_val)
{
	struct al_pmdt_cctm_regs *regs;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	/* set local time stamp counter initial value */
	al_hal_pmdt_write64(cctm_ts_init_val,
			&regs->cctm_cfg.tstamp_offset_low,
			&regs->cctm_cfg.tstamp_offset_high);

	al_pmdt_cctm_timestamp_clear(pmdt_cctm);

	return 0;
}

/**
 * Enable/disable timestamp generation
 */
void al_pmdt_cctm_timestamp_enable(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_enable)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t val;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	val = al_reg_read32(&regs->cctm_cfg.tstamp_ctl1);
	AL_REG_MASK_SET_VAL(val,
			PMDT_CCTM_CCTM_CFG_TSTAMP_CTL1_TSTAMP_EN,
			is_enable);
	al_reg_write32(&regs->cctm_cfg.tstamp_ctl1, val);
}

/**
 * Pause/resume timestamp generation to all PMU units
 */
void al_pmdt_cctm_timestamp_pause(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_pause)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t val;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	val = al_reg_read32(&regs->cctm_cfg.tstamp_ctl1);
	AL_REG_MASK_SET_VAL(val,
			PMDT_CCTM_CCTM_CFG_TSTAMP_CTL1_TSTAMP_PAUSE,
			is_pause);
	al_reg_write32(&regs->cctm_cfg.tstamp_ctl1, val);
}

/**
 * Clear timestamp in all PMU units
 */
void al_pmdt_cctm_timestamp_clear(struct al_pmdt_cctm *pmdt_cctm)
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t val;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;

	val = al_reg_read32(&regs->cctm_cfg.tstamp_ctl1);
	AL_REG_MASK_SET(val,
			PMDT_CCTM_CCTM_CFG_TSTAMP_CTL1_TSTAMP_CLEAR);
	al_reg_write32(&regs->cctm_cfg.tstamp_ctl1, val);
}

/**
 * Retrieve current CCTM time stamp
 */
uint64_t al_pmdt_cctm_timestamp_get(struct al_pmdt_cctm *pmdt_cctm)
{
	struct al_pmdt_cctm_regs *regs;
	uint64_t ts;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	ts = al_hal_pmdt_read64_counter(&regs->cctm_cfg.tstamp_low,
			&regs->cctm_cfg.tstamp_high);

	return ts;
}

void al_pmdt_cctm_pmu_enable_mask(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_masked)
{
	struct al_pmdt_cctm_regs *regs;

	al_assert(pmdt_cctm);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	al_reg_write32_masked(
		&regs->cctm_cfg.en_unmask,
		PMDT_CCTM_CCTM_CFG_EN_UNMASK_VAL_MASK,
		is_masked ? 0 : PMDT_CCTM_CCTM_CFG_EN_UNMASK_VAL_MASK);
}

/**
 * Enable/disable underlying PMU devices
 */
void al_pmdt_cctm_pmu_enable(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_enable,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.event_en_0;

	set_cctm_mask(pmdt_cctm, addr_base, is_enable, AL_PMDT_CCTM_MASK_OP_RMW, *pmu_mask, AL_FALSE);
}

/**
 * Pause/resume underlying PMU devices
 */
void al_pmdt_cctm_pmu_pause(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_pause,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.pause_en_0;

	set_cctm_mask(pmdt_cctm, addr_base, is_pause, AL_PMDT_CCTM_MASK_OP_RMW, *pmu_mask, AL_FALSE);
}

/**
 * Clear underlying PMU devices
 */
void al_pmdt_cctm_pmu_clear(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.clear_en_0;

	set_cctm_mask(pmdt_cctm, addr_base, AL_TRUE, AL_PMDT_CCTM_MASK_OP_WO, *pmu_mask, AL_FALSE);
}

/**
 * Enable/disable underlying PMU GPIO trigger
 */
void al_pmdt_cctm_gpio_enable(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_enable,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.gpio_en_0;

	set_cctm_mask(pmdt_cctm, addr_base, is_enable, AL_PMDT_CCTM_MASK_OP_RMW, *pmu_mask, AL_FALSE);
}

/**
 * Force/clear SW trigger to underlying PMU devices
 */
void al_pmdt_cctm_trig_force(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_force,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.sw_set_0;

	set_cctm_mask(pmdt_cctm, addr_base, is_force, AL_PMDT_CCTM_MASK_OP_RMW, *pmu_mask, AL_FALSE);
}

/**
 * Get active PMU triggers snapshot
 */
void al_pmdt_cctm_trig_get(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.triggers_0;

	set_cctm_mask(pmdt_cctm, addr_base, 0, AL_PMDT_CCTM_MASK_OP_RO, *pmu_mask, AL_FALSE);
}

/**
 * Get active PMU GPIO trigger snapshot
 */
void al_pmdt_cctm_gpio_get(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.gpio_0;

	set_cctm_mask(pmdt_cctm, addr_base, 0, AL_PMDT_CCTM_MASK_OP_RO, *pmu_mask, AL_FALSE);
}

/**
 * Get paused PMU snapshot
 */
void al_pmdt_cctm_paused_get(
		struct al_pmdt_cctm *pmdt_cctm,
		uint32_t (*pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(pmu_mask);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cctm_cfg.pause_0;

	set_cctm_mask(pmdt_cctm, addr_base, 0, AL_PMDT_CCTM_MASK_OP_RO, *pmu_mask, AL_FALSE);
}

/**
 * Enable/disable source-PMU to pause dest-PMUs by firing an event trigger
 */
void al_pmdt_cctm_cross_pause_enable(
		struct al_pmdt_cctm *pmdt_cctm,
		al_bool is_enable,
		uint32_t dst_pmu,
		uint32_t (*src_pmu_mask)[AL_PMDT_CCTM_MATRIX_LINE_SIZE_MAX])
{
	struct al_pmdt_cctm_regs *regs;
	uint32_t *addr_base;

	al_assert(pmdt_cctm);
	al_assert(src_pmu_mask);
	al_assert(dst_pmu < 32*pmdt_cctm->line_size);

	regs = (struct al_pmdt_cctm_regs *)pmdt_cctm->regs;
	addr_base = &regs->cross_matrix[dst_pmu].infer_pause_0;

	set_cctm_mask(pmdt_cctm, addr_base, is_enable, AL_PMDT_CCTM_MASK_OP_RMW,
			*src_pmu_mask, AL_TRUE);
}

/** @} end of PMDT CCTM group */
