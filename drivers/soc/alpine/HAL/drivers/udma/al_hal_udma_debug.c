/*******************************************************************************
Copyright (C) 2015 Annapurna Labs Ltd.

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
 * @file   al_hal_udma_debug.c
 *
 * @brief  Universal DMA HAL driver for debug
 *
 */

#define DEBUG

#include <al_hal_common.h>
#include <al_hal_udma_regs.h>
#include <al_hal_udma_debug.h>

/* Assuming an average packet size of 256B, and taking into consideration that for ethernet case
 * there is an internal FIFO in EC of size ~10KB, this means approx 40 packets can be held before
 * receiving completion acks. Assuming 1 desc/packet, and aligning to power of 2 we print 64 descs
 */
#define AL_UDMA_RING_END_PRE_MAX_NUM_OF_DESCS 64

/* Consider two pointers : PTR A and PTR B within a queue, where PTR A points to the Consumer and Ptr B points to the
 * Producer for the queue. i.e  PTR A is always supposed to chase PTR B.
 *
 * The two correct working scenarios are detailed as follows.
 * Condition 1 : Ring ids match
 *       Ring ID
 *        | Desc ID
 *        |   |
 *        V   V
 *       |-------------------------|
 *       |00|0000|                 |
 *       |00|0001|                 |<--PTR A (eg: HP)
 *       |00|0010|                 |
 *       |00|0011|                 |
 *       |00|0100|                 |<- PTR B (eg: TP)
 *       |00|0101|                 |
 *       |00|0110|                 |
 *       |.....  |                 |
 *       |.....  |                 |
 *       |00|1111|                 |
 *       |-------------------------|
 *
 * if Ptr A > Ptr B : Then the consumer has gone beyond the producer and is consuming garbage resulting in error.
 *
 * Condition 2 : Ring ids dont match
 *       Ring ID
 *        | Desc ID
 *        |   |
 *        V   V
 *       |------------------------|
 *       |01|0000|                |
 *       |01|0001|                |<--PTR B (eg: TP)
 *       |00|0010|                |
 *       |00|0011|                |
 *       |00|0100|                |<- PTR A (eg: HP)
 *       |00|0101|                |
 *       |00|0110|                |
 *       |.....  |                |
 *       |.....  |                |
 *       |00|1111|                |
 *       |------------------------|
 *
 * if Ptr B > Ptr A : Then the producer has overwritten the descriptors before the consumer was able to get to it.
 * Resulting in error.
 */
#define Q_PTR_A_Q_PTR_B_OVERRUN_OCCURRED(ptr_a_offset, ptr_b_offset, same_ring_id)	\
	(((same_ring_id) && ((ptr_a_offset) > (ptr_b_offset))) ||	\
	((!(same_ring_id)) && ((ptr_b_offset) > (ptr_a_offset))))

static void al_udma_regs_m2s_axi_print(struct al_udma *udma)
{
	al_udma_print("M2S AXI regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, comp_wr_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, comp_wr_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, data_rd_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, data_rd_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, desc_rd_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, desc_rd_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, data_rd_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, desc_rd_cfg_3);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, desc_wr_cfg_1);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, axi_m2s,
			desc_wr_cfg_1,
			max_axi_beats,
			UDMA_AXI_M2S_DESC_WR_CFG_1_MAX_AXI_BEATS);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, axi_m2s,
			desc_wr_cfg_1,
			min_axi_beats,
			UDMA_AXI_M2S_DESC_WR_CFG_1_MIN_AXI_BEATS);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, axi_m2s, ostand_cfg);
}

static void al_udma_regs_m2s_general_print(struct al_udma *udma)
{
	al_udma_print("M2S general regs:\n");

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, state);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s, state,
			comp_ctrl,
			UDMA_M2S_STATE_COMP_CTRL);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s, state,
			stream_if,
			UDMA_M2S_STATE_STREAM_IF);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s, state,
			rd_ctrl,
			UDMA_M2S_STATE_DATA_RD_CTRL);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s, state,
			desc_pref,
			UDMA_M2S_STATE_DESC_PREF);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, err_log_mask);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, log_0);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, log_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, log_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, log_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, data_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, header_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, unack_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, check_en);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, fifo_en);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, cfg_len);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, stream_cfg);
}

static void al_udma_regs_m2s_rd_print(struct al_udma *udma)
{
	al_udma_print("M2S read regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_rd, desc_pref_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_rd, desc_pref_cfg_2);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_rd, desc_pref_cfg_3);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_rd,
			desc_pref_cfg_3,
			min_burst_below_thr,
			UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_BELOW_THR);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_rd,
			desc_pref_cfg_3,
			min_burst_above_thr,
			UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_ABOVE_THR);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_rd,
			desc_pref_cfg_3,
			pref_thr,
			UDMA_M2S_RD_DESC_PREF_CFG_3_PREF_THR);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_rd, data_cfg);
}

static void al_udma_regs_m2s_dwrr_print(struct al_udma *udma)
{
	al_udma_print("M2S DWRR regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_dwrr, cfg_sched);
}

static void al_udma_regs_m2s_rate_limiter_print(struct al_udma *udma)
{
	al_udma_print("M2S rate limiter regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_rate_limiter, gen_cfg);
}

static void al_udma_regs_m2s_stream_rate_limiter_print(struct al_udma *udma)
{
	al_udma_print("M2S stream rate limiter regs:\n");

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stream_rate_limiter,
			cfg_1s);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stream_rate_limiter,
			cfg_cycle);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stream_rate_limiter,
			cfg_token_size_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stream_rate_limiter,
			cfg_token_size_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stream_rate_limiter,
			mask);
}

static void al_udma_regs_m2s_comp_print(struct al_udma *udma)
{
	al_udma_print("M2S completion regs:\n");

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_comp, cfg_1c);

	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_comp, cfg_1c,
			comp_fifo_depth,
			UDMA_M2S_COMP_CFG_1C_COMP_FIFO_DEPTH);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_comp, cfg_1c,
			unack_fifo_depth,
			UDMA_M2S_COMP_CFG_1C_UNACK_FIFO_DEPTH);
	AL_UDMA_PRINT_REG_BIT(udma, "  ", "\n", m2s, m2s_comp, cfg_1c,
			q_promotion,
			UDMA_M2S_COMP_CFG_1C_Q_PROMOTION);
	AL_UDMA_PRINT_REG_BIT(udma, "  ", "\n", m2s, m2s_comp, cfg_1c,
			force_rr,
			UDMA_M2S_COMP_CFG_1C_FORCE_RR);
	AL_UDMA_PRINT_REG_FIELD(udma, "  ", "\n", "%d", m2s, m2s_comp, cfg_1c,
			q_free_min,
			UDMA_M2S_COMP_CFG_1C_Q_FREE_MIN);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_comp, cfg_coal);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_comp, cfg_application_ack);
}

static void al_udma_regs_m2s_stat_print(struct al_udma *udma)
{
	al_udma_print("M2S statistics regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, cfg_st);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, tx_pkt);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, tx_bytes_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, tx_bytes_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, prefed_desc);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, comp_pkt);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, comp_desc);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_stat, ack_pkts);
}

static void al_udma_regs_m2s_feature_print(struct al_udma *udma)
{
	al_udma_print("M2S feature regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_feature, reg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_feature, reg_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_feature, reg_4);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_feature, reg_5);
}

static void al_udma_regs_m2s_q_print(struct al_udma *udma, uint32_t qid)
{
	al_udma_print("M2S Q[%d] status regs:\n", qid);
	al_reg_write32(&udma->udma_regs->m2s.m2s.indirect_ctrl, qid);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, sel_pref_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, sel_comp_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, sel_rate_limit_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s, sel_dwrr_status);

	al_udma_print("M2S Q[%d] regs:\n", qid);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdrbp_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdrbp_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdrl);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdrhp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdrtp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tdcp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tcrbp_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tcrbp_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], tcrhp);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], rate_limit_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], rate_limit_cfg_cycle);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid],
			rate_limit_cfg_token_size_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid],
			rate_limit_cfg_token_size_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], rate_limit_mask);

	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], dwrr_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], dwrr_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], dwrr_cfg_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], comp_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", m2s, m2s_q[qid], q_tx_pkt);
}

static void al_udma_regs_s2m_axi_print(struct al_udma *udma)
{
	al_udma_print("S2M AXI regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, data_wr_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, data_wr_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, desc_rd_cfg_4);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, desc_rd_cfg_5);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, comp_wr_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, comp_wr_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, data_wr_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, desc_rd_cfg_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, desc_wr_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, ostand_cfg_rd);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, axi_s2m, ostand_cfg_wr);
}

static void al_udma_regs_s2m_general_print(struct al_udma *udma)
{
	al_udma_print("S2M general regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, state);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, err_log_mask);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, log_0);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, log_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, log_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, log_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, s_data_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, s_header_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, axi_data_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, unack_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, check_en);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, fifo_en);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, stream_cfg);
}

static void al_udma_regs_s2m_rd_print(struct al_udma *udma)
{
	al_udma_print("S2M read regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_rd, desc_pref_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_rd, desc_pref_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_rd, desc_pref_cfg_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_rd, desc_pref_cfg_4);
}

static void al_udma_regs_s2m_wr_print(struct al_udma *udma)
{
	al_udma_print("S2M write regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_wr, data_cfg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_wr, data_cfg_2);
}

static void al_udma_regs_s2m_comp_print(struct al_udma *udma)
{
	al_udma_print("S2M completion regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_comp, cfg_1c);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_comp, cfg_2c);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_comp, cfg_application_ack);
}

static void al_udma_regs_s2m_stat_print(struct al_udma *udma)
{
	al_udma_print("S2M statistics regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, drop_pkt);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, rx_bytes_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, rx_bytes_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, prefed_desc);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, comp_pkt);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, comp_desc);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_stat, ack_pkts);
}

static void al_udma_regs_s2m_feature_print(struct al_udma *udma)
{
	al_udma_print("S2M feature regs:\n");
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_feature, reg_1);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_feature, reg_3);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_feature, reg_4);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_feature, reg_5);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_feature, reg_6);
}

static void al_udma_regs_s2m_q_print(struct al_udma *udma, uint32_t qid)
{
	al_udma_print("S2M Q[%d] status regs:\n", qid);
	al_reg_write32(&udma->udma_regs->s2m.s2m.indirect_ctrl, qid);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, sel_pref_fifo_status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m, sel_comp_fifo_status);

	al_dbg("S2M Q[%d] regs:\n", qid);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], status);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdrbp_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdrbp_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdrl);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdrhp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdrtp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rdcp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rcrbp_low);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rcrbp_high);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rcrhp);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], rcrhp_internal);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], comp_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], comp_cfg_2);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], pkt_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], qos_cfg);
	AL_UDMA_PRINT_REG(udma, " ", "\n", s2m, s2m_q[qid], q_rx_pkt);
}

void al_udma_regs_print(struct al_udma *udma, unsigned int mask)
{
	uint32_t i;

	if (!udma)
		return;

	if (udma->type == UDMA_TX) {
		if (mask & AL_UDMA_DEBUG_AXI)
			al_udma_regs_m2s_axi_print(udma);
		if (mask & AL_UDMA_DEBUG_GENERAL)
			al_udma_regs_m2s_general_print(udma);
		if (mask & AL_UDMA_DEBUG_READ)
			al_udma_regs_m2s_rd_print(udma);
		if (mask & AL_UDMA_DEBUG_DWRR)
			al_udma_regs_m2s_dwrr_print(udma);
		if (mask & AL_UDMA_DEBUG_RATE_LIMITER)
			al_udma_regs_m2s_rate_limiter_print(udma);
		if (mask & AL_UDMA_DEBUG_STREAM_RATE_LIMITER)
			al_udma_regs_m2s_stream_rate_limiter_print(udma);
		if (mask & AL_UDMA_DEBUG_COMP)
			al_udma_regs_m2s_comp_print(udma);
		if (mask & AL_UDMA_DEBUG_STAT)
			al_udma_regs_m2s_stat_print(udma);
		if (mask & AL_UDMA_DEBUG_FEATURE)
			al_udma_regs_m2s_feature_print(udma);
		for (i = 0; i < udma->num_of_queues; i++) {
			if (mask & AL_UDMA_DEBUG_QUEUE(i))
				al_udma_regs_m2s_q_print(udma, i);
		}
	} else {
		if (mask & AL_UDMA_DEBUG_AXI)
			al_udma_regs_s2m_axi_print(udma);
		if (mask & AL_UDMA_DEBUG_GENERAL)
			al_udma_regs_s2m_general_print(udma);
		if (mask & AL_UDMA_DEBUG_READ)
			al_udma_regs_s2m_rd_print(udma);
		if (mask & AL_UDMA_DEBUG_WRITE)
			al_udma_regs_s2m_wr_print(udma);
		if (mask & AL_UDMA_DEBUG_COMP)
			al_udma_regs_s2m_comp_print(udma);
		if (mask & AL_UDMA_DEBUG_STAT)
			al_udma_regs_s2m_stat_print(udma);
		if (mask & AL_UDMA_DEBUG_FEATURE)
			al_udma_regs_s2m_feature_print(udma);
		for (i = 0; i < udma->num_of_queues; i++) {
			if (mask & AL_UDMA_DEBUG_QUEUE(i))
				al_udma_regs_s2m_q_print(udma, i);
		}
	}
}

void al_udma_q_struct_print(struct al_udma *udma, uint32_t qid)
{
	struct al_udma_q *queue;

	if (!udma)
		return;

	if (qid >= udma->num_of_queues)
		return;

	queue = &udma->udma_q[qid];

	al_udma_print("Q[%d] struct:\n", qid);
	al_udma_print(" size_mask = 0x%08x\n", queue->size_mask);
	al_udma_print(" q_regs = %p\n", queue->q_regs);
	al_udma_print(" q_dp_regs = %p\n", queue->q_dp_regs);
	al_udma_print(" desc_base_ptr = %p\n", queue->desc_base_ptr);
	al_udma_print(" next_desc_idx = %d\n", queue->next_desc_idx);
	al_udma_print(" desc_ring_id = %d\n", queue->desc_ring_id);
	al_udma_print(" cdesc_base_ptr = %p\n", queue->cdesc_base_ptr);
	al_udma_print(" cdesc_size = %d\n", queue->cdesc_size);
	al_udma_print(" next_cdesc_idx = %d\n", queue->next_cdesc_idx);
	al_udma_print(" end_cdesc_ptr = %p\n", queue->end_cdesc_ptr);
	al_udma_print(" comp_head_idx = %d\n", queue->comp_head_idx);
	al_udma_print(" comp_head_ptr = %p\n", queue->comp_head_ptr);
	al_udma_print(" pkt_crnt_descs = %d\n", queue->pkt_crnt_descs);
	al_udma_print(" comp_ring_id = %d\n", queue->comp_ring_id);
	al_udma_print(" desc_phy_base = 0x%016" PRIx64 "\n", (uint64_t)queue->desc_phy_base);
	al_udma_print(" cdesc_phy_base = 0x%016" PRIx64 "\n",
			(uint64_t)queue->cdesc_phy_base);
	al_udma_print(" flags = 0x%08x\n", queue->flags);
	al_udma_print(" size = %d\n", queue->size);
	al_udma_print(" status = %d\n", queue->status);
	al_udma_print(" udma = %p\n", queue->udma);
	al_udma_print(" qid = %d\n", queue->qid);
}

void al_udma_ring_print(struct al_udma *udma, uint32_t qid,
		enum al_udma_ring_type rtype)
{
	struct al_udma_q *queue;
	uint32_t desc_size;
	uint8_t *base_ptr;
	uint32_t i;

	if (!udma)
		return;

	if (qid >= udma->num_of_queues)
		return;

	queue = &udma->udma_q[qid];
	if (rtype == AL_RING_SUBMISSION) {
		base_ptr = (uint8_t *)queue->desc_base_ptr;
		desc_size = sizeof(union al_udma_desc);
		if (base_ptr)
			al_udma_print("Q[%d] submission ring pointers:\n", qid);
		else {
			al_udma_print("Q[%d] submission ring is not allocated\n", qid);
			return;
		}
	} else {
		base_ptr = (uint8_t *)queue->cdesc_base_ptr;
		desc_size = queue->cdesc_size;
		if (base_ptr)
			al_udma_print("Q[%d] completion ring pointers:\n", qid);
		else {
			al_udma_print("Q[%d] completion ring is not allocated\n", qid);
			return;
		}
	}

	for (i = 0; i < queue->size; i++) {
		uint32_t *curr_addr = (uint32_t *)(base_ptr + i * desc_size);
		if (desc_size == 16)
			al_udma_print("[%04d](%p): %08x %08x %08x %08x\n",
					i,
					curr_addr,
					(uint32_t)*curr_addr,
					(uint32_t)*(curr_addr+1),
					(uint32_t)*(curr_addr+2),
					(uint32_t)*(curr_addr+3));
		else if (desc_size == 8)
			al_udma_print("[%04d](%p): %08x %08x\n",
					i,
					curr_addr,
					(uint32_t)*curr_addr,
					(uint32_t)*(curr_addr+1));
		else if (desc_size == 4)
			al_udma_print("[%04d](%p): %08x\n",
					i,
					curr_addr,
					(uint32_t)*curr_addr);
		else
			break;
	}
}

void al_udma_ring_descs_around_comp_print(struct al_udma *udma, uint32_t qid,
					  enum al_udma_ring_type rtype,
					  unsigned int num_of_descs_pre_comp_head)
{
	struct al_udma_q *queue;
	uint32_t desc_size;
	uint8_t *base_ptr;
	uint32_t i;
	uint32_t crhp;
	uint32_t drtp;
	unsigned int pre_next_to_complete_num;
	unsigned int pre_next_to_complete_idx;
	unsigned int num_of_descs_to_print;
	unsigned int end_idx;

	al_assert(udma);
	al_assert(qid < udma->num_of_queues);

	queue = &udma->udma_q[qid];
	if (rtype == AL_RING_SUBMISSION) {
		base_ptr = (uint8_t *)queue->desc_base_ptr;
		desc_size = sizeof(union al_udma_desc);
		if (base_ptr)
			al_info("Q[%d] submission ring around comp head pointers:\n", qid);
		else {
			al_warn("Q[%d] submission ring is not allocated\n", qid);
			return;
		}
	} else {
		base_ptr = (uint8_t *)queue->cdesc_base_ptr;
		desc_size = queue->cdesc_size;
		if (base_ptr)
			al_info("Q[%d] completion ring pre comp header pointers:\n", qid);
		else {
			al_warn("Q[%d] completion ring is not allocated\n", qid);
			return;
		}
	}

	al_info("desc_size: %d\n", desc_size);
	al_assert((desc_size == 16) || (desc_size == 8) || (desc_size == 4));

	crhp = al_reg_read32(&queue->q_regs->rings.crhp);
	pre_next_to_complete_num = al_min_t(typeof(queue->size - 1), queue->size - 1,
					    num_of_descs_pre_comp_head);
	pre_next_to_complete_idx = (crhp - pre_next_to_complete_num)
				   & queue->size_mask;

	if (rtype == AL_RING_SUBMISSION) {
		drtp = al_reg_read32(&queue->q_regs->rings.drhp);
		num_of_descs_to_print = (drtp - pre_next_to_complete_idx) & queue->size_mask;
		al_info("drtp: 0x%08x\n", drtp);
	} else {
		num_of_descs_to_print = pre_next_to_complete_num + 1;
	}

	al_info("crhp: 0x%08x\n", crhp);
	al_info("ring_size: 0x%08x\n", queue->size);

	end_idx = pre_next_to_complete_idx + num_of_descs_to_print;
	for (i = pre_next_to_complete_idx; i <= end_idx; i++) {
		uint32_t *curr_addr = (uint32_t *)(base_ptr + (i & queue->size_mask) * desc_size);

		if (desc_size == 16)
			al_info("[%04d](%p): %08x %08x %08x %08x\n",
					(i & queue->size_mask),
					curr_addr,
					(uint32_t)*curr_addr,
					(uint32_t)*(curr_addr+1),
					(uint32_t)*(curr_addr+2),
					(uint32_t)*(curr_addr+3));
		else if (desc_size == 8)
			al_info("[%04d](%p): %08x %08x\n",
					(i & queue->size_mask),
					curr_addr,
					(uint32_t)*curr_addr,
					(uint32_t)*(curr_addr+1));
		else if (desc_size == 4)
			al_info("[%04d](%p): %08x\n",
					(i & queue->size_mask),
					curr_addr,
					(uint32_t)*curr_addr);
	}
}

struct al_udma_reg_mask_and_shift {
	uint32_t drhp_ring_id_mask;
	uint32_t drtp_ring_id_mask;
	uint32_t dcp_ring_id_mask;
	uint32_t drhp_offset_mask;
	uint32_t drtp_offset_mask;
	uint32_t dcp_offset_mask;
	uint32_t drhp_ring_id_shift;
	uint32_t drtp_ring_id_shift;
	uint32_t dcp_ring_id_shift;
};

enum al_udma_q_ptr_type {
	AL_UDMA_Q_HEAD,
	AL_UDMA_Q_TAIL,
	AL_UDMA_Q_CURRENT,
	AL_UDMA_Q_BP_LO,
	AL_UDMA_Q_BP_HI,
	AL_UDMA_Q_LENGTH
};

static const struct al_udma_reg_mask_and_shift udma_field_mask_and_shift_const[] = {
{
	.drhp_ring_id_mask = UDMA_M2S_Q_TDRHP_RING_ID_MASK,
	.drtp_ring_id_mask = UDMA_M2S_Q_TDRTP_RING_ID_MASK,
	.dcp_ring_id_mask = UDMA_M2S_Q_TDCP_RING_ID_MASK,
	.drhp_offset_mask =  UDMA_M2S_Q_TDRHP_OFFSET_MASK,
	.drtp_offset_mask =  UDMA_M2S_Q_TDRTP_OFFSET_MASK,
	.dcp_offset_mask =  UDMA_M2S_Q_TDCP_OFFSET_MASK,
	.drhp_ring_id_shift = UDMA_M2S_Q_TDRHP_RING_ID_SHIFT,
	.drtp_ring_id_shift = UDMA_M2S_Q_TDRTP_RING_ID_SHIFT,
	.dcp_ring_id_shift = UDMA_M2S_Q_TDCP_RING_ID_SHIFT
},
{
	.drhp_ring_id_mask = UDMA_S2M_Q_RDRHP_RING_ID_MASK,
	.drtp_ring_id_mask = UDMA_S2M_Q_RDRTP_RING_ID_MASK,
	.dcp_ring_id_mask = UDMA_S2M_Q_RDCP_RING_ID_MASK,
	.drhp_offset_mask =  UDMA_S2M_Q_RDRHP_OFFSET_MASK,
	.drtp_offset_mask =  UDMA_S2M_Q_RDRTP_OFFSET_MASK,
	.dcp_offset_mask =  UDMA_S2M_Q_RDCP_OFFSET_MASK,
	.drhp_ring_id_shift = UDMA_S2M_Q_RDRHP_RING_ID_SHIFT,
	.drtp_ring_id_shift = UDMA_S2M_Q_RDRTP_RING_ID_SHIFT,
	.dcp_ring_id_shift = UDMA_S2M_Q_RDCP_RING_ID_SHIFT
}
};

static INLINE const char *al_udma_get_q_ptr_convert_to_str(enum al_udma_q_ptr_type q_ptr_type)
{
	switch (q_ptr_type) {
	case AL_UDMA_Q_HEAD:
		return "HEAD";
	case AL_UDMA_Q_TAIL:
		return "TAIL";
	case AL_UDMA_Q_CURRENT:
		return "CURRENT";
	case AL_UDMA_Q_BP_LO:
		return "BASE POINTER LO";
	case AL_UDMA_Q_BP_HI:
		return "BASE POINTER HIGH";
	case AL_UDMA_Q_LENGTH:
		return "LENGTH";
	default:
		return "INCORRECT";
	}
}


#define AL_UDMA_Q_PTR_REG_VAL_UNINITIALIZED	0xffffffff

static void al_udma_q_ptr_val_print(uint32_t q_ptr_val,
					   enum al_udma_q_ptr_type q_ptr_type,
					   enum al_udma_ring_type ring_type)
{
	const char * const submission_completion = (ring_type == AL_RING_COMPLETION) ?
					"COMPLETION" : "SUBMISSION";

	if (q_ptr_val != AL_UDMA_Q_PTR_REG_VAL_UNINITIALIZED)
		al_info("\t%s %s POINTER value : 0x%08x\n",
			submission_completion, al_udma_get_q_ptr_convert_to_str(q_ptr_type), (uint32_t)(q_ptr_val));
	else
		al_err("\t%s %s POINTER : uninitialized\n",
			submission_completion, al_udma_get_q_ptr_convert_to_str(q_ptr_type));
}

void al_udma_q_ptrs_print_and_validate(struct al_udma *udma, uint32_t qid)
{
	uint32_t drhp_ring_id;
	uint32_t drhp_offset;
	uint32_t drtp_ring_id;
	uint32_t drtp_offset;
	uint32_t drcp_ring_id;
	uint32_t drcp_offset;
	uint32_t drhp;
	uint32_t drtp;
	uint32_t drcp;
	uint32_t cdrhp;
	struct udma_rings_regs *udma_ring_reg_handle;
	const struct al_udma_reg_mask_and_shift *udma_field_mask_and_shift;

	al_assert(udma);
	al_assert(qid < udma->num_of_queues);

	/* Read cfg register to check if queue is enabled */
	if (!al_udma_q_is_enabled(&udma->udma_q[qid])) {
		al_info("QUEUE DISABLED for QID %d\n", qid);
		return;
	}

	al_info("QUEUE POINTERS for QID : %d\n", qid);
	if (udma->type == UDMA_TX) {
		udma_ring_reg_handle = (struct udma_rings_regs *)&(udma->udma_regs->m2s.m2s_q[qid]);
		udma_field_mask_and_shift = &udma_field_mask_and_shift_const[UDMA_TX];
	} else {
		udma_ring_reg_handle = (struct udma_rings_regs *)&(udma->udma_regs->s2m.s2m_q[qid]);
		udma_field_mask_and_shift = &udma_field_mask_and_shift_const[UDMA_RX];
	}

	drhp = al_reg_read32(&(udma_ring_reg_handle->drhp));
	al_udma_q_ptr_val_print(drhp, AL_UDMA_Q_HEAD, AL_RING_SUBMISSION);
	drtp = al_reg_read32(&(udma_ring_reg_handle->drtp));
	al_udma_q_ptr_val_print(drtp, AL_UDMA_Q_TAIL, AL_RING_SUBMISSION);
	drcp = al_reg_read32(&(udma_ring_reg_handle->dcp));
	al_udma_q_ptr_val_print(drcp, AL_UDMA_Q_CURRENT, AL_RING_SUBMISSION);
	cdrhp = al_reg_read32(&(udma_ring_reg_handle->crhp));
	al_udma_q_ptr_val_print(cdrhp, AL_UDMA_Q_HEAD, AL_RING_COMPLETION);

	drhp_ring_id = AL_REG_FIELD_GET(drhp,
					udma_field_mask_and_shift->drhp_ring_id_mask,
					udma_field_mask_and_shift->drhp_ring_id_shift);
	drtp_ring_id = AL_REG_FIELD_GET(drtp,
					udma_field_mask_and_shift->drtp_ring_id_mask,
					udma_field_mask_and_shift->drtp_ring_id_shift);
	drcp_ring_id = AL_REG_FIELD_GET(drcp,
					udma_field_mask_and_shift->dcp_ring_id_mask,
					udma_field_mask_and_shift->dcp_ring_id_shift);
	drhp_offset = (drhp & udma_field_mask_and_shift->drhp_offset_mask);
	drtp_offset = (drtp & udma_field_mask_and_shift->drtp_offset_mask);
	drcp_offset = (drcp & udma_field_mask_and_shift->dcp_offset_mask);

	/*  logically tp should always be >=  hp */
	if (Q_PTR_A_Q_PTR_B_OVERRUN_OCCURRED(drhp_offset,
					     drtp_offset,
					     (drhp_ring_id == drtp_ring_id)))
		al_err("tail pointer and head pointer overrun detected\n");

	/* logically cp should always be <= hp */
	if (Q_PTR_A_Q_PTR_B_OVERRUN_OCCURRED(drcp_offset,
					     drhp_offset,
					     (drcp_ring_id == drhp_ring_id)))
		al_err("current pointer and head pointer overrun detected\n");
}
