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
 *  @{
 * @file   al_hal_udma_main.c
 *
 * @brief  Universal DMA HAL driver for main functions (initialization, data path)
 *
 */

#include <asm/div64.h>
#include <al_hal_udma.h>
#include <al_hal_udma_config.h>
#include <al_hal_udma_iofic.h>
#include <al_hal_udma_push_mode.h>
#ifdef AL_ETH_EX
#include <al_hal_udma_config_ex.h>
#endif

/*
 * how many nSec pass for 1000 clock ticks.
 * this clock is coming from the SB block.
 */
#define AL_UDMA_REFLECTION_NSEC_PER_K_TICKS 1300

/* what is the size of the tx/rx prefetch fifos in descriptors */
#define AL_UDMA_TX_PREFETCH_FIFO_DEPTH 512 /* 0x200 */
#define AL_UDMA_RX_PREFETCH_FIFO_DEPTH 128 /* 0x80 */

#define AL_UDMA_Q_QUIESCE_TOUT	10000	/* Queue quiesce timeout [uSecs] */

#define UDMA_STATE_IDLE		0x0
#define UDMA_STATE_NORMAL	0x1
#define UDMA_STATE_ABORT	0x2
#define UDMA_STATE_RESERVED	0x3

const char *const al_udma_states_name[] = {
	"Disable",
	"Idle",
	"Normal",
	"Abort",
	"Reset"
};

#define AL_UDMA_INITIAL_RING_ID	1

/* Shadow access permissions */
#define AL_UDMA_RX_ACCESS_CTRL_REGS_LIMIT \
	((sizeof(struct udma_s2m_shadow_access) * 8) / 2)
#define AL_UDMA_TX_ACCESS_CTRL_REGS_LIMIT \
	((sizeof(struct udma_m2s_shadow_access) * 8) / 2)
#define AL_UDMA_INTC_ACCESS_CTRL_REGS_LIMIT \
	((sizeof(struct udma_gen_intc_shadow_access) * 8) / 2)

enum udma_shadow_permission_dir {
	PERMISSION_DIR_RD,
	PERMISSION_DIR_WR
};

/* rev 4 and rev 6 have the same max number of queues */
al_static_assert(DMA_MAX_Q_V4 == DMA_MAX_Q_V6, "mismatch maximal number or queues");

static void al_udma_set_defaults(struct al_udma *udma)
{
	uint8_t rev_id = udma->rev_id;

	if (udma->type == UDMA_TX) {
		struct al_udma_m2s_pkt_len_conf conf = {
			.encode_64k_as_zero = AL_TRUE,
			.max_pkt_size = UDMA_M2S_CFG_LEN_MAX_PKT_SIZE_MASK,
		};

		/*
		 * UDMA V4:
		 * Setting the data fifo depth to 32K (1024 beats of 256 bits)
		 * This allows the UDMA to have 128 outstanding writes
		 *
		 * UDMA V2:
		 * Setting the data fifo depth to 4K (256 beats of 128 bits)
		 * This allows the UDMA to have 16 outstanding writes
		 */
		if (rev_id >= AL_UDMA_REV_ID_2) {
			unsigned int num_beats = (rev_id >= AL_UDMA_REV_ID_4) ? 1024 : 256;

			al_reg_write32_masked(&udma->udma_regs->m2s.m2s_rd.data_cfg,
			      UDMA_M2S_RD_DATA_CFG_DATA_FIFO_DEPTH_MASK,
			      num_beats << UDMA_M2S_RD_DATA_CFG_DATA_FIFO_DEPTH_SHIFT);
		}

		/* Set M2S max number of outstanding transactions */
		if (rev_id >= AL_UDMA_REV_ID_4) {
			al_reg_write32_masked(&udma->udma_regs->m2s.axi_m2s.ostand_cfg,
				UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_MASK |
				UDMA_AXI_M2S_OSTAND_CFG_MAX_DESC_RD_MASK |
				UDMA_AXI_M2S_OSTAND_CFG_MAX_COMP_REQ_MASK,
				(128 << UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_SHIFT) |
				(128 << UDMA_AXI_M2S_OSTAND_CFG_MAX_DESC_RD_SHIFT) |
				(128 << UDMA_AXI_M2S_OSTAND_CFG_MAX_COMP_REQ_SHIFT));
		}

		/* set AXI timeout to 5M (~10 ms) */
		al_reg_write32(&udma->gen_axi_regs->cfg_1, 5000000);

		/* Ack time out */
		al_reg_write32(&udma->udma_regs->m2s.m2s_comp.cfg_application_ack, 0);

		/* set max packet size to maximum */
		al_udma_m2s_packet_size_cfg_set(udma, &conf);

		/* Set addr_hi selectors */
		if (rev_id >= AL_UDMA_REV_ID_4) {
			struct udma_gen_ex_regs *gen_ex_regs =
				(struct udma_gen_ex_regs *)udma->gen_ex_regs;
			unsigned int i;

			/*
			 * revisions 4,5,6 have the same max number of queues
			 * this is asserted in the head of this function.
			 */
			for (i = 0; i < DMA_MAX_Q_V4; i++)
				al_reg_write32(&gen_ex_regs->vmpr_v4[i].tx_sel, 0xffffffff);
		}
	}

	if (udma->type == UDMA_RX) {
		/* Ack time out */
		al_reg_write32(&udma->udma_regs->s2m.s2m_comp.cfg_application_ack, 0);

		/* Set addr_hi selectors */
		if (rev_id >= AL_UDMA_REV_ID_4) {
			struct udma_gen_ex_regs *gen_ex_regs =
				(struct udma_gen_ex_regs *)udma->gen_ex_regs;
			unsigned int i;

			/*
			 * revisions 4,5,6 have the same max number of queues
			 * this is asserted in the head of this function.
			 */
			for (i = 0; i < DMA_MAX_Q_V4; i++) {
				al_reg_write32(&gen_ex_regs->vmpr_v4[i].rx_sel[0], 0xffffffff);
				al_reg_write32(&gen_ex_regs->vmpr_v4[i].rx_sel[1], 0xffffffff);
				al_reg_write32(&gen_ex_regs->vmpr_v4[i].rx_sel[2], 0xffffffff);
			}
		}

		/* Set S2M max number of outstanding transactions */
		if (rev_id >= AL_UDMA_REV_ID_4) {
			al_reg_write32_masked(&udma->udma_regs->s2m.axi_s2m.ostand_cfg_rd,
				UDMA_AXI_S2M_OSTAND_CFG_RD_MAX_DESC_RD_OSTAND_MASK,
				(128 << UDMA_AXI_S2M_OSTAND_CFG_RD_MAX_DESC_RD_OSTAND_SHIFT));

			al_reg_write32_masked(&udma->udma_regs->s2m.axi_s2m.ostand_cfg_wr,
				UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_DATA_WR_OSTAND_MASK |
				UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_COMP_REQ_MASK,
				(128 << UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_DATA_WR_OSTAND_SHIFT) |
				(128 << UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_COMP_REQ_SHIFT));
		}
	}
}
/**
 * misc queue configurations
 *
 * @param udma_q udma queue data structure
 *
 * @return 0
 */
static int al_udma_q_config(struct al_udma_q *udma_q)
{
	uint32_t *reg_addr;
	uint32_t val;

	if (udma_q->udma->type == UDMA_TX) {
		reg_addr = &udma_q->q_regs->m2s_q.rate_limit_mask;
		val = al_reg_read32(reg_addr);
		// enable DMB
		val &= ~UDMA_M2S_Q_RATE_LIMIT_MASK_INTERNAL_PAUSE_DMB;
		al_reg_write32(reg_addr, val);
	}

	return 0;
}

/**
 * set the queue's completion configuration register
 *
 * @param udma_q udma queue data structure
 * @param q_hw_params hw configuration parameters
 *
 * @return 0
 */
static int al_udma_q_config_compl(struct al_udma_q *udma_q,
	const struct al_udma_q_hw_params *q_hw_params)
{
	uint32_t *reg_addr;
	uint32_t val;

	if (udma_q->udma->type == UDMA_TX)
		reg_addr = &udma_q->q_regs->m2s_q.comp_cfg;
	else
		reg_addr = &udma_q->q_regs->s2m_q.comp_cfg;

	val = al_reg_read32(reg_addr);

	if (q_hw_params->comp_cfg & UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE)
		val |= UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE;
	else
		val &= ~UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE;

	val |= UDMA_M2S_Q_COMP_CFG_DIS_COMP_COAL;

	al_reg_write32(reg_addr, val);

	/* set the completion queue size */
	if (udma_q->udma->type == UDMA_RX) {
		/* Verify validity of cdesc size */
		switch (q_hw_params->cdesc_size) {
		case AL_UDMA_RX_CDESC_SIZE_8:
		case AL_UDMA_RX_CDESC_SIZE_16:
		case AL_UDMA_RX_CDESC_SIZE_32:
			break;
		default:
			al_assert_msg(0, "Unsupported UDMA RX cdesc size %d", q_hw_params->cdesc_size);
		}

		val = al_reg_read32(
				&udma_q->udma->udma_regs->s2m.s2m_comp.cfg_1c);
		val &= ~UDMA_S2M_COMP_CFG_1C_DESC_SIZE_MASK;
		/* the register expects it to be in words */
		val |= (q_hw_params->cdesc_size >> 2)
					& UDMA_S2M_COMP_CFG_1C_DESC_SIZE_MASK;
		al_reg_write32(&udma_q->udma->udma_regs->s2m.s2m_comp.cfg_1c
							, val);
	} else
		/* Verify validity of cdesc size */
		if (q_hw_params->cdesc_phy_base && q_hw_params->cdesc_size != AL_UDMA_TX_CDESC_SIZE)
			al_warn("UDMA TX cdesc size %d is not supported. HW will use cdesc size of %d\n",
					q_hw_params->cdesc_size, AL_UDMA_TX_CDESC_SIZE);
	return 0;
}

/**
 * reset the queues pointers (Head, Tail, etc) and set the base addresses
 *
 * @param udma_q udma queue data structure
 * @param q_hw_params hw configuration parameters
 */
static int al_udma_q_set_pointers(struct al_udma_q *udma_q,
	const struct al_udma_q_hw_params *q_hw_params)
{
	/* reset the descriptors ring pointers */
	/* assert descriptor base address aligned. */
	al_assert((AL_ADDR_LOW(q_hw_params->desc_phy_base) &
		   ~UDMA_M2S_Q_TDRBP_LOW_ADDR_MASK) == 0);
	al_reg_write32(&udma_q->q_regs->rings.drbp_low,
		       AL_ADDR_LOW(q_hw_params->desc_phy_base));
	al_reg_write32(&udma_q->q_regs->rings.drbp_high,
		       AL_ADDR_HIGH(q_hw_params->desc_phy_base));

	al_reg_write32(&udma_q->q_regs->rings.drl, q_hw_params->ring_size);

	/* if completion ring update enabled */
	if (q_hw_params->comp_cfg & UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE) {
		/* reset the completion descriptors ring pointers */
		/* assert completion base address aligned. */
		al_assert((AL_ADDR_LOW(q_hw_params->cdesc_phy_base) &
			   ~UDMA_M2S_Q_TCRBP_LOW_ADDR_MASK) == 0);
		al_reg_write32(&udma_q->q_regs->rings.crbp_low,
			       AL_ADDR_LOW(q_hw_params->cdesc_phy_base));
		al_reg_write32(&udma_q->q_regs->rings.crbp_high,
			       AL_ADDR_HIGH(q_hw_params->cdesc_phy_base));
	}

	al_udma_q_config_compl(udma_q, q_hw_params);

	return 0;
}

/** enable/disable udma queue */
void al_udma_q_enable(struct al_udma_q *udma_q, int enable)
{
	uint32_t reg;

	al_assert(udma_q);

	reg = al_reg_read32(&udma_q->q_regs->rings.cfg);
	if (enable) {
		reg |= (UDMA_M2S_Q_CFG_EN_PREF | UDMA_M2S_Q_CFG_EN_SCHEDULING);
		udma_q->status = AL_QUEUE_ENABLED;
	} else {
		reg &= ~(UDMA_M2S_Q_CFG_EN_PREF | UDMA_M2S_Q_CFG_EN_SCHEDULING);
		udma_q->status = AL_QUEUE_DISABLED;
	}
	al_reg_write32(&udma_q->q_regs->rings.cfg, reg);
}

static enum al_udma_queue_status al_udma_q_status_get(struct al_udma_q *udma_q)
{
	if (al_udma_q_is_enabled(udma_q))
		return AL_QUEUE_ENABLED;

	return AL_QUEUE_DISABLED;

}

/************************ API functions ***************************************/


/* UDMA get revision */
unsigned int al_udma_revision_get(void __iomem *regs_base)
{
	/*
	 * this function uses v4 structure which remains valid also in
	 * future revisions.
	 */
	struct udma_m2s_regs_v4 __iomem *m2s_regs = (struct udma_m2s_regs_v4 __iomem *)regs_base;
	uint32_t dma_version;

	al_assert(regs_base);

	dma_version = al_reg_read32(&m2s_regs->m2s_feature.dma_version);

	if (dma_version	< AL_UDMA_REV_ID_REV4) {
		struct unit_regs_v3 __iomem *unit_regs =
		(struct unit_regs_v3 __iomem *)regs_base;

		dma_version = (al_reg_read32(&unit_regs->gen.dma_misc.revision)
			& UDMA_GEN_DMA_MISC_REVISION_PROGRAMMING_ID_MASK) >>
			UDMA_GEN_DMA_MISC_REVISION_PROGRAMMING_ID_SHIFT;
	}

	return dma_version;
}

/* Initializations functions */

/**
 * Initialize UDMA handle and allow to read current statuses from registers
 */
static int al_udma_handle_init_aux(struct al_udma *udma, struct al_udma_params *udma_params,
	al_bool read_regs)
{
	int i;

	/* mapping from udma revision into max num of queues */
	static const uint8_t rev2numQ[AL_UDMA_REV_ID_7] = {
		DMA_MAX_Q_V3,
		DMA_MAX_Q_V3,
		DMA_MAX_Q_V3,
		DMA_MAX_Q_V4,
		DMA_MAX_Q_V5,
		DMA_MAX_Q_V6,
		DMA_MAX_Q_V7
	};

	udma->rev_id = al_udma_revision_get(udma_params->udma_regs_base);

	al_assert((udma_params->access_mode == UDMA_MAIN) || (udma->rev_id >= AL_UDMA_REV_ID_5));

	/* avoid exceeding rev2numQ array boundries */
	al_assert(udma->rev_id <= AL_UDMA_REV_ID_7);
	al_assert(udma->rev_id >= AL_UDMA_REV_ID_1);

	udma->num_of_queues_max = rev2numQ[udma->rev_id-1];

	udma->type = udma_params->type;

	if (udma_params->num_of_queues == AL_UDMA_NUM_QUEUES_MAX)
		udma->num_of_queues = udma->num_of_queues_max;
	else
		udma->num_of_queues = udma_params->num_of_queues;

	if (udma->num_of_queues > udma->num_of_queues_max) {
		al_err("udma: invalid num_of_queues parameter\n");
		return -EINVAL;
	}

	if (udma->rev_id < AL_UDMA_REV_ID_4) {
		struct unit_regs_v3 __iomem *unit_regs =
			(struct unit_regs_v3 __iomem *)udma_params->udma_regs_base;

		udma->unit_regs_base = (void __iomem *)unit_regs;
		udma->gen_regs = &unit_regs->gen;
		udma->gen_axi_regs = &unit_regs->gen.axi;
		udma->gen_int_regs = &unit_regs->gen.interrupt_regs;
		udma->gen_ex_regs = &unit_regs->gen_ex;
		if (udma->type == UDMA_TX)
			udma->udma_regs = (union udma_regs *)&unit_regs->m2s;
		else
			udma->udma_regs = (union udma_regs *)&unit_regs->s2m;
	} else {
		struct unit_regs_v6 __iomem *unit_regs =
			(struct unit_regs_v6 __iomem *)udma_params->udma_regs_base;

		udma->unit_regs_base = (void __iomem *)unit_regs;
		udma->gen_regs = &unit_regs->gen;
		udma->gen_axi_regs = &unit_regs->gen.axi;
		udma->gen_int_regs = &unit_regs->gen.interrupt_regs;
		switch (udma_params->access_mode) {
		case UDMA_MAIN:
			udma->shadow_int_regs = NULL;
			break;
		case UDMA_SHADOW:
			udma->shadow_int_regs = (struct al_iofic_regs *)
			 &((struct unit_regs_v6 __iomem *)unit_regs)->shadow_view.q_shadow[0].int_c;
			break;
		default:
			al_assert_msg(0, "Unsupported virtualization mode %d\n",
				udma_params->access_mode);
		}
		udma->gen_ex_regs = &unit_regs->gen_ex;
		if (udma->type == UDMA_TX)
			udma->udma_regs = (union udma_regs *)&unit_regs->m2s;
		else
			udma->udma_regs = (union udma_regs *)&unit_regs->s2m;
	}

	if (udma_params->name == NULL)
		udma->name = "";
	else
		udma->name = udma_params->name;

	for (i = 0; i < udma->num_of_queues_max; i++) {
		struct unit_regs_v6 __iomem *unit_regs =
			(struct unit_regs_v6 __iomem *)udma_params->udma_regs_base;

		udma->udma_q[i].q_regs = (udma->type == UDMA_TX) ?
			(union udma_q_regs __iomem *)&udma->udma_regs->m2s.m2s_q[i] :
			(union udma_q_regs __iomem *)&udma->udma_regs->s2m.s2m_q[i];

		switch (udma_params->access_mode) {
		case UDMA_MAIN:
			udma->udma_q[i].q_dp_regs = udma->udma_q[i].q_regs;
			break;
		case UDMA_SHADOW:
			udma->udma_q[i].q_dp_regs = (udma->type == UDMA_TX) ?
				(union udma_q_regs __iomem *)&unit_regs->
					shadow_view.q_shadow[i].m2s_q :
				(union udma_q_regs __iomem *)&unit_regs->
					shadow_view.q_shadow[i].s2m_q;
			break;
		default:
			al_assert_msg(0, "Unsupported virtualization mode %d\n",
				udma_params->access_mode);
		}
		udma->udma_q[i].udma = udma;
		udma->udma_q[i].status = (read_regs ?
			al_udma_q_status_get(&udma->udma_q[i]) : AL_QUEUE_DISABLED);
	}

	udma->state = (read_regs ? al_udma_state_get(udma) : UDMA_DISABLE);

	udma->status_reflection_enabled = (udma_params->reflection_config != NULL);

	return 0;
}

/*
 * Initialize the udma engine handle
 */
int al_udma_handle_init(struct al_udma *udma, struct al_udma_params *udma_params)
{
	al_assert(udma);
	al_assert(udma_params);

	return al_udma_handle_init_aux(udma, udma_params, AL_TRUE);
}

unsigned int al_udma_rev_id_get(struct al_udma *udma)
{
	al_assert(udma);

	return udma->rev_id;
}

/* Performance params printout */
void al_udma_perf_params_print(
	struct al_udma		*m2s_udma,
	struct al_udma		*s2m_udma)
{
	struct al_udma_m2s_desc_pref_conf m2s_conf;
	struct al_udma_s2m_desc_pref_conf s2m_conf;
	unsigned int m2s_ostand_max_data_read;
	unsigned int m2s_ostand_max_desc_read;
	unsigned int m2s_ostand_max_comp_req;
	unsigned int s2m_ostand_max_desc_read;
	unsigned int s2m_ostand_max_data_req;
	unsigned int s2m_ostand_max_comp_req;
	uint32_t reg;
	int err;
	unsigned int i;

	err = al_udma_m2s_pref_get(m2s_udma, &m2s_conf);
	if (err) {
		al_err("%s: al_udma_handle_init failed!\n", __func__);
		return;
	}

	reg = al_reg_read32(&m2s_udma->udma_regs->m2s.axi_m2s.ostand_cfg);
	m2s_ostand_max_data_read = AL_REG_FIELD_GET(reg,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_MASK,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_SHIFT);
	m2s_ostand_max_desc_read = AL_REG_FIELD_GET(reg,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_DESC_RD_MASK,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_DESC_RD_SHIFT);
	m2s_ostand_max_comp_req = AL_REG_FIELD_GET(reg,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_COMP_REQ_MASK,
		UDMA_AXI_M2S_OSTAND_CFG_MAX_COMP_REQ_SHIFT);

	err = al_udma_s2m_pref_get(s2m_udma, &s2m_conf);
	if (err) {
		al_err("%s: al_udma_handle_init failed!\n", __func__);
		return;
	}

	reg = al_reg_read32(&s2m_udma->udma_regs->s2m.axi_s2m.ostand_cfg_rd);
	s2m_ostand_max_desc_read = AL_REG_FIELD_GET(reg,
		UDMA_AXI_S2M_OSTAND_CFG_RD_MAX_DESC_RD_OSTAND_MASK,
		UDMA_AXI_S2M_OSTAND_CFG_RD_MAX_DESC_RD_OSTAND_SHIFT);

	reg = al_reg_read32(&s2m_udma->udma_regs->s2m.axi_s2m.ostand_cfg_wr);
	s2m_ostand_max_data_req = AL_REG_FIELD_GET(reg,
		UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_DATA_WR_OSTAND_MASK,
		UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_DATA_WR_OSTAND_SHIFT);
	s2m_ostand_max_comp_req = AL_REG_FIELD_GET(reg,
		UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_COMP_REQ_MASK,
		UDMA_AXI_S2M_OSTAND_CFG_WR_MAX_COMP_REQ_SHIFT);

	al_print("- M2S\n");
	al_print("  * outstanding reads:\n");
	al_print("    > ostand_max_data_read = %u\n", m2s_ostand_max_data_read);
	al_print("    > ostand_max_desc_read = %u\n", m2s_ostand_max_desc_read);
	al_print("  * outstanding writes:\n");
	al_print("    > ostand_max_comp_req = %u\n", m2s_ostand_max_comp_req);
	al_print("  * prefetch:\n");
	al_print("    > desc_fifo_depth = %u\n", m2s_conf.desc_fifo_depth);
	al_print("    > max_desc_per_packet = %u\n", m2s_conf.max_desc_per_packet);
	al_print("    > pref_thr = %u\n", m2s_conf.pref_thr);
	al_print("    > min_burst_above_thr = %u\n", m2s_conf.min_burst_above_thr);
	al_print("    > min_burst_below_thr = %u\n", m2s_conf.min_burst_below_thr);
	al_print("    > max_pkt_limit = %u\n", m2s_conf.max_pkt_limit);
	al_print("    > data_fifo_depth = %u\n", m2s_conf.data_fifo_depth);

	al_print("- S2M\n");
	al_print("  * outstanding reads:\n");
	al_print("    > ostand_max_desc_read = %u\n", s2m_ostand_max_desc_read);
	al_print("  * outstanding writes:\n");
	al_print("    > ostand_max_data_req = %u\n", s2m_ostand_max_data_req);
	al_print("    > ostand_max_comp_req = %u\n", s2m_ostand_max_comp_req);
	al_print("  * prefetch:\n");
	al_print("    > desc_fifo_depth = %u\n", s2m_conf.desc_fifo_depth);
	al_print("    > pref_thr = %u\n", s2m_conf.pref_thr);
	al_print("    > min_burst_above_thr = %u\n", s2m_conf.min_burst_above_thr);
	al_print("    > min_burst_below_thr = %u\n", s2m_conf.min_burst_below_thr);

	for (i = 0; i < al_udma_num_queues_get(s2m_udma); i++) {
		struct al_udma_q *s2m_udma_q;
		struct al_udma_s2m_q_comp_conf comp_conf;

		err = al_udma_q_handle_get(s2m_udma, i, &s2m_udma_q);
		if (err) {
			al_err("%s: al_udma_q_handle_get failed!\n", __func__);
			return;
		}

		if (!al_udma_q_is_enabled(s2m_udma_q)) {
			al_print("  * Queue %u: Disabled\n", i);
			continue;
		}

		al_udma_s2m_q_comp_get(s2m_udma_q, &comp_conf);

		al_print("  * Queue %u\n", i);
		al_print("    > dis_comp_coal = %s\n",
			comp_conf.dis_comp_coal ? "enabled" : "disabled");
		al_print("    > en_comp_ring_update = %s\n",
			comp_conf.en_comp_ring_update ? "enabled" : "disabled");
		al_print("    > comp_timer = %u\n", comp_conf.comp_timer);
		al_print("    > q_qos = %u\n", comp_conf.q_qos);
	}
}

/* Num available queues */
unsigned int al_udma_num_queues_get(
	const struct al_udma *udma)
{
	al_assert(udma);

	return udma->num_of_queues;
}

/**
 * set permissions for shadowed q registers access
 *
 * @param udma udma handle
 * @param q_reg_offset register offset in bytes
 * @param dir read/write
 * @param en enable/disable access
 *
 * @return 0
 */
static int al_udma_shadow_q_reg_permission_set(struct al_udma *udma,
	unsigned int q_reg_offset,
	enum udma_shadow_permission_dir dir,
	al_bool en)
{
	uint32_t *permissions_reg;
	unsigned int reg_idx, bit_idx;

	al_assert(udma->rev_id >= AL_UDMA_REV_ID_5);
	al_assert((q_reg_offset & (sizeof(uint32_t) - 1)) == 0); /* Multiple of register size */

	reg_idx = q_reg_offset / sizeof(uint32_t); /* Bytes to register index */

	if (udma->type == UDMA_TX) {
		al_assert(reg_idx < AL_UDMA_TX_ACCESS_CTRL_REGS_LIMIT);
		permissions_reg = (uint32_t *)(((struct udma_m2s_shadow_access_shadow_index *)
			&udma->udma_regs->m2s.m2s_shadow_access) + (reg_idx / 32));
	} else {
		al_assert(reg_idx < AL_UDMA_RX_ACCESS_CTRL_REGS_LIMIT);
		permissions_reg = (uint32_t *)(((struct udma_s2m_shadow_access_shadow_index *)
			&udma->udma_regs->s2m.s2m_shadow_access) + (reg_idx / 32));
	}

	bit_idx = reg_idx % 32;

	al_dbg("%s: Setting %s permission to %d @%p[%d]\n", __func__,
		dir == PERMISSION_DIR_RD ? "RD" : "WR", en, permissions_reg + dir, bit_idx);
	al_reg_write32_masked(permissions_reg + dir, AL_BIT(bit_idx), en << bit_idx);

	return 0;
}

/**
 * set permissions for shadowed interrupt group control register access
 *
 * @param udma udma handle
 * @param int_grp interrupts group
 * @param grp_reg_offset register offset inside group in bytes
 * @param dir read/write
 * @param en enable/disable access
 *
 * @return 0
 */
static int al_udma_shadow_int_grp_reg_permission_set(struct al_udma *udma,
	int int_grp,
	unsigned int grp_reg_offset,
	enum udma_shadow_permission_dir dir,
	al_bool en)
{
	uint32_t *permissions_reg;
	void __iomem *int_access_base;
	unsigned int grp_regs_num;
	unsigned int reg_idx, bit_idx;

	al_assert(udma->rev_id >= AL_UDMA_REV_ID_5);

	al_assert(int_grp < AL_IOFIC_MAX_GROUPS);
	al_assert((grp_reg_offset & (sizeof(uint32_t) - 1)) == 0); /* Multiple of register size */

	int_access_base = &((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->intc_shadow_access;

	grp_regs_num = sizeof(struct al_iofic_grp_ctrl) / sizeof(uint32_t); /* Bytes to regs */
	reg_idx = (grp_regs_num * int_grp) + (grp_reg_offset / sizeof(uint32_t));

	al_assert(reg_idx < AL_UDMA_INTC_ACCESS_CTRL_REGS_LIMIT);

	/* Each permissions bit corresponds to a register */
	permissions_reg = (uint32_t *)(((struct udma_gen_intc_shadow_access_shadow_index *)
		int_access_base + (reg_idx  / 32)));

	bit_idx = reg_idx % 32;

	al_dbg("%s: Setting %s permission to %d @%p[%d]\n", __func__,
		dir == PERMISSION_DIR_RD ? "RD" : "WR", en, permissions_reg + dir, bit_idx);
	al_reg_write32_masked(permissions_reg + dir, AL_BIT(bit_idx), en << bit_idx);

	return 0;
}

/* Enable shadowed access to subset of data path registers */
static int al_udma_shadow_dp_enable(struct al_udma *udma)
{
	/* Enable tail pointer increment */
	al_udma_shadow_q_reg_permission_set(udma,
		al_offsetof(struct udma_dp_regs, drtp_inc), PERMISSION_DIR_WR, AL_TRUE);
	/* Enable reading completion ring head pointer value */
	al_udma_shadow_q_reg_permission_set(udma,
		al_offsetof(struct udma_dp_regs, crhp), PERMISSION_DIR_RD, AL_TRUE);

	return 0;
}

/* Enable shadowed access to subset of interrupt contorl registers */
static int al_udma_shadow_ints_enable(struct al_udma *udma)
{
	al_assert(udma->type == UDMA_RX);

	/* Grant access to unmasking group C interrupts */
	al_udma_shadow_int_grp_reg_permission_set(udma,
		AL_INT_GROUP_C, al_offsetof(struct al_iofic_grp_ctrl, int_mask_clear_grp),
		PERMISSION_DIR_WR, AL_TRUE);

	return 0;
}

static void al_udma_status_reflection_control(
	struct udma_gen_reflect __iomem *reflect_rba,
	al_bool enable)
{
	/* determime whether we should activate or deactivate the reflection */
	uint32_t reg_val = (enable ? UDMA_GEN_REFLECT_CFG_GLOBAL_EN : 0);

	al_reg_write32_masked(&reflect_rba->cfg,
			UDMA_GEN_REFLECT_CFG_GLOBAL_EN_MASK,
			reg_val);
}

static void al_udma_reflection_timer_set(
	struct udma_gen_reflect __iomem *reflect_rba,
	uint32_t update_cycle_time_nsec)
{
	uint64_t update_cycle_time_clock_ticks = 0;

	/* input validation */
	al_assert(update_cycle_time_nsec);

	/* use 64 bit variable as we multiply in 1000 */
	/* we multiply in 1000 and divide by 1000 to avoid floating point calculations */
#ifdef __arm__
	update_cycle_time_clock_ticks = ((uint64_t)update_cycle_time_nsec*1000);
	do_div(update_cycle_time_clock_ticks, AL_UDMA_REFLECTION_NSEC_PER_K_TICKS);
#else
	update_cycle_time_clock_ticks = ((uint64_t)update_cycle_time_nsec*1000) /
		AL_UDMA_REFLECTION_NSEC_PER_K_TICKS;
#endif

	/* if the calculation gave us less then 1 clock tick, set it to 1 */
	if (!update_cycle_time_clock_ticks)
		update_cycle_time_clock_ticks = 1;

	al_reg_write32(&reflect_rba->timer_cfg, update_cycle_time_clock_ticks);
}

static void al_udma_config_reflection_agent(uint32_t __iomem *agent_reg,
	const struct al_udma_reflect_agent *agent_config)
{
	al_static_assert(UDMA_GEN_REFLECT_RX_HEAD_AGENT_ENABLE == UDMA_GEN_REFLECT_TX_HEAD_AGENT_ENABLE,
		"this common function assumes rx and tx registers have the same layout");

	/* this means we also disable bypass enable bit on init */
	uint32_t reg_val = (agent_config->enable ? UDMA_GEN_REFLECT_RX_HEAD_AGENT_ENABLE : 0);

	al_reg_write32_masked(agent_reg,
			UDMA_GEN_REFLECT_RX_HEAD_AGENT_ENABLE_MASK,
			reg_val);
}

static int al_udma_status_reflection_init(struct al_udma *udma,
	const struct al_udma_reflect_config *reflect_config)
{
	/* the status reflection feature is available starting rev 6 */
	al_assert((udma->rev_id >= AL_UDMA_REV_ID_6) || (reflect_config == NULL));

	if (udma->rev_id >= AL_UDMA_REV_ID_6) {

		/* pointer to the global reflection RBA */
		struct udma_gen_reflect __iomem *reflect_rba =
			&((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->reflect;

		/* NULL means disable the reflection */
		if (reflect_config == NULL) {

			/* disable the reflection engine */
			al_udma_status_reflection_control(reflect_rba, AL_FALSE);

		} else {

			/* set the timer */
			al_udma_reflection_timer_set(reflect_rba,
					reflect_config->update_cycle_time_nsec);

			/* configure head update agents */
			al_udma_config_reflection_agent(&reflect_rba->tx_head_agent, &reflect_config->tx_head_agent);
			al_udma_config_reflection_agent(&reflect_rba->rx_head_agent, &reflect_config->rx_head_agent);

			/* finally, enable the reflection engine */
			al_udma_status_reflection_control(reflect_rba, AL_TRUE);
		}
	}

	return 0;
}

al_phys_addr_t al_udma_push_get_q_ring_address(uint16_t queue_id,
					enum al_udma_push_tx_rx_select tx_rx_select,
					al_phys_addr_t axi_stream_udma_push_base)
{
	/* the default returned value indicates failure */
	al_phys_addr_t ret_val = 0;

	/* currently the only used mode is descriptor */
	enum al_udma_push_desc_db_select desc_doorbell_select = AL_UDMA_PUSH_DESC_DB_SELECT_DESCRIPTOR;

	/**
	 * the base address shell not exceed the MSB mask.
	 * LSB's are used to indicate the address within the ring
	 */
	if (axi_stream_udma_push_base & AL_UDMA_PUSH_AXI_STREAM_MSB_MASK)
		return ret_val;

	if (!axi_stream_udma_push_base)
		return ret_val;

	/* apply the base address */
	ret_val = axi_stream_udma_push_base;

	/* apply the doorbell/descriptor mode */
	ret_val = AL_MASK_VAL(AL_UDMA_PUSH_DESCR_DOORBELL_SEL_MASK,
			((uint32_t)desc_doorbell_select << AL_UDMA_PUSH_DESCR_DOORBELL_SEL_OFFSET),
			ret_val);

	/* apply the queue id field */
	ret_val = AL_MASK_VAL(AL_UDMA_PUSH_QUEUE_ID_FIELD_MASK,
			((uint32_t)queue_id << AL_UDMA_PUSH_QUEUE_ID_FIELD_OFFSET),
			ret_val);

	/* apply the tx/rx field */
	ret_val = AL_MASK_VAL(AL_UDMA_PUSH_TX_RX_FIELD_MASK,
			((uint32_t)tx_rx_select << AL_UDMA_PUSH_TX_RX_FIELD_OFFSET),
			ret_val);

	return ret_val;
}

static int al_udma_push_mode_init(struct al_udma *udma,
		const struct al_udma_push_config *push_config)
{
	int ret_val = 0;
	al_bool push_mode_disabled = ((!push_config->tx_enable) && (!push_config->rx_enable));

	/* initialize it as 0 to reset the pushback and *_err_mask fields */
	uint32_t control_register_val = 0;

	/* pointer to the push packet RBA */
	struct udma_gen_pp __iomem *push_rba =
		&((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->pp;

	/* the push packet mode is available starting rev 6 */
	al_assert((udma->rev_id >= AL_UDMA_REV_ID_6) || push_mode_disabled);

	/* don't execute this for previos revisions */
	if (udma->rev_id >= AL_UDMA_REV_ID_6) {

		if (push_config->rx_enable)
			control_register_val |= UDMA_GEN_PP_CFG_RX_PP_MODE_EN;

		if (push_config->tx_enable)
			control_register_val |= UDMA_GEN_PP_CFG_TX_PP_MODE_EN;

		/* configure the push mode control register */
		al_reg_write32(&push_rba->cfg, control_register_val);

		/* configure the field extract offsets the udma needs in this mode */
		al_reg_write32_masked(&push_rba->queue_fe.offset,
				CELL_FE_OFFSET_VAL_MASK,
				(AL_UDMA_PUSH_QUEUE_ID_FIELD_OFFSET << CELL_FE_OFFSET_VAL_SHIFT));

		al_reg_write32_masked(&push_rba->tx_fe.offset,
				CELL_FE_OFFSET_VAL_MASK,
				(AL_UDMA_PUSH_TX_RX_FIELD_OFFSET << CELL_FE_OFFSET_VAL_SHIFT));
	}

	return ret_val;
}

/*
 * Initialize the udma engine
 */
int al_udma_init(struct al_udma *udma, struct al_udma_params *udma_params)
{
	int ret;

	al_assert(udma);
	al_assert(udma_params);

	ret = al_udma_handle_init_aux(udma, udma_params, AL_FALSE);
	if (ret)
		return ret;

	/* initialize configuration registers to correct values */
	al_udma_set_defaults(udma);

	if (udma_params->access_mode == UDMA_SHADOW) {
		al_udma_shadow_dp_enable(udma);
		if (udma->type == UDMA_RX)
			al_udma_shadow_ints_enable(udma);
	}

	/* unmask error interrupts */
	if (udma->type == UDMA_TX)
		al_udma_iofic_m2s_error_ints_unmask(udma);
	else
		al_udma_iofic_s2m_error_ints_unmask(udma	);

	/* initialize queue status reflection engine */
	ret = al_udma_status_reflection_init(udma, udma_params->reflection_config);
	if (ret)
		return ret;

	ret = al_udma_push_mode_init(udma, &udma_params->push_config);
	if (ret)
		return ret;

	al_dbg("udma [%s] initialized. base %p\n", udma->name,
		udma->udma_regs);
	return 0;
}

/* Reset (and disable) all UDMA queues */
int al_udma_q_reset_all(struct al_udma *udma)
{
	unsigned int i;
	int err;

	al_assert(udma);

	for (i = 0; i < udma->num_of_queues; i++) {
		struct al_udma_q *q = NULL;

		err = al_udma_q_handle_get(udma, i, &q);
		if (err) {
			al_err("%s: al_udma_q_handle_get failed(%u)!\n", __func__, i);
			return err;
		}

		if (!al_udma_q_is_enabled(q))
			continue;

		err = al_udma_q_reset(q);
		if (err) {
			al_err("%s: al_udma_q_reset failed(%u)!\n", __func__, i);
			return err;
		}
	}

	return 0;
}

static int al_udma_q_ring_params_set(struct al_udma_q *udma_q,
	const struct al_udma_q_ring_params *q_ring_params)
{
	uint32_t *hp_set_reg_addr;
	uint32_t *tp_set_reg_addr;

	al_assert(udma_q->udma->rev_id >= AL_UDMA_REV_ID_5);

	if (udma_q->udma->type == UDMA_TX) {
		hp_set_reg_addr = &udma_q->q_regs->m2s_q.tdrhp_set;
		tp_set_reg_addr = &udma_q->q_regs->m2s_q.tdrtp_set;

		al_reg_write32(hp_set_reg_addr,
			(((q_ring_params->head_idx << UDMA_M2S_Q_TDRHP_SET_PTR_VALUE_SHIFT) &
				UDMA_M2S_Q_TDRHP_SET_PTR_VALUE_MASK) |
			((q_ring_params->comp_ring_id << UDMA_M2S_Q_TDRHP_SET_RIND_IG_SHIFT) &
				UDMA_M2S_Q_TDRHP_SET_RIND_IG_MASK)));
		al_reg_write32(tp_set_reg_addr,
			(((q_ring_params->tail_idx << UDMA_M2S_Q_TDRTP_SET_PTR_VAL_SHIFT) &
				UDMA_M2S_Q_TDRTP_SET_PTR_VAL_MASK) |
			((q_ring_params->desc_ring_id << UDMA_M2S_Q_TDRTP_SET_RING_ID_SHIFT) &
				UDMA_M2S_Q_TDRTP_SET_RING_ID_MASK)));
	} else {
		hp_set_reg_addr = &udma_q->q_regs->s2m_q.rdrhp_set;
		tp_set_reg_addr = &udma_q->q_regs->s2m_q.rdrtp_set;

		al_reg_write32(hp_set_reg_addr,
			(((q_ring_params->head_idx << UDMA_S2M_Q_RDRHP_SET_PTR_VALUE_SHIFT) &
				UDMA_S2M_Q_RDRHP_SET_PTR_VALUE_MASK) |
			((q_ring_params->comp_ring_id << UDMA_S2M_Q_RDRHP_SET_RING_ID_SHIFT) &
				UDMA_S2M_Q_RDRHP_SET_RING_ID_MASK)));
		al_reg_write32(tp_set_reg_addr,
			(((q_ring_params->tail_idx << UDMA_S2M_Q_RDRTP_SET_PTR_VALUE_SHIFT) &
				UDMA_S2M_Q_RDRTP_SET_PTR_VALUE_MASK) |
			((q_ring_params->desc_ring_id << UDMA_S2M_Q_RDRTP_SET_RING_ID_SHIFT) &
				UDMA_S2M_Q_RDRTP_SET_RING_ID_MASK)));
	}

	return 0;

}

static void al_udma_q_configure_descr_prefetch_fifo(struct al_udma_q *udma_q)
{
	uint32_t *prefetch_config1_address = NULL;
	uint32_t *prefetch_config2_address = NULL;

	uint32_t fifo_start_address = 0;
	uint32_t fifo_end_address = 0;
	uint32_t fifo_depth = 0;

	uint32_t fifo_start_mask = 0;
	uint32_t fifo_start_shift = 0;
	uint32_t fifo_depth_mask = 0;
	uint32_t fifo_depth_shift = 0;
	uint32_t fifo_end_mask = 0;
	uint32_t fifo_end_shift = 0;

	/* set descriptor prefetch fifo depth. available for v6 or higher */
	if (udma_q->udma->rev_id < AL_UDMA_REV_ID_6)
		return;

	/* determine the registers and values that needs to be written according to handle type */
	if (udma_q->udma->type == UDMA_TX) {

		prefetch_config1_address = &udma_q->q_regs->m2s_q.desc_pref_cfg;
		prefetch_config2_address = &udma_q->q_regs->m2s_q.desc_pref_cfg2;

		fifo_depth = AL_UDMA_TX_PREFETCH_FIFO_DEPTH;

		fifo_start_mask = UDMA_M2S_Q_DESC_PREF_CFG_FIFO_START_ADDR_MASK;
		fifo_start_shift = UDMA_M2S_Q_DESC_PREF_CFG_FIFO_START_ADDR_SHIFT;
		fifo_depth_mask = UDMA_M2S_Q_DESC_PREF_CFG_FIFO_DEPTH_MASK;
		fifo_depth_shift = UDMA_M2S_Q_DESC_PREF_CFG_FIFO_DEPTH_SHIFT;
		fifo_end_mask = UDMA_M2S_Q_DESC_PREF_CFG2_FIFO_END_ADDR_MASK;
		fifo_end_shift = UDMA_M2S_Q_DESC_PREF_CFG_FIFO_DEPTH_SHIFT;

	} else {

		prefetch_config1_address = &udma_q->q_regs->s2m_q.desc_pref_cfg;
		prefetch_config2_address = &udma_q->q_regs->s2m_q.desc_pref_cfg2;

		fifo_depth = AL_UDMA_RX_PREFETCH_FIFO_DEPTH;

		fifo_start_mask = UDMA_S2M_Q_DESC_PREF_CFG_FIFO_START_ADDR_MASK;
		fifo_start_shift = UDMA_S2M_Q_DESC_PREF_CFG_FIFO_START_ADDR_SHIFT;
		fifo_depth_mask = UDMA_S2M_Q_DESC_PREF_CFG_FIFO_DEPTH_MASK;
		fifo_depth_shift = UDMA_S2M_Q_DESC_PREF_CFG_FIFO_DEPTH_SHIFT;
		fifo_end_mask = UDMA_S2M_Q_DESC_PREF_CFG2_FIFO_END_ADDR_MASK;
		fifo_end_shift = UDMA_S2M_Q_DESC_PREF_CFG_FIFO_DEPTH_SHIFT;
	}

	/* set the start and end address using data we already have */
	fifo_start_address = udma_q->qid * fifo_depth;
	fifo_end_address = fifo_start_address + fifo_depth - 1;

	/* write all registers */
	al_reg_write32_masked(prefetch_config1_address,
			fifo_start_mask,
			(fifo_start_address << fifo_start_shift));

	al_reg_write32_masked(prefetch_config1_address,
			fifo_depth_mask,
			(fifo_depth << fifo_depth_shift));

	al_reg_write32_masked(prefetch_config2_address,
			fifo_end_mask,
			(fifo_end_address << fifo_end_shift));
}

static int al_udma_q_hw_init(struct al_udma_q *udma_q,
	const struct al_udma_q_hw_params *q_hw_params)
{
	al_udma_q_config(udma_q);
	/* reset the queue pointers */
	al_udma_q_set_pointers(udma_q, q_hw_params);

	/* set descriptor prefetch fifo depth */
	al_udma_q_configure_descr_prefetch_fifo(udma_q);

	return 0;
}

int al_udma_q_handle_init(struct al_udma *udma,
	struct al_udma_q *udma_q,
	uint32_t qid,
	struct al_udma_q_params *q_params)
{
	al_assert(udma);
	al_assert(udma_q);
	al_assert(q_params);

	udma_q->qid = qid;

	if (udma_q->status == AL_QUEUE_ENABLED) {
		al_err("udma: queue (%d) already enabled!\n", udma_q->qid);
		return -EIO;
	}

	if (q_params->size < AL_UDMA_MIN_Q_SIZE) {
		al_err("udma: queue (%d) size (%d) too small\n",
			udma_q->qid, q_params->size);
		return -EINVAL;
	}

	if (q_params->size > AL_UDMA_MAX_Q_SIZE) {
		al_err("udma: queue (%d) size (%d) too large\n",
			udma_q->qid, q_params->size);
		return -EINVAL;
	}

	if (q_params->size & (q_params->size - 1)) {
		al_err("udma: queue (%d) size (%d) must be power of 2\n",
			udma_q->qid, q_params->size);
		return -EINVAL;
	}

	udma_q->adapter_rev_id = q_params->adapter_rev_id;
	udma_q->size = q_params->size;
	udma_q->size_mask = q_params->size - 1;
	udma_q->desc_base_ptr = q_params->desc_base;
	udma_q->desc_phy_base = q_params->desc_phy_base;
	udma_q->cdesc_base_ptr = q_params->cdesc_base;
	udma_q->cdesc_phy_base = q_params->cdesc_phy_base;
	udma_q->cdesc_size = q_params->cdesc_size;
	udma_q->next_desc_idx = 0;
	udma_q->next_cdesc_idx = 0;
	udma_q->end_cdesc_ptr = (uint8_t *) udma_q->cdesc_base_ptr +
		(udma_q->size - 1) * udma_q->cdesc_size;
	udma_q->comp_head_idx = 0;
	udma_q->comp_head_ptr = (union al_udma_cdesc *)udma_q->cdesc_base_ptr;
	udma_q->desc_ring_id = AL_UDMA_INITIAL_RING_ID;
	udma_q->comp_ring_id = AL_UDMA_INITIAL_RING_ID;
	udma_q->pkt_crnt_descs = 0;
	udma_q->flags = 0;

	udma_q->status_reflection.enable = (q_params->q_reflection_config != NULL);

	/* reflection parameters */
	if (udma_q->status_reflection.enable) {

		/* queue reflection can be enabled only if the global feature is enabled */
		al_assert(udma->status_reflection_enabled);

		/* if you want that feature on, you have to assign these addresses */
		al_assert(q_params->q_reflection_config->status_virt_address);
		al_assert(q_params->q_reflection_config->status_phy_address);

		/* the addresses must be cache line aligned */
		al_assert(((uintptr_t)q_params->q_reflection_config->status_virt_address % AL_CACHE_LINE_SIZE) == 0);
		al_assert((q_params->q_reflection_config->status_phy_address % AL_CACHE_LINE_SIZE) == 0);

		udma_q->status_reflection.phy_address = q_params->q_reflection_config->status_phy_address;

		udma_q->status_reflection.crhp_reflection = (udma_q->udma->type == UDMA_TX ?
			&(q_params->q_reflection_config->status_virt_address->data.tx_crhp) :
			&(q_params->q_reflection_config->status_virt_address->data.rx_crhp));
	} else {

		/* avoid garbage values */
		udma_q->status_reflection.phy_address = 0;
		udma_q->status_reflection.crhp_reflection = NULL;
	}
#if 0
	udma_q->desc_ctrl_bits = AL_UDMA_INITIAL_RING_ID <<
						AL_M2S_DESC_RING_ID_SHIFT;
#endif
	udma_q->status = AL_QUEUE_DISABLED;
	udma_q->udma = udma;

	return 0;
}

static void  al_udma_q_set_reflection_address(
	struct udma_m2s_q_reflect __iomem *q_reflect_rba,
	al_phys_addr_t status_address)
{
	al_reg_write32(&q_reflect_rba->addr_high, AL_ADDR_HIGH(status_address));
	al_reg_write32(&q_reflect_rba->addr_low, AL_ADDR_LOW(status_address));
}

static void al_udma_q_status_reflection_control(
	struct udma_m2s_q_reflect __iomem *q_reflect_rba,
	al_bool enable)
{
	/*
	 * determime whether we should activate or deactivate this q reflection.
	 * if we do, enable the promotion feature by default.
	 */
	uint32_t reg_val = (enable ?
			(UDMA_M2S_Q_REFLECT_CTRL_EN | UDMA_M2S_Q_REFLECT_CTRL_PROMO_EN) :
			0);

	al_reg_write32_masked(&q_reflect_rba->ctrl,
			UDMA_M2S_Q_REFLECT_CTRL_EN_MASK | UDMA_M2S_Q_REFLECT_CTRL_PROMO_EN_MASK,
			reg_val);
}

static int al_udma_q_status_reflection_init(struct al_udma *udma,
	struct al_udma_q *udma_q)
{
	/* by default indicate success */
	int ret_val = 0;

	/* the status reflection feature is available starting rev 6 */
	al_assert((udma->rev_id >= AL_UDMA_REV_ID_6) || (!udma_q->status_reflection.enable));

	if (udma->rev_id >= AL_UDMA_REV_ID_6) {

		/* pointer to the queue reflection RBA */
		struct udma_m2s_q_reflect __iomem *q_reflect_rba =
			&((struct unit_regs_v6 __iomem *)udma->unit_regs_base)->m2s.m2s_q[udma_q->qid].reflect;

		/* disable the queue reflection as we cannot update its config while enabled */
		al_udma_q_status_reflection_control(q_reflect_rba, AL_FALSE);

		/* NULL means disable the reflection - we already did it */
		if (udma_q->status_reflection.enable) {

			/*
			 * IMPORTANT NOTE: the address is written into two high/low registers
			 * if the reflection is active while the address is updated, the hw
			 * might calculate the address to access when the first register has
			 * been written but the second didn't. this will have bad impact.
			 */
			al_udma_q_set_reflection_address(q_reflect_rba, udma_q->status_reflection.phy_address);

			/* finally, enable this q reflection */
			al_udma_q_status_reflection_control(q_reflect_rba, AL_TRUE);
		}
	}

	return ret_val;
}

/*
 * Initialize the udma queue data structure
 */
int al_udma_q_init(struct al_udma *udma, uint32_t qid,
					struct al_udma_q_params *q_params)
{
	struct al_udma_q *udma_q;
	struct al_udma_q_hw_params hw_params = {};
	int ret;
	uint32_t reg;

	al_assert(udma);
	al_assert(q_params);

	if (qid >= udma->num_of_queues) {
		al_err("udma: invalid queue id (%d)\n", qid);
		return -EINVAL;
	}

	udma_q = &udma->udma_q[qid];

	ret = al_udma_q_handle_init(udma, udma_q, qid, q_params);
	if (ret)
		return ret;

	hw_params.ring_size = q_params->size;
	hw_params.desc_phy_base = q_params->desc_phy_base;
	hw_params.cdesc_phy_base = q_params->cdesc_phy_base;
	hw_params.cdesc_size = q_params->cdesc_size;
	hw_params.comp_cfg = (q_params->cdesc_base != NULL) ?
		UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE : 0;

	/* start hardware configuration */
	al_udma_q_hw_init(udma_q, &hw_params);

	/* configure queue status reflection registers */
	ret = al_udma_q_status_reflection_init(udma, udma_q);
	if (ret)
		return ret;

	/* Configure force full line */
	reg = al_reg_read32(&udma_q->q_regs->rings.cfg);
	if (q_params->cdesc_force_full_line)
		reg |= UDMA_M2S_Q_CFG_FORCE_FULL_LINE;
	else
		reg &= ~(UDMA_M2S_Q_CFG_FORCE_FULL_LINE);

	al_reg_write32(&udma_q->q_regs->rings.cfg, reg);

	/* enable the q */
	al_udma_q_enable(udma_q, 1);

	al_dbg("udma [%s %d]: %s q init. size 0x%x\n"
			"  desc ring info: phys base 0x%" PRIx64 " virt base %p\n",
			udma_q->udma->name, udma_q->qid,
			udma->type == UDMA_TX ? "Tx" : "Rx",
			q_params->size,
			q_params->desc_phy_base,
			q_params->desc_base);
	al_dbg("  cdesc ring info: phys base 0x%" PRIx64 " virt base %p entry size 0x%x\n",
			q_params->cdesc_phy_base,
			q_params->cdesc_base,
			q_params->cdesc_size);

	return 0;
}

al_bool al_udma_q_is_enabled(struct al_udma_q *udma_q)
{
	uint32_t reg;

	al_assert(udma_q);

	reg = al_reg_read32(&udma_q->q_regs->rings.cfg);
	if (reg & (UDMA_M2S_Q_CFG_EN_PREF | UDMA_M2S_Q_CFG_EN_SCHEDULING))
		return AL_TRUE;

	return AL_FALSE;
}

/*
 * pause a udma queue
 */
int al_udma_q_pause(struct al_udma_q *udma_q)
{
	unsigned int remaining_time = AL_UDMA_Q_QUIESCE_TOUT;
	uint32_t *status_reg;
	uint32_t *dcp_reg;
	uint32_t *crhp_reg;

	al_assert(udma_q);

	/* De-assert scheduling and prefetch */
	al_udma_q_enable(udma_q, 0);

	/* Wait for scheduling and prefetch to stop */
	status_reg = &udma_q->q_regs->rings.status;

	while (remaining_time) {
		uint32_t status = al_reg_read32(status_reg);

		if (!(status & (UDMA_M2S_Q_STATUS_PREFETCH |
						UDMA_M2S_Q_STATUS_SCHEDULER)))
			break;

		remaining_time--;
		al_udelay(1);
	}

	if (!remaining_time) {
		al_err("udma [%s %d]: %s timeout waiting for prefetch and "
			"scheduler disable\n", udma_q->udma->name, udma_q->qid,
			__func__);
		return -ETIME;
	}

	/* Wait for the completion queue to reach to the same pointer as the
	 * prefetch stopped at ([TR]DCP == [TR]CRHP) */
	dcp_reg = &udma_q->q_regs->rings.dcp;
	crhp_reg = &udma_q->q_regs->rings.crhp;

	while (remaining_time) {
		uint32_t dcp = al_reg_read32(dcp_reg);
		uint32_t crhp = al_reg_read32(crhp_reg);

		if (dcp == crhp)
			break;

		remaining_time--;
		al_udelay(1);
	};

	if (!remaining_time) {
		al_err("udma [%s %d]: %s timeout waiting for dcp==crhp\n",
			udma_q->udma->name, udma_q->qid, __func__);
		return -ETIME;
	}

	return 0;
}

int al_udma_q_resume(struct al_udma_q *udma_q)
{
	al_assert(udma_q);

	al_udma_q_enable(udma_q, 1);

	return 0;
}

/*
 * Reset a udma queue
 */
int al_udma_q_reset(struct al_udma_q *udma_q)
{
	uint32_t *q_sw_ctrl_reg;
	int rc;

	al_assert(udma_q);

	rc = al_udma_q_pause(udma_q);

	if (rc)
		return rc;

	/* Assert the queue reset */
	if (udma_q->udma->type == UDMA_TX)
		q_sw_ctrl_reg = &udma_q->q_regs->m2s_q.q_sw_ctrl;
	else
		q_sw_ctrl_reg = &udma_q->q_regs->s2m_q.q_sw_ctrl;

	al_reg_write32(q_sw_ctrl_reg, UDMA_M2S_Q_SW_CTRL_RST_Q);

	return 0;
}

/*
 * return (by reference) a pointer to a specific queue date structure.
 */
int al_udma_q_handle_get(struct al_udma *udma, uint32_t qid,
						struct al_udma_q **q_handle)
{

	al_assert(udma);
	al_assert(q_handle);

	if (unlikely(qid >= udma->num_of_queues)) {
		al_err("udma [%s]: invalid queue id (%d)\n", udma->name, qid);
		return -EINVAL;
	}
	*q_handle = &udma->udma_q[qid];
	return 0;
}

/*
 * Change the UDMA's state
 */
int al_udma_state_set(struct al_udma *udma, enum al_udma_state state)
{
	uint32_t reg;

	al_assert(udma != NULL);
	if (state == udma->state)
		al_dbg("udma [%s]: requested state identical to "
			"current state (%d)\n", udma->name, state);

	al_dbg("udma [%s]: change state from (%s) to (%s)\n",
		 udma->name, al_udma_states_name[udma->state],
		 al_udma_states_name[state]);

	reg = 0;
	switch (state) {
	case UDMA_DISABLE:
		reg |= UDMA_M2S_CHANGE_STATE_DIS;
		break;
	case UDMA_NORMAL:
		reg |= UDMA_M2S_CHANGE_STATE_NORMAL;
		break;
	case UDMA_ABORT:
		reg |= UDMA_M2S_CHANGE_STATE_ABORT;
		break;
	default:
		al_err("udma: invalid state (%d)\n", state);
		return -EINVAL;
	}

	if (udma->type == UDMA_TX)
		al_reg_write32(&udma->udma_regs->m2s.m2s.change_state, reg);
	else
		al_reg_write32(&udma->udma_regs->s2m.s2m.change_state, reg);

	udma->state = state;
	return 0;
}

static void al_udma_s2m_fifos_flush(struct al_udma *udma)
{
	al_assert(udma);
	al_assert(udma->type == UDMA_RX);

	/** FIFO reset - toggle enable bit*/
	al_reg_write32(&udma->udma_regs->s2m.s2m.fifo_en, 0);
	/** restart udma control state machines */
	al_reg_write32(&udma->udma_regs->s2m.s2m.clear_ctrl, 0xFFFFFFFF);
	al_udelay(1);

	/** Re enable fifo's */
	al_reg_write32(&udma->udma_regs->s2m.s2m.fifo_en, 0xFFFFFFFF);
	/** restart udma state machines */
	al_reg_write32(&udma->udma_regs->s2m.s2m.clear_ctrl, 0xFFFFFFFF);
	al_udelay(1);
}

#define AL_UDMA_S2M_STREAM_FLUSH \
				(UDMA_S2M_STREAM_CFG_DISABLE_STREAM | \
				UDMA_S2M_STREAM_CFG_FLUSH	| \
				UDMA_S2M_STREAM_CFG_STOP_PREFETCH)

static void udma_s2m_stream_set_mode(struct al_udma *udma, al_bool state)
{
	uint32_t stream_dis = AL_UDMA_S2M_STREAM_FLUSH;

	al_reg_write32_masked(&udma->udma_regs->s2m.s2m.stream_cfg,
		stream_dis,
		state ? 0 : stream_dis);
}

/**
 * Enable/disable stream queues
 *
 * @param udma udma handle
 * @param state enable/disable
 * @param en_queues_mask save enabled queues before disabling, or queues to
 * enable when enabling
 */
static void udma_s2m_stream_queues_set_mode(struct al_udma *udma, al_bool state,
					    uint32_t *en_queues_mask)
{
	int i = 0;
	int rc;

	/* Expecting uint32_t is enough to hold all queues */
	al_assert(udma->num_of_queues <= 32);

	for (i = 0; i < udma->num_of_queues; i++) {
		struct al_udma_q *dma_q = NULL;
		uint32_t *reg_addr;
		rc = al_udma_q_handle_get(udma, i, &dma_q);
		if (rc != 0) {
			al_assert_msg(!rc, "%s : Failed at al_udma_q_handle_get (rc = %d)\n",
				__func__, rc);
			return;
		}

		reg_addr = &dma_q->q_regs->s2m_q.cfg;
		if (!state && al_reg_read32(reg_addr) & UDMA_S2M_Q_CFG_EN_STREAM)
			*en_queues_mask |= (1 << i);

		al_reg_write32_masked(reg_addr, UDMA_S2M_Q_CFG_EN_STREAM,
				      state && ((*en_queues_mask) & (1 << i)) ?
				      UDMA_S2M_Q_CFG_EN_STREAM : 0);
	}
}

/**
 * Determine if stream is enabled or disbled (flushing new packets)
 */
static al_bool al_udma_s2m_stream_status_get(struct al_udma *udma)
{
	unsigned int i;
	uint32_t stream_cfg = 0;
	int rc = 0;
	al_bool queue_stream_status;
	al_bool queue_stream_status_valid = AL_FALSE;
	al_bool stream_status = AL_TRUE;

	al_assert(udma);
	al_assert(udma->type == UDMA_RX);

	stream_cfg = al_reg_read32(&udma->udma_regs->s2m.s2m.stream_cfg);
	stream_cfg &= AL_UDMA_S2M_STREAM_FLUSH;

	if (stream_cfg == AL_UDMA_S2M_STREAM_FLUSH)
		stream_status = AL_FALSE; /** stream is disabled */

	queue_stream_status = AL_FALSE;
	for (i = 0; i < udma->num_of_queues; i++) {
		struct al_udma_q *dma_q = NULL;
		uint32_t *reg_addr;
		uint32_t reg_val;

		rc = al_udma_q_handle_get(udma, i, &dma_q);
		if (rc != 0)
			al_err("%s : Failed at al_udma_q_handle_get (rc = %d)\n", __func__, rc);

		if (!al_udma_q_is_enabled(dma_q))
			continue;

		queue_stream_status_valid = AL_TRUE;

		reg_addr = &dma_q->q_regs->s2m_q.cfg;
		reg_val = al_reg_read32(reg_addr);

		/** Need all queues stream interface disabled */
		if (reg_val & UDMA_S2M_Q_CFG_EN_STREAM) {
			queue_stream_status = AL_TRUE;
			break;
		}
	}

	if (queue_stream_status_valid && (queue_stream_status != stream_status)) {
		al_warn("%s: Bad configurations, stream & queue stream interface status are"
			" different! assuming strem is enabled\n",
			__func__);
		/** Need both interfaces disabled to stop stream */
		return AL_TRUE;
	}

	return stream_status;
}

static enum al_udma_state udma_attr_hw_state_to_udma_state(uint32_t attr_hw_state)
{
	enum al_udma_state result = UDMA_DISABLE;

	if (attr_hw_state == UDMA_STATE_IDLE)
		result = UDMA_IDLE;
	else if (attr_hw_state == UDMA_STATE_NORMAL)
		result = UDMA_NORMAL;
	else if (attr_hw_state == UDMA_STATE_ABORT)
		result = UDMA_ABORT;
	else
		al_assert_msg(0, "Unexpected UDMA attribute hw state(%d)\n", attr_hw_state);

	return result;
}

/*
 * return the current UDMA hardware state advanced
 */
int al_udma_state_get_adv(struct al_udma *udma,
	struct al_udma_state_adv *udma_state_adv)
{
	uint32_t state_reg;

	if (udma->type == UDMA_TX) {
		state_reg = al_reg_read32(&udma->udma_regs->m2s.m2s.state);
		udma_state_adv->stream_enabled = AL_TRUE;
	} else {
		state_reg = al_reg_read32(&udma->udma_regs->s2m.s2m.state);
		udma_state_adv->stream_enabled = al_udma_s2m_stream_status_get(udma);
	}

	udma_state_adv->comp_ctrl = udma_attr_hw_state_to_udma_state(
		AL_REG_FIELD_GET(state_reg,
				UDMA_M2S_STATE_COMP_CTRL_MASK,
				UDMA_M2S_STATE_COMP_CTRL_SHIFT));
	udma_state_adv->stream_if = udma_attr_hw_state_to_udma_state(
		AL_REG_FIELD_GET(state_reg,
				UDMA_M2S_STATE_STREAM_IF_MASK,
				UDMA_M2S_STATE_STREAM_IF_SHIFT));
	udma_state_adv->data_rd = udma_attr_hw_state_to_udma_state(
		AL_REG_FIELD_GET(state_reg,
				UDMA_M2S_STATE_DATA_RD_CTRL_MASK,
				UDMA_M2S_STATE_DATA_RD_CTRL_SHIFT));
	udma_state_adv->desc_pref = udma_attr_hw_state_to_udma_state(
		AL_REG_FIELD_GET(state_reg,
				UDMA_M2S_STATE_DESC_PREF_MASK,
				UDMA_M2S_STATE_DESC_PREF_SHIFT));

	return 0;
}

/*
 * return the current UDMA hardware state
 */
enum al_udma_state al_udma_state_get(struct al_udma *udma)
{
	struct al_udma_state_adv udma_state_adv;

	enum al_udma_state comp_ctrl;
	enum al_udma_state stream_if;
	enum al_udma_state data_rd;
	enum al_udma_state desc_pref;
	al_bool stream_enabled;

	al_udma_state_get_adv(udma, &udma_state_adv);

	comp_ctrl = udma_state_adv.comp_ctrl;
	stream_if = udma_state_adv.stream_if;
	data_rd = udma_state_adv.data_rd;
	desc_pref = udma_state_adv.desc_pref;
	stream_enabled = udma_state_adv.stream_enabled;

	/**
	 * Due to a HW bug, in case stream is disabled but there are packets waiting to enter
	 * the UDMA, the stream_if might be "stuck" at "1"
	 *
	 * We can ignore the stream_if indication if :
	 * 1. The stream is disabled for UDMA and for each queue -
	 *    new packets cannot enter the UDMA or its queues.
	 * 3. We waited at least 1us for the FIFO's to be emptied
	 */

	/* if any of the states is abort then return abort */
	if (stream_enabled) {
		if ((comp_ctrl == UDMA_ABORT) || (stream_if == UDMA_ABORT) ||
			(data_rd == UDMA_ABORT) || (desc_pref == UDMA_ABORT))
				return UDMA_ABORT;
	} else {
		if ((comp_ctrl == UDMA_ABORT) || (desc_pref == UDMA_ABORT) ||
			(data_rd == UDMA_ABORT))
				return UDMA_ABORT;
	}

	if (stream_enabled) {
		/* if any of the states is normal then return normal */
		if ((comp_ctrl == UDMA_NORMAL) || (stream_if == UDMA_NORMAL) ||
			(data_rd == UDMA_NORMAL) || (desc_pref == UDMA_NORMAL))
				return UDMA_NORMAL;
	} else {
		if ((comp_ctrl == UDMA_NORMAL) || (desc_pref == UDMA_NORMAL) ||
			(data_rd == UDMA_NORMAL))
				return UDMA_NORMAL;
	}

	return UDMA_IDLE;
}

int al_udma_state_set_wait(struct al_udma *dma, enum al_udma_state new_state, al_bool flush_stream)
{
	enum al_udma_state state;
	enum al_udma_state expected_state = new_state;
	uint32_t en_queues_mask = 0;
	int count = 1000;
	int rc;

	/**
	 * When we wish to move to DISABLE or NORMAL state & the UDMA can receive traffic,
	 * we might never change state & the UDMA will be busy processing new requests.
	 * Therefore we perform the following operations :
	 * 1. disable the stream of the UDMA & stream enable of each queue
	 * 2. wait untill all FIFO's are emptied - 1us is enough.
	 * 3. Issue FIFO's flush for "stuck" packets - This can occur if there are "garbage" packets
	 *    For example : if the EC hasn't been initialized but transferred traffic
	 */
	if (flush_stream) {
		al_assert(dma->type == UDMA_RX);
		al_assert((new_state == UDMA_DISABLE) || (new_state == UDMA_NORMAL));

		/** stop stream */
		udma_s2m_stream_set_mode(dma, AL_FALSE);
		udma_s2m_stream_queues_set_mode(dma, AL_FALSE, &en_queues_mask);
		al_udelay(1);					/** wait until empty */
		al_udma_s2m_fifos_flush(dma);			/** flush remaining packets */
	}

	rc = al_udma_state_set(dma, new_state);
	if (rc != 0) {
		al_warn("[%s] warn: failed to change state, error %d\n", dma->name, rc);
		return rc;
	}

	if ((new_state == UDMA_NORMAL) || (new_state == UDMA_DISABLE))
		expected_state = UDMA_IDLE;

	do {
		state = al_udma_state_get(dma);
		if (state == expected_state)
			break;
		al_udelay(1);
		if (count-- == 0) {
			al_warn("[%s] warn: dma state didn't change to %s\n",
				 dma->name, al_udma_states_name[new_state]);
			return -ETIMEDOUT;
		}
	} while (1);

	if (flush_stream) {
		if ((new_state == UDMA_NORMAL) && (dma->rev_id >= AL_UDMA_REV_ID_4)) {
			al_udma_ttt_default_config(dma); /** Configure TTT table */
#ifdef AL_ETH_EX
			al_udma_ex_ttt_default_config(dma);
#endif
		}

		/** Enable the stream */
		udma_s2m_stream_queues_set_mode(dma, AL_TRUE, &en_queues_mask);
		udma_s2m_stream_set_mode(dma, AL_TRUE);
	}

	return 0;
}

/*
 * Action handling
 */

/*
 * get next completed packet from completion ring of the queue
 *
 * This implementation assumes that udma drop mode is set - if it is removed we should remove
 * the marked code segment as its not needed.
 */
uint32_t al_udma_cdesc_packet_get(
	struct al_udma_q		*udma_q,
	volatile union al_udma_cdesc	**cdesc)
{
	uint32_t count;
	volatile union al_udma_cdesc *curr;
	uint32_t comp_flags;

	/* this function requires the completion ring update */
	al_assert(udma_q->cdesc_base_ptr);

	/* comp_head points to the last comp desc that was processed */
	curr = udma_q->comp_head_ptr;
	comp_flags = swap32_from_le(curr->al_desc_comp_tx.ctrl_meta);

	/* check if the completion descriptor is new */
	if (unlikely(al_udma_new_cdesc(udma_q, comp_flags) == AL_FALSE))
		return 0;

	/**
	 * If continue to process a packet, check it was not "trimmed" by udma drop
	 * i.e, making sure processed desc is not a "first" desc
	 */
	if (unlikely(udma_q->pkt_crnt_descs && cdesc_is_first(comp_flags))) {
		uint32_t last_desc =
			(udma_q->next_cdesc_idx + udma_q->pkt_crnt_descs - 1) & (udma_q->size_mask);
		curr = al_udma_cdesc_idx_to_ptr(udma_q, last_desc);
		curr->al_desc_comp_rx.ctrl_meta |=
			swap32_to_le(AL_S2M_DESC_ERR | AL_UDMA_CDESC_LAST);
		count = udma_q->pkt_crnt_descs;
		/**
		 * rewinding the cdesc sw ptr, therefore needs to rewind sw comp ring id as well
		 * (effectively it will only change upon queue wrap around, scenraio detailed below)
		 */
		udma_q->comp_ring_id = al_udma_cdesc_ring_id_get(swap32_from_le(curr->al_desc_comp_rx.ctrl_meta));
		goto done;
	}

	/* if new desc found, increment the current packets descriptors */
	count = udma_q->pkt_crnt_descs + 1;

	/**
	 * Stopping condition is end of packet - indication of last descriptor
	 */
	while (!cdesc_is_last(comp_flags)) {
		curr = al_cdesc_next_update(udma_q, curr);
		comp_flags = swap32_from_le(curr->al_desc_comp_tx.ctrl_meta);
		if (unlikely(al_udma_new_cdesc(udma_q, comp_flags)
								== AL_FALSE)) {
			/* the current packet here doesn't have all  */
			/* descriptors completed. log the current desc */
			/* location and number of completed descriptors so */
			/*  far. then return */
			udma_q->pkt_crnt_descs = count;
			udma_q->comp_head_ptr = curr;
			return 0;
		}

		/**
		 * If udma drop mode is on (call to al_udma_s2m_no_desc_cfg_set())
		 * "middle/last" descs can be dropped.
		 * A possible error scenarios is :
		 * F, M, M, F, L : In that case we will pass on 2 packets as one.
		 * F, F , F, FL     : Pass 4 packets as one.
		 *
		 * Therefore we should make sure we are not processing a new "F/FL" desc.
		 */
		if (unlikely(cdesc_is_first(comp_flags))) {
		/*
		 * Return to actual last desc and mark it as last with L2 err, then stop
		 * F, M, M, F => last desc is at : first + 2 => first + (count - 1)
		 * F, F       => last desc is at : first + 0
		 * count >= 1
		 *
		 * A special case is when there is a drop exactly at a queue wrap around :
		 * CURR    [              *  ]
		 * IDX     [0  1  2 .... SZ-1]
		 * FLAG    [F  FL FL      F  ]
		 * Ring ID [2  1  1 ....  1  ]
		 *
		 * Once SW ends processing the cdesc at the end of the queue it looks at the next cdesc with
		 * al_cdesc_next_update() which increments the SW ring ID (udma_q->comp_ring_id) due to wrap around.
		 * Curr desc has a F flag so we identify a drop and return to current packet last cdesc at end of queue
		 * Therefore we need to rewind the ring ID as well.
		 *
		 * If it is not rewinded, then at the end of function when we prepare for handling of the next packet
		 * We will cause another queue wrap around with al_cdesc_next_update() therefore the SW ring ID will be
		 * incremented twice causing loss of ring ID sync between HW and SW and leaving the queue stuck
		 */
			uint32_t last_desc =
				(udma_q->next_cdesc_idx + count - 1) & (udma_q->size_mask);
			curr = al_udma_cdesc_idx_to_ptr(udma_q, last_desc);
			curr->al_desc_comp_rx.ctrl_meta |= swap32_to_le(
				AL_S2M_DESC_ERR | AL_UDMA_CDESC_LAST);
			udma_q->comp_ring_id =
				al_udma_cdesc_ring_id_get(swap32_from_le(curr->al_desc_comp_rx.ctrl_meta));
			break;
		}

		count++;
		/* check against max descs per packet. */
		al_assert(count <= udma_q->size);
	}

done:
	/* return back the first descriptor of the packet */
	*cdesc = al_udma_cdesc_idx_to_ptr(udma_q, udma_q->next_cdesc_idx);
	udma_q->pkt_crnt_descs = 0;
	udma_q->comp_head_ptr = al_cdesc_next_update(udma_q, curr);

	al_dbg("udma [%s %d]: packet completed. first desc %p (ixd 0x%x)"
		 " descs %d\n", udma_q->udma->name, udma_q->qid, *cdesc,
		 udma_q->next_cdesc_idx, count);

	return count;
}

void al_udma_mailbox_read(struct al_udma *udma, unsigned int mailbox_id, uint32_t *val)
{
	uint32_t *reg;

	if (udma->rev_id < AL_UDMA_REV_ID_4) {
		struct udma_gen_regs_v3 *udma_gen_regs = udma->gen_regs;

		al_assert(mailbox_id < AL_ARR_SIZE(udma_gen_regs->mailbox));

		reg = &udma_gen_regs->mailbox[mailbox_id].msg_in;
	} else {
		/* v4+ */
		struct udma_gen_regs_v6 *udma_gen_regs = udma->gen_regs;

		al_assert(mailbox_id < AL_ARR_SIZE(udma_gen_regs->mailbox));

		reg = &udma_gen_regs->mailbox[mailbox_id].msg_in;
	}

	*val = al_reg_read32(reg);
}

void al_udma_mailbox_write(struct al_udma *udma, unsigned int mailbox_id, uint32_t val)
{
	uint32_t *reg;

	if (udma->rev_id < AL_UDMA_REV_ID_4) {
		struct udma_gen_regs_v3 *udma_gen_regs = udma->gen_regs;

		al_assert(mailbox_id < AL_ARR_SIZE(udma_gen_regs->mailbox));

		reg = &udma_gen_regs->mailbox[mailbox_id].msg_out;
	} else {
		/* v4+ */
		struct udma_gen_regs_v6 *udma_gen_regs = udma->gen_regs;

		al_assert(mailbox_id < AL_ARR_SIZE(udma_gen_regs->mailbox));

		reg = &udma_gen_regs->mailbox[mailbox_id].msg_out;
	}

	al_reg_write32(reg, val);
}

/*
 * Live migration
 */

void al_udma_q_hw_state_save(const struct al_udma_q *udma_q,
	struct al_udma_q_hw_state *q_state)
{
	uint32_t reg_1;
	uint32_t reg_2;
	uint32_t *reg_addr;

	al_assert(udma_q);
	al_assert(q_state);

	al_memset(q_state, 0, sizeof(struct al_udma_q_hw_state));

	/**
	 *  Static state save
	 */
	q_state->static_params.ring_size = al_reg_read32(&udma_q->q_regs->rings.drl);

	/* Completion descriptor size */
	if (udma_q->udma->type == UDMA_RX) {
		reg_1 = al_reg_read32(&udma_q->udma->udma_regs->s2m.s2m_comp.cfg_1c);
		q_state->static_params.cdesc_size = AL_REG_FIELD_GET(reg_1,
							UDMA_S2M_COMP_CFG_1C_DESC_SIZE_MASK,
							UDMA_S2M_COMP_CFG_1C_DESC_SIZE_SHIFT) << 2;
	}

	if (udma_q->udma->type == UDMA_TX)
		reg_addr = &udma_q->q_regs->m2s_q.comp_cfg;
	else
		reg_addr = &udma_q->q_regs->s2m_q.comp_cfg;

	q_state->static_params.comp_cfg = al_reg_read32(reg_addr);

	if (q_state->static_params.comp_cfg & UDMA_M2S_Q_COMP_CFG_EN_COMP_RING_UPDATE) {
		reg_1 = al_reg_read32(&udma_q->q_regs->rings.crbp_low);
		reg_2 = al_reg_read32(&udma_q->q_regs->rings.crbp_high);
		q_state->static_params.cdesc_phy_base = ((al_phys_addr_t)reg_2 << 32) | reg_1;
	}


	reg_1 = al_reg_read32(&udma_q->q_regs->rings.drbp_low);
	reg_2 = al_reg_read32(&udma_q->q_regs->rings.drbp_high);

	q_state->static_params.desc_phy_base = ((al_phys_addr_t)reg_2 << 32) | reg_1;

	/* TODO: consider storing reflection register like agents config (also for the global config) */

	/**
	 *  Dynamic state save
	 */
	reg_1 = al_reg_read32(&udma_q->q_regs->rings.crhp);
	reg_2 = al_reg_read32(&udma_q->q_regs->rings.drtp);

	if (udma_q->udma->type == UDMA_TX) {
		q_state->dynamic_params.head_idx =
			AL_REG_FIELD_GET(reg_1, UDMA_M2S_Q_TCRHP_OFFSET_MASK,
			UDMA_M2S_Q_TCRHP_OFFSET_SHIFT);
		q_state->dynamic_params.comp_ring_id =
			AL_REG_FIELD_GET(reg_1, UDMA_M2S_Q_TCRHP_RING_ID_MASK,
			UDMA_M2S_Q_TCRHP_RING_ID_SHIFT);
		q_state->dynamic_params.tail_idx =
			AL_REG_FIELD_GET(reg_2, UDMA_M2S_Q_TDRTP_OFFSET_MASK,
			UDMA_M2S_Q_TDRTP_OFFSET_SHIFT);
		q_state->dynamic_params.desc_ring_id =
			AL_REG_FIELD_GET(reg_2, UDMA_M2S_Q_TDRTP_RING_ID_MASK,
			UDMA_M2S_Q_TDRTP_RING_ID_SHIFT);
	} else {
		q_state->dynamic_params.head_idx =
			AL_REG_FIELD_GET(reg_1, UDMA_S2M_Q_RCRHP_OFFSET_MASK,
			UDMA_S2M_Q_RCRHP_OFFSET_SHIFT);
		q_state->dynamic_params.comp_ring_id =
			AL_REG_FIELD_GET(reg_1, UDMA_S2M_Q_RCRHP_RING_ID_MASK,
			UDMA_S2M_Q_RCRHP_RING_ID_SHIFT);
		q_state->dynamic_params.tail_idx =
			AL_REG_FIELD_GET(reg_2, UDMA_S2M_Q_RDRTP_OFFSET_MASK,
			UDMA_S2M_Q_RDRTP_OFFSET_SHIFT);
		q_state->dynamic_params.desc_ring_id =
			AL_REG_FIELD_GET(reg_2, UDMA_S2M_Q_RDRTP_RING_ID_MASK,
			UDMA_S2M_Q_RDRTP_RING_ID_SHIFT);
	}
}

void al_udma_hw_state_save(const struct al_udma *dma,
	struct al_udma_hw_state *hw_state_out)
{
	unsigned int i;

	al_assert(dma);
	al_assert(hw_state_out);

	for (i = 0; i < dma->num_of_queues; i++)
		al_udma_q_hw_state_save(&dma->udma_q[i], &hw_state_out->q_state[i]);

}

int al_udma_q_hw_state_load(struct al_udma_q *udma_q,
	const struct al_udma_q_hw_state *q_state)
{
	al_assert(udma_q);
	al_assert(q_state);

	al_udma_q_hw_init(udma_q, &q_state->static_params);
	al_udma_q_ring_params_set(udma_q, &q_state->dynamic_params);

	return 0;
}

int al_udma_hw_state_load(struct al_udma *dma,
	const struct al_udma_hw_state *hw_state_in)
{
	unsigned int i;

	al_assert(dma);
	al_assert(hw_state_in);

	for (i = 0; i < dma->num_of_queues; i++)
		al_udma_q_hw_state_load(&dma->udma_q[i], &hw_state_in->q_state[i]);

	return 0;
}

void al_udma_q_copy(const struct al_udma_q *src_q,
	struct al_udma *dst_dma, unsigned int dst_qid)
{
	struct al_udma_q *dst_q;

	al_assert(src_q);
	al_assert(dst_dma);
	al_assert(dst_qid < dst_dma->num_of_queues);

	dst_q = &(dst_dma->udma_q[dst_qid]);

	al_assert_msg((src_q->status == AL_QUEUE_DISABLED) && (dst_q->status == AL_QUEUE_DISABLED),
		"Source and destination queues must be disabled\n");


	al_memcpy(dst_q, src_q, sizeof(struct al_udma_q));

	/* Parameters fixup */
	dst_q->udma = dst_dma;
	dst_q->qid = dst_qid;
}

int al_udma_reflect_set_timer(struct al_udma *udma,
	uint32_t update_cycle_time_nsec)
{
	struct udma_gen_reflect __iomem *reflect_rba = NULL;

	al_assert(udma);

	/* the status reflection feature is available starting rev 6 */
	al_assert(udma->rev_id >= AL_UDMA_REV_ID_6);

	/* pointer to the global reflection RBA */
	reflect_rba = &((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->reflect;

	/* set the timer */
	al_udma_reflection_timer_set(reflect_rba, update_cycle_time_nsec);

	return 0;
}

static int al_udma_reflect_app_status_agent_control(struct al_udma *udma,
					al_bool enable_agent)
{
	struct udma_gen_reflect __iomem *reflect_rba = NULL;
	uint32_t register_val = 0;

	al_assert(udma);

	/* the status reflection feature is available starting rev 6 */
	al_assert(udma->rev_id >= AL_UDMA_REV_ID_6);

	/* pointer to the global reflection RBA */
	reflect_rba = &((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->reflect;

	if (enable_agent) {

		/*
		 * the app status agent shall work in bypass mode only
		 * this mode triggers DRAM update whenever the relevant
		 * app status bits of a queue are detected using its mask.
		 */
		register_val = UDMA_GEN_REFLECT_TX_HEAD_AGENT_ENABLE |
				UDMA_GEN_REFLECT_TX_HEAD_AGENT_BYPASS_TIMER_EN;
	}

	al_reg_write32_masked(&reflect_rba->app_stat_agent,
			UDMA_GEN_REFLECT_APP_STAT_AGENT_ENABLE_MASK |
			UDMA_GEN_REFLECT_APP_STAT_AGENT_BYPASS_TIMER_EN_MASK,
			register_val);
	return 0;
}

int al_udma_reflect_app_status_agent_enable(struct al_udma *udma)
{
	return al_udma_reflect_app_status_agent_control(udma, AL_TRUE);
}

int al_udma_reflect_app_status_agent_disable(struct al_udma *udma)
{
	return al_udma_reflect_app_status_agent_control(udma, AL_FALSE);
}

int al_udma_reflect_config_app_status_agent(struct al_udma_q *udma_q,
		struct al_udma_reflect_app_status_agent *app_status_agent)
{
	struct udma_m2s_q_reflect __iomem *q_reflect_rba;

	al_assert(udma_q);
	al_assert(app_status_agent);

	/* the status reflection feature is available starting rev 6 */
	al_assert(udma_q->udma->rev_id >= AL_UDMA_REV_ID_6);

	/* pointer to the queue reflection RBA */
	q_reflect_rba =
		&((struct unit_regs_v6 __iomem *)udma_q->udma->unit_regs_base)->m2s.m2s_q[udma_q->qid].reflect;

	al_reg_write32(&q_reflect_rba->app_status_mask, app_status_agent->app_status_mask);

	return 0;
}

static void al_udma_reflect_update_agent_mask_reg(uint32_t  __iomem *match_reg,
			uint32_t  __iomem *match_mask_reg,
			uint32_t  __iomem *error_mask_reg,
			uint32_t match,
			uint32_t match_mask,
			uint32_t error_mask)
{
	/* entire register writing as we need all 32 bits */
	al_reg_write32(match_reg, match);
	al_reg_write32(match_mask_reg, match_mask);
	al_reg_write32(error_mask_reg, error_mask);
}

static void al_udma_reflect_update_agent_mask(
		struct udma_gen_reflect __iomem *reflect_rba,
		struct al_udma_rx_error_detection_agent *rx_err_agent,
		struct al_udma_tx_error_detection_agent *tx_err_agent,
		uint16_t reg_idx,
		uint16_t mask_index,
		uint16_t mask_reg_idx,
		al_bool tx)
{
	if (tx) {
		al_udma_reflect_update_agent_mask_reg(
			&reflect_rba->tx_cmpl_agent.error_detect[reg_idx].match,
			&reflect_rba->tx_cmpl_agent.error_detect[reg_idx].match_mask,
			&reflect_rba->tx_cmpl_agent.error_detect[reg_idx].error_mask,
			tx_err_agent->error_detect[mask_index].match[mask_reg_idx],
			tx_err_agent->error_detect[mask_index].match_mask[mask_reg_idx],
			tx_err_agent->error_detect[mask_index].error_mask[mask_reg_idx]);
	} else {
		al_udma_reflect_update_agent_mask_reg(
			&reflect_rba->rx_cmpl_agent.error_detect[reg_idx].match,
			&reflect_rba->rx_cmpl_agent.error_detect[reg_idx].match_mask,
			&reflect_rba->rx_cmpl_agent.error_detect[reg_idx].error_mask,
			rx_err_agent->error_detect[mask_index].match[mask_reg_idx],
			rx_err_agent->error_detect[mask_index].match_mask[mask_reg_idx],
			rx_err_agent->error_detect[mask_index].error_mask[mask_reg_idx]);
	}
}

static int al_udma_reflect_config_err_agent(struct udma_gen_reflect __iomem *reflect_rba,
	struct al_udma_rx_error_detection_agent *rx_err_agent,
	struct al_udma_tx_error_detection_agent *tx_err_agent,
	uint32_t number_of_masks,
	uint32_t number_of_register_per_mask,
	al_bool tx)
{
	uint8_t i = 0;

	/* we shall enable the agent if the corresponding config was set */
	const al_bool enable_agent = ((tx && tx_err_agent) || (!tx && rx_err_agent));

	/* pointer to the agent enable register */
	uint32_t __iomem *enable_reg = (tx ?
		&reflect_rba->tx_cmpl_agent.enable :
		&reflect_rba->rx_cmpl_agent.enable);

	/* disable the agent if required to */
	if (!enable_agent) {

		/* we asserted that rx and tx registers have the same masks */
		al_reg_write32_masked(enable_reg,
			UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_ENABLE_MASK,
			0);

		return 0;
	}

	/* for each mask */
	for ( ; i < number_of_masks ; i++) {

		uint8_t j = 0;

		/* for each mask register */
		for ( ; j < number_of_register_per_mask ; j++) {

			/*
			 * while j is the register index inside the mask in user's struct,
			 * reg_idx is the index in the reg_file which is ordered a little
			 * diffrently.
			 */
			const uint8_t reg_idx =
				((i * number_of_register_per_mask) + j);

			al_udma_reflect_update_agent_mask(
					reflect_rba,
					rx_err_agent,
					tx_err_agent,
					reg_idx,
					i,
					j,
					tx);
		}
	}

	/*
	 * Finally, enable the agent.
	 * According to the requirements, this agent shall
	 * be configured to work in bypass mode always.
	 * it means direct DRAM update on every detected error.
	 */
	al_reg_write32_masked(enable_reg,
				UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_ENABLE_MASK |
				UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN_MASK,
				UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_ENABLE |
				UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN);
	return 0;
}

al_static_assert(UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_ENABLE_MASK ==
		 UDMA_GEN_REFLECT_TX_CMPL_AGENT_ENABLE_ENABLE_MASK,
		 "mask mismatch. above function assumes it");

al_static_assert(UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN_MASK ==
		 UDMA_GEN_REFLECT_TX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN_MASK,
		 "mask mismatch. above function assumes it");

al_static_assert(UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_ENABLE ==
		 UDMA_GEN_REFLECT_TX_CMPL_AGENT_ENABLE_ENABLE,
		 "value mismatch. above function assumes it");

al_static_assert(UDMA_GEN_REFLECT_RX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN ==
		 UDMA_GEN_REFLECT_TX_CMPL_AGENT_ENABLE_BYPASS_TIMER_EN,
		 "value mismatch. above function assumes it");

int al_udma_reflect_config_rx_err_agent(struct al_udma *udma,
	struct al_udma_rx_error_detection_agent *rx_err_agent)
{
	struct udma_gen_reflect __iomem *reflect_rba = NULL;

	/* this is used to make sure we don't exceed error_detect array boundaries */
	al_static_assert((AL_UDMA_REFLECT_ERR_DETECT_NUM *
			  AL_UDMA_REFLECT_RX_ERR_DETECT_MASK_SIZE) ==
		((sizeof(reflect_rba->rx_cmpl_agent.error_detect) /
		  sizeof(struct udma_gen_reflect_rx_cmpl_agent_error_detect))),
		"mask number mismatch");

	/* the status reflection feature is available starting rev 6 */
	if (!udma || udma->rev_id < AL_UDMA_REV_ID_6)
		return -1;

	reflect_rba = &((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->reflect;

	return al_udma_reflect_config_err_agent(
			reflect_rba,
			rx_err_agent,
			NULL,
			AL_UDMA_REFLECT_ERR_DETECT_NUM,
			AL_UDMA_REFLECT_RX_ERR_DETECT_MASK_SIZE,
			AL_FALSE);
}

int al_udma_reflect_config_tx_err_agent(struct al_udma *udma,
	struct al_udma_tx_error_detection_agent *tx_err_agent)
{
	struct udma_gen_reflect __iomem *reflect_rba = NULL;

	/* this is used to make sure we don't exceed error_detect array boundaries */
	al_static_assert((AL_UDMA_REFLECT_ERR_DETECT_NUM *
			  AL_UDMA_REFLECT_TX_ERR_DETECT_MASK_SIZE) ==
		((sizeof(reflect_rba->tx_cmpl_agent.error_detect) /
		  sizeof(struct udma_gen_reflect_tx_cmpl_agent_error_detect))),
		"mask number mismatch");

	/* the status reflection feature is available starting rev 6 */
	if (!udma || udma->rev_id < AL_UDMA_REV_ID_6)
		return -1;

	reflect_rba = &((struct udma_gen_regs_v6 __iomem *)udma->gen_regs)->reflect;

	return al_udma_reflect_config_err_agent(
			reflect_rba,
			NULL,
			tx_err_agent,
			AL_UDMA_REFLECT_ERR_DETECT_NUM,
			AL_UDMA_REFLECT_TX_ERR_DETECT_MASK_SIZE,
			AL_TRUE);
}

int al_hal_udma_q_set_fill_level_thresholds(
	struct al_udma_q	*udma_q,
	uint8_t			preset_index,
	struct al_hal_udma_q_fill_level_thresholds *fill_level_thresholds)
{
	struct udma_s2m_q_fill_th *q_thresholds = NULL;

	/* fill level advertising is available for v6 or higher */
	if (udma_q->udma->rev_id < AL_UDMA_REV_ID_6)
		return -1;

	/* input validation */
	al_assert(udma_q);
	al_assert(fill_level_thresholds);
	al_assert(preset_index < AL_UDMA_FILL_LEVELS_PER_QUEUE);

	/* make sure that fill levels overlap */
	al_assert(fill_level_thresholds->level_a.low < fill_level_thresholds->level_a.high);
	al_assert(fill_level_thresholds->level_a.high < fill_level_thresholds->level_b.low);
	al_assert(fill_level_thresholds->level_b.low < fill_level_thresholds->level_b.high);
	al_assert(fill_level_thresholds->level_b.high < fill_level_thresholds->level_c.low);
	al_assert(fill_level_thresholds->level_c.low < fill_level_thresholds->level_c.high);

	/* get the address of the fill thresholds RBA */
	q_thresholds = &udma_q->udma->udma_regs->s2m.s2m_q[udma_q->qid].fill_th[preset_index];

	al_reg_write32_masked(&q_thresholds->level_a_low,
			UDMA_S2M_Q_FILL_TH_LEVEL_A_LOW_VAL_MASK,
			(fill_level_thresholds->level_a.low << UDMA_S2M_Q_FILL_TH_LEVEL_A_LOW_VAL_SHIFT));

	al_reg_write32_masked(&q_thresholds->level_a_high,
			UDMA_S2M_Q_FILL_TH_LEVEL_A_HIGH_VAL_MASK,
			(fill_level_thresholds->level_a.high << UDMA_S2M_Q_FILL_TH_LEVEL_A_HIGH_VAL_SHIFT));

	al_reg_write32_masked(&q_thresholds->level_b_low,
			UDMA_S2M_Q_FILL_TH_LEVEL_B_LOW_VAL_MASK,
			(fill_level_thresholds->level_b.low << UDMA_S2M_Q_FILL_TH_LEVEL_B_LOW_VAL_SHIFT));

	al_reg_write32_masked(&q_thresholds->level_b_high,
			UDMA_S2M_Q_FILL_TH_LEVEL_B_HIGH_VAL_MASK,
			(fill_level_thresholds->level_b.high << UDMA_S2M_Q_FILL_TH_LEVEL_B_HIGH_VAL_SHIFT));

	al_reg_write32_masked(&q_thresholds->level_c_low,
			UDMA_S2M_Q_FILL_TH_LEVEL_C_LOW_VAL_MASK,
			(fill_level_thresholds->level_c.low << UDMA_S2M_Q_FILL_TH_LEVEL_C_LOW_VAL_SHIFT));

	al_reg_write32_masked(&q_thresholds->level_c_high,
			UDMA_S2M_Q_FILL_TH_LEVEL_C_HIGH_VAL_MASK,
			(fill_level_thresholds->level_c.high << UDMA_S2M_Q_FILL_TH_LEVEL_C_HIGH_VAL_SHIFT));

	return 0;
}

/** @} end of UDMA group */
