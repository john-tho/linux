/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 *  @{
 * @file   al_hal_udma_regs_internal.h
 *
 * @brief udma internal registers - this file was created manually based on the auto generator UDMA regs file.
 * For future UDMA rev-id, register struct should be added to this file under (regs_type)_v(rev-id)
 *
 */

#ifndef __AL_HAL_UDMA_REGS_INTERNAL_H__
#define __AL_HAL_UDMA_REGS_INTERNAL_H__

#include "al_hal_common.h"
#include "al_hal_udma_iofic_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

struct udma_gen_sram_ctrl {
	/* [0x0] Timing configuration */
	uint32_t timing;
};

/**** sram_ctrl (timing) register ****/
/*
 * Write margin
 * Reset: 0x0         Access: RW
 */
#define UDMA_GEN_SRAM_CTRL_TIMING_RMA_MASK 0x0000000F
#define UDMA_GEN_SRAM_CTRL_TIMING_RMA_SHIFT 0
/*
 * Write margin enable
 * Reset: 0x0         Access: RW
 */
#define UDMA_GEN_SRAM_CTRL_TIMING_RMEA   (1 << 8)
/*
 * Read margin
 * Reset: 0x0         Access: RW
 */
#define UDMA_GEN_SRAM_CTRL_TIMING_RMB_MASK 0x000F0000
#define UDMA_GEN_SRAM_CTRL_TIMING_RMB_SHIFT 16
/*
 * Read margin enable
 * Reset: 0x0         Access: RW
 */
#define UDMA_GEN_SRAM_CTRL_TIMING_RMEB   (1 << 24)

/*
 * TGTADDR
 */
struct udma_gen_tgtaddr_v3 {
	/* [0x0] TX queue 0/1 TGTADDR */
	uint32_t cfg_tgtaddr_0;
	/* [0x4] TX queue 2/3 TGTADDR */
	uint32_t cfg_tgtaddr_1;
	/* [0x8] RX queue 0/1 TGTADDR */
	uint32_t cfg_tgtaddr_2;
	/* [0xc] RX queue 2/3 TGTADDR */
	uint32_t cfg_tgtaddr_3;
};

struct udma_gen_tgtid_v3 {
	/* [0x0] Target-ID control */
	uint32_t cfg_tgtid_0;
	/* [0x4] TX queue 0/1 Target-ID - V3 only */
	uint32_t cfg_tgtid_1;
	/* [0x8] TX queue 2/3 Target-ID - V3 only */
	uint32_t cfg_tgtid_2;
	/* [0xc] RX queue 0/1 Target-ID - V3 only */
	uint32_t cfg_tgtid_3;
	/* [0x10] RX queue 2/3 Target-ID - V3 only */
	uint32_t cfg_tgtid_4;
};


/**************************************************************************************************
 * udma regs m2s
 **************************************************************************************************/
struct udma_m2s_regs_v3 {
	uint32_t rsrvd_0[64];
	struct udma_axi_m2s axi_m2s;                     /* [0x100] */
	struct udma_m2s m2s;                             /* [0x200] */
	struct udma_m2s_rd m2s_rd;                       /* [0x300] */
	struct udma_m2s_dwrr m2s_dwrr;                   /* [0x340] */
	struct udma_m2s_rate_limiter m2s_rate_limiter;   /* [0x380] */
	struct udma_m2s_stream_rate_limiter m2s_stream_rate_limiter; /* [0x3c0] */
	struct udma_m2s_comp m2s_comp;                   /* [0x400] */
	struct udma_m2s_stat m2s_stat;                   /* [0x500] */
	struct udma_m2s_feature m2s_feature;             /* [0x600] */
	uint32_t rsrvd_1[576];
	struct udma_m2s_q m2s_q[4];                      /* [0x1000] */
};

struct udma_m2s_regs_v4 {
	uint32_t rsrvd_0[64];
	struct udma_axi_m2s axi_m2s;                     /* [0x100] */
	struct udma_m2s m2s;                             /* [0x200] */
	struct udma_m2s_rd m2s_rd;                       /* [0x300] */
	struct udma_m2s_dwrr m2s_dwrr;                   /* [0x340] */
	struct udma_m2s_rate_limiter m2s_rate_limiter;   /* [0x380] */
	struct udma_m2s_stream_rate_limiter m2s_stream_rate_limiter; /* [0x3c0] */
	struct udma_m2s_comp m2s_comp;                   /* [0x400] */
	struct udma_m2s_stat m2s_stat;                   /* [0x500] */
	struct udma_m2s_feature m2s_feature;             /* [0x600] */
	uint32_t rsrvd_1[576];
	struct udma_m2s_q m2s_q[16];                     /* [0x1000] */
};

struct udma_m2s_regs_v5 {
	uint32_t rsrvd_0[64];
	struct udma_axi_m2s axi_m2s;                     /* [0x100] */
	struct udma_m2s m2s;                             /* [0x200] */
	struct udma_m2s_rd m2s_rd;                       /* [0x300] */
	struct udma_m2s_dwrr m2s_dwrr;                   /* [0x340] */
	struct udma_m2s_rate_limiter m2s_rate_limiter;   /* [0x380] */
	struct udma_m2s_stream_rate_limiter m2s_stream_rate_limiter; /* [0x3c0] */
	struct udma_m2s_comp m2s_comp;                   /* [0x400] */
	struct udma_m2s_stat m2s_stat;                   /* [0x500] */
	struct udma_m2s_feature m2s_feature;             /* [0x600] */
	struct udma_m2s_shadow_access m2s_shadow_access; /* [0x700] */
	uint32_t rsrvd_1[512];
	struct udma_m2s_q m2s_q[16];                     /* [0x1000] */
};

struct udma_m2s_regs_v6 {
	/* [0x0] */
	uint32_t rsrvd_0[64];
	/* [0x100] AXI M2S configuration */
	struct udma_axi_m2s axi_m2s;
	/* [0x200] M2S general configuration */
	struct udma_m2s m2s;
	/* [0x300] M2S descriptor and data read  configuration */
	struct udma_m2s_rd m2s_rd;
	/* [0x340] M2S DWRR scheduler configuration */
	struct udma_m2s_dwrr m2s_dwrr;
	/* [0x380] M2S rate limiter configuration */
	struct udma_m2s_rate_limiter m2s_rate_limiter;
	/* [0x3c0] M2S rate limiter configuration */
	struct udma_m2s_stream_rate_limiter m2s_stream_rate_limiter;
	/* [0x400] M2S completion control configuration */
	struct udma_m2s_comp m2s_comp;
	/* [0x500] M2S statistics */
	struct udma_m2s_stat m2s_stat;
	/* [0x600] M2S Feature registers */
	struct udma_m2s_feature m2s_feature;
	/* [0x700] */
	struct udma_m2s_shadow_access m2s_shadow_access;
	/* [0x800] */
	uint32_t rsrvd_1[512];
	/* [0x1000] */
	struct udma_m2s_q m2s_q[16];
	/* [0x11000] */
	uint32_t rsrvd_2[6144];
	/* [0x17000] Memory controller mapped regfile */
	uint32_t mem_ctrl[1024];
	/* [0x18000] PMU memory mapped regfile */
	uint32_t pmu[4096];
};

/**************************************************************************************************
 * udma regs s2m
 **************************************************************************************************/
struct udma_s2m_regs_v3 {
	uint32_t rsrvd_0[64];
	struct udma_axi_s2m axi_s2m;                     /* [0x100] */
	struct udma_s2m s2m;                             /* [0x200] */
	struct udma_s2m_rd s2m_rd;                       /* [0x300] */
	struct udma_s2m_wr s2m_wr;                       /* [0x340] */
	struct udma_s2m_comp s2m_comp;                   /* [0x380] */
	uint32_t rsrvd_1[80];
	struct udma_s2m_stat s2m_stat;                   /* [0x500] */
	struct udma_s2m_feature s2m_feature;             /* [0x600] */
	uint32_t rsrvd_2[576];
	struct udma_s2m_q s2m_q[4];                      /* [0x1000] */
};

struct udma_s2m_regs_v4 {
	uint32_t rsrvd_0[64];
	struct udma_axi_s2m axi_s2m;                     /* [0x100] */
	struct udma_s2m s2m;                             /* [0x200] */
	struct udma_s2m_rd s2m_rd;                       /* [0x300] */
	struct udma_s2m_wr s2m_wr;                       /* [0x340] */
	struct udma_s2m_comp s2m_comp;                   /* [0x380] */
	uint32_t rsrvd_1[80];
	struct udma_s2m_stat s2m_stat;                   /* [0x500] */
	struct udma_s2m_feature s2m_feature;             /* [0x600] */
	uint32_t rsrvd_2[576];
	struct udma_s2m_q s2m_q[16];                     /* [0x1000] */
};

struct udma_s2m_regs_v5 {
	uint32_t rsrvd_0[64];
	struct udma_axi_s2m axi_s2m;                     /* [0x100] */
	struct udma_s2m s2m;                             /* [0x200] */
	struct udma_s2m_rd s2m_rd;                       /* [0x300] */
	struct udma_s2m_wr s2m_wr;                       /* [0x340] */
	struct udma_s2m_comp s2m_comp;                   /* [0x380] */
	uint32_t rsrvd_1[80];
	struct udma_s2m_stat s2m_stat;                   /* [0x500] */
	struct udma_s2m_feature s2m_feature;             /* [0x600] */
	struct udma_s2m_shadow_access s2m_shadow_access; /* [0x700] */
	uint32_t rsrvd_2[512];
	struct udma_s2m_q s2m_q[16];                     /* [0x1000] */
};

struct udma_s2m_regs_v6 {
	/* [0x0] */
	uint32_t rsrvd_0[64];
	/* [0x100] AXI S2M configuration */
	struct udma_axi_s2m axi_s2m;
	/* [0x200] S2M general configuration */
	struct udma_s2m s2m;
	/* [0x300] S2M descriptor read  configuration */
	struct udma_s2m_rd s2m_rd;
	/* [0x340] S2M data write  configuration */
	struct udma_s2m_wr s2m_wr;
	/* [0x380] S2M completion control configuration */
	struct udma_s2m_comp s2m_comp;
	/* [0x3c0] */
	uint32_t rsrvd_1[80];
	/* [0x500] S2M statistics */
	struct udma_s2m_stat s2m_stat;
	/* [0x600] S2M Feature registers */
	struct udma_s2m_feature s2m_feature;
	/* [0x700] */
	struct udma_s2m_shadow_access s2m_shadow_access;
	/* [0x800] */
	uint32_t rsrvd_2[512];
	/* [0x1000] */
	struct udma_s2m_q s2m_q[16];
};

/**************************************************************************************************
 * udma regs gen
 **************************************************************************************************/
struct udma_gen_regs_v3 {
	struct udma_iofic_regs interrupt_regs;               /* [0x0000] */
	struct udma_gen_dma_misc dma_misc;                   /* [0x2080] */
	struct udma_gen_mailbox mailbox[4];                  /* [0x2180] */
	struct udma_gen_axi axi;                             /* [0x2280] */
	uint32_t rsrvd_0[32];
	struct udma_gen_sram_ctrl sram_ctrl[25];             /* [0x2380] */
	uint32_t rsrvd_1[2];
	struct udma_gen_tgtid_v3 tgtid;                      /* [0x23ec] */
	struct udma_gen_tgtaddr_v3 tgtaddr;                  /* [0x2400] */
	uint32_t rsrvd_2[252];
};

struct udma_gen_regs_v4 {
	struct udma_iofic_regs interrupt_regs;               /* [0x0000] */
	uint32_t rsrvd_0[32];
	struct udma_gen_dma_misc dma_misc;                   /* [0x2100] */
	struct udma_gen_mailbox mailbox[4];                  /* [0x2200] */
	struct udma_gen_axi axi;                             /* [0x2300] */
	uint32_t rsrvd_00[32];
	struct udma_gen_sram_ctrl sram_ctrl[25];             /* [0x2400] */
	uint32_t rsrvd_1[39];
	struct udma_gen_tgtid tgtid;                         /* [0x2500] */
	uint32_t rsrvd_2[63];
	struct udma_gen_tgtaddr tgtaddr[16];                 /* [0x2600] */
	struct udma_gen_axi_outstanding_cnt axi_outstanding_cnt[16]; /* [0x2700] */
	uint32_t rsrvd_3[16];
	struct udma_gen_axi_error_detection_table axi_error_detection_table[7]; /* [0x2900] */
	uint32_t rsrvd_4[16];
	struct udma_gen_axi_error_control axi_error_control[7]; /* [0x2b00] */
	uint32_t rsrvd_5[50];
	struct udma_gen_axi_queue axi_queue[16];             /* [0x2c00] */
	uint32_t rsrvd_6[16];
	uint32_t iofic_base_m2s_desc_rd;                     /* [0x2d00] */
	uint32_t rsrvd_m7[63];
	uint32_t iofic_base_m2s_data_rd;                     /* [0x2e00] */
	uint32_t rsrvd_m8[63];
	uint32_t iofic_base_m2s_cmpl_wr;                     /* [0x2f00] */
	uint32_t rsrvd_m9[63];
	uint32_t iofic_base_s2m_desc_rd;                     /* [0x3000] */
	uint32_t rsrvd_m10[63];
	uint32_t iofic_base_s2m_data_wr;                     /* [0x3100] */
	uint32_t rsrvd_m11[63];
	uint32_t iofic_base_s2m_cmpl_wr;                     /* [0x3200] */
	uint32_t rsrvd_m12[63];
	uint32_t iofic_base_msix;                            /* [0x3300] */
	uint32_t rsrvd_m13[63];
	struct udma_gen_pmu pmu;                             /* [0x3400] */
	uint32_t rsrvd_7[4];
	struct udma_gen_init_memory init_memory;             /* [0x3420] */
	uint32_t rsrvd_8[8];
	struct udma_gen_spare_reg spare_reg;                 /* [0x3460] */
};

struct udma_gen_regs_v5 {
	struct udma_iofic_regs interrupt_regs;               /* [0x0000] */
	uint32_t rsrvd_0[32];
	struct udma_gen_dma_misc dma_misc;                   /* [0x2100] */
	struct udma_gen_mailbox mailbox[4];                  /* [0x2200] */
	struct udma_gen_axi axi;                             /* [0x2300] */
	uint32_t rsrvd_00[32];
	struct udma_gen_sram_ctrl sram_ctrl[25];             /* [0x2400] */
	uint32_t rsrvd_1[39];
	struct udma_gen_tgtid tgtid;                         /* [0x2500] */
	uint32_t rsrvd_2[63];
	struct udma_gen_tgtaddr tgtaddr[16];                 /* [0x2600] */
	struct udma_gen_axi_outstanding_cnt axi_outstanding_cnt[16]; /* [0x2700] */
	uint32_t rsrvd_3[16];
	struct udma_gen_axi_error_detection_table axi_error_detection_table[7]; /* [0x2900] */
	uint32_t rsrvd_4[16];
	struct udma_gen_axi_error_control axi_error_control[7]; /* [0x2b00] */
	uint32_t rsrvd_5[50];
	struct udma_gen_axi_queue axi_queue[16];             /* [0x2c00] */
	uint32_t rsrvd_6[16];
	uint32_t iofic_base_m2s_desc_rd;                     /* [0x2d00] */
	uint32_t rsrvd_m7[63];
	uint32_t iofic_base_m2s_data_rd;                     /* [0x2e00] */
	uint32_t rsrvd_m8[63];
	uint32_t iofic_base_m2s_cmpl_wr;                     /* [0x2f00] */
	uint32_t rsrvd_m9[63];
	uint32_t iofic_base_s2m_desc_rd;                     /* [0x3000] */
	uint32_t rsrvd_m10[63];
	uint32_t iofic_base_s2m_data_wr;                     /* [0x3100] */
	uint32_t rsrvd_m11[63];
	uint32_t iofic_base_s2m_cmpl_wr;                     /* [0x3200] */
	uint32_t rsrvd_m12[63];
	uint32_t iofic_base_msix;                            /* [0x3300] */
	uint32_t rsrvd_m13[63];
	struct udma_gen_pmu pmu;                             /* [0x3400] */
	uint32_t rsrvd_7[4];
	struct udma_gen_init_memory init_memory;             /* [0x3420] */
	uint32_t rsrvd_8[8];
	struct udma_gen_spare_reg spare_reg;                 /* [0x3460] */
	uint32_t rsrvd_9[36];
	struct udma_gen_intc_shadow_access intc_shadow_access; /* [0x3500] */
};

struct udma_gen_regs_v6 {
	/* [0x0000] */
	struct udma_iofic_regs interrupt_regs;
	uint32_t rsrvd_10[32];
	/* [0x2100] AXI configuration */
	struct udma_gen_dma_misc dma_misc;
	/* [0x2200] Mailbox between DMAs */
	struct udma_gen_mailbox mailbox[4];
	/* [0x2300] AXI configuration */
	struct udma_gen_axi axi;
	/* [0x2380] Reflect configuration */
	struct udma_gen_reflect reflect;
	/* [0x24a8] */
	uint32_t rsrvd_0[22];
	/* [0x2500] Target-ID configurations */
	struct udma_gen_tgtid tgtid;
	/* [0x2514] */
	uint32_t rsrvd_1[63];
	/* [0x2600] TGTADDR */
	struct udma_gen_tgtaddr tgtaddr[16];
	/* [0x2700] */
	struct udma_gen_axi_outstanding_cnt axi_outstanding_cnt[16];
	/* [0x28c0] */
	uint32_t rsrvd_2[16];
	/*
	 * [0x2900] Each axi generator has an error detecion table, the Addr to the table is
	 * {axi_parity_error,axi_timeout_error,axi_response_error} Instance 0 - M2S - Desc read
	 * Instance 1 -  M2S - Data read
	 * Instance 2 -  M2s -  Cmpl Write
	 * Instance 3 -  S2M -  Desc read
	 * Instance 4 - S2M -   Data Write
	 * Instance 5 - S2M -  Cmpl Write
	 * Instance 6 - MSIX
	 */
	struct udma_gen_axi_error_detection_table axi_error_detection_table[7];
	/* [0x2ac0] */
	uint32_t rsrvd_3[16];
	/*
	 * [0x2b00] Each axi generator has an error control table, the Addr to the table is
	 * {queue_number,error_type (from error detection_table}
	 * Instance 0 - M2S - Desc read
	 * Instance 1 -  M2S - Data read
	 * Instance 2 -  M2s -  Cmpl Write
	 * Instance 3 -  S2M -  Desc read
	 * Instance 4 - S2M -   Data Write
	 * Instance 5 - S2M -  Cmpl Write
	 * Instance 6 - MSIX
	 */
	struct udma_gen_axi_error_control axi_error_control[7];
	/* [0x2b38] */
	uint32_t rsrvd_4[50];
	/* [0x2c00] */
	struct udma_gen_axi_queue axi_queue[16];
	/* [0x2cc0] Push packet desc cfg */
	struct udma_gen_pp pp;
	/* [0x2cd4] */
	uint32_t rsrvd_5[11];
	/* [0x2d00] m2s_desc_read */
	uint32_t int_ctrl0[64];
	/* [0x2e00] m2s_data_read */
	uint32_t int_ctrl1[64];
	/* [0x2f00] m2s_cmpl_write */
	uint32_t int_ctrl2[64];
	/* [0x3000] s2m_desc_read */
	uint32_t int_ctrl3[64];
	/* [0x3100] s2m_data_write */
	uint32_t int_ctrl4[64];
	/* [0x3200] s2m_cmpl_write */
	uint32_t int_ctrl5[64];
	/* [0x3300] msix_write */
	uint32_t int_ctrl6[64];
	/* [0x3400] Here you can configure in which q PMU counter will count */
	struct udma_gen_pmu pmu;
	/* [0x3410] */
	uint32_t rsrvd_6[4];
	/* [0x3420] */
	struct udma_gen_init_memory init_memory;
	/* [0x3440] */
	uint32_t rsrvd_7[8];
	/* [0x3460] spare registers for ECO */
	struct udma_gen_spare_reg spare_reg;
	/* [0x3470] */
	uint32_t rsrvd_8[36];
	/* [0x3500] */
	struct udma_gen_intc_shadow_access intc_shadow_access;
	/* [0x3600] Stat for shadow access */
	struct udma_gen_shadow_access_stat shadow_access_stat;
};

#ifndef AL_UDMA_EX
/**************************************************************************************************
 * udma regs gen ex
 **************************************************************************************************/
struct udma_gen_ex_vmpr_v4 {
	uint32_t tx_sel;
	uint32_t rx_sel[3];
	uint32_t rsrvd[12];
};

struct udma_gen_ex_regs {
	uint32_t rsrvd[0x100];
	struct udma_gen_ex_vmpr_v4 vmpr_v4[16];              /* [0x400] */
};
#endif

/**************************************************************************************************
 * udma regs
 **************************************************************************************************/

/**** cfg_tgtid_0 register ****/
/* For M2S queues 3:0, enable usage of the Target-ID from the buffer address 63:56 */
/* V3 only */
#define UDMA_GEN_TGTID_CFG_TGTID_0_TX_Q_TGTID_DESC_EN_MASK 0x0000000F
#define UDMA_GEN_TGTID_CFG_TGTID_0_TX_Q_TGTID_DESC_EN_SHIFT 0
#define UDMA_GEN_TGTID_CFG_TX_DESC_EN(qid) (AL_BIT((qid) + 0))

/*
 * For M2S queues 3:0, enable usage of the Target-ID from the configuration register
 * (cfg_tgtid_1/2 used for M2S queue_x)
 */
/* V3 only */
#define UDMA_GEN_TGTID_CFG_TGTID_0_TX_Q_TGTID_QUEUE_EN_MASK 0x000000F0
#define UDMA_GEN_TGTID_CFG_TGTID_0_TX_Q_TGTID_QUEUE_EN_SHIFT 4
#define UDMA_GEN_TGTID_CFG_TX_QUEUE_EN(qid) (AL_BIT((qid) + 4))

/* For S2M queues 3:0, enable usage of the Target-ID from the buffer address 63:56 */
/* V3 only */
#define UDMA_GEN_TGTID_CFG_TGTID_0_RX_Q_TGTID_DESC_EN_MASK 0x000F0000
#define UDMA_GEN_TGTID_CFG_TGTID_0_RX_Q_TGTID_DESC_EN_SHIFT 16
#define UDMA_GEN_TGTID_CFG_RX_DESC_EN(qid) (AL_BIT((qid) + 16))

/*
 * For S2M queues 3:0, enable usage of the Target-ID from the configuration register
 * (cfg_tgtid_3/4 used for M2S queue_x)
 */
/* V3 only */
#define UDMA_GEN_TGTID_CFG_TGTID_0_RX_Q_TGTID_QUEUE_EN_MASK 0x00F00000
#define UDMA_GEN_TGTID_CFG_TGTID_0_RX_Q_TGTID_QUEUE_EN_SHIFT 20
#define UDMA_GEN_TGTID_CFG_RX_QUEUE_EN(qid) (AL_BIT((qid) + 20))

#define UDMA_GEN_TGTID_CFG_TGTID_SHIFT(qid)	(((qid) & 0x1) ? 16 : 0)
#define UDMA_GEN_TGTID_CFG_TGTID_MASK(qid)	(((qid) & 0x1) ? 0xFFFF0000 : 0x0000FFFF)

/**** cfg_tgtid_1 register ****/
/* V3 only */
/* TX queue 0 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_1_TX_Q_0_TGTID_MASK 0x0000FFFF
#define UDMA_GEN_TGTID_CFG_TGTID_1_TX_Q_0_TGTID_SHIFT 0
/* TX queue 1 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_1_TX_Q_1_TGTID_MASK 0xFFFF0000
#define UDMA_GEN_TGTID_CFG_TGTID_1_TX_Q_1_TGTID_SHIFT 16

/**** cfg_tgtid_2 register ****/
/* V3 only */
/* TX queue 2 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_2_TX_Q_2_TGTID_MASK 0x0000FFFF
#define UDMA_GEN_TGTID_CFG_TGTID_2_TX_Q_2_TGTID_SHIFT 0
/* TX queue 3 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_2_TX_Q_3_TGTID_MASK 0xFFFF0000
#define UDMA_GEN_TGTID_CFG_TGTID_2_TX_Q_3_TGTID_SHIFT 16

/**** cfg_tgtid_3 register ****/
/* V3 only */
/* RX queue 0 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_3_RX_Q_0_TGTID_MASK 0x0000FFFF
#define UDMA_GEN_TGTID_CFG_TGTID_3_RX_Q_0_TGTID_SHIFT 0
/* RX queue 1 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_3_RX_Q_1_TGTID_MASK 0xFFFF0000
#define UDMA_GEN_TGTID_CFG_TGTID_3_RX_Q_1_TGTID_SHIFT 16

/**** cfg_tgtid_4 register ****/
/* V3 only */
/* RX queue 2 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_4_RX_Q_2_TGTID_MASK 0x0000FFFF
#define UDMA_GEN_TGTID_CFG_TGTID_4_RX_Q_2_TGTID_SHIFT 0
/* RX queue 3 Target-ID value */
#define UDMA_GEN_TGTID_CFG_TGTID_4_RX_Q_3_TGTID_MASK 0xFFFF0000
#define UDMA_GEN_TGTID_CFG_TGTID_4_RX_Q_3_TGTID_SHIFT 16

#define UDMA_GEN_TGTADDR_CFG_SHIFT(qid)	(((qid) & 0x1) ? 16 : 0)
#define UDMA_GEN_TGTADDR_CFG_MASK(qid)	(((qid) & 0x1) ? 0xFFFF0000 : 0x0000FFFF)

/**** desc_pref_cfg_1 register (V3 ?) ****/
/*
 * Size of the descriptor prefetch FIFO.
 * (descriptors)
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_1_FIFO_DEPTH_MASK 0x000000FF
#define UDMA_S2M_RD_DESC_PREF_CFG_1_FIFO_DEPTH_SHIFT 0

/**** desc_pref_cfg_2 register ****/
/*
 * Enable promotion of the current queue in progress
 * Reset: 0x1         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_2_Q_PROMOTION (1 << 0)
/*
 * Force promotion of the current queue in progress
 * Reset: 0x1         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_2_FORCE_PROMOTION (1 << 1)
/*
 * Enable prefetch prediction of next packet in line.
 * Reset: 0x1         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_2_EN_PREF_PREDICTION (1 << 2)
/*
 * Threshold for queue promotion.
 * Queue is promoted for prefetch if there are less descriptors in the prefetch FIFO than the
 * threshold
 * Reset: 0x4         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_2_PROMOTION_TH_MASK 0x0000FF00
#define UDMA_S2M_RD_DESC_PREF_CFG_2_PROMOTION_TH_SHIFT 8
/*
 * Force RR arbitration in the prefetch arbiter.
 * 0 - Standard arbitration based on queue QoS
 * 1 - Force round robin arbitration
 * Reset: 0x1         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_2_PREF_FORCE_RR (1 << 16)

/**** desc_pref_cfg_3 register ****/
/*
 * Minimum descriptor burst size when prefetch FIFO level is below the descriptor prefetch threshold
 * (must be 1)
 * Reset: 0x1         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_3_MIN_BURST_BELOW_THR_MASK 0x0000000F
#define UDMA_S2M_RD_DESC_PREF_CFG_3_MIN_BURST_BELOW_THR_SHIFT 0
/*
 * Minimum descriptor burst size when prefetch FIFO level is above the descriptor prefetch threshold
 * Reset: 0x4         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_3_MIN_BURST_ABOVE_THR_MASK 0x000000F0
#define UDMA_S2M_RD_DESC_PREF_CFG_3_MIN_BURST_ABOVE_THR_SHIFT 4
/*
 * Descriptor fetch threshold.
 * Used as a threshold to determine the allowed minimum descriptor burst size.
 * (Must be at least "max_desc_per_pkt")
 * Reset: 0x8         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_3_PREF_THR_MASK 0x0000FF00
#define UDMA_S2M_RD_DESC_PREF_CFG_3_PREF_THR_SHIFT 8

/**** desc_pref_cfg_4 register ****/
/*
 * Used as a threshold for generating almost FULL indication to the application
 * Reset: 0x8         Access: RW
 */
#define UDMA_S2M_RD_DESC_PREF_CFG_4_A_FULL_THR_MASK 0x0000FFFF
#define UDMA_S2M_RD_DESC_PREF_CFG_4_A_FULL_THR_SHIFT 0

/**** desc_pref_cfg_1 register (V3 ?) ****/
/* Size of the descriptor prefetch FIFO (in descriptors) */
#define UDMA_M2S_RD_DESC_PREF_CFG_1_FIFO_DEPTH_MASK 0x000000FF
#define UDMA_M2S_RD_DESC_PREF_CFG_1_FIFO_DEPTH_SHIFT 0

/**** desc_pref_cfg_2 register ****/
/*
 * Maximum number of descriptors per packet
 * Reset: 0x40        Access: RW
 */
#define UDMA_M2S_RD_DESC_PREF_CFG_2_MAX_DESC_PER_PKT_MASK 0x00000FFF
#define UDMA_M2S_RD_DESC_PREF_CFG_2_MAX_DESC_PER_PKT_SHIFT 0
/*
 * Force RR arbitration in the prefetch arbiter (V3)
 * Force RR arbitration in the prefetch arbiter and packet scheduler(post rate
 * control) (V4_V5)
 * 0 -Standard arbitration based on queue QoS
 * 1 - Force Round Robin arbitration
 * Reset: 0x1         Access: RW
 */
#define UDMA_M2S_RD_DESC_PREF_CFG_2_PREF_FORCE_RR (1 << 16)

/**** desc_pref_cfg_3 register ****/
/*
 * Minimum descriptor burst size when prefetch FIFO level is below the descriptor prefetch threshold
 * (must be 1)
 * Reset: 0x1         Access: RW
 */
#define UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_BELOW_THR_MASK 0x0000000F
#define UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_BELOW_THR_SHIFT 0
/*
 * Minimum descriptor burst size when prefetch FIFO level is above the descriptor prefetch threshold
 * Reset: 0x4         Access: RW
 */
#define UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_ABOVE_THR_MASK 0x000000F0
#define UDMA_M2S_RD_DESC_PREF_CFG_3_MIN_BURST_ABOVE_THR_SHIFT 4
/*
 * Descriptor fetch threshold.
 * Used as a threshold to determine the allowed minimum descriptor burst size.
 * (Must be at least max_desc_per_pkt)
 * Reset: 0x10        Access: RW
 */
#define UDMA_M2S_RD_DESC_PREF_CFG_3_PREF_THR_MASK 0x0000FF00
#define UDMA_M2S_RD_DESC_PREF_CFG_3_PREF_THR_SHIFT 8

/* Legacy defines for supporting HAL clients (CVTE/libdx) */
#ifndef UDMA_M2S_Q_COMP_CFG_DIS_COMP_COAL
#define UDMA_M2S_Q_COMP_CFG_DIS_COMP_COAL UDMA_M2S_M2S_Q_COMP_CFG_DIS_COMP_COAL
#endif

#ifndef UDMA_S2M_Q_COMP_CFG_DIS_COMP_COAL
#define UDMA_S2M_Q_COMP_CFG_DIS_COMP_COAL UDMA_S2M_S2M_Q_COMP_CFG_DIS_COMP_COAL
#endif

#ifndef UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_MASK
#define UDMA_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_MASK UDMA_M2S_AXI_M2S_OSTAND_CFG_MAX_DATA_RD_MASK
#endif
#ifdef __cplusplus
}
#endif

#endif

/** @} */
