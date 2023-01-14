/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

/**
 *  @{
 * @file   al_hal_iofic_regs.h
 *
 * @brief IOFIC registers
 *
 * This a hand written file and not auto-generated.
 * Only part of the IOFIC registers are auto-generated.
 *
 */

#ifndef __AL_HAL_IOFIC_REGS_H__
#define __AL_HAL_IOFIC_REGS_H__

#include "al_hal_iofic_grp_ctrl_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

 /*
  * Unit Registers
  */

/**
 * Keep backward compatibly on grp_ctrl structure name
 */
#define al_iofic_grp_ctrl	al_iofic_grp_ctrl_regs

struct al_iofic_grp_mod {
	/*
	 * [0x0] Interrupt Moderation Register
	 * Dedicated moderation interval per bit.
	 */
	uint32_t grp_int_mod_reg;
	/* [0x4] Target ID */
	uint32_t grp_int_tgtid_reg;
};

/**
 * This is a maximal IOFIC registers representation.
 *
 * IOFIC instantiations vary on number of 'ctrl' groups (1, 2 or 4).
 * Not all IOFIC instantiations have 'grp_mod' registers.
 */
struct al_iofic_regs {
	struct al_iofic_grp_ctrl_regs ctrl[0];
	uint32_t rsrvd1[0x400 >> 2];
	struct al_iofic_grp_mod grp_int_mod[0][32];
};


/*
 * Registers Fields
 */

/**** int_control_grp register ****/
/**
 * These definitions here are for backward compatibly.
 * See actual register file for these registers documentation.
 */
#define INT_CONTROL_GRP_CLEAR_ON_READ \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_CLEAR_ON_READ
#define INT_CONTROL_GRP_AUTO_MASK \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_AUTO_MASK
#define INT_CONTROL_GRP_AUTO_CLEAR \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_AUTO_CLEAR
#define INT_CONTROL_GRP_SET_ON_POSEDGE \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_SET_ON_POSEDGE
#define INT_CONTROL_GRP_MOD_RST \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MOD_RST
#define INT_CONTROL_GRP_MASK_MSI_X \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MASK_MSI_X
#define INT_CONTROL_GRP_AWID_MASK \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_AWID_MASK
#define INT_CONTROL_GRP_AWID_SHIFT \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_AWID_SHIFT
#define INT_CONTROL_GRP_MOD_INTV_MASK \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MOD_INTV_MASK
#define INT_CONTROL_GRP_MOD_INTV_SHIFT \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MOD_INTV_SHIFT
#define INT_CONTROL_GRP_MOD_RES_MASK \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MOD_RES_MASK
#define INT_CONTROL_GRP_MOD_RES_SHIFT \
	IOFIC_GRP_CTRL_INT_CONTROL_GRP_MOD_RES_SHIFT

/*
 * Asserting this bit results in MSIX Address[63:20] sharing by the whole group. Default is per
 * entry.
 */
#define INT_CONTROL_GRP_ADDR_HI_COMPAT_MODE (1 << 7)

/**** grp0_int_mod_reg register ****/
/*
 * Interrupt Moderation Interval Register
 * A dedicated register is allocated per bit in the group. This value determines the interval
 * between interrupts for the associated bit in the group. Writing ZERO disables Moderation.
 */
#define INT_MOD_INTV_MASK 0x000000FF
#define INT_MOD_INTV_SHIFT 0

/**** grp_int_tgtid_reg register ****/
/* Target ID field. */
#define INT_MSIX_TGTID_MASK 0x0000FFFF
#define INT_MSIX_TGTID_SHIFT 0
/* Target ID enable. */
#define INT_MSIX_TGTID_EN_SHIFT 31

#ifdef __cplusplus
}
#endif

#endif /* __AL_HAL_IOFIC_REG_H */




