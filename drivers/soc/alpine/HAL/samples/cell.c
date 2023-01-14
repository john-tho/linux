/*
 * Copyright 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 */

#include "samples.h"

#include "al_hal_cell_fe_regs.h"
#include "al_hal_cell_fm_regs.h"
#include "al_hal_cell_fe_fm_regs.h"
#include "al_hal_cell_alu_regs.h"

/**
 * @defgroup group_cell_samples Code Samples
 * @ingroup group_cell
 * @{
 * cell.c: this file can be found in samples directory.
 *
 * @code */

int main(void)
{
	struct al_cell_fe_regs fe;
	struct al_cell_fm_regs fm;
	struct al_cell_fe_fm_regs fe_fm;
	struct al_cell_alu_regs alu;

	/* Very basic example for checking compilation */
	(void)fe;
	(void)fm;
	(void)fe_fm;
	(void)alu;

	return 0;
}

/** @endcode */

/** @} */
