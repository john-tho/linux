/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#ifndef __AL_ERR_EVENTS_PMDT_H__
#define __AL_ERR_EVENTS_PMDT_H__

#include "al_pmdt.h"

/*
 * Inizialize PMDT units with TRAP and TIMEOUT features.
 * @param	pmdt
 *		the PMDT handle
 *
 * @return	0 in case of success.
 */
int al_err_events_pmdt_init(
		struct al_pmdt_handle *pmdt);

/**
 * Print PMDT info
 *
 * @param pmdt
 *		the PMDT handle
 * @param info
 *		the PMDT isr info
 */
void al_err_events_pmdt_print_info(
		struct al_pmdt_handle *pmdt,
		const struct al_pmdt_isr_info *info);

#endif
