/*
 * Copyright 2019, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#ifndef __SRAM_AGENT_H__
#define __SRAM_AGENT_H__

enum al_sram_agent_sram_type {
	AL_ETH_SRAM_TYPE
};

/**
 * ETH SRAM agent init
 * copy code compiled to SRAM address
 *
 * @returns	0 upon success
 */
int al_sram_agent_init(enum al_sram_agent_sram_type type, const unsigned char *arr, size_t size);

/**
 * SRAM agent run
 *
 *
 * @returns	0 upon success
 */
int al_sram_agent_run(void);


#endif /* __SRAM_AGENT_H__ */

