/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */
#ifndef _AL_HAL_ETH_V4_LM_LIVE_UPDATE_H_
#define _AL_HAL_ETH_V4_LM_LIVE_UPDATE_H_

#include "al_eth_v4_lm.h"
#include "al_hal_live_update.h"

/** 76 - size of al_eth_lm_v4_packed, Multiplied by 2 for double buffering +
 *   AL_LU_ETH_LM_PACKED_META_DATA_LEN = 280.
 *   We round up to 512
 */
#define AL_LU_ETH_LM_PACKED_DATA_LEN (512 / 2)
#define AL_LU_ETH_LM_PACKED_BUFFER_COUNT (2)
#define AL_LU_ETH_LM_PACKED_META_DATA_LEN (128)

/* NOTE: This struct must be allocated 64bit aligned */
struct al_hal_eth_lm_v4_packed_container {
	uint8_t packed[AL_LU_ETH_LM_PACKED_DATA_LEN * AL_LU_ETH_LM_PACKED_BUFFER_COUNT +
			AL_LU_ETH_LM_PACKED_META_DATA_LEN];
} __packed_a8;

/** Pack an al_eth_v4_lm struct
 *
 * @param lm_packed_cont pointer to an initialized al_eth_v4_lm_packed_cont struct
 * @param eth_lm pointer to a al_eth struct to pack
 * @param idx instance index. For distinguishign between packed structs if having multiple instances
 * @param dst_ver HAL destination version to pack to. Use HAL_CURR_VER for current version
 *
 * @return 0 on success. Error code otherwise.
 */
int al_eth_lm_v4_handle_pack(struct al_hal_eth_lm_v4_packed_container *lm_packed_cont,
				struct al_eth_v4_lm *eth_lm,
				int idx,
				enum al_lu_hal_packed_ver dst_ver);

/** UnPack an al_hal_eth_lm_v4_packed_container struct, into an al_eth struct
 *
 * @param eth_lm pointer a the destination al_eth struct, previously initialized with handle_init
 * @param lm_packed_cont pointer to an al_hal_eth_lm_v4_packed_container struct to unpack
 *
 * @return 0 on success. Error code otherwise.
 */
int al_eth_lm_v4_handle_unpack(struct al_eth_v4_lm *eth_lm,
				struct al_hal_eth_lm_v4_packed_container *lm_packed_cont);

/** Get the idx value of an al_eth_lm_v4_packed struct
 * @param lm_packed_cont pointer to an al_eth_lm_v4_packed struct
 */
unsigned int al_eth_lm_v4_packed_idx_get(
		struct al_hal_eth_lm_v4_packed_container *lm_packed_cont);

#endif
