/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */
#include "al_hal_live_update.h"
#include "al_hal_live_update_internal.h"
#include "al_eth_v4_lm_live_update.h"
#include "al_eth_v4_lm.h"

#define AL_LU_ETH_PACKED_MAGIC (0xCAFE)
#define AL_LU_ETH_LM_PACKED_ID	(0x210)

struct al_hal_eth_lm_v4_packed_v0 {
	int state;
	unsigned int state_sleep_request;
	al_bool	state_return;
	al_bool	state_start_time_set;
	uint64_t state_start_time;
	unsigned int state_counter;
	uint64_t state_visit_req;
	int mac_mode_detect_stage;
	uint64_t link_down_counter;
	al_bool debug;
	/* al_eth_v4_lm_status status */
	al_bool status_mode_detection_ongoing;
	al_bool status_link_up;
	al_bool status_link_local_fault;
	al_bool status_link_remote_fault;
	/* status.detected_mode */
	al_bool status_detected_mode_detected;
	enum al_eth_mac_mode status_detected_mode_mac_mode;
	enum al_eth_mac_mode mac_mode[AL_ETH_V4_LM_MAC_MAX];
} __packed;

struct al_hal_eth_lm_v4_packed {
	enum al_lu_hal_packed_ver ver;
	uint16_t magic;
	uint16_t id;
	uint16_t idx;
	uint16_t data_len;
	int8_t active_buff;  /* Indicates which of the 2 buffers in the union is active */
	uint8_t pad[3];
	uint8_t reserved[16];
	char packed[AL_LU_ETH_LM_PACKED_BUFFER_COUNT][AL_LU_ETH_LM_PACKED_DATA_LEN];
} __packed;


/* These functions have to check what is the current active buffer,
 * update the not active, and set it to active
 * Note: The version in the tuple, is the source version to convert from
 */
static struct al_lu_ver_to_func_tuple eth_lm_convert_up_funcs[0] = {
};
static struct al_lu_ver_to_func_tuple eth_lm_convert_down_funcs[0] = {
};

static enum al_lu_hal_packed_ver eth_lm_get_ver_func(void *packed)
{
	return ((struct al_hal_eth_lm_v4_packed *)packed)->ver;
}

/*
 * Pack a V0 packed lm struct out of a struct al_eth_lm
 */
static int al_eth_lm_pack_v0(struct al_hal_eth_lm_v4_packed *lm_packed,
				struct al_eth_v4_lm *eth_lm,
				__attribute__((unused)) enum al_lu_hal_packed_ver dst_ver)
{
	struct al_hal_eth_lm_v4_packed_v0 *packed_v0 =
		(struct al_hal_eth_lm_v4_packed_v0 *)lm_packed->packed[lm_packed->active_buff];
	int mac;

	/* Pack the fields */
	packed_v0->state = eth_lm->state;
	packed_v0->state_sleep_request = eth_lm->state_sleep_request;
	packed_v0->state_return = eth_lm->state_return;
	packed_v0->state_start_time_set = eth_lm->state_start_time_set;
	packed_v0->state_start_time = eth_lm->state_start_time;
	packed_v0->state_counter = eth_lm->state_counter;
	packed_v0->state_visit_req = eth_lm->state_visit_req;
	packed_v0->mac_mode_detect_stage = eth_lm->mac_mode_detect_stage;
	packed_v0->link_down_counter = eth_lm->link_down_counter;
	packed_v0->debug = eth_lm->debug;
	packed_v0->status_mode_detection_ongoing = eth_lm->status.mode_detection_ongoing;
	packed_v0->status_link_up = eth_lm->status.link_up;
	packed_v0->status_link_local_fault = eth_lm->status.link_local_fault;
	packed_v0->status_link_remote_fault = eth_lm->status.link_remote_fault;
	packed_v0->status_detected_mode_detected = eth_lm->status.detected_mode.detected;
	packed_v0->status_detected_mode_mac_mode = eth_lm->status.detected_mode.mac_mode;
	for (mac = 0; mac < AL_ETH_V4_LM_MAC_MAX; mac++)
		packed_v0->mac_mode[mac] = eth_lm->mac_obj[mac].mac_mode;

	lm_packed->active_buff = 0;

	return 0;
}

/*
 * Unpack a V0 packed struct into a struct al_eth
 */
static int al_eth_lm_unpack_v0(struct al_eth_v4_lm *eth_lm,
				struct al_hal_eth_lm_v4_packed *lm_packed)
{
	struct al_hal_eth_lm_v4_packed_v0 *packed_v0 =
		(struct al_hal_eth_lm_v4_packed_v0 *)lm_packed->packed[lm_packed->active_buff];
	int mac;

	al_static_assert(sizeof(struct al_hal_eth_lm_v4_packed_v0) <
			AL_LU_ETH_LM_PACKED_DATA_LEN, "Packed v0 struct to big");

	eth_lm->state = packed_v0->state;
	eth_lm->state_sleep_request = packed_v0->state_sleep_request;
	eth_lm->state_return = packed_v0->state_return;
	eth_lm->state_start_time_set = packed_v0->state_start_time_set;
	eth_lm->state_start_time = packed_v0->state_start_time;
	eth_lm->state_counter = packed_v0->state_counter;
	eth_lm->state_visit_req = packed_v0->state_visit_req;
	eth_lm->mac_mode_detect_stage = packed_v0->mac_mode_detect_stage;
	eth_lm->link_down_counter = packed_v0->link_down_counter;
	eth_lm->debug = packed_v0->debug;
	eth_lm->status.mode_detection_ongoing = packed_v0->status_mode_detection_ongoing;
	eth_lm->status.link_up = packed_v0->status_link_up;
	eth_lm->status.link_local_fault = packed_v0->status_link_local_fault;
	eth_lm->status.link_remote_fault = packed_v0->status_link_remote_fault;
	eth_lm->status.detected_mode.detected = packed_v0->status_detected_mode_detected;
	eth_lm->status.detected_mode.mac_mode = packed_v0->status_detected_mode_mac_mode;
	for (mac = 0; mac < AL_ETH_V4_LM_MAC_MAX; mac++)
		eth_lm->mac_obj[mac].mac_mode = packed_v0->mac_mode[mac];

	return 0;
}

/*****************************API Functions  **********************************/
int al_eth_lm_v4_handle_pack(struct al_hal_eth_lm_v4_packed_container *lm_packed_cont,
				struct al_eth_v4_lm *eth_lm,
				int idx,
				enum al_lu_hal_packed_ver dst_ver)
{
	struct al_hal_eth_lm_v4_packed *lm_packed =
				(struct al_hal_eth_lm_v4_packed *)lm_packed_cont;

	al_static_assert(sizeof(struct al_hal_eth_lm_v4_packed_v0) <
			AL_LU_ETH_LM_PACKED_DATA_LEN, "Packed v0 struct to big");
	al_static_assert(sizeof(struct al_hal_eth_lm_v4_packed_container) >=
		sizeof(struct al_hal_eth_lm_v4_packed), "Packed struct larger than container");
	al_static_assert(sizeof(struct al_hal_eth_lm_v4_packed) >=
			(AL_LU_ETH_LM_PACKED_BUFFER_COUNT *
			 sizeof(struct al_hal_eth_lm_v4_packed_v0)),
			"2*Packed_v0 larger than packed");

	al_assert(dst_ver < AL_LU_HAL_VERSION_MAX);

	al_memset(lm_packed_cont, 0x0, sizeof(struct al_hal_eth_lm_v4_packed_container));

	/* Put the last HAL version that changed the eth */
	if (AL_ARR_SIZE(eth_lm_convert_down_funcs) > 0)
		lm_packed->ver =
			eth_lm_convert_down_funcs[AL_ARR_SIZE(eth_lm_convert_down_funcs) - 1].ver;
	else
		lm_packed->ver = AL_LU_HAL_VERSION_V0;

	lm_packed->magic = AL_LU_ETH_PACKED_MAGIC;
	lm_packed->id = AL_LU_ETH_LM_PACKED_ID;
	lm_packed->idx = idx;
	lm_packed->active_buff = 0;
	lm_packed->data_len = sizeof(struct al_hal_eth_lm_v4_packed_v0);

	al_eth_lm_pack_v0(lm_packed, eth_lm, dst_ver);

	if (al_lu_packed_convert(lm_packed,
				dst_ver,
				eth_lm_convert_up_funcs,
				eth_lm_convert_down_funcs,
				eth_lm_get_ver_func,
				AL_ARR_SIZE(eth_lm_convert_up_funcs)) != 0)
		return -EINVAL;

	return 0;
}

int al_eth_lm_v4_handle_unpack(struct al_eth_v4_lm *eth_lm,
				struct al_hal_eth_lm_v4_packed_container *lm_packed_cont)
{
	struct al_hal_eth_lm_v4_packed *lm_packed =
				(struct al_hal_eth_lm_v4_packed *)lm_packed_cont;
	int rc;

	/* Assertions for checking enum differences between HAL versions.
	 * When changing an ENUM, create a new HAL version, and implement a convert function
	 */
	al_static_assert(AL_ETH_MDIO_IF_1G_MAC  == 0, "Enum mismatch");
	al_static_assert(AL_ETH_MDIO_IF_10G_MAC == 1, "Enum mismatch");
	al_static_assert(AL_ETH_MDIO_IF_MAX  == 2, "Enum mismatch");
	al_static_assert(AL_ETH_MDIO_TYPE_CLAUSE_22 == 0, "Enum mismatch");
	al_static_assert(AL_ETH_MDIO_TYPE_CLAUSE_45 == 1, "Enum mismatch");
	al_static_assert(AL_ETH_MDIO_TYPE_MAX  == 2, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_RGMII == 0, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_SGMII == 1, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_SGMII_2_5G == 2, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_10GbE_Serial == 3, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_10G_SGMII == 4, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_XLG_LL_40G == 5, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_KR_LL_25G == 6, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_XLG_LL_50G == 7, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_XLG_LL_25G == 8, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_CG_100G == 9, "Enum mismatch");
	al_static_assert(AL_ETH_MAC_MODE_NUM == 10, "Enum mismatch");
	al_static_assert(AL_ETH_LM_RETIMER_TYPE_NONE == 0, "Enum mismatch");
	al_static_assert(AL_ETH_LM_RETIMER_TYPE_BR_210 == 1, "Enum mismatch");
	al_static_assert(AL_ETH_LM_RETIMER_TYPE_BR_410 == 2, "Enum mismatch");
	al_static_assert(AL_ETH_LM_RETIMER_TYPE_DS_25 == 3, "Enum mismatch");
	al_static_assert(AL_ETH_LM_RETIMER_TYPE_NUM == 4, "Enum mismatch");
	al_static_assert(AL_I2C_MASTER_PRIMARY == 0, "Enum mismatch");
	al_static_assert(AL_I2C_MASTER_PLD == 0, "Enum mismatch");
	al_static_assert(AL_I2C_MASTER_SECONDARY == 1, "Enum mismatch");
	al_static_assert(AL_I2C_MASTER_GEN == 1, "Enum mismatch");
	al_static_assert(AL_I2C_MASTER_NUM == 2, "Enum mismatch");
	al_static_assert(AL_ETH_V4_LM_TIME_UNITS_MSEC == 0, "Enum mismatch");
	al_static_assert(AL_ETH_V4_LM_TIME_UNITS_USEC == 1, "Enum mismatch");
	al_static_assert(AL_ETH_V4_LM_MAC_MAX == 4, "Enum mismatch");

	al_assert(lm_packed->magic == AL_LU_ETH_PACKED_MAGIC);

	/* Convert the packed structure to a format we know how to work with */
	if (al_lu_packed_convert(lm_packed,
					AL_LU_HAL_CURR_VER,
					eth_lm_convert_up_funcs,
					eth_lm_convert_down_funcs,
					eth_lm_get_ver_func,
					AL_ARR_SIZE(eth_lm_convert_up_funcs)) != 0)
		return -EINVAL;

	/* Unpacks the lm struct, and all of the structs under it */
	rc = al_eth_lm_unpack_v0(eth_lm, lm_packed);
	if (rc != 0) {
		al_err("Failed unpacked lm_packed struct\n");
		return -EINVAL;
	}

	return 0;
}

unsigned int al_eth_lm_v4_packed_idx_get(
		struct al_hal_eth_lm_v4_packed_container *lm_packed_cont)
{
	struct al_hal_eth_lm_v4_packed *eth_lm_packed =
		(struct al_hal_eth_lm_v4_packed *)lm_packed_cont;

	return eth_lm_packed->idx;
}
