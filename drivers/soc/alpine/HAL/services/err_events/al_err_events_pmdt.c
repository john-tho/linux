/*
 * Copyright 2018, Amazon.com, Inc. or its affiliates. All Rights Reserved
 */

#include "al_err_events_pmdt.h"
#include "al_hal_pmdt_map.h"

int al_err_events_pmdt_init(
		struct al_pmdt_handle *pmdt)
{
	struct al_pmdt_special_setting setting;
	struct al_pmdt_components_info info;
	uint32_t units_bitmap[AL_PMDT_MAP_ACTIVE_UNIT_ENTRIES];
	enum al_pmdt_unit unit;
	int err;

	al_assert(pmdt);

	al_info("Error events: Initialize PMDT Units\n");

	al_memset(&setting, 0, sizeof(struct al_pmdt_special_setting));

	setting.force_features = AL_PMDT_FEATURE_AXI_TRAP |
			AL_PMDT_FEATURE_AXI_TIMEOUT;

	for (unit = 0; unit < AL_PMDT_UNIT_MAX; unit++) {
		err = al_pmdt_unit_components_info_get(unit, &info);
		if (err) {
			al_err("al_pmdt_unit_components_info_get failed!\n");
			return err;
		}
		if (info.axi_mon_num)
			AL_REG_BIT_SET(units_bitmap[unit/32], unit%32);
	}

	setting.units = &units_bitmap;
	setting.ext_trig = NULL;
	err = al_pmdt_monitor(pmdt, NULL, AL_TRUE, &setting);
	if (err)
		al_err("al_pmdt_monitor failed!\n");

	return err;
}

void al_err_events_pmdt_print_info(
		struct al_pmdt_handle *pmdt,
		const struct al_pmdt_isr_info *info)
{
	al_assert(pmdt);
	al_assert(info);

	al_info("PMDT %d trapped AXI error of type %d\n",
			info->pmdt_unit, info->err);

	if (info->err_info.read_timeout_info_valid)
		al_info("Read Timeout Info: latency: %" PRId64 ", "
				"address: %" PRId64 ", attribute: %" PRId64 "\n",
				info->err_info.read_timeout_info.latency,
				info->err_info.read_timeout_info.address,
				info->err_info.read_timeout_info.attr);

	if (info->err_info.write_timeout_info_valid)
		al_info("Write Timeout Info: latency: %" PRId64 ", "
				"address: %" PRId64 ", attribute: %" PRId64 "\n",
				info->err_info.write_timeout_info.latency,
				info->err_info.write_timeout_info.address,
				info->err_info.write_timeout_info.attr);

	al_pmdt_info_print(pmdt, info->pmdt_unit);
}
