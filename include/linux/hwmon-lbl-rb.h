#ifndef _LINUX_HWMON_LBL_RB_H
#define _LINUX_HWMON_LBL_RB_H

enum HWMON_LABEL {
	HWMON_LBL_NONE,
	HWMON_LBL_FAN,
	HWMON_LBL_PSU,
	HWMON_LBL_BOARD,
	HWMON_LBL_CPU,
	HWMON_LBL_SFP,
	HWMON_LBL_POE,
	HWMON_LBL_SYSTEM,
	HWMON_LBL_TOTAL_POE_POWER,
	HWMON_LBL_GPIO_EXT_PIN2,
	HWMON_LBL_GPIO_EXT_PIN3,
	HWMON_LBL_HIDE,
	HWMON_LBL_HIDE_REMOTE,
	HWMON_LBL_HIDE_CPU,
	HWMON_LBL_HIDE_SOC,
	HWMON_LBL_HIDE_AVDD,
	HWMON_LBL_CNT
};

#define HWMON_LABELS_ARRAY \
		"",		/* HWMON_LBL_NONE */ \
		"fan",		/* HWMON_LBL_FAN */ \
		"psu",		/* HWMON_LBL_PSU */ \
		"board",	/* HWMON_LBL_BOARD */ \
		"cpu",		/* HWMON_LBL_CPU */ \
		"sfp",		/* HWMON_LBL_SFP */ \
		"poe",		/* HWMON_LBL_POE */ \
		"system",	/* HWMON_LBL_SYSTEM */ \
		"total_poe_power", /* HWMON_LBL_TOTAL_POE_POWER */ \
		"ext-pin2",	/* HWMON_LBL_GPIO_EXT_PIN2 */ \
		"ext-pin3",	/* HWMON_LBL_GPIO_EXT_PIN3 */ \
		"hide",		/* HWMON_LBL_HIDE */ \
		"hide_remote",	/* HWMON_LBL_HIDE_REMOTE */ \
		"hide_cpu",	/* HWMON_LBL_HIDE_CPU */ \
		"hide_soc",	/* HWMON_LBL_HIDE_SOC */ \
		"hide_avdd",	/* HWMON_LBL_HIDE_AVDD */ \


#endif /* _LINUX_HWMON_LBL_RB_H */
