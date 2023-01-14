#ifndef _LINUX_NRF52_RB_H
#define _LINUX_NRF52_RB_H

#include <linux/hwmon-lbl-rb.h>

#define NRF52_MAX_SENSOR_COUNT 4
#define NRF52_MAX_GPIO_COUNT 2

struct nrf52_platform_data {
	unsigned irq_gpio;
	/* temp1, in0, in1, in2 */
	enum HWMON_LABEL labels[NRF52_MAX_SENSOR_COUNT];
	struct {
		unsigned pins[NRF52_MAX_GPIO_COUNT];
		const char *names[NRF52_MAX_GPIO_COUNT];
	} gpio;
};

#endif /* _LINUX_NRF52_RB_H */
