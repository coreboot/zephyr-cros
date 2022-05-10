/*
 * Copyright (c) 2021 Eug Krashtan
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#if DT_HAS_COMPAT_STATUS_OKAY(st_stm32_temp)
#define TEMP_NODE DT_INST(0, st_stm32_temp)
#elif DT_HAS_COMPAT_STATUS_OKAY(st_stm32_temp_cal)
#define TEMP_NODE DT_INST(0, st_stm32_temp_cal)
#else
#error "Could not find a compatible temperature sensor"
#endif

void main(void)
{
	struct sensor_value val;
	int rc;
	const struct device *dev = DEVICE_DT_GET(TEMP_NODE);

	if (!device_is_ready(dev)) {
		printk("Temperature sensor is not ready\n");
		return;
	}

	printk("STM32 Die temperature sensor test\n");

	while (1) {
		k_sleep(K_MSEC(300));

		/* fetch sensor samples */
		rc = sensor_sample_fetch(dev);
		if (rc) {
			printk("Failed to fetch sample (%d)\n", rc);
			continue;
		}

		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &val);
		if (rc) {
			printk("Failed to get data (%d)\n", rc);
			continue;
		}

		printk("Current temperature: %.1f °C\n", sensor_value_to_double(&val));
	}
}
