/*
 * Copyright (c) 2021 Jimmy Johnson <catch22@fastmail.net>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/sensor.h>
#include <logging/log.h>
#include <kernel.h>

#include "tmp108.h"

#define TMP108_ONE_SHOT_RETRY_TIME_IN_MS 10

LOG_MODULE_DECLARE(TMP108, CONFIG_SENSOR_LOG_LEVEL);

void tmp108_trigger_handle_one_shot(struct k_work *work)
{
	struct k_work_delayable *delayable_work = CONTAINER_OF(work,
							       struct k_work_delayable,
							       work);

	struct tmp108_data *drv_data = CONTAINER_OF(delayable_work,
						    struct tmp108_data,
						    scheduled_work);

	struct sensor_trigger sensor_trigger_type = {
		.chan = SENSOR_CHAN_AMBIENT_TEMP,
		.type = SENSOR_TRIG_DATA_READY
	};

	uint16_t config = 0;
	bool shutdown_mode = false;

	tmp108_reg_read(drv_data->tmp108_dev, TI_TMP108_REG_CONF, &config);

	/* check shutdown mode which indicates a one shot read was successful */
	shutdown_mode = (config & (TI_TMP108_CONF_M1 | TI_TMP108_CONF_M0)) == 0;

	if (shutdown_mode == true) {
		ti_tmp108_read_temp(drv_data->tmp108_dev);
	} else {
		LOG_ERR("Temperature one shot mode read failed, retrying");
		/* Typical wake up time is 27 ms, retry if the read fails
		 * assuming the chip should wake up and take a reading by the time
		 * 27 ms for the initial wake up time and call of this thread
		 * plus 10 ms time has passed
		 */
		k_work_reschedule(&drv_data->scheduled_work,
				  K_MSEC(TMP108_ONE_SHOT_RETRY_TIME_IN_MS));
		return;
	}

	/* Successful read, call set callbacks */
	if (drv_data->data_ready_handler) {
		drv_data->data_ready_handler(drv_data->tmp108_dev,
					     &sensor_trigger_type);
	}
}

void tmp108_trigger_handle_alert(const struct device *gpio,
				 struct gpio_callback *cb,
				 gpio_port_pins_t pins)
{

	struct tmp108_data *drv_data = CONTAINER_OF(cb,
						    struct tmp108_data,
						    temp_alert_gpio_cb);

	struct sensor_trigger sensor_trigger_type = {
		.chan = SENSOR_CHAN_AMBIENT_TEMP,
		.type = SENSOR_TRIG_THRESHOLD
	};

	/* Successful read, call set callbacks */
	if (drv_data->temp_alert_handler) {
		drv_data->temp_alert_handler(drv_data->tmp108_dev,
					     &sensor_trigger_type);
	}
}

int tmp_108_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct tmp108_data *drv_data = dev->data;

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		drv_data->data_ready_handler = handler;
		drv_data->data_ready_trigger = *trig;
		return 0;
	}

	if (trig->type == SENSOR_TRIG_THRESHOLD) {
		drv_data->temp_alert_handler = handler;
		drv_data->temp_alert_trigger = *trig;
		return 0;
	}

	return -ENOTSUP;
}
