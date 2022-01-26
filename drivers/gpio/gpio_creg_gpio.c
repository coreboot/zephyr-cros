/*
 * Copyright (c) 2021 Synopsys
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_creg_gpio

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/util.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(creg_gpio, CONFIG_GPIO_LOG_LEVEL);

#include "gpio_utils.h"

/** Runtime driver data */
struct creg_gpio_drv_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	uint32_t pin_val;
	uint32_t base_addr;
};

/** Configuration data */
struct creg_gpio_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint32_t ngpios;
	uint8_t bit_per_gpio;
	uint8_t off_val;
	uint8_t on_val;
};

static int pin_config(const struct device *dev,
		       gpio_pin_t pin,
		       gpio_flags_t flags)
{
	return -ENOTSUP;
}

static int port_get(const struct device *dev,
		    gpio_port_value_t *value)
{
	const struct creg_gpio_config *cfg = dev->config;
	struct creg_gpio_drv_data *drv_data = dev->data;
	uint32_t in = sys_read32(drv_data->base_addr);
	uint32_t tmp = 0;
	uint32_t val = 0;

	for (uint8_t i = 0; i < cfg->ngpios; i++) {
		tmp = (in & cfg->on_val << i * cfg->bit_per_gpio) ? 1 : 0;
		val |= tmp << i;
	}
	*value = drv_data->pin_val = val;

	return 0;
}

static int port_write(const struct device *dev,
		      gpio_port_pins_t mask,
		      gpio_port_value_t value,
		      gpio_port_value_t toggle)
{
	const struct creg_gpio_config *cfg = dev->config;
	struct creg_gpio_drv_data *drv_data = dev->data;
	uint32_t *pin_val = &drv_data->pin_val;
	uint32_t out = 0;
	uint32_t tmp = 0;

	*pin_val = ((*pin_val & ~mask) | (value & mask)) ^ toggle;

	for (uint8_t i = 0; i < cfg->ngpios; i++) {
		tmp = (*pin_val & 1 << i) ? cfg->on_val : cfg->off_val;
		out |= tmp << i * cfg->bit_per_gpio;
	}
	sys_write32(out, drv_data->base_addr);

	return 0;
}

static int port_set_masked(const struct device *dev,
			   gpio_port_pins_t mask,
			   gpio_port_value_t value)
{
	return port_write(dev, mask, value, 0);
}

static int port_set_bits(const struct device *dev,
			 gpio_port_pins_t pins)
{
	return port_write(dev, pins, pins, 0);
}

static int port_clear_bits(const struct device *dev,
			   gpio_port_pins_t pins)
{
	return port_write(dev, pins, 0, 0);
}

static int port_toggle_bits(const struct device *dev,
			    gpio_port_pins_t pins)
{
	return port_write(dev, 0, 0, pins);
}

static int pin_interrupt_configure(const struct device *dev,
				   gpio_pin_t pin,
				   enum gpio_int_mode mode,
				   enum gpio_int_trig trig)
{
	return -ENOTSUP;
}

/**
 * @brief Initialization function of creg_gpio
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
static int creg_gpio_init(const struct device *dev)
{
	return 0;
}

static const struct gpio_driver_api api_table = {
	.pin_configure = pin_config,
	.port_get_raw = port_get,
	.port_set_masked_raw = port_set_masked,
	.port_set_bits_raw = port_set_bits,
	.port_clear_bits_raw = port_clear_bits,
	.port_toggle_bits = port_toggle_bits,
	.pin_interrupt_configure = pin_interrupt_configure,
};

static const struct creg_gpio_config creg_gpio_cfg = {
	.common = {
		.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(0),
	},
	.ngpios = DT_INST_PROP(0, ngpios),
	.bit_per_gpio = DT_INST_PROP(0, bit_per_gpio),
	.off_val = DT_INST_PROP(0, off_val),
	.on_val = DT_INST_PROP(0, on_val),
};

static struct creg_gpio_drv_data creg_gpio_drvdata = {
	.base_addr = DT_INST_REG_ADDR(0),
};

DEVICE_DT_INST_DEFINE(0, creg_gpio_init, NULL,
		      &creg_gpio_drvdata, &creg_gpio_cfg,
		      POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,
		      &api_table);
