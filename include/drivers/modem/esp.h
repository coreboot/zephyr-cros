/**
 * @file
 * @brief ESP modem public API header file.
 */

/*
 * Copyright (c) 2022, Commonwealth Scientific and Industrial Research
 * Organisation (CSIRO) ABN 41 687 119 230.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_MODEM_ESP_H_
#define ZEPHYR_INCLUDE_DRIVERS_MODEM_ESP_H_

#include <stdint.h>
#include <stdlib.h>

#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Query modem AT version information
 *
 * @param dev Modem to query.
 * @param version Modem AT version output.
 *
 * @retval 0 On success.
 * @retval -errno Negative error code on failure.
 */
int esp_firmware_at_version(const struct device *dev, uint8_t version[4]);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_ESP_H_ */
