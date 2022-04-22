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
 * @brief Query the currently connected WiFi network parameters
 *
 * @param dev Modem to query.
 * @param channel WiFi channel used by the network.
 *                If UINT8_MAX no network is connected.
 * @param rssi Current RSSI of the network in dBm.
 *
 * @retval 0 On success.
 * @retval -errno Negative error code on failure.
 */
int esp_network_parameters(const struct device *dev, uint8_t *channel, int8_t *rssi);

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

/**
 * @brief Upgrade modem firmware through Wi-Fi
 *
 * Trigger a blocking OTA upgrade of the ESP Modem. See the documentation for
 * AT+CIUPDATE for more complete information on behaviour.
 *
 * @note This function updates from Espressif's official release server.
 *
 * @note The functionality of this API has changed over ESP-AT releases, therefore
 *       only the common functionality set is implemented.
 *
 * @param dev Modem to upgrade.
 * @param https OTA via HTTPS when true, HTTP otherwise.
 * @param version Explicit AT version string (Optional).
 *
 * @retval 0 On success.
 * @retval -errno Negative error code on failure.
 */
int esp_firmware_ota_update(const struct device *dev, bool https, char *version);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_MODEM_ESP_H_ */
