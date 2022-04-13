/*
 * Copyright (c) 2022, Commonwealth Scientific and Industrial Research
 * Organisation (CSIRO) ABN 41 687 119 230.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
#include <drivers/modem/esp.h>

#include "esp.h"

LOG_MODULE_REGISTER(wifi_esp_mgmt, CONFIG_WIFI_LOG_LEVEL);

MODEM_CMD_DEFINE(on_cmd_gmr_at){
	struct esp_data *dev = CONTAINER_OF(data, struct esp_data,
					    cmd_handler_data);

	dev->at_version[0] = argv[0][0] - '0';
	dev->at_version[1] = argv[0][2] - '0';
	dev->at_version[2] = argv[0][4] - '0';
	dev->at_version[2] = argv[0][6] - '0';
	LOG_INF(" AT version: %s", log_strdup(argv[0]));
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_gmr_sdk){
	LOG_INF("SDK version: %s", log_strdup(argv[0]));
	return 0;
}

MODEM_CMD_DEFINE(on_cmd_gmr_bin){
	LOG_INF("Bin version: %s", log_strdup(argv[0]));
	return 0;
}

int esp_firmware_at_version(const struct device *dev, uint8_t at_version[4])
{
	struct esp_data *data = dev->data;
	char cmd[] = "AT+GMR";
	int err;

	static const struct modem_cmd cmds[] = {
		MODEM_CMD("AT version:", on_cmd_gmr_at, 1U, ""),
		MODEM_CMD("SDK version:", on_cmd_gmr_sdk, 1U, ""),
		MODEM_CMD("Bin version:", on_cmd_gmr_bin, 1U, ""),
	};

	err = esp_cmd_send(data, cmds, ARRAY_SIZE(cmds), cmd, ESP_CMD_TIMEOUT);
	memcpy(at_version, data->at_version, 4);
	return err;
}
