/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCX_SOC_PINS_H_
#define _NUVOTON_NPCX_SOC_PINS_H_

#include <stdint.h>

#include "reg/reg_def.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief NPCX pin-mux configuration structure
 *
 * Used to indicate the device's corresponding DEVALT register/bit for
 * pin-muxing and its polarity to enable alternative functionality.
 */
struct npcx_alt {
	uint8_t group;
	uint8_t bit:3;
	uint8_t inverted:1;
	uint8_t reserved:4;
};

/**
 * @brief NPCX low-voltage configuration structure
 *
 * Used to indicate the device's corresponding LV_GPIO_CTL register/bit for
 * low-voltage detection.
 */
struct npcx_lvol {
	uint16_t io_port:5; /** A io pad's port which support low-voltage. */
	uint16_t io_bit:3; /** A io pad's bit which support low-voltage. */
	uint16_t ctrl:5; /** Related register index for low-voltage conf. */
	uint16_t bit:3; /** Related register bit for low-voltage conf. */
};

/**
 * @brief Select i2c port pads of i2c controller
 *
 * @param controller i2c controller device
 * @param port index for i2c port pads
 */
void npcx_pinctrl_i2c_port_sel(int controller, int port);

/**
 * @brief Force the internal SPI flash write-protect pin (WP) to low level to
 * protect the flash Status registers.
 */
int npcx_pinctrl_flash_write_protect_set(void);

/**
 * @brief Get write protection status
 *
 * @return 1 if write protection is set, 0 otherwise.
 */
bool npcx_pinctrl_flash_write_protect_is_set(void);

/**
 * @brief Restore all connections between IO pads that support low-voltage power
 *        supply and GPIO hardware devices. This utility is used for solving a
 *        leakage current issue found in npcx7 series. The npcx9 and later
 *        series fixed the issue and needn't it.
 */
void npcx_lvol_restore_io_pads(void);

/**
 * @brief Disable all connections between IO pads that support low-voltage power
 *        supply and GPIO hardware devices. This utility is used for solving a
 *        leakage current issue found in npcx7 series. The npcx9 and later
 *        series fixed the issue and needn't it.
 */
void npcx_lvol_suspend_io_pads(void);

/**
 * @brief Get the low-voltage power supply status of GPIO pads
 *
 * @param port port index of GPIO device
 * @param pin pin of GPIO device
 * @return 1 means the low-voltage power supply is enabled, otherwise disabled.
 */
bool npcx_lvol_is_enabled(int port, int pin);

/**
 * @brief Select the host interface type
 *
 * @param hif_type host interface type
 */
void npcx_host_interface_sel(enum npcx_hif_type hif_type);

#ifdef __cplusplus
}
#endif

#endif /* _NUVOTON_NPCX_SOC_PINS_H_ */
