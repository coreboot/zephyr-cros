/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _SOC__H_
#define _SOC__H_

#include <sys/util.h>

#ifndef _ASMLANGUAGE

#include <fsl_common.h>

/* Add include for DTS generated information */
#include <devicetree.h>

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_DISK_DRIVER_SDMMC &&					\
	(DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc1), okay) ||	\
	 DT_NODE_HAS_STATUS(DT_NODELABEL(usdhc2), okay))

typedef void (*usdhc_pin_cfg_cb)(uint16_t nusdhc, bool init,
	uint32_t speed, uint32_t strength);

void imxrt_usdhc_pinmux(uint16_t nusdhc,
	bool init, uint32_t speed, uint32_t strength);

void imxrt_usdhc_pinmux_cb_register(usdhc_pin_cfg_cb cb);

typedef void (*usdhc_dat3_cfg_cb)(bool pullup);

void imxrt_usdhc_dat3_cb_register(usdhc_dat3_cfg_cb cb);

void imxrt_usdhc_dat3_pull(bool pullup);

#endif

#if CONFIG_I2S_MCUX_SAI
void imxrt_audio_codec_pll_init(uint32_t clock_name, uint32_t clk_src,
					uint32_t clk_pre_div, uint32_t clk_src_div);

#endif

#ifdef __cplusplus
}
#endif

#endif /* !_ASMLANGUAGE */

#endif /* _SOC__H_ */
