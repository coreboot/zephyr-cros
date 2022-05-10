/*
 *
 * Copyright (c) 2018 Ilya Tagunov
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "clock_stm32_ll_common.h"


#if STM32_SYSCLK_SRC_PLL

/* Macros to fill up multiplication and division factors values */
#define z_pll_mul(v) LL_RCC_PLL_MUL_ ## v
#define pll_mul(v) z_pll_mul(v)

#define z_pll_div(v) LL_RCC_PLL_DIV_ ## v
#define pll_div(v) z_pll_div(v)

/**
 * @brief Set up pll configuration
 */
int config_pll_sysclock(void)
{
	uint32_t pll_source, pll_mul, pll_div;

	pll_mul = pll_mul(STM32_PLL_MULTIPLIER);
	pll_div = pll_div(STM32_PLL_DIVISOR);

	/* Configure PLL source */
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		pll_source = LL_RCC_PLLSOURCE_HSI;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		pll_source = LL_RCC_PLLSOURCE_HSE;
	} else {
		return -ENOTSUP;
	}

	LL_RCC_PLL_ConfigDomain_SYS(pll_source, pll_mul, pll_div);

	return 0;
}

#endif /* STM32_SYSCLK_SRC_PLL */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
#if defined(CONFIG_EXTI_STM32) || defined(CONFIG_USB_DC_STM32) || \
	(defined(CONFIG_SOC_SERIES_STM32L0X) &&			  \
	 defined(CONFIG_ENTROPY_STM32_RNG))
	/* Enable System Configuration Controller clock. */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
#endif
}
