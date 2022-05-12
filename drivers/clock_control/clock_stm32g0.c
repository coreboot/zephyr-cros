/*
 *
 * Copyright (c) 2019 Ilya Tagunov
 * Copyright (c) 2019 STMicroelectronics
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
#define z_pll_div(v) LL_RCC_PLLM_DIV_ ## v
#define pll_div(v) z_pll_div(v)

#define z_pllr(v) LL_RCC_PLLR_DIV_ ## v
#define pllr(v) z_pllr(v)

/**
 * @brief Return PLL source
 */
__unused
static uint32_t get_pll_source(void)
{
	/* Configure PLL source */
	if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		return LL_RCC_PLLSOURCE_HSI;
	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		return LL_RCC_PLLSOURCE_HSE;
	}

	__ASSERT(0, "Invalid source");
	return 0;
}

/**
 * @brief Set up pll configuration
 */
__unused
void config_pll_sysclock(void)
{
	LL_RCC_PLL_ConfigDomain_SYS(get_pll_source(),
				    pll_div(STM32_PLL_M_DIVISOR),
				    STM32_PLL_N_MULTIPLIER,
				    pllr(STM32_PLL_R_DIVISOR));

	LL_RCC_PLL_EnableDomain_SYS();
}


/**
 * @brief Return pllout frequency
 */
__unused
uint32_t get_pllout_frequency(void)
{
	return __LL_RCC_CALC_PLLCLK_FREQ(get_pll_source(),
					 pll_div(STM32_PLL_M_DIVISOR),
					 STM32_PLL_N_MULTIPLIER,
					 pllr(STM32_PLL_R_DIVISOR));
}

#endif /* STM32_SYSCLK_SRC_PLL */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
	/* Enable the power interface clock */
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
}
