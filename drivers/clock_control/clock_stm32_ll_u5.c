/*
 *
 * Copyright (c) 2021 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <soc.h>
#include <stm32_ll_bus.h>
#include <stm32_ll_pwr.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_utils.h>
#include <stm32_ll_system.h>
#include <drivers/clock_control.h>
#include <sys/util.h>
#include <stm32_ll_utils.h>
#include <drivers/clock_control/stm32_clock_control.h>

/* Macros to fill up prescaler values */
#define z_ahb_prescaler(v) LL_RCC_SYSCLK_DIV_ ## v
#define ahb_prescaler(v) z_ahb_prescaler(v)

#define z_apb1_prescaler(v) LL_RCC_APB1_DIV_ ## v
#define apb1_prescaler(v) z_apb1_prescaler(v)

#define z_apb2_prescaler(v) LL_RCC_APB2_DIV_ ## v
#define apb2_prescaler(v) z_apb2_prescaler(v)

#define z_apb3_prescaler(v) LL_RCC_APB3_DIV_ ## v
#define apb3_prescaler(v) z_apb3_prescaler(v)


#ifdef STM32_SYSCLK_SRC_PLL
/**
 * @brief fill in pll configuration structure
 */
static void config_pll_init(LL_UTILS_PLLInitTypeDef *pllinit)
{
	pllinit->PLLM = STM32_PLL_M_DIVISOR;
	pllinit->PLLN = STM32_PLL_N_MULTIPLIER;
	pllinit->PLLR = STM32_PLL_R_DIVISOR;
}
#endif /* STM32_SYSCLK_SRC_PLL */

/**
 * @brief Activate default clocks
 */
void config_enable_default_clocks(void)
{
	/* Enable the power interface clock */
	LL_AHB3_GRP1_EnableClock(LL_AHB3_GRP1_PERIPH_PWR);

	if (IS_ENABLED(STM32_LSE_ENABLED)) {
		if (!LL_PWR_IsEnabledBkUpAccess()) {
			/* Enable write access to Backup domain */
			LL_PWR_EnableBkUpAccess();
			while (!LL_PWR_IsEnabledBkUpAccess()) {
				/* Wait for Backup domain access */
			}
		}

		/* Enable LSE Oscillator */
		LL_RCC_LSE_Enable();
		/* Wait for LSE ready */
		while (!LL_RCC_LSE_IsReady()) {
		}

		/* Enable LSESYS additionally */
		LL_RCC_LSE_EnablePropagation();
		/* Wait till LSESYS is ready */
		while (!LL_RCC_LSESYS_IsReady()) {
		}

		LL_PWR_DisableBkUpAccess();
	}
}

/**
 * @brief fill in AHB/APB buses configuration structure
 */
static void config_bus_clk_init(LL_UTILS_ClkInitTypeDef *clk_init)
{
	clk_init->AHBCLKDivider = ahb_prescaler(STM32_AHB_PRESCALER);
	clk_init->APB1CLKDivider = apb1_prescaler(STM32_APB1_PRESCALER);
	clk_init->APB2CLKDivider = apb2_prescaler(STM32_APB2_PRESCALER);
	clk_init->APB3CLKDivider = apb3_prescaler(STM32_APB3_PRESCALER);
}

static uint32_t get_bus_clock(uint32_t clock, uint32_t prescaler)
{
	return clock / prescaler;
}

static inline int stm32_clock_control_on(const struct device *dev,
					 clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
		LL_AHB1_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB2:
		LL_AHB2_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB3:
		LL_AHB3_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1:
		LL_APB1_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1_2:
		LL_APB1_GRP2_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB2:
		LL_APB2_GRP1_EnableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB3:
		LL_APB3_GRP1_EnableClock(pclken->enr);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static inline int stm32_clock_control_off(const struct device *dev,
					  clock_control_subsys_t sub_system)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sub_system);

	ARG_UNUSED(dev);

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
		LL_AHB1_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB2:
		LL_AHB2_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_AHB3:
		LL_AHB3_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1:
		LL_APB1_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB1_2:
		LL_APB1_GRP2_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB2:
		LL_APB2_GRP1_DisableClock(pclken->enr);
		break;
	case STM32_CLOCK_BUS_APB3:
		LL_APB3_GRP1_DisableClock(pclken->enr);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int stm32_clock_control_get_subsys_rate(const struct device *dev,
					       clock_control_subsys_t sys,
					       uint32_t *rate)
{
	struct stm32_pclken *pclken = (struct stm32_pclken *)(sys);
	/*
	 * Get AHB Clock (= SystemCoreClock = SYSCLK/prescaler)
	 * SystemCoreClock is preferred to CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC
	 * since it will be updated after clock configuration and hence
	 * more likely to contain actual clock speed
	 */
	uint32_t ahb_clock = SystemCoreClock;
	uint32_t apb1_clock = get_bus_clock(ahb_clock, STM32_APB1_PRESCALER);
	uint32_t apb2_clock = get_bus_clock(ahb_clock, STM32_APB2_PRESCALER);
	uint32_t apb3_clock = get_bus_clock(ahb_clock, STM32_APB3_PRESCALER);

	ARG_UNUSED(dev);

	switch (pclken->bus) {
	case STM32_CLOCK_BUS_AHB1:
	case STM32_CLOCK_BUS_AHB2:
	case STM32_CLOCK_BUS_AHB3:
		*rate = ahb_clock;
		break;
	case STM32_CLOCK_BUS_APB1:
	case STM32_CLOCK_BUS_APB1_2:
		*rate = apb1_clock;
		break;
	case STM32_CLOCK_BUS_APB2:
		*rate = apb2_clock;
		break;
	case STM32_CLOCK_BUS_APB3:
		*rate = apb3_clock;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static struct clock_control_driver_api stm32_clock_control_api = {
	.on = stm32_clock_control_on,
	.off = stm32_clock_control_off,
	.get_rate = stm32_clock_control_get_subsys_rate,
};

static void set_regu_voltage(uint32_t hclk_freq)
{
	if (hclk_freq < MHZ(25)) {
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE4);
	} else if (hclk_freq < MHZ(55)) {
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
	} else if (hclk_freq < MHZ(110)) {
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE2);
	} else {
		LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	}
	while (LL_PWR_IsActiveFlag_VOS() == 0) {
	}
}

/*
 * Unconditionally switch the system clock source to HSI.
 */
__unused
static void clock_switch_to_hsi(uint32_t ahb_prescaler)
{
	/* Enable HSI if not enabled */
	if (LL_RCC_HSI_IsReady() != 1) {
		/* Enable HSI */
		LL_RCC_HSI_Enable();
		while (LL_RCC_HSI_IsReady() != 1) {
		/* Wait for HSI ready */
		}
	}

	/* Set HSI as SYSCLCK source */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);
	LL_RCC_SetAHBPrescaler(ahb_prescaler);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI) {
	}
}

static void set_up_clk_msis(void)
{
#if defined(STM32_SYSCLK_SRC_MSIS) || defined(STM32_PLL_SRC_MSIS)
	/* Set MSIS Range */
	LL_RCC_MSI_EnableRangeSelection();

	LL_RCC_MSIS_SetRange(STM32_MSIS_RANGE << RCC_ICSCR1_MSISRANGE_Pos);

	if (IS_ENABLED(STM32_MSIS_PLL_MODE)) {
		BUILD_ASSERT(STM32_LSE_ENABLED,
			"MSI Hardware auto calibration needs LSE clock activation");
		/* Enable MSI hardware auto calibration */
		LL_RCC_MSI_EnablePLLMode();
	}

	/* Set MSIS Range */
	LL_RCC_MSIS_Enable();

	/* Wait till MSIS is ready */
	while (LL_RCC_MSIS_IsReady() != 1) {
	}

#endif /* STM32_SYSCLK_SRC_MSIS || STM32_PLL_SRC_MSIS */
}

/*
 * Configure PLL as source of SYSCLK
 */
void config_src_sysclk_pll(LL_UTILS_ClkInitTypeDef s_ClkInitStruct)
{
#ifdef STM32_SYSCLK_SRC_PLL
	LL_UTILS_PLLInitTypeDef s_PLLInitStruct;

	/* configure PLL input settings */
	config_pll_init(&s_PLLInitStruct);

	/*
	 * Switch to HSI and disable the PLL before configuration.
	 * (Switching to HSI makes sure we have a SYSCLK source in
	 * case we're currently running from the PLL we're about to
	 * turn off and reconfigure.)
	 *
	 * Don't use s_ClkInitStruct.AHBCLKDivider as the AHB
	 * prescaler here. In this configuration, that's the value to
	 * use when the SYSCLK source is the PLL, not HSI.
	 */
	clock_switch_to_hsi(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_PLL1_Disable();

	if (IS_ENABLED(STM32_PLL_Q_DIVISOR)) {
		LL_RCC_PLL1_SetQ(STM32_PLL_Q_DIVISOR);
	}

	set_regu_voltage(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

	if (IS_ENABLED(STM32_PLL_SRC_MSIS)) {
		set_up_clk_msis();

		/* Switch to PLL with MSI as clock source */
		LL_PLL_ConfigSystemClock_MSI(&s_PLLInitStruct, &s_ClkInitStruct);

		/* Disable other clocks */
		LL_RCC_HSI_Disable();
		LL_RCC_HSE_Disable();

	} else if (IS_ENABLED(STM32_PLL_SRC_HSI)) {
		/* Switch to PLL with HSI as clock source */
		LL_PLL_ConfigSystemClock_HSI(&s_PLLInitStruct, &s_ClkInitStruct);

		/* Disable other clocks */
		LL_RCC_HSE_Disable();
		LL_RCC_MSIS_Disable();

	} else if (IS_ENABLED(STM32_PLL_SRC_HSE)) {
		int hse_bypass;

		if (IS_ENABLED(STM32_HSE_BYPASS)) {
			hse_bypass = LL_UTILS_HSEBYPASS_ON;
		} else {
			hse_bypass = LL_UTILS_HSEBYPASS_OFF;
		}

		/* Switch to PLL with HSE as clock source */
		LL_PLL1_ConfigSystemClock_HSE(CONFIG_CLOCK_STM32_HSE_CLOCK,
					      hse_bypass,
					      &s_PLLInitStruct,
					      &s_ClkInitStruct);

		/* Disable other clocks */
		LL_RCC_HSI_Disable();
		LL_RCC_MSIS_Disable();
	}

#endif /* STM32_SYSCLK_SRC_PLL */
}


/*
 * Configure HSE as source of SYSCLK
 */
void config_src_sysclk_hse(LL_UTILS_ClkInitTypeDef s_ClkInitStruct)
{
#ifdef STM32_SYSCLK_SRC_HSE

	uint32_t old_hclk_freq;
	uint32_t new_hclk_freq;

	old_hclk_freq = HAL_RCC_GetHCLKFreq();

	/* Calculate new SystemCoreClock variable based on HSE freq */
	new_hclk_freq = __LL_RCC_CALC_HCLK_FREQ(CONFIG_CLOCK_STM32_HSE_CLOCK,
						s_ClkInitStruct.AHBCLKDivider);

	__ASSERT(new_hclk_freq == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
			 "Config mismatch HCLK frequency %u %u",
			 CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, new_hclk_freq);

	/* If freq increases, set flash latency before any clock setting */
	if (new_hclk_freq > old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	/* Enable HSE if not enabled */
	if (LL_RCC_HSE_IsReady() != 1) {
		/* Check if need to enable HSE bypass feature or not */
		if (IS_ENABLED(STM32_HSE_BYPASS)) {
			LL_RCC_HSE_EnableBypass();
		} else {
			LL_RCC_HSE_DisableBypass();
		}

		/* Enable HSE */
		LL_RCC_HSE_Enable();
		while (LL_RCC_HSE_IsReady() != 1) {
		/* Wait for HSE ready */
		}
	}

	/* Set HSE as SYSCLCK source */
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSE);
	LL_RCC_SetAHBPrescaler(s_ClkInitStruct.AHBCLKDivider);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSE) {
	}

	/* Update SystemCoreClock variable */
	LL_SetSystemCoreClock(new_hclk_freq);

	/* Set peripheral busses prescalers */
	LL_RCC_SetAPB1Prescaler(s_ClkInitStruct.APB1CLKDivider);
	LL_RCC_SetAPB2Prescaler(s_ClkInitStruct.APB2CLKDivider);
	LL_RCC_SetAPB3Prescaler(s_ClkInitStruct.APB3CLKDivider);

	/* If freq not increased, set flash latency after all clock setting */
	if (new_hclk_freq <= old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	/* Disable other clocks */
	LL_RCC_HSI_Disable();
	LL_RCC_MSIS_Disable();
	LL_RCC_PLL1_Disable();

#endif	/* STM32_SYSCLK_SRC_HSE */
}

/*
 * Configure MSI as source of SYSCLK
 */
void config_src_sysclk_msis(LL_UTILS_ClkInitTypeDef s_ClkInitStruct)
{
#ifdef STM32_SYSCLK_SRC_MSIS

	uint32_t old_hclk_freq;
	uint32_t new_hclk_freq;

	old_hclk_freq = HAL_RCC_GetHCLKFreq();

	/* Calculate new SystemCoreClock variable with MSI freq */
	/* MSI freq is defined from RUN range selection */
	new_hclk_freq =	__LL_RCC_CALC_HCLK_FREQ(
				__LL_RCC_CALC_MSIS_FREQ(LL_RCC_MSIRANGESEL_RUN,
				STM32_MSIS_RANGE << RCC_ICSCR1_MSISRANGE_Pos),
				s_ClkInitStruct.AHBCLKDivider);

	__ASSERT(new_hclk_freq == CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
			 "Config mismatch HCLK frequency %u %u",
			 CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC, new_hclk_freq);

	/* If freq increases, set flash latency before any clock setting */
	if (new_hclk_freq > old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	set_regu_voltage(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC);

	/* Set MSIS as SYSCLCK source */
	set_up_clk_msis();
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_MSIS);
	LL_RCC_SetAHBPrescaler(s_ClkInitStruct.AHBCLKDivider);
	while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_MSIS) {
	}

	/* Update SystemCoreClock variable */
	LL_SetSystemCoreClock(new_hclk_freq);

	/* Set peripheral busses prescalers */
	LL_RCC_SetAPB1Prescaler(s_ClkInitStruct.APB1CLKDivider);
	LL_RCC_SetAPB2Prescaler(s_ClkInitStruct.APB2CLKDivider);
	LL_RCC_SetAPB3Prescaler(s_ClkInitStruct.APB3CLKDivider);

	/* If freq not increased, set flash latency after all clock setting */
	if (new_hclk_freq <= old_hclk_freq) {
		LL_SetFlashLatency(new_hclk_freq);
	}

	/* Disable other clocks */
	LL_RCC_HSE_Disable();
	LL_RCC_HSI_Disable();
	LL_RCC_PLL1_Disable();

#endif	/* STM32_SYSCLK_SRC_MSIS */
}

/*
 * Configure HSI as source of SYSCLK
 */
void config_src_sysclk_hsi(LL_UTILS_ClkInitTypeDef s_ClkInitStruct)
{
#ifdef STM32_SYSCLK_SRC_HSI

	clock_switch_to_hsi(s_ClkInitStruct.AHBCLKDivider);

	/* Update SystemCoreClock variable */
	LL_SetSystemCoreClock(__LL_RCC_CALC_HCLK_FREQ(HSI_VALUE,
						s_ClkInitStruct.AHBCLKDivider));

	/* Set peripheral busses prescalers */
	LL_RCC_SetAPB1Prescaler(s_ClkInitStruct.APB1CLKDivider);
	LL_RCC_SetAPB2Prescaler(s_ClkInitStruct.APB2CLKDivider);
	LL_RCC_SetAPB3Prescaler(s_ClkInitStruct.APB3CLKDivider);

	/* Set flash latency */
	/* HSI used as SYSCLK, set latency to 0 */
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);

	/* Disable other clocks */
	LL_RCC_HSE_Disable();
	LL_RCC_MSIS_Disable();
	LL_RCC_PLL1_Disable();

#endif	/* STM32_SYSCLK_SRC_HSI */
}

int stm32_clock_control_init(const struct device *dev)
{
	LL_UTILS_ClkInitTypeDef s_ClkInitStruct;

	ARG_UNUSED(dev);

	/* configure clock for AHB/APB buses */
	config_bus_clk_init((LL_UTILS_ClkInitTypeDef *)&s_ClkInitStruct);

	/* Some clocks would be activated by default */
	config_enable_default_clocks();

	if (IS_ENABLED(STM32_SYSCLK_SRC_PLL)) {
		/* Configure PLL as source of SYSCLK */
		config_src_sysclk_pll(s_ClkInitStruct);
	} else if (IS_ENABLED(STM32_SYSCLK_SRC_HSE)) {
		/* Configure HSE as source of SYSCLK */
		config_src_sysclk_hse(s_ClkInitStruct);
	} else if (IS_ENABLED(STM32_SYSCLK_SRC_MSIS)) {
		/* Configure MSIS as source of SYSCLK */
		config_src_sysclk_msis(s_ClkInitStruct);
	} else if (IS_ENABLED(STM32_SYSCLK_SRC_HSI)) {
		/* Configure HSI as source of SYSCLK */
		config_src_sysclk_hsi(s_ClkInitStruct);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief RCC device, note that priority is intentionally set to 1 so
 * that the device init runs just after SOC init
 */
DEVICE_DT_DEFINE(DT_NODELABEL(rcc),
		    &stm32_clock_control_init,
		    NULL,
		    NULL, NULL,
		    PRE_KERNEL_1,
		    CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		    &stm32_clock_control_api);
