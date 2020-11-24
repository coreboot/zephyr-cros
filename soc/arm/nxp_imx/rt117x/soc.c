/*
 * Copyright (c) 2017,2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <linker/sections.h>
#include <linker/linker-defs.h>
#include <fsl_clock.h>
#include <fsl_gpc.h>
#include <fsl_pmu.h>
#include <fsl_dcdc.h>
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>
#include <fsl_flexspi_nor_boot.h>
#if CONFIG_USB_DC_NXP_EHCI
#include "usb_phy.h"
#include "usb_dc_mcux.h"
#endif

#if CONFIG_USB_DC_NXP_EHCI /* USB PHY condfiguration */
#define BOARD_USB_PHY_D_CAL (0x0CU)
#define BOARD_USB_PHY_TXCAL45DP (0x06U)
#define BOARD_USB_PHY_TXCAL45DM (0x06U)
#endif

static const clock_sys_pll2_config_t sysPll2Config = {
	.ssEnable = false,
};

#ifdef CONFIG_INIT_ENET_PLL
static const clock_sys_pll1_config_t sysPll1Config = {
	.pllDiv2En = true,
};
#endif

#if CONFIG_USB_DC_NXP_EHCI
	usb_phy_config_struct_t usbPhyConfig = {
		BOARD_USB_PHY_D_CAL,
		BOARD_USB_PHY_TXCAL45DP,
		BOARD_USB_PHY_TXCAL45DM,
	};
#endif

#ifdef CONFIG_NXP_IMX_RT_BOOT_HEADER
const __imx_boot_data_section BOOT_DATA_T boot_data = {
	.start = CONFIG_FLASH_BASE_ADDRESS,
	.size = KB(CONFIG_FLASH_SIZE),
	.plugin = PLUGIN_FLAG,
	.placeholder = 0xFFFFFFFF,
};

extern char __start[];
const __imx_boot_ivt_section ivt image_vector_table = {
	.hdr = IVT_HEADER,
	.entry = (uint32_t) __start,
	.reserved1 = IVT_RSVD,
#ifdef CONFIG_DEVICE_CONFIGURATION_DATA
	.dcd = (uint32_t) dcd_data,
#else
	.dcd = (uint32_t) NULL,
#endif
	.boot_data = (uint32_t) &boot_data,
	.self = (uint32_t) &image_vector_table,
	.csf = (uint32_t)CSF_ADDRESS,
	.reserved2 = IVT_RSVD,
};
#endif

/**
 *
 * @brief Initialize the system clock
 *
 * @return N/A
 *
 */
static ALWAYS_INLINE void clock_init(void)
{
	clock_root_config_t rootCfg = {0};

	/* Keep core clock ungated during WFI */
	CCM->GPR_PRIVATE1_SET = 0x1;
#if CONFIG_ADJUST_DCDC
	DCDC_SetVDD1P0BuckModeTargetVoltage(DCDC, kDCDC_1P0BuckTarget1P15V);
#endif

#if CONFIG_ENABLE_FBB
	PMU_EnableBodyBias(ANADIG_PMU, kPMU_FBB_CM7, true);
#endif

#if CONFIG_BYPASS_LDO_LPSR
	PMU_StaticEnableLpsrAnaLdoBypassMode(ANADIG_LDO_SNVS, true);
	PMU_StaticEnableLpsrDigLdoBypassMode(ANADIG_LDO_SNVS, true);
#endif

#if CONFIG_ADJUST_LDO
	pmu_static_lpsr_ana_ldo_config_t lpsrAnaConfig;
	pmu_static_lpsr_dig_config_t lpsrDigConfig;

	if ((ANADIG_LDO_SNVS->PMU_LDO_LPSR_ANA &
		ANADIG_LDO_SNVS_PMU_LDO_LPSR_ANA_BYPASS_MODE_EN_MASK) == 0UL) {
		PMU_StaticGetLpsrAnaLdoDefaultConfig(&lpsrAnaConfig);
		PMU_StaticLpsrAnaLdoInit(ANADIG_LDO_SNVS, &lpsrAnaConfig);
	}

	if ((ANADIG_LDO_SNVS->PMU_LDO_LPSR_DIG &
		ANADIG_LDO_SNVS_PMU_LDO_LPSR_DIG_BYPASS_MODE_MASK) == 0UL) {
		PMU_StaticGetLpsrDigLdoDefaultConfig(&lpsrDigConfig);
		lpsrDigConfig.targetVoltage =
			kPMU_LpsrDigTargetStableVoltage1P117V;
		PMU_StaticLpsrDigLdoInit(ANADIG_LDO_SNVS, &lpsrDigConfig);
	}
#endif

	/* PLL LDO shall be enabled first before enable PLLs */
	CLOCK_OSC_EnableOsc24M();

#if CONFIG_UPDATE_CM7_CLOCK
	if (CLOCK_GetRootClockMux(kCLOCK_Root_M7)
			== kCLOCK_M7_ClockRoot_MuxArmPllOut) {
		rootCfg.mux = kCLOCK_M7_ClockRoot_MuxOscRc400M;
		rootCfg.div = 1;
		CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);
	}
	CLOCK_InitArmPllWithFreq(CONFIG_CM7_FREQ);

	rootCfg.mux = kCLOCK_M7_ClockRoot_MuxArmPllOut;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_M7, &rootCfg);
#endif

#ifdef CONFIG_INIT_ENET_PLL
	CLOCK_InitSysPll1(&sysPll1Config);
#endif

	CLOCK_InitSysPll2(&sysPll2Config);


#ifdef CONFIG_UPDATE_CM4_CLOCK
	if (CLOCK_GetRootClockMux(kCLOCK_Root_M4)
			== kCLOCK_M4_ClockRoot_MuxSysPll3Pfd3) {
		rootCfg.mux = kCLOCK_M4_ClockRoot_MuxOscRc400M;
		rootCfg.div = 1;
		CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);
	}

	if (CLOCK_GetRootClockMux(kCLOCK_Root_Bus_Lpsr)
			== kCLOCK_BUS_LPSR_ClockRoot_MuxSysPll3Out) {
		rootCfg.mux = kCLOCK_M4_ClockRoot_MuxOscRc400M;
		rootCfg.div = 3;
		CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);
	}
#endif

	CLOCK_InitSysPll3();
	CLOCK_InitPfd(kCLOCK_PllSys3, kCLOCK_Pfd3, 22);

#ifdef CONFIG_UPDATE_CM4_CLOCK
	/* Configure M4 using SysPll3Pfd3 divided by 1 */
	rootCfg.mux = kCLOCK_M4_ClockRoot_MuxSysPll3Pfd3;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_M4, &rootCfg);

	/* SysPll3 divide by 3 */
	rootCfg.mux = kCLOCK_BUS_LPSR_ClockRoot_MuxSysPll3Out;
	rootCfg.div = 3;
	CLOCK_SetRootClock(kCLOCK_Root_Bus_Lpsr, &rootCfg);
#endif

	/* SysPll3 divide by 2 */
	rootCfg.mux = kCLOCK_BUS_ClockRoot_MuxSysPll3Out;
	rootCfg.div = 2;
	CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootCfg);

#ifdef CONFIG_UART_MCUX_LPUART
	/* Configure Lpuart1 using SysPll2*/
	rootCfg.mux = kCLOCK_LPUART1_ClockRoot_MuxSysPll2Out;
	rootCfg.div = 22;
	CLOCK_SetRootClock(kCLOCK_Root_Lpuart1, &rootCfg);

	/* Configure Lpuart2 using SysPll2*/
	rootCfg.mux = kCLOCK_LPUART2_ClockRoot_MuxSysPll2Out;
	rootCfg.div = 22;
	CLOCK_SetRootClock(kCLOCK_Root_Lpuart2, &rootCfg);
#endif

	CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd1, 16);
	/* Configure Semc using SysPll2Pfd1 divided by 3 */
	rootCfg.mux = kCLOCK_SEMC_ClockRoot_MuxSysPll2Pfd1;
	rootCfg.div = 3;
	CLOCK_SetRootClock(kCLOCK_Root_Semc, &rootCfg);

	/* Configure Bus using SysPll3 divided by 2 */
	rootCfg.mux = kCLOCK_BUS_ClockRoot_MuxSysPll3Out;
	rootCfg.div = 2;
	CLOCK_SetRootClock(kCLOCK_Root_Bus, &rootCfg);

#ifdef CONFIG_I2C_MCUX_LPI2C
	/* Configure Lpi2c1 using Osc48MDiv2 */
	rootCfg.mux = kCLOCK_LPI2C1_ClockRoot_MuxOscRc48MDiv2;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_Lpi2c1, &rootCfg);

	/* Configure Lpi2c5 using Osc48MDiv2 */
	rootCfg.mux = kCLOCK_LPI2C5_ClockRoot_MuxOscRc48MDiv2;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_Lpi2c5, &rootCfg);
#endif

#ifdef CONFIG_SPI_MCUX_LPSPI
	/* Configure lpspi using Osc48MDiv2 */
	rootCfg.mux = kCLOCK_LPSPI1_ClockRoot_MuxOscRc48MDiv2;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_Lpspi1, &rootCfg);
#endif

#ifdef CONFIG_DISPLAY_MCUX_ELCDIF
	rootCfg.mux = kCLOCK_LCDIF_ClockRoot_MuxSysPll2Out;
	rootCfg.div = 15;
	CLOCK_SetRootClock(kCLOCK_Root_Lcdif, &rootCfg);
#endif

#if CONFIG_USB_DC_NXP_EHCI
	CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usb480M,
				DT_PROP_BY_PHANDLE(DT_INST(0, nxp_kinetis_usbd),
				clocks, clock_frequency));
	CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M,
				DT_PROP_BY_PHANDLE(DT_INST(0, nxp_kinetis_usbd),
				clocks, clock_frequency));
	USB_EhciPhyInit(kUSB_ControllerEhci0, CPU_XTAL_CLK_HZ, &usbPhyConfig);
#endif

#ifdef CONFIG_DISK_ACCESS_USDHC1
	CLOCK_InitPfd(kCLOCK_PllSys2, kCLOCK_Pfd2, 24);
	/* Configure USDHC clock source and divider */
	rootCfg.mux = kCLOCK_USDHC1_ClockRoot_MuxSysPll2Pfd2;
	rootCfg.div = 2;
	CLOCK_SetRootClock(kCLOCK_Root_Usdhc1, &rootCfg);

	/* ERR050396
	 * Errata description:
	 *  AXI to AHB conversion for CM7 AHBS port (port to access CM7 to
	 * TCM) is by a NIC301 block, instead of XHB400 block. NIC301 doesn’t
	 * support sparse write conversion. Any AXI to AHB conversion need
	 * XHB400, not by NIC. This will result in data corruption in case
	 * of AXI sparse write reaches the NIC301 ahead of AHBS.
	 *
	 * Errata workaround:
	 *  For uSDHC, don’t set the bit#1 of IOMUXC_GPR28 (AXI transaction
	 *  is cacheable), if write data to TCM aligned in 4 * bytes; No
	 *  such write access limitation for OCRAM or external RAM
	 */
	IOMUXC_GPR->GPR28 &= (~IOMUXC_GPR_GPR28_AWCACHE_USDHC_MASK);
#endif

#ifdef CONFIG_CAN_MCUX_FLEXCAN
	rootCfg.mux = kCLOCK_CAN3_ClockRoot_MuxOsc24MOut;
	rootCfg.div = 1;
	CLOCK_SetRootClock(kCLOCK_Root_Can3, &rootCfg);
#endif

	GPC_CM_SetNextCpuMode(GPC_CPU_MODE_CTRL_0, kGPC_RunMode);
	GPC_CM_SetNextCpuMode(GPC_CPU_MODE_CTRL_1, kGPC_RunMode);
}

#if defined(CONFIG_DISK_ACCESS_USDHC1) ||	\
	defined(CONFIG_DISK_ACCESS_USDHC2)

/* Usdhc driver needs to re-configure pinmux
 * Pinmux depends on board design.
 * From the perspective of Usdhc driver,
 * it can't access board specific function.
 * So SoC provides this for board to register
 * its usdhc pinmux and for usdhc to access
 * pinmux.
 */

static usdhc_pin_cfg_cb g_usdhc_pin_cfg_cb;

void imxrt_usdhc_pinmux_cb_register(usdhc_pin_cfg_cb cb)
{
	g_usdhc_pin_cfg_cb = cb;
}

void imxrt_usdhc_pinmux(uint16_t nusdhc, bool init,
	uint32_t speed, uint32_t strength)
{
	if (g_usdhc_pin_cfg_cb)
		g_usdhc_pin_cfg_cb(nusdhc, init,
			speed, strength);
}
#endif

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */

static int imxrt_init(const struct device *arg)
{
	ARG_UNUSED(arg);

	unsigned int oldLevel; /* old interrupt lock level */

	/* disable interrupts */
	oldLevel = irq_lock();

#if CONFIG_SOC_MIMXRT1176_CM7 == 7
	if (SCB_CCR_IC_Msk != (SCB_CCR_IC_Msk & SCB->CCR)) {
		SCB_EnableICache();
	}
	if (SCB_CCR_DC_Msk != (SCB_CCR_DC_Msk & SCB->CCR)) {
		SCB_EnableDCache();
	}
#endif

#if CONFIG_SOC_MIMXRT1176_CM4 == 4
	/* Initialize Cache */
	/* Enable Code Bus Cache */
	if (0U == (LMEM->PCCCR & LMEM_PCCCR_ENCACHE_MASK)) {
		/*
		 * set command to invalidate all ways,
		 * and write GO bit to initiate command
		 */
		LMEM->PCCCR |= LMEM_PCCCR_INVW1_MASK
			| LMEM_PCCCR_INVW0_MASK | LMEM_PCCCR_GO_MASK;
		/* Wait until the command completes */
		while ((LMEM->PCCCR & LMEM_PCCCR_GO_MASK) != 0U) {
		}
		/* Enable cache, enable write buffer */
		LMEM->PCCCR |= (LMEM_PCCCR_ENWRBUF_MASK
				| LMEM_PCCCR_ENCACHE_MASK);
	}

	/* Enable System Bus Cache */
	if (0U == (LMEM->PSCCR & LMEM_PSCCR_ENCACHE_MASK)) {
		/*
		 * set command to invalidate all ways,
		 * and write GO bit to initiate command
		 */
		LMEM->PSCCR |= LMEM_PSCCR_INVW1_MASK
			| LMEM_PSCCR_INVW0_MASK | LMEM_PSCCR_GO_MASK;
		/* Wait until the command completes */
		while ((LMEM->PSCCR & LMEM_PSCCR_GO_MASK) != 0U) {
		}
		/* Enable cache, enable write buffer */
		LMEM->PSCCR |= (LMEM_PSCCR_ENWRBUF_MASK
				| LMEM_PSCCR_ENCACHE_MASK);
	}
#endif

	/* Initialize system clock */
	clock_init();

	/*
	 * install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* Clear bit 13 to its reset value since it might be set by ROM. */
	IOMUXC_GPR->GPR28 &= ~IOMUXC_GPR_GPR28_CACHE_USB_MASK;

	/* restore interrupt state */
	irq_unlock(oldLevel);
	return 0;
}

SYS_INIT(imxrt_init, PRE_KERNEL_1, 0);
