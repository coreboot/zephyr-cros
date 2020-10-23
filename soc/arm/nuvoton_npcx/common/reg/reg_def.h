/*
 * Copyright (c) 2020 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _NUVOTON_NPCX_REG_DEF_H
#define _NUVOTON_NPCX_REG_DEF_H

/*
 * NPCX register structure size/offset checking macro function to mitigate
 * the risk of unexpected compiling results. All addresses of NPCX registers
 * must meet the alignment requirement of cortex-m4.
 * DO NOT use 'packed' attribute if module contains different length ie.
 * 8/16/32 bits registers.
 */
#define NPCX_REG_SIZE_CHECK(reg_def, size) \
	BUILD_ASSERT(sizeof(struct reg_def) == size, \
		"Failed in size check of register structure!")
#define NPCX_REG_OFFSET_CHECK(reg_def, member, offset) \
	BUILD_ASSERT(offsetof(struct reg_def, member) == offset, \
		"Failed in offset check of register structure member!")

/*
 * NPCX register access checking via structure macro function to mitigate the
 * risk of unexpected compiling results if module contains different length
 * registers. For example, a word register access might break into two byte
 * register accesses by adding 'packed' attribute.
 *
 * For example, add this macro for word register 'PRSC' of PWM module in its
 * device init function for checking violation. Once it occurred, core will be
 * stalled forever and easy to find out what happens.
 */
#define NPCX_REG_WORD_ACCESS_CHECK(reg, val) { \
		uint16_t placeholder = reg; \
		reg = val; \
		__ASSERT(reg == val, "16-bit reg access failed!"); \
		reg = placeholder; \
	}
#define NPCX_REG_DWORD_ACCESS_CHECK(reg, val) { \
		uint32_t placeholder = reg; \
		reg = val; \
		__ASSERT(reg == val, "32-bit reg access failed!"); \
		reg = placeholder; \
	}
/*
 * Core Domain Clock Generator (CDCG) device registers
 */
struct cdcg_reg {
	/* High Frequency Clock Generator (HFCG) registers */
	/* 0x000: HFCG Control */
	volatile uint8_t HFCGCTRL;
	volatile uint8_t reserved1;
	/* 0x002: HFCG M Low Byte Value */
	volatile uint8_t HFCGML;
	volatile uint8_t reserved2;
	/* 0x004: HFCG M High Byte Value */
	volatile uint8_t HFCGMH;
	volatile uint8_t reserved3;
	/* 0x006: HFCG N Value */
	volatile uint8_t HFCGN;
	volatile uint8_t reserved4;
	/* 0x008: HFCG Prescaler */
	volatile uint8_t HFCGP;
	volatile uint8_t reserved5[7];
	/* 0x010: HFCG Bus Clock Dividers */
	volatile uint8_t HFCBCD;
	volatile uint8_t reserved6;
	/* 0x012: HFCG Bus Clock Dividers */
	volatile uint8_t HFCBCD1;
	volatile uint8_t reserved7;
	/* 0x014: HFCG Bus Clock Dividers */
	volatile uint8_t HFCBCD2;
	volatile uint8_t reserved8[235];

	/* Low Frequency Clock Generator (LFCG) registers */
	/* 0x100: LFCG Control */
	volatile uint8_t  LFCGCTL;
	volatile uint8_t reserved9;
	/* 0x102: High-Frequency Reference Divisor I */
	volatile uint16_t HFRDI;
	/* 0x104: High-Frequency Reference Divisor F */
	volatile uint16_t HFRDF;
	/* 0x106: FRCLK Clock Divisor */
	volatile uint16_t FRCDIV;
	/* 0x108: Divisor Correction Value 1 */
	volatile uint16_t DIVCOR1;
	/* 0x10A: Divisor Correction Value 2 */
	volatile uint16_t DIVCOR2;
	volatile uint8_t reserved10[8];
	/* 0x114: LFCG Control 2 */
	volatile uint8_t  LFCGCTL2;
	volatile uint8_t  reserved11;
};

/* CDCG register fields */
#define NPCX_HFCGCTRL_LOAD                    0
#define NPCX_HFCGCTRL_LOCK                    2
#define NPCX_HFCGCTRL_CLK_CHNG                7

/*
 * Power Management Controller (PMC) device registers
 */
struct pmc_reg {
	/* 0x000: Power Management Controller */
	volatile uint8_t PMCSR;
	volatile uint8_t reserved1[2];
	/* 0x003: Enable in Sleep Control */
	volatile uint8_t ENIDL_CTL;
	/* 0x004: Disable in Idle Control */
	volatile uint8_t DISIDL_CTL;
	/* 0x005: Disable in Idle Control 1 */
	volatile uint8_t DISIDL_CTL1;
	volatile uint8_t reserved2[2];
	/* 0x008 - 0D: Power-Down Control 1 - 6 */
	volatile uint8_t PWDWN_CTL1[6];
	volatile uint8_t reserved3[18];
	/* 0x020 - 21: Power-Down Control 1 - 2 */
	volatile uint8_t RAM_PD[2];
	volatile uint8_t reserved4[2];
	/* 0x024: Power-Down Control 7 */
	volatile uint8_t PWDWN_CTL7[1];
};

/* PMC multi-registers */
#define NPCX_PWDWN_CTL_OFFSET(n) (((n) < 6) ? (0x008 + n) : (0x024 + (n - 6)))
#define NPCX_PWDWN_CTL(base, n) (*(volatile uint8_t *)(base + \
						NPCX_PWDWN_CTL_OFFSET(n)))

/* PMC register fields */
#define NPCX_PMCSR_DI_INSTW                   0
#define NPCX_PMCSR_DHF                        1
#define NPCX_PMCSR_IDLE                       2
#define NPCX_PMCSR_NWBI                       3
#define NPCX_PMCSR_OHFC                       6
#define NPCX_PMCSR_OLFC                       7
#define NPCX_DISIDL_CTL_RAM_DID               5
#define NPCX_ENIDL_CTL_ADC_LFSL               7
#define NPCX_ENIDL_CTL_LP_WK_CTL              6
#define NPCX_ENIDL_CTL_PECI_ENI               2
#define NPCX_ENIDL_CTL_ADC_ACC_DIS            1

/*
 * System Configuration (SCFG) device registers
 */
struct scfg_reg {
	/* 0x000: Device Control */
	volatile uint8_t DEVCNT;
	/* 0x001: Straps Status */
	volatile uint8_t STRPST;
	/* 0x002: Reset Control and Status */
	volatile uint8_t RSTCTL;
	volatile uint8_t reserved1[3];
	/* 0x006: Device Control 4 */
	volatile uint8_t DEV_CTL4;
	volatile uint8_t reserved2[9];
	/* 0x010 - 1F: Device Alternate Function 0 - F */
	volatile uint8_t DEVALT0[16];
	volatile uint8_t reserved3[6];
	/* 0x026: Low-Voltage GPIO Pins Control 5 */
	volatile uint8_t LV_GPIO_CTL5[1];
	volatile uint8_t reserved4;
	/* 0x028: Pull-Up/Pull-Down Enable 0 */
	volatile uint8_t PUPD_EN0;
	/* 0x029: Pull-Up/Pull-Down Enable 1 */
	volatile uint8_t PUPD_EN1;
	/* 0x02A - 2E: Low-Voltage GPIO Pins Control 0 - 4 */
	volatile uint8_t LV_GPIO_CTL0[5];
};

/* SCFG multi-registers */
#define NPCX_DEVALT_OFFSET(n) (0x010 + (n))
#define NPCX_DEVALT(base, n) (*(volatile uint8_t *)(base + \
						NPCX_DEVALT_OFFSET(n)))

#define NPCX_LV_GPIO_CTL_OFFSET(n) (((n) < 5) ? (0x02A + (n)) \
						: (0x026 + (n - 5)))
#define NPCX_LV_GPIO_CTL(base, n) (*(volatile uint8_t *)(base + \
						NPCX_LV_GPIO_CTL_OFFSET(n)))

/* SCFG register fields */
#define NPCX_DEVCNT_F_SPI_TRIS                6
#define NPCX_DEVCNT_HIF_TYP_SEL_FIELD         FIELD(2, 2)
#define NPCX_DEVCNT_JEN1_HEN                  5
#define NPCX_DEVCNT_JEN0_HEN                  4
#define NPCX_STRPST_TRIST                     1
#define NPCX_STRPST_TEST                      2
#define NPCX_STRPST_JEN1                      4
#define NPCX_STRPST_JEN0                      5
#define NPCX_STRPST_SPI_COMP                  7
#define NPCX_RSTCTL_VCC1_RST_STS              0
#define NPCX_RSTCTL_DBGRST_STS                1
#define NPCX_RSTCTL_VCC1_RST_SCRATCH          3
#define NPCX_RSTCTL_LRESET_PLTRST_MODE        5
#define NPCX_RSTCTL_HIPRST_MODE               6
#define NPCX_DEV_CTL4_F_SPI_SLLK              2
#define NPCX_DEV_CTL4_SPI_SP_SEL              4
#define NPCX_DEV_CTL4_WP_IF                   5
#define NPCX_DEV_CTL4_VCC1_RST_LK             6
#define NPCX_DEVPU0_I2C0_0_PUE                0
#define NPCX_DEVPU0_I2C0_1_PUE                1
#define NPCX_DEVPU0_I2C1_0_PUE                2
#define NPCX_DEVPU0_I2C2_0_PUE                4
#define NPCX_DEVPU0_I2C3_0_PUE                6
#define NPCX_DEVPU1_F_SPI_PUD_EN              7

/*
 * Universal Asynchronous Receiver-Transmitter (UART) device registers
 */
struct uart_reg {
	/* 0x000: Transmit Data Buffer */
	volatile uint8_t UTBUF;
	volatile uint8_t reserved1;
	/* 0x002: Receive Data Buffer */
	volatile uint8_t URBUF;
	volatile uint8_t reserved2;
	/* 0x004: Interrupt Control */
	volatile uint8_t UICTRL;
	volatile uint8_t reserved3;
	/* 0x006: Status */
	volatile uint8_t USTAT;
	volatile uint8_t reserved4;
	/* 0x008: Frame Select */
	volatile uint8_t UFRS;
	volatile uint8_t reserved5;
	/* 0x00A: Mode Select */
	volatile uint8_t UMDSL;
	volatile uint8_t reserved6;
	/* 0x00C: Baud Rate Divisor */
	volatile uint8_t UBAUD;
	volatile uint8_t reserved7;
	/* 0x00E: Baud Rate Prescaler */
	volatile uint8_t UPSR;
	volatile uint8_t reserved8[17];
	/* 0x020: FIFO Mode Transmit Status */
	volatile uint8_t UFTSTS;
	volatile uint8_t reserved9;
	/* 0x022: FIFO Mode Receive Status */
	volatile uint8_t UFRSTS;
	volatile uint8_t reserved10;
	/* 0x024: FIFO Mode Transmit Control */
	volatile uint8_t UFTCTL;
	volatile uint8_t reserved11;
	/* 0x026: FIFO Mode Receive Control */
	volatile uint8_t UFRCTL;
};

/* UART register fields */
#define NPCX_UICTRL_TBE                       0
#define NPCX_UICTRL_RBF                       1
#define NPCX_UICTRL_ETI                       5
#define NPCX_UICTRL_ERI                       6
#define NPCX_UICTRL_EEI                       7
#define NPCX_USTAT_PE                         0
#define NPCX_USTAT_FE                         1
#define NPCX_USTAT_DOE                        2
#define NPCX_USTAT_ERR                        3
#define NPCX_USTAT_BKD                        4
#define NPCX_USTAT_RB9                        5
#define NPCX_USTAT_XMIP                       6
#define NPCX_UFRS_CHAR_FIELD                  FIELD(0, 2)
#define NPCX_UFRS_STP                         2
#define NPCX_UFRS_XB9                         3
#define NPCX_UFRS_PSEL_FIELD                  FIELD(4, 2)
#define NPCX_UFRS_PEN                         6
#define NPCX_UMDSL_FIFO_MD                    0
#define NPCX_UFTSTS_TEMPTY_LVL                FIELD(0, 5)
#define NPCX_UFTSTS_TEMPTY_LVL_STS            5
#define NPCX_UFTSTS_TFIFO_EMPTY_STS           6
#define NPCX_UFTSTS_NXMIP                     7
#define NPCX_UFRSTS_RFULL_LVL_STS             5
#define NPCX_UFRSTS_RFIFO_NEMPTY_STS          6
#define NPCX_UFRSTS_ERR                       7
#define NPCX_UFTCTL_TEMPTY_LVL_SEL            FIELD(0, 5)
#define NPCX_UFTCTL_TEMPTY_LVL_EN             5
#define NPCX_UFTCTL_TEMPTY_EN                 6
#define NPCX_UFTCTL_NXMIPEN                   7
#define NPCX_UFRCTL_RFULL_LVL_SEL             FIELD(0, 5)
#define NPCX_UFRCTL_RFULL_LVL_EN              5
#define NPCX_UFRCTL_RNEMPTY_EN                6
#define NPCX_UFRCTL_ERR_EN                    7

/*
 * Multi-Input Wake-Up Unit (MIWU) device registers
 */

/* MIWU multi-registers */
#define NPCX_WKEDG_OFFSET(n)    (0x000 + ((n) * 2L) + ((n) < 5 ? 0 : 0x1E))
#define NPCX_WKAEDG_OFFSET(n)   (0x001 + ((n) * 2L) + ((n) < 5 ? 0 : 0x1E))
#define NPCX_WKPND_OFFSET(n)    (0x00A + ((n) * 4L) + ((n) < 5 ? 0 : 0x10))
#define NPCX_WKPCL_OFFSET(n)    (0x00C + ((n) * 4L) + ((n) < 5 ? 0 : 0x10))
#define NPCX_WKEN_OFFSET(n)     (0x01E + ((n) * 2L) + ((n) < 5 ? 0 : 0x12))
#define NPCX_WKINEN_OFFSET(n)   (0x01F + ((n) * 2L) + ((n) < 5 ? 0 : 0x12))
#define NPCX_WKMOD_OFFSET(n)    (0x070 + (n))

#define NPCX_WKEDG(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKEDG_OFFSET(n)))
#define NPCX_WKAEDG(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKAEDG_OFFSET(n)))
#define NPCX_WKPND(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKPND_OFFSET(n)))
#define NPCX_WKPCL(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKPCL_OFFSET(n)))
#define NPCX_WKEN(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKEN_OFFSET(n)))
#define NPCX_WKINEN(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKINEN_OFFSET(n)))
#define NPCX_WKMOD(base, n) (*(volatile uint8_t *)(base + \
						NPCX_WKMOD_OFFSET(n)))

/*
 * General-Purpose I/O (GPIO) device registers
 */
struct gpio_reg {
	/* 0x000: Port GPIOx Data Out */
	volatile uint8_t PDOUT;
	/* 0x001: Port GPIOx Data In */
	volatile uint8_t PDIN;
	/* 0x002: Port GPIOx Direction */
	volatile uint8_t PDIR;
	/* 0x003: Port GPIOx Pull-Up or Pull-Down Enable */
	volatile uint8_t PPULL;
	/* 0x004: Port GPIOx Pull-Up/Down Selection */
	volatile uint8_t PPUD;
	/* 0x005: Port GPIOx Drive Enable by VDD Present */
	volatile uint8_t PENVDD;
	/* 0x006: Port GPIOx Output Type */
	volatile uint8_t PTYPE;
	/* 0x007: Port GPIOx Lock Control */
	volatile uint8_t PLOCK_CTL;
};

/*
 * Pulse Width Modulator (PWM) device registers
 */
struct pwm_reg {
	/* 0x000: Clock Prescaler */
	volatile uint16_t PRSC;
	/* 0x002: Cycle Time */
	volatile uint16_t CTR;
	/* 0x004: PWM Control */
	volatile uint8_t PWMCTL;
	volatile uint8_t reserved1;
	/* 0x006: Duty Cycle */
	volatile uint16_t DCR;
	volatile uint8_t reserved2[4];
	/* 0x00C: PWM Control Extended */
	volatile uint8_t PWMCTLEX;
	volatile uint8_t reserved3;
};

/* PWM register fields */
#define NPCX_PWMCTL_INVP                      0
#define NPCX_PWMCTL_CKSEL                     1
#define NPCX_PWMCTL_HB_DC_CTL_FIELD           FIELD(2, 2)
#define NPCX_PWMCTL_PWR                       7
#define NPCX_PWMCTLEX_FCK_SEL_FIELD           FIELD(4, 2)
#define NPCX_PWMCTLEX_OD_OUT                  7

/*
 * Enhanced Serial Peripheral Interface (eSPI) device registers
 */
struct espi_reg {
	/* 0x000: eSPI Identification */
	volatile uint32_t ESPIID;
	/* 0x004: eSPI Configuration */
	volatile uint32_t ESPICFG;
	/* 0x008: eSPI Status */
	volatile uint32_t ESPISTS;
	/* 0x00C: eSPI Interrupt Enable */
	volatile uint32_t ESPIIE;
	/* 0x010: eSPI Wake-Up Enable */
	volatile uint32_t ESPIWE;
	/* 0x014: Virtual Wire Register Index */
	volatile uint32_t VWREGIDX;
	/* 0x018: Virtual Wire Register Data */
	volatile uint32_t VWREGDATA;
	/* 0x01C: OOB Receive Buffer Read Head */
	volatile uint32_t OOBRXRDHEAD;
	/* 0x020: OOB Transmit Buffer Write Head */
	volatile uint32_t OOBTXWRHEAD;
	/* 0x024: OOB Channel Control */
	volatile uint32_t OOBCTL;
	/* 0x028: Flash Receive Buffer Read Head */
	volatile uint32_t FLASHRXRDHEAD;
	/* 0x02C: Flash Transmit Buffer Write Head */
	volatile uint32_t FLASHTXWRHEAD;
	volatile uint32_t reserved1;
	/* 0x034: Flash Channel Configuration */
	volatile uint32_t FLASHCFG;
	/* 0x038: Flash Channel Control */
	volatile uint32_t FLASHCTL;
	/* 0x03C: eSPI Error Status */
	volatile uint32_t ESPIERR;
	/* 0x040: Peripheral Bus Master Receive Buffer Read Head */
	volatile uint32_t PBMRXRDHEAD;
	/* 0x044: Peripheral Bus Master Transmit Buffer Write Head */
	volatile uint32_t PBMTXWRHEAD;
	/* 0x048: Peripheral Channel Configuration */
	volatile uint32_t PERCFG;
	/* 0x04C: Peripheral Channel Control */
	volatile uint32_t PERCTL;
	volatile uint32_t reserved2[44];
	/* 0x100 - 127: Virtual Wire Event Slave-to-Master 0 - 9 */
	volatile uint32_t VWEVSM[10];
	volatile uint32_t reserved3[6];
	/* 0x140 - 16F: Virtual Wire Event Master-to-Slave 0 - 11 */
	volatile uint32_t VWEVMS[12];
	volatile uint32_t reserved4[99];
	/* 0x2FC: Virtual Wire Channel Control */
	volatile uint32_t VWCTL;
	/* 0x300 - 34F: OOB Receive Buffer 0 - 19 */
	volatile uint32_t OOBRXBUF[20];
	volatile uint32_t reserved5[12];
	/* 0x380 - 3CF: OOB Transmit Buffer 0-19 */
	volatile uint32_t OOBTXBUF[20];
	volatile uint32_t reserved6[11];
	/* 0x3FC: OOB Channel Control used in 'direct' mode */
	volatile uint32_t OOBCTL_DIRECT;
	/* 0x400 - 443: Flash Receive Buffer 0-16 */
	volatile uint32_t FLASHRXBUF[17];
	volatile uint32_t reserved7[15];
	/* 0x480 - 497: Flash Transmit Buffer 0-5 */
	volatile uint32_t FLASHTXBUF[6];
	volatile uint32_t reserved8[25];
	/* 0x4FC: Flash Channel Control used in 'direct' mode */
	volatile uint32_t FLASHCTL_DIRECT;
};

/* eSPI register fields */
#define NPCX_ESPICFG_PCHANEN             0
#define NPCX_ESPICFG_VWCHANEN            1
#define NPCX_ESPICFG_OOBCHANEN           2
#define NPCX_ESPICFG_FLASHCHANEN         3
#define NPCX_ESPICFG_HPCHANEN            4
#define NPCX_ESPICFG_HVWCHANEN           5
#define NPCX_ESPICFG_HOOBCHANEN          6
#define NPCX_ESPICFG_HFLASHCHANEN        7
#define NPCX_ESPICFG_CHANS_FIELD         FIELD(0, 4)
#define NPCX_ESPICFG_HCHANS_FIELD        FIELD(4, 4)
#define NPCX_ESPICFG_IOMODE_FIELD        FIELD(8, 9)
#define NPCX_ESPICFG_MAXFREQ_FIELD       FIELD(10, 12)
#define NPCX_ESPICFG_PCCHN_SUPP          24
#define NPCX_ESPICFG_VWCHN_SUPP          25
#define NPCX_ESPICFG_OOBCHN_SUPP         26
#define NPCX_ESPICFG_FLASHCHN_SUPP       27
#define NPCX_ESPIIE_IBRSTIE              0
#define NPCX_ESPIIE_CFGUPDIE             1
#define NPCX_ESPIIE_BERRIE               2
#define NPCX_ESPIIE_OOBRXIE              3
#define NPCX_ESPIIE_FLASHRXIE            4
#define NPCX_ESPIIE_SFLASHRDIE           5
#define NPCX_ESPIIE_PERACCIE             6
#define NPCX_ESPIIE_DFRDIE               7
#define NPCX_ESPIIE_VWUPDIE              8
#define NPCX_ESPIIE_ESPIRSTIE            9
#define NPCX_ESPIIE_PLTRSTIE             10
#define NPCX_ESPIIE_AMERRIE              15
#define NPCX_ESPIIE_AMDONEIE             16
#define NPCX_ESPIIE_BMTXDONEIE           19
#define NPCX_ESPIIE_PBMRXIE              20
#define NPCX_ESPIIE_PMSGRXIE             21
#define NPCX_ESPIIE_BMBURSTERRIE         22
#define NPCX_ESPIIE_BMBURSTDONEIE        23
#define NPCX_ESPIWE_IBRSTWE              0
#define NPCX_ESPIWE_CFGUPDWE             1
#define NPCX_ESPIWE_BERRWE               2
#define NPCX_ESPIWE_OOBRXWE              3
#define NPCX_ESPIWE_FLASHRXWE            4
#define NPCX_ESPIWE_PERACCWE             6
#define NPCX_ESPIWE_DFRDWE               7
#define NPCX_ESPIWE_VWUPDWE              8
#define NPCX_ESPIWE_ESPIRSTWE            9
#define NPCX_ESPIWE_PBMRXWE              20
#define NPCX_ESPIWE_PMSGRXWE             21
#define NPCX_ESPISTS_IBRST               0
#define NPCX_ESPISTS_CFGUPD              1
#define NPCX_ESPISTS_BERR                2
#define NPCX_ESPISTS_OOBRX               3
#define NPCX_ESPISTS_FLASHRX             4
#define NPCX_ESPISTS_PERACC              6
#define NPCX_ESPISTS_DFRD                7
#define NPCX_ESPISTS_VWUPD               8
#define NPCX_ESPISTS_ESPIRST             9
#define NPCX_ESPISTS_PLTRST              10
#define NPCX_ESPISTS_AMERR               15
#define NPCX_ESPISTS_AMDONE              16
#define NPCX_ESPISTS_VWUPDW              17
#define NPCX_ESPISTS_BMTXDONE            19
#define NPCX_ESPISTS_PBMRX               20
#define NPCX_ESPISTS_PMSGRX              21
#define NPCX_ESPISTS_BMBURSTERR          22
#define NPCX_ESPISTS_BMBURSTDONE         23
#define NPCX_ESPISTS_ESPIRST_LVL         24
#define NPCX_VWEVMS_WIRE                 FIELD(0, 4)
#define NPCX_VWEVMS_VALID                FIELD(4, 4)
#define NPCX_VWEVMS_IE                   18
#define NPCX_VWEVMS_WE                   20
#define NPCX_VWEVSM_WIRE                 FIELD(0, 4)
#define NPCX_VWEVSM_VALID                FIELD(4, 4)
#define NPCX_VWEVSM_BIT_VALID(n)         (4+n)
#define NPCX_OOBCTL_OOB_FREE             0
#define NPCX_OOBCTL_OOB_AVAIL            1
#define NPCX_OOBCTL_RSTBUFHEADS          2
#define NPCX_OOBCTL_OOBPLSIZE            FIELD(10, 3)
#define NPCX_FLASHCFG_FLASHBLERSSIZE     FIELD(7, 3)
#define NPCX_FLASHCFG_FLASHPLSIZE        FIELD(10, 3)
#define NPCX_FLASHCFG_FLASHREQSIZE       FIELD(13, 3)
#define NPCX_FLASHCTL_FLASH_NP_FREE      0
#define NPCX_FLASHCTL_FLASH_TX_AVAIL     1
#define NPCX_FLASHCTL_STRPHDR            2
#define NPCX_FLASHCTL_DMATHRESH          FIELD(3, 2)
#define NPCX_FLASHCTL_AMTSIZE            FIELD(5, 8)
#define NPCX_FLASHCTL_RSTBUFHEADS        13
#define NPCX_FLASHCTL_CRCEN              14
#define NPCX_FLASHCTL_CHKSUMSEL          15
#define NPCX_FLASHCTL_AMTEN              16

/*
 * Mobile System Wake-Up Control (MSWC) device registers
 */
struct mswc_reg {
	/* 0x000: MSWC Control Status 1 */
	volatile uint8_t MSWCTL1;
	volatile uint8_t reserved1;
	/* 0x002: MSWC Control Status 2 */
	volatile uint8_t MSWCTL2;
	volatile uint8_t reserved2[5];
	/* 0x008: Host Configuration Base Address Low */
	volatile uint8_t HCBAL;
	volatile uint8_t reserved3;
	/* 0x00A: Host Configuration Base Address High */
	volatile uint8_t HCBAH;
	volatile uint8_t reserved4;
	/* 0X00C: MSWC INTERRUPT ENABLE 2 */
	volatile uint8_t MSIEN2;
	volatile uint8_t reserved5;
	/* 0x00E: MSWC Host Event Status 0 */
	volatile uint8_t MSHES0;
	volatile uint8_t reserved6;
	/* 0x010: MSWC Host Event Interrupt Enable */
	volatile uint8_t MSHEIE0;
	volatile uint8_t reserved7;
	/* 0x012: Host Control */
	volatile uint8_t HOST_CTL;
	volatile uint8_t reserved8;
	/* 0x014: SMI Pulse Length */
	volatile uint8_t SMIP_LEN;
	volatile uint8_t reserved9;
	/* 0x016: SCI Pulse Length */
	volatile uint8_t SCIP_LEN;
	volatile uint8_t reserved10[5];
	/* 0x01C: SRID Core Access */
	volatile uint8_t SRID_CR;
	volatile uint8_t reserved11[3];
	/* 0x020: SID Core Access */
	volatile uint8_t SID_CR;
	volatile uint8_t reserved12;
	/* 0x022: DEVICE_ID Core Access */
	volatile uint8_t DEVICE_ID_CR;
	volatile uint8_t reserved13[5];
	/* 0x028: Chip Revision Core Access */
	volatile uint8_t CHPREV_CR;
	volatile uint8_t reserved14[5];
	/* 0x02E: Virtual Wire Sleep States */
	volatile uint8_t VW_SLPST1;
	volatile uint8_t reserved15;
};

/* MSWC register fields */
#define NPCX_MSWCTL1_HRSTOB              0
#define NPCS_MSWCTL1_HWPRON              1
#define NPCX_MSWCTL1_PLTRST_ACT          2
#define NPCX_MSWCTL1_VHCFGA              3
#define NPCX_MSWCTL1_HCFGLK              4
#define NPCX_MSWCTL1_PWROFFB             6
#define NPCX_MSWCTL1_A20MB               7

/*
 * Shared Memory (SHM) device registers
 */
struct shm_reg {
	/* 0x000: Shared Memory Core Status */
	volatile uint8_t SMC_STS;
	/* 0x001: Shared Memory Core Control */
	volatile uint8_t SMC_CTL;
	/* 0x002: Shared Memory Host Control */
	volatile uint8_t SHM_CTL;
	volatile uint8_t reserved1[2];
	/* 0x005: Indirect Memory Access Window Size */
	volatile uint8_t IMA_WIN_SIZE;
	volatile uint8_t reserved2;
	/* 0x007: Shared Access Windows Size */
	volatile uint8_t WIN_SIZE;
	/* 0x008: Shared Access Window 1, Semaphore */
	volatile uint8_t SHAW1_SEM;
	/* 0x009: Shared Access Window 2, Semaphore */
	volatile uint8_t SHAW2_SEM;
	volatile uint8_t reserved3;
	/* 0x00B: Indirect Memory Access, Semaphore */
	volatile uint8_t IMA_SEM;
	volatile uint8_t reserved4[2];
	/* 0x00E: Shared Memory Configuration */
	volatile uint16_t SHCFG;
	/* 0x010: Shared Access Window 1 Write Protect */
	volatile uint8_t WIN1_WR_PROT;
	/* 0x011: Shared Access Window 1 Read Protect */
	volatile uint8_t WIN1_RD_PROT;
	/* 0x012: Shared Access Window 2 Write Protect */
	volatile uint8_t WIN2_WR_PROT;
	/* 0x013: Shared Access Window 2 Read Protect */
	volatile uint8_t WIN2_RD_PROT;
	volatile uint8_t reserved5[2];
	/* 0x016: Indirect Memory Access Write Protect */
	volatile uint8_t IMA_WR_PROT;
	/* 0x017: Indirect Memory Access Read Protect */
	volatile uint8_t IMA_RD_PROT;
	volatile uint8_t reserved6[8];
	/* 0x020: Shared Access Window 1 Base */
	volatile uint32_t WIN_BASE1;
	/* 0x024: Shared Access Window 2 Base */
	volatile uint32_t WIN_BASE2;
	volatile uint32_t reserved7;
	/* 0x02C: Indirect Memory Access Base */
	volatile uint32_t IMA_BASE;
	volatile uint8_t reserved8[10];
	/* 0x03A: Reset Configuration */
	volatile uint8_t RST_CFG;
	volatile uint8_t reserved9[5];
	/* 0x040: Debug Port 80 Buffered Data */
	volatile uint16_t DP80BUF;
	/* 0x042: Debug Port 80 Status */
	volatile uint8_t DP80STS;
	volatile uint8_t reserved10;
	/* 0x044: Debug Port 80 Control */
	volatile uint8_t DP80CTL;
	volatile uint8_t reserved11[3];
	/* 0x048: Host_Offset in Windows 1, 2 Status */
	volatile uint8_t HOFS_STS;
	/* 0x049: Host_Offset in Windows 1, 2 Control */
	volatile uint8_t HOFS_CTL;
	/* 0x04A: Core_Offset in Window 2 Address */
	volatile uint16_t COFS2;
	/* 0x04C: Core_Offset in Window 1 Address */
	volatile uint16_t COFS1;
	volatile uint16_t reserved12;
};

/* SHM register fields */
#define NPCX_SMC_STS_HRERR               0
#define NPCX_SMC_STS_HWERR               1
#define NPCX_SMC_STS_HSEM1W              4
#define NPCX_SMC_STS_HSEM2W              5
#define NPCX_SMC_STS_SHM_ACC             6
#define NPCX_SMC_CTL_HERR_IE             2
#define NPCX_SMC_CTL_HSEM1_IE            3
#define NPCX_SMC_CTL_HSEM2_IE            4
#define NPCX_SMC_CTL_ACC_IE              5
#define NPCX_SMC_CTL_PREF_EN             6
#define NPCX_SMC_CTL_HOSTWAIT            7
#define NPCX_FLASH_SIZE_STALL_HOST       6
#define NPCX_FLASH_SIZE_RD_BURST         7
#define NPCX_WIN_PROT_RW1L_RP            0
#define NPCX_WIN_PROT_RW1L_WP            1
#define NPCX_WIN_PROT_RW1H_RP            2
#define NPCX_WIN_PROT_RW1H_WP            3
#define NPCX_WIN_PROT_RW2L_RP            4
#define NPCX_WIN_PROT_RW2L_WP            5
#define NPCX_WIN_PROT_RW2H_RP            6
#define NPCX_WIN_PROT_RW2H_WP            7
#define NPCX_PWIN_SIZEI_RPROT            13
#define NPCX_PWIN_SIZEI_WPROT            14
#define NPCX_CSEM2                       6
#define NPCX_CSEM3                       7
#define NPCX_DP80STS_FWR                 5
#define NPCX_DP80STS_FNE                 6
#define NPCX_DP80STS_FOR                 7
#define NPCX_DP80CTL_DP80EN              0
#define NPCX_DP80CTL_SYNCEN              1
#define NPCX_DP80CTL_ADV                 2
#define NPCX_DP80CTL_RAA                 3
#define NPCX_DP80CTL_RFIFO               4
#define NPCX_DP80CTL_CIEN                5
#define NPCX_DP80CTL_DP80_HF_CFG         7

/*
 * Keyboard and Mouse Controller (KBC) device registers
 */
struct kbc_reg {
	/* 0x000h: Host Interface Control */
	volatile uint8_t HICTRL;
	volatile uint8_t reserved1;
	/* 0x002h: Host Interface IRQ Control */
	volatile uint8_t HIIRQC;
	volatile uint8_t reserved2;
	/* 0x004h: Host Interface Keyboard/Mouse Status */
	volatile uint8_t HIKMST;
	volatile uint8_t reserved3;
	/* 0x006h: Host Interface Keyboard Data Out Buffer */
	volatile uint8_t HIKDO;
	volatile uint8_t reserved4;
	/* 0x008h: Host Interface Mouse Data Out Buffer */
	volatile uint8_t HIMDO;
	volatile uint8_t reserved5;
	/* 0x00Ah: Host Interface Keyboard/Mouse Data In Buffer */
	volatile uint8_t HIKMDI;
	/* 0x00Bh: Host Interface Keyboard/Mouse Shadow Data In Buffer */
	volatile uint8_t SHIKMDI;
};

/* KBC register field */
#define NPCX_HICTRL_OBFKIE               0
#define NPCX_HICTRL_OBFMIE               1
#define NPCX_HICTRL_OBECIE               2
#define NPCX_HICTRL_IBFCIE               3
#define NPCX_HICTRL_PMIHIE               4
#define NPCX_HICTRL_PMIOCIE              5
#define NPCX_HICTRL_PMICIE               6
#define NPCX_HICTRL_FW_OBF               7
#define NPCX_HIKMST_OBF                  0
#define NPCX_HIKMST_IBF                  1
#define NPCX_HIKMST_F0                   2
#define NPCX_HIKMST_A2                   3
#define NPCX_HIKMST_ST0                  4
#define NPCX_HIKMST_ST1                  5
#define NPCX_HIKMST_ST2                  6
#define NPCX_HIKMST_ST3                  7

/*
 * Power Management Channel (PMCH) device registers
 */

struct pmch_reg {
	/* 0x000: Host Interface PM Status */
	volatile uint8_t HIPMST;
	volatile uint8_t reserved1;
	/* 0x002: Host Interface PM Data Out Buffer */
	volatile uint8_t HIPMDO;
	volatile uint8_t reserved2;
	/* 0x004: Host Interface PM Data In Buffer */
	volatile uint8_t HIPMDI;
	/* 0x005: Host Interface PM Shadow Data In Buffer */
	volatile uint8_t SHIPMDI;
	/* 0x006: Host Interface PM Data Out Buffer with SCI */
	volatile uint8_t HIPMDOC;
	volatile uint8_t reserved3;
	/* 0x008: Host Interface PM Data Out Buffer with SMI */
	volatile uint8_t HIPMDOM;
	volatile uint8_t reserved4;
	/* 0x00A: Host Interface PM Data In Buffer with SCI */
	volatile uint8_t HIPMDIC;
	volatile uint8_t reserved5;
	/* 0x00C: Host Interface PM Control */
	volatile uint8_t HIPMCTL;
	/* 0x00D: Host Interface PM Control 2 */
	volatile uint8_t HIPMCTL2;
	/* 0x00E: Host Interface PM Interrupt Control */
	volatile uint8_t HIPMIC;
	volatile uint8_t reserved6;
	/* 0x010: Host Interface PM Interrupt Enable */
	volatile uint8_t HIPMIE;
	volatile uint8_t reserved7;
};

/* PMCH register field */
#define NPCX_HIPMIE_SCIE                 1
#define NPCX_HIPMIE_SMIE                 2
#define NPCX_HIPMCTL_IBFIE               0
#define NPCX_HIPMCTL_SCIPOL              6
#define NPCX_HIPMST_OBF                  0
#define NPCX_HIPMST_IBF                  1
#define NPCX_HIPMST_F0                   2
#define NPCX_HIPMST_CMD                  3
#define NPCX_HIPMST_ST0                  4
#define NPCX_HIPMST_ST1                  5
#define NPCX_HIPMST_ST2                  6
#define NPCX_HIPMIC_SMIB                 1
#define NPCX_HIPMIC_SCIB                 2
#define NPCX_HIPMIC_SMIPOL               6

/*
 * Core Access to Host (C2H) device registers
 */
struct c2h_reg {
	/* 0x000: Indirect Host I/O Address */
	volatile uint16_t IHIOA;
	/* 0x002: Indirect Host Data */
	volatile uint8_t IHD;
	volatile uint8_t reserved1;
	/* 0x004: Lock Host Access */
	volatile uint16_t LKSIOHA;
	/* 0x006: Access Lock Violation */
	volatile uint16_t SIOLV;
	/* 0x008: Core-to-Host Modules Access Enable */
	volatile uint16_t CRSMAE;
	/* 0x00A: Module Control */
	volatile uint8_t SIBCTRL;
	volatile uint8_t reserved3;
};

/* C2H register fields */
#define NPCX_LKSIOHA_LKCFG               0
#define NPCX_LKSIOHA_LKSPHA              2
#define NPCX_LKSIOHA_LKHIKBD             11
#define NPCX_CRSMAE_CFGAE                0
#define NPCX_CRSMAE_HIKBDAE              11
#define NPCX_SIOLV_SPLV                  2
#define NPCX_SIBCTRL_CSAE                0
#define NPCX_SIBCTRL_CSRD                1
#define NPCX_SIBCTRL_CSWR                2

#endif /* _NUVOTON_NPCX_REG_DEF_H */
