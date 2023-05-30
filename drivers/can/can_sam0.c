/*
 * Copyright (c) 2022 Vestas Wind Systems A/S
 * Copyright (c) 2021 Alexander Wachter
 * Copyright (c) 2022 Kamil Serwus
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/can.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <soc.h>

#include "can_mcan.h"

LOG_MODULE_REGISTER(can_sam0, CONFIG_CAN_LOG_LEVEL);

#define DT_DRV_COMPAT atmel_sam0_can

struct can_sam0_config {
	mm_reg_t base;
	void (*config_irq)(void);
	const struct pinctrl_dev_config *pcfg;
	volatile uint32_t *mclk;
	uint32_t mclk_mask;
	uint16_t gclk_core_id;
	int divider;
};

struct can_sam0_data {
	struct can_mcan_msg_sram msg_ram;
};

static int can_sam0_read_reg(const struct device *dev, uint16_t reg, uint32_t *val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_sam0_config *sam_config = mcan_config->custom;

	return can_mcan_sys_read_reg(sam_config->base, reg, val);
}

static int can_sam0_write_reg(const struct device *dev, uint16_t reg, uint32_t val)
{
	const struct can_mcan_config *mcan_config = dev->config;
	const struct can_sam0_config *sam_config = mcan_config->custom;

	switch (reg) {
	case CAN_MCAN_ILS:
		/* All interrupts are assigned to MCAN_INT0 */
		val = 0;
		break;
	case CAN_MCAN_ILE:
		/* SAM0 has only one line to handle interrupts */
		val = CAN_MCAN_ILE_EINT0;
		break;
	default:
		/* No field remap needed */
		break;
	};

	return can_mcan_sys_write_reg(sam_config->base, reg, val);
}

void can_sam0_line_x_isr(const struct device *dev)
{
	can_mcan_line_0_isr(dev);
	can_mcan_line_1_isr(dev);
}

static int can_sam0_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_sam0_config *sam_cfg = mcan_cfg->custom;

	*rate = SOC_ATMEL_SAM0_OSC48M_FREQ_HZ / (sam_cfg->divider);

	return 0;
}

static void can_sam0_clock_enable(const struct can_sam0_config *cfg)
{
	/* Enable the GLCK7 with DIV*/
	GCLK->GENCTRL[7].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_OSC48M)
			     | GCLK_GENCTRL_DIV(cfg->divider)
			     | GCLK_GENCTRL_GENEN;

	/* Route channel */
	GCLK->PCHCTRL[cfg->gclk_core_id].reg = GCLK_PCHCTRL_GEN_GCLK7
					     | GCLK_PCHCTRL_CHEN;

	/* Enable CAN clock in MCLK */
	*cfg->mclk |= cfg->mclk_mask;
}

static int can_sam0_init(const struct device *dev)
{
	const struct can_mcan_config *mcan_cfg = dev->config;
	const struct can_sam0_config *sam_cfg = mcan_cfg->custom;
	int ret;

	can_sam0_clock_enable(sam_cfg);

	ret = pinctrl_apply_state(sam_cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("failed to apply pinctrl");
		return ret;
	}

	ret = can_mcan_configure_message_ram(dev, 0U);
	if (ret != 0) {
		LOG_ERR("failed to configure message ram");
		return ret;
	}

	ret = can_mcan_init(dev);
	if (ret != 0) {
		LOG_ERR("failed to mcan init");
		return ret;
	}

	sam_cfg->config_irq();

	return ret;
}

static const struct can_driver_api can_sam0_driver_api = {
	.get_capabilities = can_mcan_get_capabilities,
	.start = can_mcan_start,
	.stop = can_mcan_stop,
	.set_mode = can_mcan_set_mode,
	.set_timing = can_mcan_set_timing,
	.send = can_mcan_send,
	.add_rx_filter = can_mcan_add_rx_filter,
	.remove_rx_filter = can_mcan_remove_rx_filter,
	.get_state = can_mcan_get_state,
#ifndef CONFIG_CAN_AUTO_BUS_OFF_RECOVERY
	.recover = can_mcan_recover,
#endif /* CONFIG_CAN_AUTO_BUS_OFF_RECOVERY */
	.get_core_clock = can_sam0_get_core_clock,
	.get_max_filters = can_mcan_get_max_filters,
	.get_max_bitrate = can_mcan_get_max_bitrate,
	.set_state_change_callback =  can_mcan_set_state_change_callback,
	.timing_min = {
		.sjw = 0x1,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_max = {
		.sjw = 0x7f,
		.prop_seg = 0x00,
		.phase_seg1 = 0x100,
		.phase_seg2 = 0x80,
		.prescaler = 0x200
	},
#ifdef CONFIG_CAN_FD_MODE
	.set_timing_data = can_mcan_set_timing_data,
	.timing_data_min = {
		.sjw = 0x01,
		.prop_seg = 0x00,
		.phase_seg1 = 0x01,
		.phase_seg2 = 0x01,
		.prescaler = 0x01
	},
	.timing_data_max = {
		.sjw = 0x10,
		.prop_seg = 0x00,
		.phase_seg1 = 0x20,
		.phase_seg2 = 0x10,
		.prescaler = 0x20
	}
#endif /* CONFIG_CAN_FD_MODE */
};

#define CAN_SAM0_IRQ_CFG_FUNCTION(inst)							\
static void config_can_##inst##_irq(void)						\
{											\
	LOG_DBG("Enable CAN##inst## IRQ");						\
	IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, line_0, irq),				\
		    DT_INST_IRQ_BY_NAME(inst, line_0, priority), can_sam0_line_x_isr,	\
					DEVICE_DT_INST_GET(inst), 0);			\
	irq_enable(DT_INST_IRQ_BY_NAME(inst, line_0, irq));				\
}

#define CAN_SAM0_CFG_INST(inst)								\
	static const struct can_sam0_config can_sam0_cfg_##inst = {			\
		.base = (mm_reg_t)DT_INST_REG_ADDR(inst),				\
		.mclk = (volatile uint32_t *)MCLK_MASK_DT_INT_REG_ADDR(inst),		\
		.mclk_mask = BIT(DT_INST_CLOCKS_CELL_BY_NAME(inst, mclk, bit)),		\
		.gclk_core_id = DT_INST_CLOCKS_CELL_BY_NAME(inst, gclk, periph_ch),	\
		.divider = DT_INST_PROP(inst, divider),					\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),				\
		.config_irq = config_can_##inst##_irq,					\
	};										\
											\
	static const struct can_mcan_config can_mcan_cfg_##inst =			\
		CAN_MCAN_DT_CONFIG_INST_GET(inst, &can_sam0_cfg_##inst,			\
					    can_sam0_read_reg,				\
					    can_sam0_write_reg);

#define CAN_SAM0_DATA_INST(inst)							\
	static struct can_sam0_data can_sam0_data_##inst;				\
											\
	static struct can_mcan_data can_mcan_data_##inst =				\
		CAN_MCAN_DATA_INITIALIZER(&can_sam0_data_##inst.msg_ram,		\
					  &can_sam0_data_##inst);			\

#define CAN_SAM0_DEVICE_INST(inst)							\
	DEVICE_DT_INST_DEFINE(inst, &can_sam0_init, NULL,				\
			      &can_mcan_data_##inst,					\
			      &can_mcan_cfg_##inst,					\
			      POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,			\
			      &can_sam0_driver_api);

#define CAN_SAM0_INST(inst)								\
	PINCTRL_DT_INST_DEFINE(inst);							\
	CAN_SAM0_IRQ_CFG_FUNCTION(inst)							\
	CAN_SAM0_CFG_INST(inst)								\
	CAN_SAM0_DATA_INST(inst)							\
	CAN_SAM0_DEVICE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_SAM0_INST)
