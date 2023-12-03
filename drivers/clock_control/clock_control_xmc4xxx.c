/*
 * Copyright (c) 2023 SLB
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT infineon_xmc4xxx_csu

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>

#include <soc.h>
#include <xmc_scu.h>

struct clock_control_xmc4xxx_config {
#if DT_INST_NODE_HAS_PROP(0, pinctrl_0)
	const struct pinctrl_dev_config *pcfg;
#endif
};

static int clock_control_xmc4xxx_on(const struct device *dev,
				 clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int clock_control_xmc4xxx_off(const struct device *dev,
				  clock_control_subsys_t sys)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);

	return 0;
}

static int clock_control_xmc4xxx_get_rate(const struct device *dev,
				       clock_control_subsys_t sys,
				       uint32_t *rate)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(sys);
	ARG_UNUSED(rate);

	return 0;
}

static enum clock_control_status clock_control_xmc4xxx_get_status(const struct device *dev,
								  clock_control_subsys_t sys)
{
	return CLOCK_CONTROL_STATUS_ON;
}

static struct clock_control_driver_api clock_control_xmc4xxx_api = {
	.on = clock_control_xmc4xxx_on,
	.off = clock_control_xmc4xxx_off,
	.get_rate = clock_control_xmc4xxx_get_rate,
	.get_status = clock_control_xmc4xxx_get_status,
};

static int clock_control_xmc4xxx_init(const struct device *dev)
{
	int ret = 0;

#if DT_NODE_HAS_STATUS(DT_NODELABEL(usbpll), okay)

	uint32_t clock_div = DT_PROP(DT_NODELABEL(usbpll), clock_div);
	uint32_t clock_mult = DT_PROP(DT_NODELABEL(usbpll), clock_mult);

	XMC_SCU_CLOCK_SetExternalOutputClockSource(XMC_SCU_CLOCK_EXTOUTCLKSRC_USB);
	XMC_SCU_CLOCK_SetExternalOutputClockDivider(6U);

	XMC_SCU_CLOCK_EnableUsbPll();
	XMC_SCU_CLOCK_StartUsbPll(clock_mult, clock_div);

#endif

#if DT_INST_NODE_HAS_PROP(0, pinctrl_0)
	const struct clock_control_xmc4xxx_config *config = dev->config;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
#endif

	return ret;
}

#if DT_INST_NODE_HAS_PROP(0, pinctrl_0)
PINCTRL_DT_INST_DEFINE(0);
#endif

static const struct clock_control_xmc4xxx_config config = {
#if DT_INST_NODE_HAS_PROP(0, pinctrl_0)
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
#endif
};

DEVICE_DT_INST_DEFINE(0, &clock_control_xmc4xxx_init, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY,
		      &clock_control_xmc4xxx_api);
