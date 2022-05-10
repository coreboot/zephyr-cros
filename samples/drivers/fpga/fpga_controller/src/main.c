/*
 * Copyright (c) 2021 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/fpga.h>
#include "redled.h"
#include "greenled.h"
#include <eoss3_dev.h>

const struct device *fpga;

void main(void)
{
	IO_MUX->PAD_21_CTRL = (PAD_E_4MA | PAD_P_PULLDOWN | PAD_OEN_NORMAL |
			       PAD_SMT_DISABLE | PAD_REN_DISABLE | PAD_SR_SLOW |
			       PAD_CTRL_SEL_AO_REG); /* Enable red led */
	IO_MUX->PAD_22_CTRL = (PAD_E_4MA | PAD_P_PULLDOWN | PAD_OEN_NORMAL |
			       PAD_SMT_DISABLE | PAD_REN_DISABLE | PAD_SR_SLOW |
			       PAD_CTRL_SEL_AO_REG); /* Enable green led */

	fpga = device_get_binding("FPGA");

	if (!fpga) {
		printk("unable to find fpga device\n");
	}
	while (1) {
		fpga_load(fpga, axFPGABitStream_red,
			  sizeof(axFPGABitStream_red));
		k_msleep(2000);
		fpga_reset(fpga);
		fpga_load(fpga, axFPGABitStream_green,
			  sizeof(axFPGABitStream_green));
		k_msleep(2000);
		fpga_reset(fpga);
	}
}
