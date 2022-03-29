/*
 * Copyright (c) 2022, Yonatan Schachter
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>

#include <pico_pio.h>

#include <hardware/pio.h>
#include <hardware/clocks.h>

#define DT_DRV_COMPAT raspberrypi_pico_uart_pio

#define TX_STATE_MACHINE 0
#define RX_STATE_MACHINE 1
#define CYCLES_PER_BIT 8
#define SIDESET_BIT_COUNT 2

struct pio_uart_config {
	PIO pio;
	uint32_t baudrate;
	const struct pinctrl_dev_config *pcfg;
};

RPI_PICO_PIO_DECLARE_PROGRAM(uart_tx, 0, 3,
		/* .wrap_target */
	0x9fa0, /*  0: pull   block           side 1 [7]  */
	0xf727, /*  1: set    x, 7            side 0 [7]  */
	0x6001, /*  2: out    pins, 1                     */
	0x0642, /*  3: jmp    x--, 2                 [6]  */
		/* .wrap */
);

RPI_PICO_PIO_DECLARE_PROGRAM(uart_rx, 0, 8,
		/*  .wrap_target */
	0x2020, /*  0: wait   0 pin, 0                    */
	0xea27, /*  1: set    x, 7                   [10] */
	0x4001, /*  2: in     pins, 1                     */
	0x0642, /*  3: jmp    x--, 2                 [6]  */
	0x00c8, /*  4: jmp    pin, 8                      */
	0xc014, /*  5: irq    nowait 4 rel                */
	0x20a0, /*  6: wait   1 pin, 0                    */
	0x0000, /*  7: jmp    0                           */
	0x8020, /*  8: push   block                       */
		/*  .wrap */
);

static void pio_uart_tx_init(PIO pio, uint32_t pin_tx, float div)
{
	uint32_t offset;
	pio_sm_config sm_config;

	offset = pio_add_program(pio, &uart_tx_program);
	sm_config = pio_get_default_sm_config();

	sm_config_set_sideset(&sm_config, SIDESET_BIT_COUNT, true, false);
	sm_config_set_out_shift(&sm_config, true, false, 0);
	sm_config_set_out_pins(&sm_config, pin_tx, 1);
	sm_config_set_sideset_pins(&sm_config, pin_tx);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_TX);
	sm_config_set_clkdiv(&sm_config, div);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(uart_tx),
			   offset + RPI_PICO_PIO_GET_WRAP(uart_tx));

	pio_sm_set_pins_with_mask(pio, TX_STATE_MACHINE, BIT(pin_tx), BIT(pin_tx));
	pio_sm_set_pindirs_with_mask(pio, TX_STATE_MACHINE, BIT(pin_tx), BIT(pin_tx));
	pio_sm_init(pio, TX_STATE_MACHINE, offset, &sm_config);
	pio_sm_set_enabled(pio, TX_STATE_MACHINE, true);
}

static void pio_uart_rx_init(PIO pio, uint32_t pin, float div)
{
	uint32_t offset;
	pio_sm_config sm_config;

	offset = pio_add_program(pio, &uart_rx_program);
	sm_config = pio_get_default_sm_config();

	pio_sm_set_consecutive_pindirs(pio, RX_STATE_MACHINE, pin, 1, false);
	sm_config_set_in_pins(&sm_config, pin);
	sm_config_set_jmp_pin(&sm_config, pin);
	sm_config_set_in_shift(&sm_config, true, false, 0);
	sm_config_set_fifo_join(&sm_config, PIO_FIFO_JOIN_RX);
	sm_config_set_clkdiv(&sm_config, div);
	sm_config_set_wrap(&sm_config,
			   offset + RPI_PICO_PIO_GET_WRAP_TARGET(uart_rx),
			   offset + RPI_PICO_PIO_GET_WRAP(uart_rx));

	pio_sm_init(pio, RX_STATE_MACHINE, offset, &sm_config);
	pio_sm_set_enabled(pio, RX_STATE_MACHINE, true);
}

static int pio_uart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct pio_uart_config *config = dev->config;
	io_rw_8 *uart_rx_fifo_msb;
	PIO pio = config->pio;

	/*
	 * The rx FIFO is 4 bytes wide, add 3 to get the most significant
	 * byte.
	 */
	uart_rx_fifo_msb = (io_rw_8 *)&pio->rxf[RX_STATE_MACHINE] + 3;
	if (pio_sm_is_rx_fifo_empty(pio, RX_STATE_MACHINE)) {
		return -1;
	}

	/* Accessing the FIFO pops the read word from it */
	*c = (char)*uart_rx_fifo_msb;
	return 0;
}

static void pio_uart_poll_out(const struct device *dev, unsigned char c)
{
	const struct pio_uart_config *config = dev->config;

	pio_sm_put_blocking(config->pio, TX_STATE_MACHINE, (uint32_t)c);
}

static int pio_uart_init(const struct device *dev)
{
	const struct pio_uart_config *config = dev->config;
	const struct pinctrl_state *state;
	PIO pio = config->pio;
	float sm_clock_div;
	uint32_t tx_pin_index;
	int ret;

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = pinctrl_lookup_state(config->pcfg, PINCTRL_STATE_DEFAULT, &state);
	if (ret < 0) {
		return ret;
	}

	sm_clock_div = (float)clock_get_hz(clk_sys) / (CYCLES_PER_BIT * config->baudrate);

	/**
	 * UART uses two pins, which are found in the state->pins array.
	 * The tx pin can be in either index 0 or 1. If input is enabled for
	 * the pin at index 0, it means that 0 holds the rx pin, and therefore
	 * the tx pin is at index 1.
	 */
	tx_pin_index = state->pins[0].input_enable ? 1 : 0;
	pio_uart_tx_init(pio, state->pins[tx_pin_index].pin_num, sm_clock_div);
	pio_uart_rx_init(pio, state->pins[!tx_pin_index].pin_num, sm_clock_div);

	return 0;
}

static const struct uart_driver_api pio_uart_driver_api = {
	.poll_in = pio_uart_poll_in,
	.poll_out = pio_uart_poll_out,
};

#define PIO_UART_INIT(idx)							\
	PINCTRL_DT_INST_DEFINE(idx);						\
										\
	static const struct pio_uart_config pio_uart##idx##_config = {		\
		.pio = (PIO)DT_REG_ADDR(DT_INST_PARENT(idx)),			\
		.baudrate = DT_INST_PROP(idx, current_speed),			\
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),			\
	};									\
										\
	DEVICE_DT_INST_DEFINE(idx, &pio_uart_init, NULL, NULL,			\
			      &pio_uart##idx##_config, PRE_KERNEL_1,		\
			      CONFIG_SERIAL_INIT_PRIORITY,			\
			      &pio_uart_driver_api);				\

DT_INST_FOREACH_STATUS_OKAY(PIO_UART_INIT)
