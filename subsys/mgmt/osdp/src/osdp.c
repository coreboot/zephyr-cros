/*
 * Copyright (c) 2020 Siddharth Chandrasekaran <sidcha.dev@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

#include "osdp_common.h"

LOG_MODULE_REGISTER(osdp, CONFIG_OSDP_LOG_LEVEL);

#ifdef CONFIG_OSDP_SC_ENABLED
#ifdef CONFIG_OSDP_MODE_PD
#define OSDP_KEY_STRING CONFIG_OSDP_PD_SCBK
#else
#define OSDP_KEY_STRING CONFIG_OSDP_MASTER_KEY
#endif
#else
#define OSDP_KEY_STRING ""
#endif /* CONFIG_OSDP_SC_ENABLED */

#define UART_EVENT_TX_END 1UL

struct osdp_info osdp_info = {
	.baud_rate = CONFIG_OSDP_UART_BAUD_RATE,
	.skip_mark_byte = IS_ENABLED(CONFIG_OSDP_SKIP_MARK_BYTE),
	.cp_cfg = NULL,
	.pd_cfg = NULL,
};

struct osdp_device {
	struct ring_buf rx_buf;
	struct ring_buf tx_buf;
#ifdef CONFIG_OSDP_MODE_PD
	int rx_event_data;
	struct k_fifo rx_event_fifo;
#endif
	uint8_t rx_fbuf[CONFIG_OSDP_UART_BUFFER_LENGTH];
	uint8_t tx_fbuf[CONFIG_OSDP_UART_BUFFER_LENGTH];
	struct uart_config dev_config;
	const struct device *dev;
	int wait_for_mark;
	uint8_t last_byte;
};

static struct osdp osdp_ctx;
static struct osdp_pd osdp_pd_ctx[CONFIG_OSDP_NUM_CONNECTED_PD];
static struct osdp_device osdp_device;
static struct k_thread osdp_refresh_thread;
static K_THREAD_STACK_DEFINE(osdp_thread_stack, CONFIG_OSDP_THREAD_STACK_SIZE);
static K_EVENT_DEFINE(uart_tx_end);

static void osdp_handle_in_byte(struct osdp_device *p, uint8_t *buf, int len)
{
	if (p->wait_for_mark) {
		/* Check for new packet beginning with [FF,53,...] sequence */
		if (p->last_byte == 0xFF && buf[0] == 0x53) {
			buf[0] = 0xFF;
			ring_buf_put(&p->rx_buf, buf, 1); /* put last byte */
			buf[0] = 0x53;
			ring_buf_put(&p->rx_buf, buf, len); /* put rest */
			p->wait_for_mark = 0; /* Mark found. Clear flag */
		}
		p->last_byte = buf[0];
		return;
	}
	ring_buf_put(&p->rx_buf, buf, len);
}

static void osdp_uart_isr(const struct device *dev, void *user_data)
{
	size_t len;
	uint8_t buf[64];
	struct osdp_device *p = user_data;

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			len = uart_fifo_read(dev, buf, sizeof(buf));
			if (len > 0) {
				osdp_handle_in_byte(p, buf, len);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			len = ring_buf_get(&p->tx_buf, buf, 1);
			if (!len) {
				uart_irq_tx_disable(dev);
				k_event_set(&uart_tx_end, UART_EVENT_TX_END);
			} else {
				uart_fifo_fill(dev, buf, 1);
			}
		}
	}
#ifdef CONFIG_OSDP_MODE_PD
	if (p->wait_for_mark == 0) {
		/* wake osdp_refresh thread */
		k_fifo_put(&p->rx_event_fifo, &p->rx_event_data);
	}
#endif
}

static int osdp_uart_receive(void *data, uint8_t *buf, int len)
{
	struct osdp_device *p = data;

	return (int)ring_buf_get(&p->rx_buf, buf, len);
}

static int osdp_uart_send(void *data, uint8_t *buf, int len)
{
	int sent = 0;
	struct osdp_device *p = data;
	sent = (int)ring_buf_put(&p->tx_buf, buf, len);
	uart_irq_tx_enable(p->dev);
	return sent;
}

static void osdp_uart_flush(void *data)
{
	struct osdp_device *p = data;

	p->wait_for_mark = 1;
	ring_buf_reset(&p->tx_buf);
	ring_buf_reset(&p->rx_buf);
}

void osdp_refresh(void *arg1, void *arg2, void *arg3)
{
	struct osdp *ctx = osdp_get_ctx();

	while (1) {
#ifdef CONFIG_OSDP_MODE_PD
		k_fifo_get(&osdp_device.rx_event_fifo, K_FOREVER);
#else
		k_msleep(50);
#endif
		osdp_update(ctx);
	}
}

struct osdp *osdp_get_ctx()
{
	return &osdp_ctx;
}

static struct osdp *osdp_build_ctx(struct osdp_channel *channel)
{
	int i;
	struct osdp *ctx;
	struct osdp_pd *pd;
	uint8_t *pd_address;
	uint8_t num_pd;

#ifdef CONFIG_OSDP_MODE_PD
	num_pd = 1;
	pd_address = &osdp_info.pd_cfg->reader_address;
#else
	if (osdp_info.cp_cfg->connected_readers_num > CONFIG_OSDP_NUM_CONNECTED_PD) {
		return NULL;
	}
	num_pd = osdp_info.cp_cfg->connected_readers_num;
	pd_address = osdp_info.cp_cfg->connected_readers_addresses;
#endif

	/* Validate PD addresses */
	for (i = 0; i < num_pd; i++) {
		if (!osdp_is_valid_pd_address(pd_address[i])) {
			return NULL;
		}
	}

	ctx = &osdp_ctx;
	ctx->num_pd = num_pd;
	ctx->pd = &osdp_pd_ctx[0];
	SET_CURRENT_PD(ctx, 0);

	for (i = 0; i < ctx->num_pd; i++) {
		pd = osdp_to_pd(ctx, i);
		pd->idx = i;
		pd->seq_number = -1;
		pd->osdp_ctx = ctx;
		pd->address = pd_address[i];
		pd->baud_rate = osdp_info.baud_rate;
		if (osdp_info.skip_mark_byte) {
			SET_FLAG(pd, PD_FLAG_PKT_SKIP_MARK);
		}
		memcpy(&pd->channel, channel, sizeof(struct osdp_channel));
		k_mem_slab_init(&pd->cmd.slab, pd->cmd.slab_buf, sizeof(struct osdp_cmd),
				CONFIG_OSDP_PD_COMMAND_QUEUE_SIZE);
	}
	return ctx;
}

int osdp_uart_update_config()
{
	uint8_t c;
	int status = 0;
	struct osdp *ctx = osdp_get_ctx();
	struct osdp_device *p = ctx->pd->channel.data;

	if (p->dev_config.baudrate != ctx->pd->baud_rate) {
		/* Wait for sending response before reconfiguring UART */
		if (!k_event_wait(&uart_tx_end, UART_EVENT_TX_END, true, K_MSEC(30))) {
			return -ETIME;
		}
		uart_irq_rx_disable(p->dev);
		uart_irq_tx_disable(p->dev);
		p->dev_config.baudrate = ctx->pd->baud_rate;
		status = uart_configure(p->dev, &p->dev_config);
		if (status != 0) {
			ctx->pd->baud_rate = p->dev_config.baudrate;
			LOG_ERR("Baudrate change failed\n");
		}
		/* Drain UART fifo and set channel to wait for mark byte */
		while (uart_irq_rx_ready(p->dev)) {
			uart_fifo_read(p->dev, &c, 1);
		}
		p->wait_for_mark = 1;
		/* Both TX and RX are interrupt driven */
		uart_irq_rx_enable(p->dev);
	}
	return status;
}

static int osdp_configure_device(struct osdp_device *p)
{
	uint8_t c;
#ifdef CONFIG_OSDP_MODE_PD
	k_fifo_init(&p->rx_event_fifo);
#endif
	ring_buf_init(&p->rx_buf, sizeof(p->rx_fbuf), p->rx_fbuf);
	ring_buf_init(&p->tx_buf, sizeof(p->tx_fbuf), p->tx_fbuf);

	/* init OSDP uart device */
	p->dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_osdp_uart));
	if (!device_is_ready(p->dev)) {
		LOG_ERR("UART dev is not ready");
		k_panic();
	}

	/* configure uart device to 8N1 */

	if (!osdp_is_valid_baudrate(osdp_info.baud_rate)) {
		return EINVAL;
	}

	p->dev_config.baudrate = osdp_info.baud_rate;
	p->dev_config.data_bits = UART_CFG_DATA_BITS_8;
	p->dev_config.parity = UART_CFG_PARITY_NONE;
	p->dev_config.stop_bits = UART_CFG_STOP_BITS_1;
	p->dev_config.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;

	uart_configure(p->dev, &p->dev_config);

	uart_irq_rx_disable(p->dev);
	uart_irq_tx_disable(p->dev);
	uart_irq_callback_user_data_set(p->dev, osdp_uart_isr, p);

	/* Drain UART fifo and set channel to wait for mark byte */

	while (uart_irq_rx_ready(p->dev)) {
		uart_fifo_read(p->dev, &c, 1);
	}

	p->wait_for_mark = 1;

	/* Both TX and RX are interrupt driven */
	uart_irq_rx_enable(p->dev);
	return 0;
}

static uint8_t *osdp_get_key(uint8_t *key_buf)
{
	uint8_t len;
	if (IS_ENABLED(CONFIG_OSDP_SC_ENABLED)) {
		if (strcmp(OSDP_KEY_STRING, "NONE") != 0) {
			len = strlen(OSDP_KEY_STRING);
			if (len != 32) {
				LOG_ERR("Key string length must be 32");
				k_panic();
			}
			len = hex2bin(OSDP_KEY_STRING, 32, key_buf, 16);
			if (len != 16) {
				LOG_ERR("Failed to parse key buffer");
				k_panic();
			}
			return key_buf;
		}
	}
	return NULL;
}

static int osdp_init_internal()
{
	struct osdp *ctx;

	struct osdp_channel channel = {
		.send = osdp_uart_send,
		.recv = osdp_uart_receive,
		.flush = osdp_uart_flush,
		.data = &osdp_device,
	};

	/* configure OSDP device */
	if (osdp_configure_device(&osdp_device)) {
		LOG_ERR("OSDP device configuration failed!");
		k_panic();
	}

	/* setup OSDP */
	ctx = osdp_build_ctx(&channel);
	if (ctx == NULL) {
		LOG_ERR("OSDP build ctx failed!");
		k_panic();
	}

	if (osdp_setup(ctx, &osdp_info)) {
		LOG_ERR("Failed to setup OSDP device!");
		k_panic();
	}

	LOG_INF("OSDP init okay!");

	/* kick off refresh thread */
	k_thread_create(&osdp_refresh_thread, osdp_thread_stack, CONFIG_OSDP_THREAD_STACK_SIZE,
			osdp_refresh, NULL, NULL, NULL, K_PRIO_COOP(2), 0, K_NO_WAIT);
	return 0;
}

int osdp_init(const struct osdp_info *info)
{
	if (info) {
		memcpy(&osdp_info, info, sizeof(struct osdp_info));
	} else {
		uint8_t key_buf[16];
		osdp_info.key = osdp_get_key(key_buf);
#ifdef CONFIG_OSDP_MODE_PD
		osdp_info.pd_cfg = pd_get_info();
#else
		osdp_info.cp_cfg = cp_get_info();
#endif
	}
	return osdp_init_internal();
}

//SYS_INIT(osdp_init, POST_KERNEL, 10);
