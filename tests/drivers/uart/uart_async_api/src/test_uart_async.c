/*
 *  Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "test_uart.h"

K_SEM_DEFINE(tx_done, 0, 1);
K_SEM_DEFINE(tx_aborted, 0, 1);
K_SEM_DEFINE(rx_rdy, 0, 1);
K_SEM_DEFINE(rx_buf_released, 0, 1);
K_SEM_DEFINE(rx_disabled, 0, 1);

ZTEST_BMEM volatile bool failed_in_isr;
ZTEST_BMEM static const struct device *const uart_dev =
	DEVICE_DT_GET(UART_DEVICE_DEV);

static void read_abort_timeout(struct k_timer *timer);
K_TIMER_DEFINE(read_abort_timer, read_abort_timeout, NULL);


static void init_test(void)
{
	__ASSERT_NO_MSG(device_is_ready(uart_dev));
}

#ifdef CONFIG_USERSPACE
static void set_permissions(void)
{
	k_thread_access_grant(k_current_get(), &tx_done, &tx_aborted,
			      &rx_rdy, &rx_buf_released, &rx_disabled,
			      uart_dev, &read_abort_timer);
}
#endif

static void uart_async_test_init(void)
{
	static bool initialized;

	if (!initialized) {
		init_test();
		initialized = true;

#ifdef CONFIG_USERSPACE
		set_permissions();
#endif
	}
}

static void test_single_read_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		(*(uint32_t *)user_data)++;
		break;
	case UART_RX_RDY:
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

ZTEST_BMEM volatile uint32_t tx_aborted_count;

static void *single_read_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev,
			  test_single_read_callback,
			  (void *) &tx_aborted_count);

	return NULL;
}

ZTEST_USER(uart_async_single_read, test_single_read)
{
	uint8_t rx_buf[10] = {0};

	/* Check also if sending from read only memory (e.g. flash) works. */
	static const uint8_t tx_buf[5] = "test";

	zassert_not_equal(memcmp(tx_buf, rx_buf, 5), 0,
			  "Initial buffer check failed");

	uart_rx_enable(uart_dev, rx_buf, 10, 50 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");

	uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");

	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");
	zassert_not_equal(memcmp(tx_buf, rx_buf+5, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(1000)), 0,
		      "RX_DISABLED timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");

	zassert_equal(memcmp(tx_buf, rx_buf+5, 5), 0, "Buffers not equal");
	zassert_equal(tx_aborted_count, 0, "TX aborted triggered");
}

static void *multiple_rx_enable_setup(void)
{
	uart_async_test_init();

	tx_aborted_count = 0;

	/* Reuse the callback from the single_read test case, as this test case
	 * does not need anything extra in this regard.
	 */
	uart_callback_set(uart_dev,
			  test_single_read_callback,
			  (void *)&tx_aborted_count);

	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_buf_released);
	k_sem_reset(&rx_disabled);
	k_sem_reset(&tx_done);

	return NULL;
}

ZTEST_USER(uart_async_multi_rx, test_multiple_rx_enable)
{
	/* Check also if sending from read only memory (e.g. flash) works. */
	static const uint8_t tx_buf[] = "test";
	uint8_t rx_buf[sizeof(tx_buf)] = {0};
	int ret;

	/* Enable RX without a timeout. */
	ret = uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), SYS_FOREVER_US);
	zassert_equal(ret, 0, "uart_rx_enable failed");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), -EAGAIN,
		      "RX_DISABLED not expected at this point");

	/* Disable RX before any data has been received. */
	ret = uart_rx_disable(uart_dev);
	zassert_equal(ret, 0, "uart_rx_disable failed");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)), 0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");

	k_sem_reset(&rx_buf_released);
	k_sem_reset(&rx_disabled);

	/* Check that RX can be reenabled after "manual" disabling. */
	ret = uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf),
			     50 * USEC_PER_MSEC);
	zassert_equal(ret, 0, "uart_rx_enable failed");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");

	/* Send enough data to completely fill RX buffer, so that RX ends. */
	ret = uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(ret, 0, "uart_tx failed");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)), 0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
	zassert_equal(tx_aborted_count, 0, "Unexpected TX abort");

	zassert_equal(memcmp(tx_buf, rx_buf, sizeof(tx_buf)), 0,
		      "Buffers not equal");

	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_buf_released);
	k_sem_reset(&rx_disabled);
	k_sem_reset(&tx_done);
	memset(rx_buf, 0, sizeof(rx_buf));

	/* Check that RX can be reenabled after automatic disabling. */
	ret = uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf),
			     50 * USEC_PER_MSEC);
	zassert_equal(ret, 0, "uart_rx_enable failed");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "RX_RDY not expected at this point");

	/* Fill RX buffer again to confirm that RX still works properly. */
	ret = uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
	zassert_equal(ret, 0, "uart_tx failed");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), -EAGAIN,
		      "Extra RX_RDY received");
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)), 0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
	zassert_equal(tx_aborted_count, 0, "Unexpected TX abort");

	zassert_equal(memcmp(tx_buf, rx_buf, sizeof(tx_buf)), 0,
		      "Buffers not equal");
}

ZTEST_BMEM uint8_t chained_read_buf0[10];
ZTEST_BMEM uint8_t chained_read_buf1[20];
ZTEST_BMEM uint8_t chained_read_buf2[30];
ZTEST_DMEM uint8_t buf_num = 1U;
ZTEST_BMEM uint8_t *read_ptr;
ZTEST_BMEM volatile size_t read_len;

static void test_chained_read_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		read_ptr = evt->data.rx.buf + evt->data.rx.offset;
		read_len = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_REQUEST:
		if (buf_num == 1U) {
			uart_rx_buf_rsp(uart_dev,
					chained_read_buf1,
					sizeof(chained_read_buf1));
			buf_num = 2U;
		} else if (buf_num == 2U) {
			uart_rx_buf_rsp(uart_dev,
					chained_read_buf2,
					sizeof(chained_read_buf2));
			buf_num = 0U;
		}
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

static void *chained_read_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_chained_read_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_chain_read, test_chained_read)
{
	uint8_t tx_buf[10];

	uart_rx_enable(uart_dev, chained_read_buf0, 10, 50 * USEC_PER_MSEC);

	for (int i = 0; i < 6; i++) {
		zassert_not_equal(k_sem_take(&rx_disabled, K_MSEC(10)),
				  0,
				  "RX_DISABLED occurred");
		snprintf(tx_buf, sizeof(tx_buf), "Message %d", i);
		uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
		zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0,
			      "TX_DONE timeout");
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(1000)), 0,
			      "RX_RDY timeout");
		size_t read_len_temp = read_len;

		zassert_equal(read_len_temp, sizeof(tx_buf),
			      "Incorrect read length");
		zassert_equal(memcmp(tx_buf, read_ptr, sizeof(tx_buf)),
			      0,
			      "Buffers not equal");
	}
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}

ZTEST_BMEM uint8_t double_buffer[2][12];
ZTEST_DMEM uint8_t *next_buf = double_buffer[1];

static void test_double_buffer_callback(const struct device *uart_dev,
				 struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		read_ptr = evt->data.rx.buf + evt->data.rx.offset;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_REQUEST:
		uart_rx_buf_rsp(uart_dev, next_buf, sizeof(double_buffer[0]));
		break;
	case UART_RX_BUF_RELEASED:
		next_buf = evt->data.rx_buf.buf;
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}

}

static void *double_buffer_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_double_buffer_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_double_buf, test_double_buffer)
{
	uint8_t tx_buf[4];

	zassert_equal(uart_rx_enable(uart_dev,
				     double_buffer[0],
				     sizeof(double_buffer[0]),
				     50 * USEC_PER_MSEC),
		      0,
		      "Failed to enable receiving");

	for (int i = 0; i < 100; i++) {
		snprintf(tx_buf, sizeof(tx_buf), "%03d", i);
		uart_tx(uart_dev, tx_buf, sizeof(tx_buf), 100 * USEC_PER_MSEC);
		zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0,
			      "TX_DONE timeout");
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0,
			      "RX_RDY timeout");
		zassert_equal(memcmp(tx_buf, read_ptr, sizeof(tx_buf)),
			      0,
			      "Buffers not equal");
	}
	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}

static void test_read_abort_callback(const struct device *dev,
			      struct uart_event *evt, void *user_data)
{
	int err;

	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_RX_RDY:
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		err = k_sem_take(&rx_rdy, K_NO_WAIT);
		failed_in_isr |= (err < 0);
		break;
	case UART_RX_DISABLED:
		err = k_sem_take(&rx_buf_released, K_NO_WAIT);
		failed_in_isr |= (err < 0);
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static void read_abort_timeout(struct k_timer *timer)
{
	int err;

	err = uart_rx_disable(uart_dev);
	zassert_equal(err, 0, "Unexpected err:%d", err);
}

static void *read_abort_setup(void)
{
	uart_async_test_init();

	failed_in_isr = false;
	uart_callback_set(uart_dev, test_read_abort_callback, NULL);

	k_sem_reset(&rx_rdy);
	k_sem_reset(&rx_buf_released);
	k_sem_reset(&rx_disabled);
	k_sem_reset(&tx_done);

	return NULL;
}

ZTEST_USER(uart_async_read_abort, test_read_abort)
{
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];

	memset(rx_buf, 0, sizeof(rx_buf));
	memset(tx_buf, 1, sizeof(tx_buf));

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, tx_buf, 5, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, 95, 100 * USEC_PER_MSEC);

	k_timer_start(&read_abort_timer, K_USEC(300), K_NO_WAIT);

	/* RX will be aborted from k_timer timeout */

	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
	zassert_false(failed_in_isr, "Unexpected order of uart events");
	zassert_not_equal(memcmp(tx_buf, rx_buf, 100), 0, "Buffers equal");

	/* Read out possible other RX bytes
	 * that may affect following test on RX
	 */
	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);
	while (k_sem_take(&rx_rdy, K_MSEC(1000)) != -EAGAIN) {
		;
	}
	uart_rx_disable(uart_dev);
}

ZTEST_BMEM volatile size_t sent;
ZTEST_BMEM volatile size_t received;

static void test_write_abort_callback(const struct device *dev,
			       struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static void *write_abort_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_write_abort_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_write_abort, test_write_abort)
{
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];

	memset(rx_buf, 0, sizeof(rx_buf));
	memset(tx_buf, 1, sizeof(tx_buf));

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, tx_buf, 5, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(memcmp(tx_buf, rx_buf, 5), 0, "Buffers not equal");

	uart_tx(uart_dev, tx_buf, 95, 100 * USEC_PER_MSEC);
	uart_tx_abort(uart_dev);
	zassert_equal(k_sem_take(&tx_aborted, K_MSEC(100)), 0,
		      "TX_ABORTED timeout");
	if (sent != 0) {
		zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0,
			      "RX_RDY timeout");
		zassert_equal(sent, received, "Sent is not equal to received.");
	}
	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}


static void test_forever_timeout_callback(const struct device *dev,
				   struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static void *forever_timeout_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_forever_timeout_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_timeout, test_forever_timeout)
{
	uint8_t rx_buf[100];
	uint8_t tx_buf[100];

	memset(rx_buf, 0, sizeof(rx_buf));
	memset(tx_buf, 1, sizeof(tx_buf));

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), SYS_FOREVER_US);

	uart_tx(uart_dev, tx_buf, 5, SYS_FOREVER_US);
	zassert_not_equal(k_sem_take(&tx_aborted, K_MSEC(1000)), 0,
			  "TX_ABORTED timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_not_equal(k_sem_take(&rx_rdy, K_MSEC(1000)), 0,
			  "RX_RDY timeout");

	uart_tx(uart_dev, tx_buf, 95, SYS_FOREVER_US);

	zassert_not_equal(k_sem_take(&tx_aborted, K_MSEC(1000)), 0,
			  "TX_ABORTED timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");


	zassert_equal(memcmp(tx_buf, rx_buf, 100), 0, "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}


ZTEST_DMEM uint8_t chained_write_tx_bufs[2][10] = {"Message 1", "Message 2"};
ZTEST_DMEM bool chained_write_next_buf = true;
ZTEST_BMEM volatile uint8_t tx_sent;

static void test_chained_write_callback(const struct device *uart_dev,
				 struct uart_event *evt, void *user_data)
{
	switch (evt->type) {
	case UART_TX_DONE:
		if (chained_write_next_buf) {
			uart_tx(uart_dev, chained_write_tx_bufs[1], 10, 100 * USEC_PER_MSEC);
			chained_write_next_buf = false;
		}
		tx_sent = 1;
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		received = evt->data.rx.len;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static void *chained_write_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_chained_write_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_chain_write, test_chained_write)
{
	uint8_t rx_buf[20];

	memset(rx_buf, 0, sizeof(rx_buf));

	uart_rx_enable(uart_dev, rx_buf, sizeof(rx_buf), 50 * USEC_PER_MSEC);

	uart_tx(uart_dev, chained_write_tx_bufs[0], 10, 100 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&tx_done, K_MSEC(100)), 0, "TX_DONE timeout");
	zassert_equal(chained_write_next_buf, false, "Sent no message");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(100)), 0, "RX_RDY timeout");
	zassert_equal(memcmp(chained_write_tx_bufs[0], rx_buf, 10),
		      0,
		      "Buffers not equal");
	zassert_equal(memcmp(chained_write_tx_bufs[1], rx_buf + 10, 10),
		      0,
		      "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}

ZTEST_BMEM uint8_t long_rx_buf[1024];
ZTEST_BMEM uint8_t long_rx_buf2[1024];
ZTEST_BMEM uint8_t long_tx_buf[1000];
ZTEST_BMEM volatile uint8_t evt_num;
ZTEST_BMEM size_t long_received[2];

static void test_long_buffers_callback(const struct device *uart_dev,
				struct uart_event *evt, void *user_data)
{
	static bool next_buf = true;

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&tx_done);
		break;
	case UART_TX_ABORTED:
		sent = evt->data.tx.len;
		k_sem_give(&tx_aborted);
		break;
	case UART_RX_RDY:
		long_received[evt_num] = evt->data.rx.len;
		evt_num++;
		k_sem_give(&rx_rdy);
		break;
	case UART_RX_BUF_RELEASED:
		k_sem_give(&rx_buf_released);
		break;
	case UART_RX_DISABLED:
		k_sem_give(&rx_disabled);
		break;
	case UART_RX_BUF_REQUEST:
		if (next_buf) {
			uart_rx_buf_rsp(uart_dev, long_rx_buf2, 1024);
			next_buf = false;
		}
		k_sem_give(&rx_disabled);
		break;
	default:
		break;
	}
}

static void *long_buffers_setup(void)
{
	uart_async_test_init();

	uart_callback_set(uart_dev, test_long_buffers_callback, NULL);

	return NULL;
}

ZTEST_USER(uart_async_long_buf, test_long_buffers)
{
	memset(long_rx_buf, 0, sizeof(long_rx_buf));
	memset(long_tx_buf, 1, sizeof(long_tx_buf));

	uart_rx_enable(uart_dev, long_rx_buf, sizeof(long_rx_buf), 10 * USEC_PER_MSEC);

	uart_tx(uart_dev, long_tx_buf, 500, 200 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(200)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(long_received[0], 500, "Wrong number of bytes received.");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf, 500),
		      0,
		      "Buffers not equal");

	evt_num = 0;
	uart_tx(uart_dev, long_tx_buf, 1000, 200 * USEC_PER_MSEC);
	zassert_equal(k_sem_take(&tx_done, K_MSEC(200)), 0, "TX_DONE timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(k_sem_take(&rx_rdy, K_MSEC(200)), 0, "RX_RDY timeout");
	zassert_equal(long_received[0], 524, "Wrong number of bytes received.");
	zassert_equal(long_received[1], 476, "Wrong number of bytes received.");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf + 500, long_received[0]),
		      0,
		      "Buffers not equal");
	zassert_equal(memcmp(long_tx_buf, long_rx_buf2, long_received[1]),
		      0,
		      "Buffers not equal");

	uart_rx_disable(uart_dev);
	zassert_equal(k_sem_take(&rx_buf_released, K_MSEC(100)),
		      0,
		      "RX_BUF_RELEASED timeout");
	zassert_equal(k_sem_take(&rx_disabled, K_MSEC(100)), 0,
		      "RX_DISABLED timeout");
}


ZTEST_SUITE(uart_async_single_read, NULL, single_read_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_multi_rx, NULL, multiple_rx_enable_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_chain_read, NULL, chained_read_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_double_buf, NULL, double_buffer_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_read_abort, NULL, read_abort_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_chain_write, NULL, chained_write_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_long_buf, NULL, long_buffers_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_write_abort, NULL, write_abort_setup,
		NULL, NULL, NULL);

ZTEST_SUITE(uart_async_timeout, NULL, forever_timeout_setup,
		NULL, NULL, NULL);
