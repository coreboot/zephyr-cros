/*
 * Copyright (c) 2023 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "common.h"

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/iso.h>
#include <zephyr/sys/printk.h>

#define ENQUEUE_COUNT 2

extern enum bst_result_t bst_result;
static struct bt_iso_chan iso_chans[CONFIG_BT_ISO_MAX_CHAN];
static struct bt_iso_chan *default_chan = &iso_chans[0];
static struct bt_iso_cig *cig;
static uint16_t seq_num;
static volatile size_t enqueue_cnt;
static uint32_t interval_us = 10U * USEC_PER_MSEC; /* 10 ms */
NET_BUF_POOL_FIXED_DEFINE(tx_pool, ENQUEUE_COUNT, BT_ISO_SDU_BUF_SIZE(CONFIG_BT_ISO_TX_MTU),
			  CONFIG_BT_CONN_TX_USER_DATA_SIZE, NULL);

BUILD_ASSERT(CONFIG_BT_ISO_MAX_CHAN > 1, "CONFIG_BT_ISO_MAX_CHAN shall be at least 2");

CREATE_FLAG(flag_iso_connected);

static void send_data_cb(struct k_work *work)
{
	static uint8_t buf_data[CONFIG_BT_ISO_TX_MTU];
	static size_t len_to_send = 1;
	static bool data_initialized;
	struct net_buf *buf;
	int ret;

	if (!TEST_FLAG(flag_iso_connected)) {
		/* TX has been aborted */
		return;
	}

	if (!data_initialized) {
		for (int i = 0; i < ARRAY_SIZE(buf_data); i++) {
			buf_data[i] = (uint8_t)i;
		}

		data_initialized = true;
	}

	buf = net_buf_alloc(&tx_pool, K_FOREVER);
	net_buf_reserve(buf, BT_ISO_CHAN_SEND_RESERVE);

	net_buf_add_mem(buf, buf_data, len_to_send);

	ret = bt_iso_chan_send(default_chan, buf, seq_num++, BT_ISO_TIMESTAMP_NONE);
	if (ret < 0) {
		printk("Failed to send ISO data (%d)\n", ret);
		net_buf_unref(buf);

		/* Reschedule for next interval */
		k_work_reschedule(k_work_delayable_from_work(work), K_USEC(interval_us));

		return;
	}

	len_to_send++;
	if (len_to_send > ARRAY_SIZE(buf_data)) {
		len_to_send = 1;
	}

	enqueue_cnt--;
	if (enqueue_cnt > 0U) {
		/* If we have more buffers available, we reschedule the workqueue item immediately
		 * to trigger another encode + TX, but without blocking this call for too long
		 */
		k_work_reschedule(k_work_delayable_from_work(work), K_NO_WAIT);
	}
}
K_WORK_DELAYABLE_DEFINE(iso_send_work, send_data_cb);

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	int err;

	err = bt_le_scan_stop();
	if (err) {
		FAIL("Failed to stop scanning (err %d)\n", err);

		return;
	}

	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT,
				&default_conn);
	if (err) {
		FAIL("Failed to create connection (err %d)\n", err);

		return;
	}
}

static void iso_connected(struct bt_iso_chan *chan)
{
	printk("ISO Channel %p connected\n", chan);

	seq_num = 0U;
	enqueue_cnt = ENQUEUE_COUNT;

	/* Start send timer */
	k_work_schedule(&iso_send_work, K_MSEC(0));

	SET_FLAG(flag_iso_connected);
}

static void iso_disconnected(struct bt_iso_chan *chan, uint8_t reason)
{
	printk("ISO Channel %p disconnected (reason 0x%02x)\n", chan, reason);

	k_work_cancel_delayable(&iso_send_work);

	UNSET_FLAG(flag_iso_connected);
}

static void sdu_sent_cb(struct bt_iso_chan *chan)
{
	int err;

	enqueue_cnt++;

	if (!TEST_FLAG(flag_iso_connected)) {
		/* TX has been aborted */
		return;
	}

	err = k_work_schedule(&iso_send_work, K_NO_WAIT);
	if (err < 0) {
		FAIL("Failed to schedule TX for chan %p: %d\n", chan, err);
	}
}

static void init(void)
{
	static struct bt_iso_chan_ops iso_ops = {
		.connected = iso_connected,
		.disconnected = iso_disconnected,
		.sent = sdu_sent_cb,
	};
	static struct bt_iso_chan_io_qos iso_tx = {
		.sdu = CONFIG_BT_ISO_TX_MTU,
		.phy = BT_GAP_LE_PHY_2M,
		.rtn = 1,
		.path = NULL,
	};
	static struct bt_iso_chan_qos iso_qos = {
		.tx = &iso_tx,
		.rx = NULL,
	};
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		FAIL("Bluetooth enable failed (err %d)\n", err);

		return;
	}

	for (size_t i = 0U; i < ARRAY_SIZE(iso_chans); i++) {
		iso_chans[i].ops = &iso_ops;
		iso_chans[i].qos = &iso_qos;
#if defined(CONFIG_BT_SMP)
		iso_chans[i].required_sec_level = BT_SECURITY_L2;
#endif /* CONFIG_BT_SMP */
	}
}

static void create_cig(void)
{
	struct bt_iso_cig_param param;
	int err;

	param.cis_channels = &default_chan;
	param.num_cis = 1U;
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = BT_ISO_PACKING_SEQUENTIAL;
	param.framing = BT_ISO_FRAMING_UNFRAMED;
	param.latency = 10U;          /* ms */
	param.interval = interval_us; /* us */

	err = bt_iso_cig_create(&param, &cig);
	if (err != 0) {
		FAIL("Failed to create CIG (%d)\n", err);

		return;
	}
}

static void reconfigure_cig(void)
{
	struct bt_iso_chan *channels[2];
	struct bt_iso_cig_param param;
	int err;

	for (size_t i = 0U; i < ARRAY_SIZE(channels); i++) {
		channels[i] = &iso_chans[i];
	}

	/* Set parameters to same as the ones used to create the CIG */
	param.cis_channels = &default_chan;
	param.num_cis = 1U;
	param.sca = BT_GAP_SCA_UNKNOWN;
	param.packing = BT_ISO_PACKING_SEQUENTIAL;
	param.framing = BT_ISO_FRAMING_UNFRAMED;
	param.latency = 10U;          /* ms */
	param.interval = interval_us; /* us */

	/* Test modifying existing CIS */
	default_chan->qos->tx->rtn++;

	err = bt_iso_cig_reconfigure(cig, &param);
	if (err != 0) {
		FAIL("Failed to reconfigure CIS to new RTN (%d)\n", err);

		return;
	}

	/* Test modifying CIG parameter without any CIS */
	param.num_cis = 0U;
	param.interval = 7500U; /* us */

	err = bt_iso_cig_reconfigure(cig, &param);
	if (err != 0) {
		FAIL("Failed to reconfigure CIG to new interval (%d)\n", err);

		return;
	}

	/* Add CIS to the CIG and restore interval to 10ms */
	param.cis_channels = &channels[1];
	param.num_cis = 1U;
	param.interval = interval_us; /* us */

	err = bt_iso_cig_reconfigure(cig, &param);
	if (err != 0) {
		FAIL("Failed to reconfigure CIG with new CIS and original interval (%d)\n", err);

		return;
	}
}

static void connect_acl(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err != 0) {
		FAIL("Scanning failed to start (err %d)\n", err);

		return;
	}

	WAIT_FOR_FLAG_SET(flag_connected);
}

static void connect_cis(void)
{
	const struct bt_iso_connect_param connect_param = {
		.acl = default_conn,
		.iso_chan = default_chan,
	};
	int err;

	err = bt_iso_chan_connect(&connect_param, 1);
	if (err) {
		FAIL("Failed to connect ISO (%d)\n", err);

		return;
	}

	WAIT_FOR_FLAG_SET(flag_iso_connected);
}

static void disconnect_cis(void)
{
	int err;

	err = bt_iso_chan_disconnect(default_chan);
	if (err) {
		FAIL("Failed to disconnect ISO (err %d)\n", err);

		return;
	}

	WAIT_FOR_FLAG_UNSET(flag_iso_connected);
}

static void disconnect_acl(void)
{
	int err;

	err = bt_conn_disconnect(default_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	if (err) {
		FAIL("Failed to disconnect ACL (err %d)\n", err);

		return;
	}

	WAIT_FOR_FLAG_UNSET(flag_connected);
}

static void terminate_cig(void)
{
	int err;

	err = bt_iso_cig_terminate(cig);
	if (err != 0) {
		FAIL("Failed to terminate CIG (%d)\n", err);

		return;
	}

	cig = NULL;
}

static void test_main(void)
{
	init();
	create_cig();
	reconfigure_cig();
	connect_acl();
	connect_cis();

	while (seq_num < 100U) {
		k_sleep(K_USEC(interval_us));
	}

	disconnect_cis();
	disconnect_acl();
	terminate_cig();

	PASS("Test passed\n");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "central",
		.test_descr = "Central",
		.test_post_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_main,
	},
	BSTEST_END_MARKER,
};

struct bst_test_list *test_main_cis_central_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}
