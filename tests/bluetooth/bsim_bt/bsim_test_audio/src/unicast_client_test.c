/*
 * Copyright (c) 2021-2022 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BT_AUDIO_UNICAST_CLIENT)

#include <bluetooth/bluetooth.h>
#include <bluetooth/audio/audio.h>
#include <bluetooth/audio/capabilities.h>
#include "common.h"
#include "unicast_common.h"

extern enum bst_result_t bst_result;

static struct bt_audio_stream g_streams[CONFIG_BT_AUDIO_UNICAST_CLIENT_ASE_SNK_COUNT];
static struct bt_codec *g_remote_codecs[CONFIG_BT_AUDIO_UNICAST_CLIENT_PAC_COUNT];
static struct bt_audio_ep *g_sinks[CONFIG_BT_AUDIO_UNICAST_CLIENT_ASE_SNK_COUNT];
static struct bt_conn *g_conn;

/* Mandatory support preset by both client and server */
static struct bt_audio_lc3_preset preset_16_2_1 = BT_AUDIO_LC3_UNICAST_PRESET_16_2_1;

CREATE_FLAG(flag_connected);
CREATE_FLAG(flag_mtu_exchanged);
CREATE_FLAG(flag_sink_discovered);
CREATE_FLAG(flag_stream_configured);
CREATE_FLAG(flag_stream_qos);
CREATE_FLAG(flag_stream_enabled);

static void stream_configured(struct bt_audio_stream *stream,
			      const struct bt_codec_qos_pref *pref)
{
	printk("Configured stream %p\n", stream);

	/* TODO: The preference should be used/taken into account when
	 * setting the QoS
	 */

	SET_FLAG(flag_stream_configured);
}

static void stream_qos_set(struct bt_audio_stream *stream)
{
	printk("QoS set stream %p\n", stream);

	SET_FLAG(flag_stream_qos);
}

static void stream_enabled(struct bt_audio_stream *stream)
{
	printk("Enabled stream %p\n", stream);

	SET_FLAG(flag_stream_enabled);
}

static void stream_started(struct bt_audio_stream *stream)
{
	printk("Started stream %p\n", stream);
}

static void stream_metadata_updated(struct bt_audio_stream *stream)
{
	printk("Metadata updated stream %p\n", stream);
}

static void stream_disabled(struct bt_audio_stream *stream)
{
	printk("Disabled stream %p\n", stream);
}

static void stream_stopped(struct bt_audio_stream *stream)
{
	printk("Stopped stream %p\n", stream);
}

static void stream_released(struct bt_audio_stream *stream)
{
	printk("Released stream %p\n", stream);
}

static struct bt_audio_stream_ops stream_ops = {
	.configured = stream_configured,
	.qos_set = stream_qos_set,
	.enabled = stream_enabled,
	.started = stream_started,
	.metadata_updated = stream_metadata_updated,
	.disabled = stream_disabled,
	.stopped = stream_stopped,
	.released = stream_released,
};

static void add_remote_sink(struct bt_audio_ep *ep, uint8_t index)
{
	printk("Sink #%u: ep %p\n", index, ep);

	g_sinks[index] = ep;
}

static void add_remote_codec(struct bt_codec *codec, int index, uint8_t type)
{
	printk("#%u: codec %p type 0x%02x\n", index, codec, type);

	print_codec(codec);

	if (type != BT_AUDIO_SINK && type != BT_AUDIO_SOURCE) {
		return;
	}

	if (index < CONFIG_BT_AUDIO_UNICAST_CLIENT_PAC_COUNT) {
		g_remote_codecs[index] = codec;
	}
}

static void discover_sink_cb(struct bt_conn *conn,
			    struct bt_codec *codec,
			    struct bt_audio_ep *ep,
			    struct bt_audio_discover_params *params)
{
	static bool codec_found;
	static bool endpoint_found;

	if (params->err != 0) {
		FAIL("Discovery failed: %d\n", params->err);
		return;
	}

	if (codec != NULL) {
		add_remote_codec(codec, params->num_caps, params->type);
		codec_found = true;
		return;
	}

	if (ep != NULL) {
		if (params->type == BT_AUDIO_SINK) {
			add_remote_sink(ep, params->num_eps);
			endpoint_found = true;
		} else {
			FAIL("Invalid param type: %u\n", params->type);
		}

		return;
	}

	printk("Discover complete\n");

	(void)memset(params, 0, sizeof(*params));

	if (endpoint_found && codec_found) {
		SET_FLAG(flag_sink_discovered);
	} else {
		FAIL("Did not discover endpoint and codec\n");
	}
}

static void gatt_mtu_cb(struct bt_conn *conn, uint8_t err,
		   struct bt_gatt_exchange_params *params)
{
	if (err != 0) {
		FAIL("Failed to exchange MTU (%u)\n", err);
		return;
	}

	printk("MTU exchanged\n");
	SET_FLAG(flag_mtu_exchanged);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err != 0) {
		bt_conn_unref(conn);
		g_conn = NULL;

		FAIL("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected to %s\n", addr);
	g_conn = conn;
	SET_FLAG(flag_connected);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void init(void)
{
	int err;

	err = bt_enable(NULL);
	if (err != 0) {
		FAIL("Bluetooth enable failed (err %d)\n", err);
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(g_streams); i++) {
		g_streams[i].ops = &stream_ops;
	}
}

static void scan_and_connect(void)
{
	int err;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err != 0) {
		FAIL("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
	WAIT_FOR_FLAG(flag_connected);
}

static void exchange_mtu(void)
{
	struct bt_gatt_exchange_params mtu_params = {
		.func = gatt_mtu_cb
	};
	int err;

	err = bt_gatt_exchange_mtu(g_conn, &mtu_params);
	if (err != 0) {
		FAIL("Failed to exchange MTU %d\n", err);
		return;
	}

	WAIT_FOR_FLAG(flag_mtu_exchanged);
}

static void discover_sink(void)
{
	static struct bt_audio_discover_params params;
	int err;

	params.func = discover_sink_cb;
	params.type = BT_AUDIO_SINK;

	err = bt_audio_discover(g_conn, &params);
	if (err != 0) {
		printk("Failed to discover sink: %d\n", err);
		return;
	}

	WAIT_FOR_FLAG(flag_sink_discovered);
}

static int configure_stream(struct bt_audio_stream *stream,
			    struct bt_audio_ep *ep)
{
	int err;

	UNSET_FLAG(flag_stream_configured);

	err = bt_audio_stream_config(g_conn, stream, ep,
				     &preset_16_2_1.codec);
	if (err != 0) {
		FAIL("Could not configure stream: %d\n", err);
		return err;
	}

	WAIT_FOR_FLAG(flag_stream_configured);

	return 0;
}

static void test_main(void)
{
	struct bt_audio_unicast_group *unicast_group;
	uint8_t stream_cnt;
	int err;

	init();

	scan_and_connect();

	exchange_mtu();

	discover_sink();

	printk("Configuring streams\n");
	for (stream_cnt = 0; stream_cnt < ARRAY_SIZE(g_sinks); stream_cnt++) {
		if (g_sinks[stream_cnt] == NULL) {
			break;
		}

		err = configure_stream(&g_streams[stream_cnt],
				       g_sinks[stream_cnt]);
		if (err != 0) {
			FAIL("Unable to configure stream[%u]: %d",
			     stream_cnt, err);
			return;
		}
	}

	printk("Creating unicast group\n");
	err = bt_audio_unicast_group_create(g_streams, stream_cnt,
					    &unicast_group);
	if (err != 0) {
		FAIL("Unable to create unicast group: %d", err);
		return;
	}

	/* TODO: When babblesim supports ISO setup Audio streams */

	printk("Deleting unicast group\n");
	err = bt_audio_unicast_group_delete(unicast_group);
	if (err != 0) {
		FAIL("Unable to delete unicast group: %d", err);
		return;
	}
	unicast_group = NULL;

	/* Recreate unicast group to verify that it's possible */
	printk("Recreating unicast group\n");
	err = bt_audio_unicast_group_create(g_streams, stream_cnt,
					    &unicast_group);
	if (err != 0) {
		FAIL("Unable to create unicast group: %d", err);
		return;
	}

	printk("Deleting unicast group\n");
	err = bt_audio_unicast_group_delete(unicast_group);
	if (err != 0) {
		FAIL("Unable to delete unicast group: %d", err);
		return;
	}
	unicast_group = NULL;

	PASS("Unicast client passed\n");
}

static const struct bst_test_instance test_unicast_client[] = {
	{
		.test_id = "unicast_client",
		.test_post_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_main
	},
	BSTEST_END_MARKER
};

struct bst_test_list *test_unicast_client_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_unicast_client);
}

#else /* !(CONFIG_BT_AUDIO_UNICAST_CLIENT) */

struct bst_test_list *test_unicast_client_install(struct bst_test_list *tests)
{
	return tests;
}

#endif /* CONFIG_BT_AUDIO_UNICAST_CLIENT */
