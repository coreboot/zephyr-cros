/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <ztest.h>
#include "kconfig.h"

#define ULL_LLCP_UNITTEST

#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/slist.h>
#include <sys/util.h>
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu.h"
#include "ll.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll_df_types.h"
#include "lll_conn.h"

#include "ull_tx_queue.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"

#include "helper_pdu.h"
#include "helper_util.h"

static struct ll_conn conn;

static void setup(void)
{
	test_setup(&conn);
}

static bool is_instant_reached(struct ll_conn *conn, uint16_t instant)
{
	return ((event_counter(conn) - instant) & 0xFFFF) <= 0x7FFF;
}

void test_channel_map_update_mas_loc(void)
{
	uint8_t chm[5] = { 0x00, 0x04, 0x05, 0x06, 0x00 };
	/* TODO should test setup set this to valid value? */
	uint8_t defchm[5] = {};
	uint8_t err;
	struct node_tx *tx;
	struct pdu_data *pdu;
	uint16_t instant;
	struct pdu_data_llctrl_chan_map_ind chmu_ind = {
		.instant = 6,
		.chm = { 0x00, 0x04, 0x05, 0x06, 0x00 },
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	err = ull_cp_chan_map_update(&conn, chm);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CHAN_MAP_UPDATE_IND, &conn, &tx, &chmu_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.chan_map_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* spin conn events */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();

		/* check if using old channel map */
		zassert_mem_equal(conn.lll.data_chan_map, defchm, sizeof(conn.lll.data_chan_map),
				  "Channel map invalid");
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	/* at this point new channel map shall be in use */
	zassert_mem_equal(conn.lll.data_chan_map, chm, sizeof(conn.lll.data_chan_map),
			  "Channel map invalid");

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
				  "Free CTX buffers %d", ctx_buffers_free());
}

void test_channel_map_update_sla_rem(void)
{
	uint8_t chm[5] = { 0x00, 0x04, 0x05, 0x06, 0x00 };
	/* TODO should test setup set this to valid value? */
	uint8_t defchm[5] = {};
	struct pdu_data_llctrl_chan_map_ind chmu_ind = {
		.instant = 6,
		.chm = { 0x00, 0x04, 0x05, 0x06, 0x00 },
	};
	uint16_t instant = 6;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* RX */
	lt_tx(LL_CHAN_MAP_UPDATE_IND, &conn, &chmu_ind);

	/* Done */
	event_done(&conn);

	/* spin conn events */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();

		/* check if using old channel map */
		zassert_mem_equal(conn.lll.data_chan_map, defchm, sizeof(conn.lll.data_chan_map),
				  "Channel map invalid");
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	/* at this point new channel map shall be in use */
	zassert_mem_equal(conn.lll.data_chan_map, chm, sizeof(conn.lll.data_chan_map),
			  "Channel map invalid");

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
				  "Free CTX buffers %d", ctx_buffers_free());
}

void test_channel_map_update_sla_loc(void)
{
	uint8_t err;
	uint8_t chm[5] = { 0x00, 0x06, 0x06, 0x06, 0x00 };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	err = ull_cp_chan_map_update(&conn, chm);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED, NULL);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
				  "Free CTX buffers %d", ctx_buffers_free());
}

void test_main(void)
{
	ztest_test_suite(chmu,
			 ztest_unit_test_setup_teardown(test_channel_map_update_mas_loc, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_channel_map_update_sla_rem, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_channel_map_update_sla_loc, setup,
							unit_test_noop));

	ztest_run_test_suite(chmu);
}
