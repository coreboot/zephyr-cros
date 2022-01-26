/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <ztest.h>
#include "kconfig.h"

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

struct ll_conn conn;

static void setup(void)
{
	test_setup(&conn);
}

/* +-----+                     +-------+                  +-----+
 * | UT  |                     | LL_A  |                  | LT  |
 * +-----+                     +-------+                  +-----+
 *    |                            |                         |
 *    | Start                      |                         |
 *    | Min used chans Proc.       |                         |
 *    |--------------------------->|                         |
 *    |                            |                         |
 *    |                            | LL_MIN_USED_CHANS_IND   |
 *    |                            |------------------------>|
 *    |                            |                 'll_ack'|
 *    |                            |                         |
 *    |                            |                         |
 */
void test_min_used_chans_sla_loc(void)
{
	uint8_t err;
	struct node_tx *tx;

	struct pdu_data_llctrl_min_used_chans_ind local_muc_ind = { .phys = 1,
		.min_used_chans = 2 };

	struct pdu_data_llctrl_min_used_chans_ind remote_muc_ind = { .phys = 1,
		.min_used_chans = 2 };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Min number of Used Channels Procedure */
	err = ull_cp_min_used_chans(&conn, 1, 2);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_MIN_USED_CHANS_IND, &conn,  &tx, &local_muc_ind);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_MIN_USED_CHANS_IND, &conn,  &remote_muc_ind);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* Release tx node */
	ull_cp_release_tx(&conn, tx);

	/* There should not be a host notifications */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_min_used_chans_mas_loc(void)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Min number of Used Channels Procedure */
	err = ull_cp_min_used_chans(&conn, 1, 2);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED, NULL);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_min_used_chans_mas_rem(void)
{
	struct pdu_data_llctrl_min_used_chans_ind remote_muc_ind = { .phys = 1,
		.min_used_chans = 2 };
	struct pdu_data_llctrl_chan_map_ind ch_map_ind = { .chm = { 0xff, 0xff, 0xff, 0xff, 0x1f },
		.instant = 7 };

	struct node_tx *tx;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_MIN_USED_CHANS_IND, &conn,  &remote_muc_ind);

	/* Emulate a phy to trigger channel map update */
	conn.lll.phy_tx = 0x7;

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CHAN_MAP_UPDATE_IND, &conn,  &tx, &ch_map_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should not be a host notifications */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM - 1,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_main(void)
{
	ztest_test_suite(
		muc,
		ztest_unit_test_setup_teardown(test_min_used_chans_sla_loc, setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_min_used_chans_mas_loc, setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_min_used_chans_mas_rem, setup, unit_test_noop));

	ztest_run_test_suite(muc);
}
