/*
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <ztest.h>

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

/* Default connection values */
#define INTVL_MIN 6U /* multiple of 1.25 ms (min 6, max 3200) */
#define INTVL_MAX 6U /* multiple of 1.25 ms (min 6, max 3200) */
#define LATENCY 1U
#define TIMEOUT 10U /* multiple of 10 ms (min 10, max 3200) */

/* Default conn_update_ind PDU */
struct pdu_data_llctrl_conn_update_ind conn_update_ind = { .win_size = 1U,
							   .win_offset = 0U,
							   .interval = INTVL_MAX,
							   .latency = LATENCY,
							   .timeout = TIMEOUT,
							   .instant = 6U };

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
/* Default conn_param_req PDU */
struct pdu_data_llctrl_conn_param_req conn_param_req = { .interval_min = INTVL_MIN,
							 .interval_max = INTVL_MAX,
							 .latency = LATENCY,
							 .timeout = TIMEOUT,
							 .preferred_periodicity = 0U,
							 .reference_conn_event_count = 0u,
							 .offset0 = 0x0000U,
							 .offset1 = 0xffffU,
							 .offset2 = 0xffffU,
							 .offset3 = 0xffffU,
							 .offset4 = 0xffffU,
							 .offset5 = 0xffffU };

/* Default conn_param_rsp PDU */
struct pdu_data_llctrl_conn_param_rsp conn_param_rsp = { .interval_min = INTVL_MIN,
							 .interval_max = INTVL_MAX,
							 .latency = LATENCY,
							 .timeout = TIMEOUT,
							 .preferred_periodicity = 0U,
							 .reference_conn_event_count = 0u,
							 .offset0 = 0x0000U,
							 .offset1 = 0xffffU,
							 .offset2 = 0xffffU,
							 .offset3 = 0xffffU,
							 .offset4 = 0xffffU,
							 .offset5 = 0xffffU };

/* Different PDU contents for (B) */

/* Default conn_param_req PDU (B) */
struct pdu_data_llctrl_conn_param_req conn_param_req_B = {
	.interval_min = INTVL_MIN,
	.interval_max = INTVL_MAX,
	.latency = LATENCY + 1U, /* differentiate parameter */
	.timeout = TIMEOUT + 1U, /* differentiate parameter */
	.preferred_periodicity = 0U,
	.reference_conn_event_count = 0u,
	.offset0 = 0x0000U,
	.offset1 = 0xffffU,
	.offset2 = 0xffffU,
	.offset3 = 0xffffU,
	.offset4 = 0xffffU,
	.offset5 = 0xffffU
};

/* Default conn_param_rsp PDU (B) */
struct pdu_data_llctrl_conn_param_rsp conn_param_rsp_B = {
	.interval_min = INTVL_MIN,
	.interval_max = INTVL_MAX,
	.latency = LATENCY + 1U, /* differentiate parameter */
	.timeout = TIMEOUT + 1U, /* differentiate parameter */
	.preferred_periodicity = 0U,
	.reference_conn_event_count = 0u,
	.offset0 = 0x0000U,
	.offset1 = 0xffffU,
	.offset2 = 0xffffU,
	.offset3 = 0xffffU,
	.offset4 = 0xffffU,
	.offset5 = 0xffffU
};
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

/* Default conn_update_ind PDU (B) */
struct pdu_data_llctrl_conn_update_ind conn_update_ind_B = {
	.win_size = 1U,
	.win_offset = 0U,
	.interval = INTVL_MAX,
	.latency = LATENCY + 1U, /* differentiate parameter */
	.timeout = TIMEOUT + 1U, /* differentiate parameter */
	.instant = 6U
};

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
struct pdu_data_llctrl_conn_param_req *req_B = &conn_param_req_B;
struct pdu_data_llctrl_conn_param_rsp *rsp_B = &conn_param_rsp_B;
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

struct pdu_data_llctrl_conn_update_ind *cu_ind_B = &conn_update_ind_B;

static struct ll_conn conn;

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
static void test_unmask_feature_conn_param_req(struct ll_conn *conn)
{
	conn->llcp.fex.features_used &= ~BIT64(BT_LE_FEAT_BIT_CONN_PARAM_REQ);
}

static bool test_get_feature_conn_param_req(struct ll_conn *conn)
{
	return (conn->llcp.fex.features_used & BIT64(BT_LE_FEAT_BIT_CONN_PARAM_REQ));
}
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

static void setup(void)
{
	test_setup(&conn);

	/* Initialize lll conn parameters (different from new) */
	struct lll_conn *lll = &conn.lll;

	lll->interval = 0;
	lll->latency = 0;
	conn.supervision_reload = 1U;
}

static bool is_instant_reached(struct ll_conn *conn, uint16_t instant)
{
	return ((event_counter(conn) - instant) & 0xFFFF) <= 0x7FFF;
}

#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_RSP |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_accept(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_PARAM_RSP, &conn, &conn_param_rsp);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	conn_update_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host rejects.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |         LL_REJECT_EXT_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_reject(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_UNACCEPT_CONN_PARAM
	};

	struct node_rx_pu cu = { .status = BT_HCI_ERR_UNACCEPT_CONN_PARAM };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host is legacy.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |         LL_REJECT_EXT_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_remote_legacy(void)
{
	bool feature_bit_param_req;
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE
	};

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Check that feature Param Reg. is unmasked */
	feature_bit_param_req = test_get_feature_conn_param_req(&conn);
	zassert_equal(feature_bit_param_req, false, "Feature bit not unmasked");

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Controller do not
 * support Connection Parameters Request procedure, features not exchanged.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |            LL_UNKNOWN_RSP |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_unsupp_wo_feat_exch(void)
{
	bool feature_bit_param_req;
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ
	};

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown_rsp);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Check that feature Param Reg. is unmasked */
	feature_bit_param_req = test_get_feature_conn_param_req(&conn);
	zassert_equal(feature_bit_param_req, false, "Feature bit not unmasked");

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Controller do not
 * support Connection Parameters Request procedure, features exchanged.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_unsupp_w_feat_exch(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Disable feature */
	test_unmask_feature_conn_param_req(&conn);

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	conn_update_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * (A)
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * and
 *
 * (B)
 * Slave-initiated Connection Parameters Request procedure.
 * Procedure collides and is rejected.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   | (A)
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ | (B)
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                <--------------------->                |
 *    |                < PROCEDURE COLLISION >                |
 *    |                <--------------------->                |
 *    |                           |                           |
 *    |                           | LL_REJECT_EXT_IND         | (B)
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_RSP | (A)
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  | (A)
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_collision(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_LL_PROC_COLLISION
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* (A) Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* (B) Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, req_B);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/**/

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should have one LL Control PDU */
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &reject_ext_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/**/

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* (A) Rx */
	lt_tx(LL_CONNECTION_PARAM_RSP, &conn, &conn_param_rsp);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should have one LL Control PDU */
	conn_update_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (A) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (A) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* (A) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           |
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Reply                     |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_mas_rem_accept(void)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	ull_cp_conn_param_req_reply(&conn);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	conn_update_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host rejects.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           |
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Negative Reply            |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    |                           | LL_REJECT_EXT_IND         |
 *    |                           |-------------------------->|
 *    |                           |                           |
 */
void test_conn_update_mas_rem_reject(void)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_UNACCEPT_CONN_PARAM
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	ull_cp_conn_param_req_neg_reply(&conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &reject_ext_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/* Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Controller do not
 * support Connection Parameters Request procedure.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_UNKNOWN_RSP            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 */
void test_conn_update_mas_rem_unsupp_feat(void)
{
	/* TODO(thoh): Implement when Remote Request machine has feature
	 * checking
	 */
}

/*
 * (A)
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host accepts.
 *
 * and
 *
 * (B)
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * NOTE:
 * Master-initiated Connection Parameters Request procedure is paused.
 * Slave-initiated Connection Parameters Request procedure is finished.
 * Master-initiated Connection Parameters Request procedure is resumed.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------| (A)
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           | (B)
 *    |                           |                           |
 *    |               <------------------------>              |
 *    |               < LOCAL PROCEDURE PAUSED >              |
 *    |               <------------------------>              |
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           | (A)
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Reply                     |                           |
 *    |-------------------------->|                           | (A)
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->| (A)
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (A)
 *    |                           |                           |
 *    |              <------------------------->              |
 *    |              < LOCAL PROCEDURE RESUMED >              |
 *    |              <------------------------->              |
 *    |                           |                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->| (B)
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_RSP |
 *    |                           |<--------------------------| (B)
 *    |                           |                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->| (B)
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (B)
 *    |                           |                           |
 */
void test_conn_update_mas_rem_collision(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (A) Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* (B) Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, req_B->interval_min, req_B->interval_max, req_B->latency,
				 req_B->timeout);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/*******************/

	/* (A) There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	/* (A) */
	ull_cp_conn_param_req_reply(&conn);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should have one LL Control PDU */
	conn_update_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (A) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (A) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should have one LL Control PDU */
	req_B->reference_conn_event_count = event_counter(&conn) - 1;
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, req_B);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* (A) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/* Prepare */
	event_prepare(&conn);

	/* (B) Rx */
	lt_tx(LL_CONNECTION_PARAM_RSP, &conn, rsp_B);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should have one LL Control PDU */
	conn_update_ind_B.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind_B);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save Instant */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (B) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (B) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* (B) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_sla_loc_accept(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	instant = conn_update_ind.instant;
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

	/* Done */
	event_done(&conn);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host rejects.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |         LL_REJECT_EXT_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_sla_loc_reject(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_UNACCEPT_CONN_PARAM };

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_UNACCEPT_CONN_PARAM
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Controller do not
 * support Connection Parameters Request procedure, features not exchanged.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |            LL_UNKNOWN_RSP |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_sla_loc_unsupp_feat_wo_feat_exch(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE };

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Controller do not
 * support Connection Parameters Request procedure, features exchanged.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |-------------------------->|
 *    |                           |                           |
 */
void test_conn_update_sla_loc_unsupp_feat_w_feat_exch(void)
{
	uint8_t err;

	/* Disable feature */
	test_unmask_feature_conn_param_req(&conn);

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_UNSUPP_REMOTE_FEATURE, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have no LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * (A)
 * Slave-initiated Connection Parameters Request procedure.
 * Procedure collides and is rejected.
 *
 * and
 *
 * (B)
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           | (A)
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->| (A)
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------| (B)
 *    |                           |                           |
 *    |                <--------------------->                |
 *    |                < PROCEDURE COLLISION >                |
 *    |                <--------------------->                |
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           | (B)
 *    |                           |                           |
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Reply                     |                           |
 *    |-------------------------->|                           | (B)
 *    |                           |                           |
 *    |                           | LL_REJECT_EXT_IND         |
 *    |                           |-------------------------->| (A)
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (A)
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------| (B)
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (B)
 */
void test_conn_update_sla_loc_collision(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct node_rx_pu cu1 = { .status = BT_HCI_ERR_LL_PROC_COLLISION };

	struct node_rx_pu cu2 = { .status = BT_HCI_ERR_SUCCESS };

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_LL_PROC_COLLISION
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* (A) Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, &conn_param_req);
	lt_rx_q_is_empty(&conn);

	/* (B) Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, req_B);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/*******************/

	/* (B) There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, req_B);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	/* (B) */
	ull_cp_conn_param_req_reply(&conn);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should have one LL Control PDU */
	rsp_B->reference_conn_event_count = req_B->reference_conn_event_count;
	lt_rx(LL_CONNECTION_PARAM_RSP, &conn, &tx, rsp_B);
	lt_rx_q_is_empty(&conn);

	/* (A) Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* (A) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu1);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/* Prepare */
	event_prepare(&conn);

	/* (B) Rx */
	cu_ind_B->instant = instant = event_counter(&conn) + 6;
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, cu_ind_B);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (B) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (B) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* (B) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu2);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           |
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Reply                     |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    |                           | LL_CONNECTION_PARAM_RSP   |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_sla_rem_accept(void)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	ull_cp_conn_param_req_reply(&conn);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_RSP, &conn, &tx, &conn_param_rsp);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	instant = conn_update_ind.instant;
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host rejects.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           |
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Negative Reply            |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    |                           | LL_REJECT_EXT_IND         |
 *    |                           |-------------------------->|
 *    |                           |                           |
 */
void test_conn_update_sla_rem_reject(void)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONN_PARAM_REQ,
		.error_code = BT_HCI_ERR_UNACCEPT_CONN_PARAM
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	ull_cp_conn_param_req_neg_reply(&conn, BT_HCI_ERR_UNACCEPT_CONN_PARAM);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &reject_ext_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Controller do not
 * support Connection Parameters Request procedure.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           | LL_UNKNOWN_RSP            |
 *    |                           |-------------------------->|
 *    |                           |                           |
 */
void test_conn_update_sla_rem_unsupp_feat(void)
{
	/* TODO(thoh): Implement when Remote Request machine has feature
	 * checking
	 */
}

/*
 * (A)
 * Master-initiated Connection Parameters Request procedure.
 * Master requests change in LE connection parameters, slave’s Host accepts.
 *
 * and
 *
 * (B)
 * Slave-initiated Connection Parameters Request procedure.
 * Slave requests change in LE connection parameters, master’s Host accepts.
 *
 * NOTE:
 * Slave-initiated Connection Parameters Request procedure is paused.
 * Master-initiated Connection Parameters Request procedure is finished.
 * Slave-initiated Connection Parameters Request procedure is resumed.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |   LL_CONNECTION_PARAM_REQ |
 *    |                           |<--------------------------| (A)
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           | (B)
 *    |                           |                           |
 *    |               <------------------------>              |
 *    |               < LOCAL PROCEDURE PAUSED >              |
 *    |               <------------------------>              |
 *    |                           |                           |
 *    |      LE Remote Connection |                           |
 *    |         Parameter Request |                           |
 *    |<--------------------------|                           | (A)
 *    | LE Remote Connection      |                           |
 *    | Parameter Request         |                           |
 *    | Reply                     |                           |
 *    |-------------------------->|                           | (A)
 *    |                           |                           |
 *    |                           | LL_CONNECTION_PARAM_RSP   |
 *    |                           |-------------------------->| (A)
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------| (A)
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (A)
 *    |                           |                           |
 *    |              <------------------------->              |
 *    |              < LOCAL PROCEDURE RESUMED >              |
 *    |              <------------------------->              |
 *    |                           |                           |
 *    |                           | LL_CONNECTION_PARAM_REQ   |
 *    |                           |-------------------------->| (B)
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------| (B)
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           | (B)
 *    |                           |                           |
 */
void test_conn_update_sla_rem_collision(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (A) Rx */
	lt_tx(LL_CONNECTION_PARAM_REQ, &conn, &conn_param_req);

	/* Done */
	event_done(&conn);

	/*******************/

	/* (B) Initiate a Connection Parameter Request Procedure */
	err = ull_cp_conn_update(&conn, req_B->interval_min, req_B->interval_max, req_B->latency,
				 req_B->timeout);
	zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/*******************/

	/* (A) There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_PARAM_REQ, &ntf, &conn_param_req);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/*******************/

	/* (A) */
	ull_cp_conn_param_req_reply(&conn);

	/*******************/

	/* Prepare */
	event_prepare(&conn);

	/* (A) Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_RSP, &conn, &tx, &conn_param_rsp);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* (A) Rx */
	instant = conn_update_ind.instant;
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (A) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (A) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should have one LL Control PDU */
	lt_rx(LL_CONNECTION_PARAM_REQ, &conn, &tx, req_B);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* (A) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* (B) Rx */
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, cu_ind_B);

	/* Done */
	event_done(&conn);

	/* */
	while (!is_instant_reached(&conn, instant)) {
		/* Prepare */
		event_prepare(&conn);

		/* (B) Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* (B) There should NOT be a host notification */
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);

	/* (B) Tx Queue should NOT have a LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* (B) There should be one host notification */
	ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
	ut_rx_q_is_empty();

	/* Release Ntf */
	ull_cp_release_ntf(ntf);
	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}
#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */

/*
 * Parameter Request Procedure not supported.
 * Master-initiated Connection Update procedure.
 * Master requests update of LE connection.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_UPDATE_IND  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 *    | (If conn. parameters are  |                           |
 *    |  unchanged, host should   |                           |
 *    |  not receive a ntf.)      |                           |
 *    |                           |                           |
 */
void test_conn_update_mas_loc_accept_no_param_req(void)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	/* Test with and without parameter change  */
	uint8_t parameters_changed = 1U;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	do {
		/* Initiate a Connection Update Procedure */
		err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
		zassert_equal(err, BT_HCI_ERR_SUCCESS, NULL);

		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should have one LL Control PDU */
		conn_update_ind.instant = event_counter(&conn) + 6U;
		lt_rx(LL_CONNECTION_UPDATE_IND, &conn, &tx, &conn_update_ind);
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		/* Release Tx */
		ull_cp_release_tx(&conn, tx);

		/* Save Instant */
		pdu = (struct pdu_data *)tx->pdu;
		instant = sys_le16_to_cpu(pdu->llctrl.conn_update_ind.instant);

		/* Release Tx */
		ull_cp_release_tx(&conn, tx);

		/* */
		while (!is_instant_reached(&conn, instant)) {
			/* Prepare */
			event_prepare(&conn);

			/* Tx Queue should NOT have a LL Control PDU */
			lt_rx_q_is_empty(&conn);

			/* Done */
			event_done(&conn);

			/* There should NOT be a host notification */
			ut_rx_q_is_empty();
		}

		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		if (parameters_changed == 0U) {
			/* There should NOT be a host notification */
			ut_rx_q_is_empty();
		} else {
			/* There should be one host notification */
			ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
			ut_rx_q_is_empty();

			/* Release Ntf */
			ull_cp_release_ntf(ntf);
		}
	} while (parameters_changed-- > 0U);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Parameter Request Procedure not supported.
 * Slave-initiated Connection Update procedure.
 * Master receives Connection Update parameters.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_M  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    |                           |           LL_UNKNOWN_RSP  |
 *    |                           |-------------------------->|
 *    |                           |                           |
 *    |                           |                           |
 */
void test_conn_update_mas_rem_accept_no_param_req(void)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_CONN_UPDATE_IND
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_UNKNOWN_RSP, &conn, &tx, &unknown_rsp);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should NOT be a host notification */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Parameter Request Procedure not supported.
 * Master-initiated Connection Update procedure.
 * Slave receives Connection Update parameters.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    |                           |  LL_CONNECTION_UPDATE_IND |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |                           |                           |
 *    |      LE Connection Update |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 *    | (If conn. parameters are  |                           |
 *    |  unchanged, host should   |                           |
 *    |  not receive a ntf.)      |                           |
 *    |                           |                           |
 */
void test_conn_update_sla_rem_accept_no_param_req(void)
{
	struct node_rx_pdu *ntf;
	uint16_t instant;

	/* Test with and without parameter change  */
	uint8_t parameters_changed = 1U;

	struct node_rx_pu cu = { .status = BT_HCI_ERR_SUCCESS };

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	do {
		/* Prepare */
		event_prepare(&conn);

		/* Rx */
		instant = conn_update_ind.instant;
		lt_tx(LL_CONNECTION_UPDATE_IND, &conn, &conn_update_ind);

		/* Done */
		event_done(&conn);

		/* */
		while (!is_instant_reached(&conn, instant)) {
			/* Prepare */
			event_prepare(&conn);

			/* Tx Queue should NOT have a LL Control PDU */
			lt_rx_q_is_empty(&conn);

			/* Done */
			event_done(&conn);

			/* There should NOT be a host notification */
			ut_rx_q_is_empty();
		}

		/* Prepare */
		event_prepare(&conn);

		/* Tx Queue should NOT have a LL Control PDU */
		lt_rx_q_is_empty(&conn);

		/* Done */
		event_done(&conn);

		if (parameters_changed == 0U) {
			/* There should NOT be a host notification */
			ut_rx_q_is_empty();
		} else {
			/* There should be one host notification */
			ut_rx_node(NODE_CONN_UPDATE, &ntf, &cu);
			ut_rx_q_is_empty();

			/* Release Ntf */
			ull_cp_release_ntf(ntf);
		}
	} while (parameters_changed-- > 0U);

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

/*
 * Parameter Request Procedure not supported.
 * Slave-initiated Connection Update procedure (not allowed).
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_S  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |                           |
 *    | LE Connection Update      |                           |
 *    |-------------------------->|                           |
 *    |                           |                           |
 *    |      ERR CMD Disallowed   |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
void test_conn_update_sla_loc_disallowed_no_param_req(void)
{
	uint8_t err;

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Update Procedure */
	err = ull_cp_conn_update(&conn, INTVL_MIN, INTVL_MAX, LATENCY, TIMEOUT);
	zassert_equal(err, BT_HCI_ERR_CMD_DISALLOWED, NULL);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have no LL Control PDU */
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	zassert_equal(ctx_buffers_free(), CONFIG_BT_CTLR_LLCP_PROC_CTX_BUF_NUM,
		      "Free CTX buffers %d", ctx_buffers_free());
}

void test_main(void)
{
#if defined(CONFIG_BT_CTLR_CONN_PARAM_REQ)
	ztest_test_suite(
		mas_loc,
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_accept, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_reject, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_remote_legacy, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_unsupp_wo_feat_exch, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_unsupp_w_feat_exch, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_mas_loc_collision, setup,
					       unit_test_noop));

	ztest_test_suite(mas_rem,
			 ztest_unit_test_setup_teardown(test_conn_update_mas_rem_accept, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_mas_rem_reject, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_mas_rem_unsupp_feat, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_mas_rem_collision, setup,
							unit_test_noop));

	ztest_test_suite(
		sla_loc,
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_accept, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_reject, setup,
					       unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_unsupp_feat_wo_feat_exch,
					       setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_unsupp_feat_w_feat_exch,
					       setup, unit_test_noop),
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_collision, setup,
					       unit_test_noop));

	ztest_test_suite(sla_rem,
			 ztest_unit_test_setup_teardown(test_conn_update_sla_rem_accept, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_sla_rem_reject, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_sla_rem_unsupp_feat, setup,
							unit_test_noop),
			 ztest_unit_test_setup_teardown(test_conn_update_sla_rem_collision, setup,
							unit_test_noop));

	ztest_run_test_suite(mas_loc);
	ztest_run_test_suite(mas_rem);
	ztest_run_test_suite(sla_loc);
	ztest_run_test_suite(sla_rem);

#else /* !CONFIG_BT_CTLR_CONN_PARAM_REQ */

	ztest_test_suite(mas_loc_no_param_req, ztest_unit_test_setup_teardown(
				 test_conn_update_mas_loc_accept_no_param_req,
				 setup, unit_test_noop));

	ztest_test_suite(mas_rem_no_param_req, ztest_unit_test_setup_teardown(
				 test_conn_update_mas_rem_accept_no_param_req,
				 setup, unit_test_noop));

	ztest_test_suite(
		sla_loc_no_param_req,
		ztest_unit_test_setup_teardown(test_conn_update_sla_loc_disallowed_no_param_req,
					       setup, unit_test_noop));

	ztest_test_suite(sla_rem_no_param_req, ztest_unit_test_setup_teardown(
				 test_conn_update_sla_rem_accept_no_param_req,
				 setup, unit_test_noop));

	ztest_run_test_suite(mas_loc_no_param_req);
	ztest_run_test_suite(mas_rem_no_param_req);
	ztest_run_test_suite(sla_loc_no_param_req);
	ztest_run_test_suite(sla_rem_no_param_req);

#endif /* CONFIG_BT_CTLR_CONN_PARAM_REQ */
}
