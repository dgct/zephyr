/*
 * Copyright (c) 2024 Demant A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/ztest.h>

#define ULL_LLCP_UNITTEST

#include <zephyr/bluetooth/hci.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>
#include "hal/ccm.h"

#include "util/util.h"
#include "util/mem.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"
#include "ll.h"
#include "ll_feat.h"
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_internal.h"
#include "ull_conn_types.h"
#include "ull_llcp.h"
#include "ull_conn_internal.h"
#include "ull_llcp_internal.h"
#include "ull_llcp_features.h"

#include "helper_pdu.h"
#include "helper_util.h"
#include "helper_features.h"

static struct ll_conn conn;

static void subrate_setup(void *data)
{
	test_setup(&conn);

	/* Emulate a completed feature exchange in which the peer supports
	 * LE Connection Subrating, so that locally-initiated requests are
	 * permitted (ull_cp_subrate() is gated on feature_subrating()).
	 */
	conn.llcp.fex.features_used |= BIT64(BT_LE_FEAT_BIT_CONN_SUBRATING);
	conn.llcp.fex.valid = 1U;
}

/*
 * Remotely-initiated Connection Subrate Update procedure - valid parameters.
 *
 * Connection Subrating has no procedure instant; the peripheral applies the
 * new parameters immediately on receiving LL_SUBRATE_IND and notifies the
 * host (Core 5.4, Vol 6, Part B, 5.1.19).
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |        LL_SUBRATE_IND      |
 *    |                           |<--------------------------|
 *    |  Subrate Change           |                           |
 *    |  (applied immediately)    |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(subrate_rem, test_subrate_peripheral_rem)
{
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_subrate_ind remote_ind = {
		.subrate_factor = 4,
		.subrate_base_event = 10,
		.latency = 2,
		.continuation_number = 1,
		.timeout = 1000,
	};

	struct pdu_data_llctrl_subrate_ind exp_ntf = {
		.subrate_factor = 4,
		.subrate_base_event = 10,
		.latency = 2,
		.continuation_number = 1,
		.timeout = 1000,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx an unsolicited LL_SUBRATE_IND from the central */
	lt_tx(LL_SUBRATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* There should be one host notification carrying the applied params */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* Verify the subrate parameters were applied immediately */
	zassert_equal(conn.lll.subrate.factor, 4, "subrate factor %u", conn.lll.subrate.factor);
	zassert_equal(conn.lll.subrate.base_event, 10, "subrate base_event %u",
		      conn.lll.subrate.base_event);
	zassert_equal(conn.lll.subrate.continuation_number, 1, "subrate cont %u",
		      conn.lll.subrate.continuation_number);
	zassert_equal(conn.lll.latency, 2, "latency %u", conn.lll.latency);
	zassert_equal(conn.supervision_timeout, 1000, "supervision timeout %u",
		      conn.supervision_timeout);

	release_ntf(ntf);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely-initiated Connection Subrate Update procedure - invalid parameters.
 *
 * An LL_SUBRATE_IND whose parameters fail the validity checks (here the
 * supervision timeout is too small for the requested subrate factor/latency)
 * must result in connection termination with BT_HCI_ERR_INVALID_LL_PARAM and
 * no host notification.
 */
ZTEST(subrate_rem, test_subrate_peripheral_rem_invalid)
{
	struct pdu_data_llctrl_subrate_ind remote_ind = {
		.subrate_factor = 200,
		.subrate_base_event = 0,
		.latency = 10,
		.continuation_number = 1,
		/* timeout*4 = 40 is NOT > factor*(latency+1)*interval = 200*11*6 = 13200 */
		.timeout = 10,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);

	/* Rx an unsolicited, invalid LL_SUBRATE_IND */
	lt_tx(LL_SUBRATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* Termination 'triggered' */
	zassert_equal(conn.llcp_terminate.reason_final, BT_HCI_ERR_INVALID_LL_PARAM,
		      "Terminate reason %d", conn.llcp_terminate.reason_final);

	/* There should be no host notifications */
	ut_rx_q_is_empty();

	/* No subrate parameters applied */
	zassert_equal(conn.lll.subrate.factor, 0, "subrate factor unexpectedly set to %u",
		      conn.lll.subrate.factor);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Subrate Update procedure - peer accepts.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    | Subrate Request           |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_SUBRATE_REQ            |
 *    |                           |-------------------------->|
 *    |                           |        LL_SUBRATE_IND      |
 *    |                           |<--------------------------|
 *    | Subrate Change            |                           |
 *    |<--------------------------|                           |
 *    |                           |                           |
 */
ZTEST(subrate_loc, test_subrate_peripheral_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_subrate_req exp_req = {
		.subrate_factor_min = 2,
		.subrate_factor_max = 8,
		.max_latency = 20,
		.continuation_number = 2,
		.timeout = 1000,
	};

	struct pdu_data_llctrl_subrate_ind remote_ind = {
		.subrate_factor = 8,
		.subrate_base_event = 5,
		.latency = 3,
		.continuation_number = 2,
		.timeout = 1000,
	};

	struct pdu_data_llctrl_subrate_ind exp_ntf = {
		.subrate_factor = 8,
		.subrate_base_event = 5,
		.latency = 3,
		.continuation_number = 2,
		.timeout = 1000,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Subrate Update Procedure */
	err = ull_cp_subrate(&conn, 2, 8, 20, 2, 1000);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_SUBRATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* There should be one host notification carrying the applied params */
	ut_rx_pdu(LL_SUBRATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* Verify the subrate parameters were applied */
	zassert_equal(conn.lll.subrate.factor, 8, "subrate factor %u", conn.lll.subrate.factor);
	zassert_equal(conn.lll.subrate.base_event, 5, "subrate base_event %u",
		      conn.lll.subrate.base_event);
	zassert_equal(conn.lll.subrate.continuation_number, 2, "subrate cont %u",
		      conn.lll.subrate.continuation_number);
	zassert_equal(conn.lll.latency, 3, "latency %u", conn.lll.latency);
	zassert_equal(conn.supervision_timeout, 1000, "supervision timeout %u",
		      conn.supervision_timeout);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);
	release_ntf(ntf);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Subrate Update procedure - peer rejects.
 *
 * The central rejects the LL_SUBRATE_REQ with LL_REJECT_EXT_IND; the host is
 * notified of the rejection and no subrate parameters are applied.
 */
ZTEST(subrate_loc, test_subrate_peripheral_loc_reject)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_subrate_req exp_req = {
		.subrate_factor_min = 2,
		.subrate_factor_max = 8,
		.max_latency = 20,
		.continuation_number = 2,
		.timeout = 1000,
	};

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	struct pdu_data_llctrl_reject_ext_ind exp_ntf = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Subrate Update Procedure */
	err = ull_cp_subrate(&conn, 2, 8, 20, 2, 1000);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* The host is notified of the rejection */
	ut_rx_pdu(LL_REJECT_EXT_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* No subrate parameters applied */
	zassert_equal(conn.lll.subrate.factor, 0, "subrate factor unexpectedly set to %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);
	release_ntf(ntf);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Subrate Update procedure - peer unsupported.
 *
 * The central does not support subrating and answers with LL_UNKNOWN_RSP; the
 * feature is unmasked for the connection and the host is notified.
 */
ZTEST(subrate_loc, test_subrate_peripheral_loc_unknown_rsp)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_subrate_req exp_req = {
		.subrate_factor_min = 2,
		.subrate_factor_max = 8,
		.max_latency = 20,
		.continuation_number = 2,
		.timeout = 1000,
	};

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
	};

	struct pdu_data_llctrl_unknown_rsp exp_ntf = {
		.type = PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ,
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* The feature is enabled by the test setup */
	zassert_true(feature_subrating(&conn), "subrating feature not enabled");

	/* Initiate a Connection Subrate Update Procedure */
	err = ull_cp_subrate(&conn, 2, 8, 20, 2, 1000);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_SUBRATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown_rsp);

	/* Done */
	event_done(&conn);

	/* The host is notified that the peer does not support the feature */
	ut_rx_pdu(LL_UNKNOWN_RSP, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* The feature must now be unmasked for this connection */
	zassert_false(feature_subrating(&conn), "subrating feature still enabled");

	/* No subrate parameters applied */
	zassert_equal(conn.lll.subrate.factor, 0, "subrate factor unexpectedly set to %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);
	release_ntf(ntf);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST_SUITE(subrate_loc, NULL, NULL, subrate_setup, NULL, NULL);
ZTEST_SUITE(subrate_rem, NULL, NULL, subrate_setup, NULL, NULL);
