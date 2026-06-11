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

/* True once the current connection event counter has reached the procedure
 * instant (wrap-safe comparison, mirrors ctrl_conn_update).
 */
static bool is_instant_reached(struct ll_conn *llconn, uint16_t instant)
{
	return ((event_counter(llconn) - instant) & 0xFFFF) <= 0x7FFF;
}

static void conn_rate_setup(void *data)
{
	test_setup(&conn);

	/* Emulate a completed feature exchange in which the peer supports
	 * Shorter Connection Intervals, so that locally-initiated procedures
	 * are permitted (ull_cp_conn_rate() is gated on
	 * feature_shorter_conn_intervals()).
	 */
	conn.llcp.fex.features_used_page1 |= LL_FEAT_P1_BIT_SHORTER_CI;
	conn.llcp.fex.valid = 1U;
}

static void conn_rate_central_setup(void *data)
{
	conn_rate_setup(data);

	/* Permissive baseline so the Central accepts the Peripheral's request
	 * in the remotely-initiated test (5.1.33 negotiation).
	 */
	ull_cp_set_default_rate_params(6U, 32000U, 1U, 500U, 499U, 0U, 3200U);
}

/*
 * Remotely-initiated Connection Rate procedure - Peripheral, valid IND.
 *
 * The Central unsolicitedly sends LL_CONNECTION_RATE_IND. The Peripheral
 * validates the parameters, defers them until the procedure instant, applies
 * them and notifies the host (BT 6.2, Vol 6, Part B, 5.1.33).
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |   LL_CONNECTION_RATE_IND  |
 *    |                           |<--------------------------|
 *    |                           |                           |
 *    ~~~~~~~~~~~~~~~~~ instant ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |       Connection Rate     |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 */
ZTEST(conn_rate_rem, test_conn_rate_peripheral_rem)
{
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct pdu_data_llctrl_conn_rate_ind remote_ind = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	struct pdu_data_llctrl_conn_rate_ind exp_ntf = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
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
	remote_ind.instant = event_counter(&conn) + 6U;
	instant = remote_ind.instant;
	exp_ntf.instant = instant;
	lt_tx(LL_CONNECTION_RATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* Nothing should be applied or notified before the instant */
	while (!is_instant_reached(&conn, instant)) {
		event_prepare(&conn);
		lt_rx_q_is_empty(&conn);
		event_done(&conn);
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);

	/* There should be one host notification carrying the applied params */
	ut_rx_pdu(LL_CONNECTION_RATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* The subrate cadence should have been applied at the instant */
	zassert_equal(conn.lll.subrate.factor, 4U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely-initiated Connection Rate procedure - Peripheral, invalid IND.
 *
 * The Central sends LL_CONNECTION_RATE_IND with parameters that fail the
 * validity check; the Peripheral declines with LL_UNKNOWN_RSP and does not
 * notify the host or apply anything.
 */
ZTEST(conn_rate_rem, test_conn_rate_peripheral_rem_invalid)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_conn_rate_ind remote_ind = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 200U,
		.latency = 10U,
		.continuation_number = 1U,
		/* timeout * 40 (= 400) must exceed factor*(latency+1)*interval
		 * (= 132000); it does not, so the IND is invalid.
		 */
		.timeout = 10U,
	};

	struct pdu_data_llctrl_unknown_rsp exp_unknown = {
		.type = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);

	/* Rx */
	remote_ind.instant = event_counter(&conn) + 6U;
	lt_tx(LL_CONNECTION_RATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* The Peripheral declines with LL_UNKNOWN_RSP */
	lt_rx(LL_UNKNOWN_RSP, &conn, &tx, &exp_unknown);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* There should NOT be a host notification */
	ut_rx_q_is_empty();

	/* Nothing applied */
	zassert_equal(conn.lll.subrate.factor, 0U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Rate procedure - Peripheral, accepted.
 *
 * The Peripheral host requests a rate change; the Peripheral sends
 * LL_CONNECTION_RATE_REQ and the Central answers with LL_CONNECTION_RATE_IND.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_P  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    | LE Set Connection Rate    |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_RATE_REQ    |
 *    |                           |-------------------------->|
 *    |                           |   LL_CONNECTION_RATE_IND  |
 *    |                           |<--------------------------|
 *    ~~~~~~~~~~~~~~~~~ instant ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |       Connection Rate     |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 */
ZTEST(conn_rate_loc, test_conn_rate_peripheral_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	uint16_t instant;

	struct pdu_data_llctrl_conn_rate_req exp_req = {
		.interval_min = 6U,
		.interval_max = 60U,
		.subrate_factor_min = 1U,
		.subrate_factor_max = 4U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
		.preferred_periodicity = 0U,
		.reference_conn_event_count = 0U,
		.offset0 = 0x0000U,
		.offset1 = 0xffffU,
		.offset2 = 0xffffU,
		.offset3 = 0xffffU,
	};

	struct pdu_data_llctrl_conn_rate_ind remote_ind = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	struct pdu_data_llctrl_conn_rate_ind exp_ntf = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Rate Procedure */
	err = ull_cp_conn_rate(&conn, 6U, 60U, 1U, 4U, 0U, 0U, 1000U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL_CONNECTION_RATE_REQ */
	lt_rx(LL_CONNECTION_RATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx - the Central accepts with the chosen IND */
	remote_ind.instant = event_counter(&conn) + 6U;
	instant = remote_ind.instant;
	exp_ntf.instant = instant;
	lt_tx(LL_CONNECTION_RATE_IND, &conn, &remote_ind);

	/* Done */
	event_done(&conn);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Nothing applied or notified before the instant */
	while (!is_instant_reached(&conn, instant)) {
		event_prepare(&conn);
		lt_rx_q_is_empty(&conn);
		event_done(&conn);
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);

	/* There should be one host notification carrying the applied params */
	ut_rx_pdu(LL_CONNECTION_RATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.lll.subrate.factor, 4U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Rate procedure - Peripheral, rejected.
 *
 * The Central rejects the request with LL_REJECT_EXT_IND; the host is notified
 * with the reject error code.
 */
ZTEST(conn_rate_loc, test_conn_rate_peripheral_loc_reject)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_conn_rate_req exp_req = {
		.interval_min = 6U,
		.interval_max = 60U,
		.subrate_factor_min = 1U,
		.subrate_factor_max = 4U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
		.preferred_periodicity = 0U,
		.reference_conn_event_count = 0U,
		.offset0 = 0x0000U,
		.offset1 = 0xffffU,
		.offset2 = 0xffffU,
		.offset3 = 0xffffU,
	};

	struct pdu_data_llctrl_reject_ext_ind reject_ext_ind = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	struct pdu_data_llctrl_reject_ext_ind exp_ntf = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ,
		.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Rate Procedure */
	err = ull_cp_conn_rate(&conn, 6U, 60U, 1U, 4U, 0U, 0U, 1000U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL_CONNECTION_RATE_REQ */
	lt_rx(LL_CONNECTION_RATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx - the Central rejects */
	lt_tx(LL_REJECT_EXT_IND, &conn, &reject_ext_ind);

	/* Done */
	event_done(&conn);

	/* There should be one host notification of the rejection */
	ut_rx_pdu(LL_REJECT_EXT_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* Nothing applied */
	zassert_equal(conn.lll.subrate.factor, 0U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Rate procedure - Peripheral, unsupported.
 *
 * The Central answers LL_UNKNOWN_RSP; the Peripheral unmasks the feature bit
 * and notifies the host.
 */
ZTEST(conn_rate_loc, test_conn_rate_peripheral_loc_unknown_rsp)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_conn_rate_req exp_req = {
		.interval_min = 6U,
		.interval_max = 60U,
		.subrate_factor_min = 1U,
		.subrate_factor_max = 4U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
		.preferred_periodicity = 0U,
		.reference_conn_event_count = 0U,
		.offset0 = 0x0000U,
		.offset1 = 0xffffU,
		.offset2 = 0xffffU,
		.offset3 = 0xffffU,
	};

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ,
	};

	struct pdu_data_llctrl_unknown_rsp exp_ntf = {
		.type = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Feature is enabled by setup */
	zassert_true(feature_shorter_conn_intervals(&conn),
		     "SCI feature should be enabled");

	/* Initiate a Connection Rate Procedure */
	err = ull_cp_conn_rate(&conn, 6U, 60U, 1U, 4U, 0U, 0U, 1000U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL_CONNECTION_RATE_REQ */
	lt_rx(LL_CONNECTION_RATE_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);

	/* Rx - the Central does not understand the procedure */
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_UNKNOWN_RSP, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	/* The feature bit should have been unmasked */
	zassert_false(feature_shorter_conn_intervals(&conn),
		      "SCI feature should have been unmasked");

	/* Nothing applied */
	zassert_equal(conn.lll.subrate.factor, 0U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally-initiated Connection Rate procedure - Central origination (5.1.32).
 *
 * The Central host requests a rate change; the Central sends
 * LL_CONNECTION_RATE_IND directly and applies it at the instant.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_C  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    | LE Set Connection Rate    |                           |
 *    |-------------------------->|                           |
 *    |                           | LL_CONNECTION_RATE_IND    |
 *    |                           |-------------------------->|
 *    ~~~~~~~~~~~~~~~~~ instant ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |       Connection Rate     |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 */
ZTEST(conn_rate_central, test_conn_rate_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct pdu_data_llctrl_conn_rate_ind exp_ind = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	struct pdu_data_llctrl_conn_rate_ind exp_ntf = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Initiate a Connection Rate Procedure */
	err = ull_cp_conn_rate(&conn, 6U, 60U, 1U, 4U, 0U, 0U, 1000U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* The Central transmits LL_CONNECTION_RATE_IND directly */
	exp_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_RATE_IND, &conn, &tx, &exp_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save the instant the IUT chose */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_rate_ind.instant);
	exp_ntf.instant = instant;

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Nothing applied or notified before the instant */
	while (!is_instant_reached(&conn, instant)) {
		event_prepare(&conn);
		lt_rx_q_is_empty(&conn);
		event_done(&conn);
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_RATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.lll.subrate.factor, 4U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely-initiated Connection Rate procedure - Central responder (5.1.33).
 *
 * The Peripheral sends LL_CONNECTION_RATE_REQ; the Central negotiates and
 * answers with LL_CONNECTION_RATE_IND, applying it at the instant.
 *
 * +-----+                    +-------+                    +-----+
 * | UT  |                    | LL_C  |                    | LT  |
 * +-----+                    +-------+                    +-----+
 *    |                           |    LL_CONNECTION_RATE_REQ |
 *    |                           |<--------------------------|
 *    |                           | LL_CONNECTION_RATE_IND    |
 *    |                           |-------------------------->|
 *    ~~~~~~~~~~~~~~~~~ instant ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *    |       Connection Rate     |                           |
 *    |                  Complete |                           |
 *    |<--------------------------|                           |
 */
ZTEST(conn_rate_central, test_conn_rate_central_rem)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint16_t instant;

	struct pdu_data_llctrl_conn_rate_req remote_req = {
		.interval_min = 6U,
		.interval_max = 60U,
		.subrate_factor_min = 1U,
		.subrate_factor_max = 4U,
		.max_latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
		.preferred_periodicity = 0U,
		.reference_conn_event_count = 0U,
		.offset0 = 0x0000U,
		.offset1 = 0xffffU,
		.offset2 = 0xffffU,
		.offset3 = 0xffffU,
	};

	struct pdu_data_llctrl_conn_rate_ind exp_ind = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	struct pdu_data_llctrl_conn_rate_ind exp_ntf = {
		.win_offset = 0U,
		.interval = 60U,
		.subrate_factor = 4U,
		.latency = 0U,
		.continuation_number = 0U,
		.timeout = 1000U,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);

	/* Rx - the Peripheral requests a rate change */
	lt_tx(LL_CONNECTION_RATE_REQ, &conn, &remote_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* The Central answers with LL_CONNECTION_RATE_IND */
	exp_ind.instant = event_counter(&conn) + 6U;
	lt_rx(LL_CONNECTION_RATE_IND, &conn, &tx, &exp_ind);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* Save the instant the IUT chose */
	pdu = (struct pdu_data *)tx->pdu;
	instant = sys_le16_to_cpu(pdu->llctrl.conn_rate_ind.instant);
	exp_ntf.instant = instant;

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	/* Nothing applied or notified before the instant */
	while (!is_instant_reached(&conn, instant)) {
		event_prepare(&conn);
		lt_rx_q_is_empty(&conn);
		event_done(&conn);
		ut_rx_q_is_empty();
	}

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_CONNECTION_RATE_IND, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	zassert_equal(conn.lll.subrate.factor, 4U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Ntf */
	release_ntf(ntf);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely-initiated Connection Rate procedure - Central, rejected.
 *
 * The Peripheral sends LL_CONNECTION_RATE_REQ with parameters the Central
 * cannot honour; the Central replies LL_REJECT_EXT_IND and does not notify
 * the host.
 */
ZTEST(conn_rate_central, test_conn_rate_central_rem_reject)
{
	struct node_tx *tx;

	struct pdu_data_llctrl_conn_rate_req remote_req = {
		.interval_min = 6U,
		.interval_max = 60U,
		.subrate_factor_min = 1U,
		.subrate_factor_max = 4U,
		.max_latency = 0U,
		/* continuation_number (4) must be < subrate_factor_max (4);
		 * it is not, so the request is invalid.
		 */
		.continuation_number = 4U,
		.timeout = 1000U,
		.preferred_periodicity = 0U,
		.reference_conn_event_count = 0U,
		.offset0 = 0x0000U,
		.offset1 = 0xffffU,
		.offset2 = 0xffffU,
		.offset3 = 0xffffU,
	};

	struct pdu_data_llctrl_reject_ext_ind exp_reject = {
		.reject_opcode = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ,
		.error_code = BT_HCI_ERR_INVALID_LL_PARAM,
	};

	/* Role */
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Prepare */
	event_prepare(&conn);
	lt_rx_q_is_empty(&conn);

	/* Rx - the Peripheral requests an invalid rate change */
	lt_tx(LL_CONNECTION_RATE_REQ, &conn, &remote_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* The Central rejects */
	lt_rx(LL_REJECT_EXT_IND, &conn, &tx, &exp_reject);
	lt_rx_q_is_empty(&conn);

	/* Done */
	event_done(&conn);

	/* A peer-initiated, rejected procedure yields no host notification */
	ut_rx_q_is_empty();

	/* Nothing applied */
	zassert_equal(conn.lll.subrate.factor, 0U, "subrate factor %u",
		      conn.lll.subrate.factor);

	/* Release Tx */
	ull_cp_release_tx(&conn, tx);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST_SUITE(conn_rate_rem, NULL, NULL, conn_rate_setup, NULL, NULL);
ZTEST_SUITE(conn_rate_loc, NULL, NULL, conn_rate_setup, NULL, NULL);
ZTEST_SUITE(conn_rate_central, NULL, NULL, conn_rate_central_setup, NULL, NULL);
