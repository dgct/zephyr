/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the page-1 (extended) feature exchange LL control procedure
 * (LL_FEATURE_EXT_REQ / LL_FEATURE_EXT_RSP, opcodes 0x2B / 0x2C), driven via
 * ull_cp_feature_ext(). Page-0 behaviour is intentionally untouched and is
 * covered by ctrl_feature_exchange.
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
#include "ll_settings.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"
#include "lll_conn_iso.h"

#include "ull_tx_queue.h"

#include "isoal.h"
#include "ull_iso_types.h"
#include "ull_conn_iso_types.h"
#include "ull_conn_types.h"

#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_internal.h"
#include "ull_conn_internal.h"

#include "ll_feat.h"

#include "helper_pdu.h"
#include "helper_util.h"
#include "helper_features.h"

static struct ll_conn conn;

static void fex_ext_setup(void *data)
{
	test_setup(&conn);
}

/* Peer page-1 feature sets exercised by the parametrised tests. The all-ones
 * entry verifies that the receiver masks the incoming bits down to the bits it
 * actually understands (LL_FEAT_PAGE1_BIT_MASK_VALID).
 */
static const uint64_t peer_page1_sets[] = {
	LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE),
	LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE) |
		LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS),
	0xFFFFFFFFFFFFFFFFULL,
	0x0ULL,
};

static void encode_feature_ext(void *pdu, uint64_t features)
{
	struct pdu_data_llctrl_feature_ext_req *p = pdu;

	p->max_page = 1U;
	p->page_number = 1U;
	sys_put_le64(features, p->features);
}

/*
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |                   |
 *    | Start                      |                   |
 *    | Ext Feature Exchange Proc. |                   |
 *    |--------------------------->|                   |
 *    |                            | LL_FEATURE_EXT_REQ|
 *    |                            |------------------>|
 *    |                            |LL_FEATURE_EXT_RSP |
 *    |                            |<------------------|
 *    |  Ext Feature Exchange Proc.|                   |
 *    |                   Complete |                   |
 *    |<---------------------------|                   |
 */
static void run_loc(uint8_t role)
{
	uint64_t err;
	uint64_t local_page1 = ll_feat_get_page1();
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_feature_ext_req exp_req;
	struct pdu_data_llctrl_feature_ext_rsp remote_rsp;
	struct pdu_data_llctrl_feature_ext_rsp exp_ntf;

	test_set_role(&conn, role);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	for (int i = 0; i < ARRAY_SIZE(peer_page1_sets); i++) {
		uint64_t peer = peer_page1_sets[i];
		uint64_t exp_stored = peer & LL_FEAT_PAGE1_BIT_MASK_VALID;
		uint64_t exp_used = local_page1 & exp_stored;

		encode_feature_ext(&exp_req, local_page1);
		encode_feature_ext(&remote_rsp, peer);
		encode_feature_ext(&exp_ntf, exp_stored);

		/* Initiate an Extended Feature Exchange Procedure */
		err = ull_cp_feature_ext(&conn, 1U);
		zassert_equal(err, BT_HCI_ERR_SUCCESS);

		event_prepare(&conn);
		/* Tx Queue should have one LL_FEATURE_EXT_REQ */
		lt_rx(LL_FEATURE_EXT_REQ, &conn, &tx, &exp_req);
		lt_rx_q_is_empty(&conn);

		/* Rx LL_FEATURE_EXT_RSP */
		lt_tx(LL_FEATURE_EXT_RSP, &conn, &remote_rsp);

		event_done(&conn);

		/* There should be one host notification carrying the peer's
		 * page-1 features (masked to the bits we understand).
		 */
		ut_rx_pdu(LL_FEATURE_EXT_RSP, &ntf, &exp_ntf);
		ut_rx_q_is_empty();

		zassert_equal(conn.llcp.fex.valid_page1, 1U, "page-1 not marked valid");
		zassert_equal(conn.llcp.fex.features_peer_page1, exp_stored,
			      "peer page-1 0x%llx != 0x%llx",
			      conn.llcp.fex.features_peer_page1, exp_stored);
		zassert_equal(conn.llcp.fex.features_used_page1, exp_used,
			      "used page-1 0x%llx != 0x%llx",
			      conn.llcp.fex.features_used_page1, exp_used);

		ull_cp_release_tx(&conn, tx);
		release_ntf(ntf);
	}

	zassert_equal(conn.lll.event_counter, ARRAY_SIZE(peer_page1_sets),
		      "Wrong event-count %d\n", conn.lll.event_counter);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(fex_ext_central, test_feat_ext_central_loc)
{
	run_loc(BT_HCI_ROLE_CENTRAL);
}

ZTEST(fex_ext_periph, test_feat_ext_periph_loc)
{
	run_loc(BT_HCI_ROLE_PERIPHERAL);
}

/* Local-initiated, with a host-set page-1 feature bit (Shorter Connection
 * Intervals Host Support) so that ll_feat_get_page1() reports it in the
 * outgoing LL_FEATURE_EXT_REQ.
 */
ZTEST(fex_ext_central, test_feat_ext_central_loc_host_feature)
{
	uint64_t err;
	uint64_t peer = LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE) |
			LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS);
	uint64_t local_page1;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;
	struct pdu_data_llctrl_feature_ext_req exp_req;
	struct pdu_data_llctrl_feature_ext_rsp remote_rsp;
	struct pdu_data_llctrl_feature_ext_rsp exp_ntf;

	/* Host enables Shorter Connection Intervals support */
	ll_set_host_feature(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP, 1);
	local_page1 = ll_feat_get_page1();

	/* The host-support bit must now be advertised */
	zassert_true(local_page1 &
			     LL_FEAT_PAGE1_BIT(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP),
		     "host page-1 bit not advertised, page1=0x%llx", local_page1);

	encode_feature_ext(&exp_req, local_page1);
	encode_feature_ext(&remote_rsp, peer);
	encode_feature_ext(&exp_ntf, peer & LL_FEAT_PAGE1_BIT_MASK_VALID);

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	err = ull_cp_feature_ext(&conn, 1U);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	event_prepare(&conn);
	lt_rx(LL_FEATURE_EXT_REQ, &conn, &tx, &exp_req);
	lt_rx_q_is_empty(&conn);
	lt_tx(LL_FEATURE_EXT_RSP, &conn, &remote_rsp);
	event_done(&conn);

	ut_rx_pdu(LL_FEATURE_EXT_RSP, &ntf, &exp_ntf);
	ut_rx_q_is_empty();

	ull_cp_release_tx(&conn, tx);
	release_ntf(ntf);

	/* Restore host feature state */
	ll_set_host_feature(BT_LE_FEAT_BIT_SHORTER_CONN_INTERVALS_HOST_SUPP, 0);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * +-----+                     +-------+            +-----+
 * | UT  |                     | LL_A  |            | LT  |
 * +-----+                     +-------+            +-----+
 *    |                            |LL_FEATURE_EXT_REQ |
 *    |                            |<------------------|
 *    |                            |LL_FEATURE_EXT_RSP |
 *    |                            |------------------>|
 */
static void run_rem(uint8_t role)
{
	uint64_t local_page1 = ll_feat_get_page1();
	struct node_tx *tx;
	struct pdu_data_llctrl_feature_ext_req remote_req;
	struct pdu_data_llctrl_feature_ext_rsp exp_rsp;

	test_set_role(&conn, role);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	for (int i = 0; i < ARRAY_SIZE(peer_page1_sets); i++) {
		uint64_t peer = peer_page1_sets[i];
		uint64_t exp_stored = peer & LL_FEAT_PAGE1_BIT_MASK_VALID;
		uint64_t exp_used = local_page1 & exp_stored;

		encode_feature_ext(&remote_req, peer);
		encode_feature_ext(&exp_rsp, exp_used);

		event_prepare(&conn);
		/* Rx LL_FEATURE_EXT_REQ */
		lt_tx(LL_FEATURE_EXT_REQ, &conn, &remote_req);
		event_done(&conn);

		event_prepare(&conn);
		/* Tx Queue should have one LL_FEATURE_EXT_RSP carrying the
		 * features we will actually use (local AND peer).
		 */
		lt_rx(LL_FEATURE_EXT_RSP, &conn, &tx, &exp_rsp);
		lt_rx_q_is_empty(&conn);
		event_done(&conn);

		/* Responder does not notify the host */
		ut_rx_q_is_empty();

		zassert_equal(conn.llcp.fex.valid_page1, 1U, "page-1 not marked valid");
		zassert_equal(conn.llcp.fex.features_peer_page1, exp_stored,
			      "peer page-1 0x%llx != 0x%llx",
			      conn.llcp.fex.features_peer_page1, exp_stored);
		zassert_equal(conn.llcp.fex.features_used_page1, exp_used,
			      "used page-1 0x%llx != 0x%llx",
			      conn.llcp.fex.features_used_page1, exp_used);

		ull_cp_release_tx(&conn, tx);
	}

	zassert_equal(conn.lll.event_counter, 2 * ARRAY_SIZE(peer_page1_sets),
		      "Wrong event-count %d\n", conn.lll.event_counter);
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

ZTEST(fex_ext_central, test_feat_ext_central_rem)
{
	run_rem(BT_HCI_ROLE_CENTRAL);
}

ZTEST(fex_ext_periph, test_feat_ext_periph_rem)
{
	run_rem(BT_HCI_ROLE_PERIPHERAL);
}

ZTEST_SUITE(fex_ext_central, NULL, NULL, fex_ext_setup, NULL, NULL);
ZTEST_SUITE(fex_ext_periph, NULL, NULL, fex_ext_setup, NULL, NULL);
