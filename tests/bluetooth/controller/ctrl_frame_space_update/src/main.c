/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
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

static void fsu_setup(void *data)
{
	test_setup(&conn);

	/* Emulate initial connection state with FSU defaults */
	ull_fsu_init(&conn);

	/* Init DLE data to have some realistic timing values */
	ull_conn_default_tx_octets_set(251);
	ull_conn_default_tx_time_set(2120);
	ull_dle_init(&conn, PHY_1M);

	/* Enable FSU feature in feature exchange (page-1 extended features) */
	conn.llcp.fex.features_used_page1 = LL_FEAT_P1_BIT_FRAME_SPACE;
	conn.llcp.fex.valid = 1;
}

/*
 * Locally triggered Frame Space Update procedure - Central Role
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Start                      |                              |
 *    | Frame Space Update Proc.   |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_REQ           |
 *    |                            | (fsu_min, fsu_max, phys,     |
 *    |                            |  spacing_type)               |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_RSP      |
 *    |                            |      (fsu, phys,             |
 *    |                            |       spacing_type)          |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    | Frame Space Update         |                              |
 *    | Notification               |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_central_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	/* Test parameters */
	uint16_t fsu_min = 200;  /* 200 us minimum frame spacing */
	uint16_t fsu_max = 300;  /* 300 us maximum frame spacing */
	uint8_t phys = PHY_1M;   /* Apply to 1M PHY */
	uint16_t spacing_type = T_IFS_ACL_CP;  /* ACL Central-to-Peripheral spacing */

	struct pdu_data_llctrl_fsu_req local_fsu_req = {
		.fsu_min = fsu_min,
		.fsu_max = fsu_max,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp remote_fsu_rsp = {
		.fsu = 250,  /* Remote negotiates to 250 us */
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 250,
		.phys = phys,
		.spacing_type = spacing_type
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Initiate a Frame Space Update Procedure */
	err = ull_cp_fsu(&conn, fsu_min, fsu_max, phys, spacing_type);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_REQ, &conn, &tx, &local_fsu_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_RSP, &conn, &remote_fsu_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify frame spacing was updated for the Central role (RX direction for ACL CP) */
	zassert_equal(conn.lll.tifs_rx_us, 250,
		      "Frame spacing RX not updated, expected 250 got %u",
		      conn.lll.tifs_rx_us);

	/* Check context buffers */
	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally triggered Frame Space Update procedure with Unknown Response - Central Role
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Start                      |                              |
 *    | Frame Space Update Proc.   |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_REQ           |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    |                            |         LL_UNKNOWN_RSP       |
 *    |                            |      (type=FRAME_SPACE_REQ)  |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *  ~~~~~~~~~~~~~~~~~~~~~~~  Unmask FSU support ~~~~~~~~~~~~~~~~~~~~
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_central_loc_unknown_rsp)
{
	uint8_t err;
	struct node_tx *tx;

	uint16_t fsu_min = 200;
	uint16_t fsu_max = 300;
	uint8_t phys = PHY_1M;
	uint16_t spacing_type = T_IFS_ACL_CP;

	struct pdu_data_llctrl_fsu_req local_fsu_req = {
		.fsu_min = fsu_min,
		.fsu_max = fsu_max,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_unknown_rsp unknown_rsp = {
		.type = PDU_DATA_LLCTRL_TYPE_FRAME_SPACE_REQ
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Initiate a Frame Space Update Procedure */
	err = ull_cp_fsu(&conn, fsu_min, fsu_max, phys, spacing_type);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_REQ, &conn, &tx, &local_fsu_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Rx */
	lt_tx(LL_UNKNOWN_RSP, &conn, &unknown_rsp);

	/* Done */
	event_done(&conn);

	/* There should be no host notification */
	ut_rx_q_is_empty();

	/* Frame spacing should not be updated */
	zassert_equal(conn.lll.tifs_rx_us, EVENT_IFS_US,
		      "Frame spacing should not change on unknown response");

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Locally triggered Frame Space Update procedure - Peripheral Role
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Start                      |                              |
 *    | Frame Space Update Proc.   |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_REQ           |
 *    |                            | (fsu_min, fsu_max, phys,     |
 *    |                            |  spacing_type=T_IFS_ACL_PC)  |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_RSP      |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    | Frame Space Update         |                              |
 *    | Notification               |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_peripheral, test_frame_space_update_peripheral_loc)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	uint16_t fsu_min = 180;
	uint16_t fsu_max = 280;
	uint8_t phys = PHY_2M;  /* Apply to 2M PHY */
	uint16_t spacing_type = T_IFS_ACL_PC;  /* ACL Peripheral-to-Central spacing */

	struct pdu_data_llctrl_fsu_req local_fsu_req = {
		.fsu_min = fsu_min,
		.fsu_max = fsu_max,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp remote_fsu_rsp = {
		.fsu = 220,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 220,
		.phys = phys,
		.spacing_type = spacing_type
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 2M */
	conn.lll.phy_tx = PHY_2M;
	conn.lll.phy_rx = PHY_2M;

	/* Initiate a Frame Space Update Procedure */
	err = ull_cp_fsu(&conn, fsu_min, fsu_max, phys, spacing_type);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_REQ, &conn, &tx, &local_fsu_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_RSP, &conn, &remote_fsu_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify frame spacing was updated for the Peripheral role (RX direction for ACL PC) */
	zassert_equal(conn.lll.tifs_rx_us, 220,
		      "Frame spacing RX not updated, expected 220 got %u",
		      conn.lll.tifs_rx_us);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely triggered Frame Space Update procedure - Central Role
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_REQ      |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_RSP           |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    | Frame Space Update         |                              |
 *    | Notification               |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_central_rem)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_fsu_req remote_fsu_req = {
		.fsu_min = 200,
		.fsu_max = 250,
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_PC  /* PC spacing, so Central updates TX */
	};

	struct pdu_data_llctrl_fsu_rsp local_fsu_rsp = {
		.fsu = 200,  /* Responder picks fastest supported = MAX(fsu_min, tIFS floor) */
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_PC
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 200,
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_PC
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_REQ, &conn, &remote_fsu_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_RSP, &conn, &tx, &local_fsu_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify frame spacing was updated for Central (TX direction for ACL PC) */
	zassert_equal(conn.lll.tifs_tx_us, 200,
		      "Frame spacing TX not updated, expected 200 got %u",
		      conn.lll.tifs_tx_us);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Remotely triggered Frame Space Update procedure - Peripheral Role
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_REQ      |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_RSP           |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    | Frame Space Update         |                              |
 *    | Notification               |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_peripheral, test_frame_space_update_peripheral_rem)
{
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	struct pdu_data_llctrl_fsu_req remote_fsu_req = {
		.fsu_min = 160,
		.fsu_max = 260,
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_CP  /* CP spacing, so Peripheral updates TX */
	};

	struct pdu_data_llctrl_fsu_rsp local_fsu_rsp = {
		.fsu = 160,
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_CP
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 160,
		.phys = PHY_1M,
		.spacing_type = T_IFS_ACL_CP
	};

	test_set_role(&conn, BT_HCI_ROLE_PERIPHERAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Prepare */
	event_prepare(&conn);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_REQ, &conn, &remote_fsu_req);

	/* Done */
	event_done(&conn);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_RSP, &conn, &tx, &local_fsu_rsp);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify frame spacing was updated for Peripheral (TX direction for ACL CP) */
	zassert_equal(conn.lll.tifs_tx_us, 160,
		      "Frame spacing TX not updated, expected 160 got %u",
		      conn.lll.tifs_tx_us);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Frame Space Update with CIS spacing type
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Start FSU with CIS         |                              |
 *    | spacing_type               |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_REQ           |
 *    |                            | (spacing_type=T_IFS_CIS)     |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_RSP      |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    | FSU Notification           |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_cis_spacing)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	uint16_t fsu_min = 190;
	uint16_t fsu_max = 290;
	uint8_t phys = PHY_CODED;
	uint16_t spacing_type = T_IFS_CIS;  /* CIS timing */

	struct pdu_data_llctrl_fsu_req local_fsu_req = {
		.fsu_min = fsu_min,
		.fsu_max = fsu_max,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp remote_fsu_rsp = {
		.fsu = 240,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 240,
		.phys = phys,
		.spacing_type = spacing_type
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to CODED */
	conn.lll.phy_tx = PHY_CODED;
	conn.lll.phy_rx = PHY_CODED;

	/* Initiate a Frame Space Update Procedure with CIS spacing */
	err = ull_cp_fsu(&conn, fsu_min, fsu_max, phys, spacing_type);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_REQ, &conn, &tx, &local_fsu_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_RSP, &conn, &remote_fsu_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify CIS frame spacing was updated */
	zassert_equal(conn.lll.tifs_cis_us, 240,
		      "CIS frame spacing not updated, expected 240 got %u",
		      conn.lll.tifs_cis_us);

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Frame Space Update with multiple PHYs
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Start FSU with all PHYs    |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    |                            | LL_FRAME_SPACE_REQ           |
 *    |                            | (phys=1M|2M|CODED)           |
 *    |                            |----------------------------->|
 *    |                            |                              |
 *    |                            |      LL_FRAME_SPACE_RSP      |
 *    |                            |<-----------------------------|
 *    |                            |                              |
 *    | FSU Notification           |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_multi_phy)
{
	uint8_t err;
	struct node_tx *tx;
	struct node_rx_pdu *ntf;

	uint16_t fsu_min = 170;
	uint16_t fsu_max = 270;
	uint8_t phys = PHY_1M | PHY_2M | PHY_CODED;  /* All PHYs */
	uint16_t spacing_type = T_IFS_ACL_CP;

	struct pdu_data_llctrl_fsu_req local_fsu_req = {
		.fsu_min = fsu_min,
		.fsu_max = fsu_max,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp remote_fsu_rsp = {
		.fsu = 220,
		.phys = phys,
		.spacing_type = spacing_type
	};

	struct pdu_data_llctrl_fsu_rsp fsu_ntf = {
		.fsu = 220,
		.phys = phys,
		.spacing_type = spacing_type
	};

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);

	/* Connect */
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Initiate a Frame Space Update Procedure for all PHYs */
	err = ull_cp_fsu(&conn, fsu_min, fsu_max, phys, spacing_type);
	zassert_equal(err, BT_HCI_ERR_SUCCESS);

	/* Prepare */
	event_prepare(&conn);

	/* Tx Queue should have one LL Control PDU */
	lt_rx(LL_FRAME_SPACE_REQ, &conn, &tx, &local_fsu_req);
	lt_rx_q_is_empty(&conn);

	/* TX Ack */
	event_tx_ack(&conn, tx);

	/* Rx */
	lt_tx(LL_FRAME_SPACE_RSP, &conn, &remote_fsu_rsp);

	/* Done */
	event_done(&conn);

	/* There should be one host notification */
	ut_rx_pdu(LL_FRAME_SPACE_RSP, &ntf, &fsu_ntf);
	ut_rx_q_is_empty();

	/* Verify per-PHY storage was updated for all PHYs */
	for (int i = 0; i < 3; i++) {
		if (phys & BIT(i)) {
			zassert_equal(conn.lll.fsu.perphy[i].fsu_min, 220,
				      "Per-PHY[%d] fsu_min not updated", i);
			zassert_equal(conn.lll.fsu.perphy[i].fsu_max, 220,
				      "Per-PHY[%d] fsu_max not updated", i);
		}
	}

	zassert_equal(llcp_ctx_buffers_free(), test_ctx_buffers_cnt(),
		      "Free CTX buffers %d", llcp_ctx_buffers_free());
}

/*
 * Frame Space Update initialization test
 */
ZTEST(fsu_central, test_frame_space_update_init)
{
	/* Verify initial FSU values are set correctly */
	zassert_equal(conn.lll.tifs_rx_us, EVENT_IFS_US,
		      "Initial tifs_rx_us incorrect");
	zassert_equal(conn.lll.tifs_tx_us, EVENT_IFS_US,
		      "Initial tifs_tx_us incorrect");
	zassert_equal(conn.lll.tifs_cis_us, EVENT_IFS_US,
		      "Initial tifs_cis_us incorrect");
	zassert_equal(conn.lll.fsu.local.fsu_min, CONFIG_BT_CTLR_EVENT_IFS_LOW_LAT_US,
		      "Initial local fsu_min incorrect");
	zassert_equal(conn.lll.fsu.local.fsu_max, EVENT_IFS_MAX_US,
		      "Initial local fsu_max incorrect");
	zassert_equal(conn.lll.fsu.eff.fsu_min, EVENT_IFS_US,
		      "Initial effective fsu_min incorrect");
	zassert_equal(conn.lll.fsu.eff.fsu_max, EVENT_IFS_US,
		      "Initial effective fsu_max incorrect");

	/* Verify per-PHY storage is initialized */
	for (int i = 0; i < 3; i++) {
		zassert_equal(conn.lll.fsu.perphy[i].fsu_min, EVENT_IFS_US,
			      "Per-PHY[%d] fsu_min not initialized", i);
		zassert_equal(conn.lll.fsu.perphy[i].fsu_max, EVENT_IFS_US,
			      "Per-PHY[%d] fsu_max not initialized", i);
		zassert_equal(conn.lll.fsu.perphy[i].phys, PHY_1M | PHY_2M | PHY_CODED,
			      "Per-PHY[%d] phys not initialized", i);
		zassert_equal(conn.lll.fsu.perphy[i].spacing_type,
			      T_IFS_ACL_PC | T_IFS_ACL_CP | T_IFS_CIS,
			      "Per-PHY[%d] spacing_type not initialized", i);
	}
}

/*
 * Frame Space Update effective value calculation test
 */
ZTEST(fsu_central, test_frame_space_update_eff_value)
{
	uint16_t fsu_min_below_config = CONFIG_BT_CTLR_EVENT_IFS_LOW_LAT_US - 10;
	uint16_t fsu_max_below_config = CONFIG_BT_CTLR_EVENT_IFS_LOW_LAT_US - 5;

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Set FSU values below CONFIG minimum */
	conn.lll.fsu.local.fsu_min = fsu_min_below_config;
	conn.lll.fsu.local.fsu_max = fsu_max_below_config;

	/* Update effective values */
	ull_fsu_update_eff_from_local(&conn);

	/* Verify that effective values are at least CONFIG minimum */
	zassert_equal(conn.lll.fsu.eff.fsu_min, CONFIG_BT_CTLR_EVENT_IFS_LOW_LAT_US,
		      "Effective fsu_min should be at least CONFIG minimum");
	zassert_equal(conn.lll.fsu.eff.fsu_max, CONFIG_BT_CTLR_EVENT_IFS_LOW_LAT_US,
		      "Effective fsu_max should be at least CONFIG minimum");
}

/*
 * Frame Space Update local TX update test - verifies fsu_max adjustment
 */
ZTEST(fsu_central, test_frame_space_update_local_tx_update)
{
	uint16_t fsu_min = 150;
	uint16_t fsu_max = 200;
	uint8_t phys = PHY_1M;
	uint16_t spacing_type = T_IFS_ACL_CP;

	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Set existing tifs values higher than requested fsu_max */
	conn.lll.tifs_tx_us = 250;
	conn.lll.tifs_rx_us = 260;

	/* Call local TX update */
	ull_fsu_local_tx_update(&conn, fsu_min, fsu_max, phys, spacing_type);

	/* Verify fsu_max was adjusted to accommodate existing tifs values */
	zassert_equal(conn.lll.fsu.local.fsu_min, fsu_min,
		      "Local fsu_min should be set as requested");
	zassert_equal(conn.lll.fsu.local.fsu_max, 260,
		      "Local fsu_max should be adjusted to max(tifs_tx_us, tifs_rx_us)");
	zassert_equal(conn.lll.fsu.local.phys, phys,
		      "Local phys should be set");
	zassert_equal(conn.lll.fsu.local.spacing_type, spacing_type,
		      "Local spacing_type should be set");
}

/*
 * Frame Space Update with PHY transition test
 * Tests that FSU per-PHY values are applied during PHY change
 *
 * +-----+                     +-------+                       +-----+
 * | UT  |                     | LL_A  |                       | LT  |
 * +-----+                     +-------+                       +-----+
 *    |                            |                              |
 *    | Setup FSU for 2M PHY       |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    | Initiate PHY Update        |                              |
 *    | to 2M PHY                  |                              |
 *    |--------------------------->|                              |
 *    |                            |                              |
 *    | Verify per-PHY FSU         |                              |
 *    | values applied             |                              |
 *    |<---------------------------|                              |
 *    |                            |                              |
 */
ZTEST(fsu_central, test_frame_space_update_phy_transition)
{
	test_set_role(&conn, BT_HCI_ROLE_CENTRAL);
	ull_cp_state_set(&conn, ULL_CP_CONNECTED);

	/* Set the current PHY to 1M */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Set up per-PHY FSU values for different PHYs */
	/* 1M PHY (index 0) */
	conn.lll.fsu.perphy[0].fsu_min = 150;
	conn.lll.fsu.perphy[0].fsu_max = 250;
	conn.lll.fsu.perphy[0].phys = PHY_1M;
	conn.lll.fsu.perphy[0].spacing_type = T_IFS_ACL_CP;

	/* 2M PHY (index 1) */
	conn.lll.fsu.perphy[1].fsu_min = 180;
	conn.lll.fsu.perphy[1].fsu_max = 280;
	conn.lll.fsu.perphy[1].phys = PHY_2M;
	conn.lll.fsu.perphy[1].spacing_type = T_IFS_ACL_CP;

	/* CODED PHY (index 2) */
	conn.lll.fsu.perphy[2].fsu_min = 200;
	conn.lll.fsu.perphy[2].fsu_max = 300;
	conn.lll.fsu.perphy[2].phys = PHY_CODED;
	conn.lll.fsu.perphy[2].spacing_type = T_IFS_ACL_CP;

	/* Start with 1M PHY */
	conn.lll.phy_tx = PHY_1M;
	conn.lll.phy_rx = PHY_1M;

	/* Verify different per-PHY storage values */
	zassert_equal(conn.lll.fsu.perphy[0].fsu_min, 150,
		      "1M PHY fsu_min incorrect");
	zassert_equal(conn.lll.fsu.perphy[1].fsu_min, 180,
		      "2M PHY fsu_min incorrect");
	zassert_equal(conn.lll.fsu.perphy[2].fsu_min, 200,
		      "CODED PHY fsu_min incorrect");

	/* The per-PHY values should be correctly stored and retrievable */
	zassert_not_equal(conn.lll.fsu.perphy[0].fsu_min,
			  conn.lll.fsu.perphy[1].fsu_min,
			  "Per-PHY storage not properly differentiated");
}

ZTEST_SUITE(fsu_central, NULL, NULL, fsu_setup, NULL, NULL);
ZTEST_SUITE(fsu_peripheral, NULL, NULL, fsu_setup, NULL, NULL);
