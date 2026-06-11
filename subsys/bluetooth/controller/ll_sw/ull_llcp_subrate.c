/*
 * Copyright (c) 2024 Demant A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/slist.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci_types.h>

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
#include "ull_conn_iso_internal.h"

#include "ull_conn_internal.h"
#include "ull_conn_types.h"

#include "ull_internal.h"
#include "ull_llcp.h"
#include "ull_llcp_features.h"
#include "ull_llcp_internal.h"

#include <soc.h>
#include "hal/debug.h"

/* Connection Subrate parameter ranges (Core 5.4, Vol 6, Part B, 5.1.19) */
#define SUBRATE_FACTOR_MIN	1U
#define SUBRATE_FACTOR_MAX	500U
#define SUBRATE_TIMEOUT_100MS	10U
#define SUBRATE_TIMEOUT_32SEC	3200U
#define SUBRATE_LATENCY_MAX	499U

/* LLCP Local Procedure Connection Subrate Update FSM states */
enum {
	LP_SR_STATE_IDLE = LLCP_STATE_IDLE,
	LP_SR_STATE_WAIT_TX_SUBRATE_REQ,
	LP_SR_STATE_WAIT_RX_SUBRATE_IND,
};

/* LLCP Local Procedure Connection Subrate Update FSM events */
enum {
	/* Procedure run */
	LP_SR_EVT_RUN,

	/* Indication received */
	LP_SR_EVT_SUBRATE_IND,

	/* Reject response received */
	LP_SR_EVT_REJECT,

	/* Unknown response received */
	LP_SR_EVT_UNKNOWN,
};

/* LLCP Remote Procedure Connection Subrate Update FSM states */
enum {
	RP_SR_STATE_IDLE = LLCP_STATE_IDLE,
};

/* LLCP Remote Procedure Connection Subrate Update FSM events */
enum {
	/* Procedure run */
	RP_SR_EVT_RUN,

	/* Indication received */
	RP_SR_EVT_SUBRATE_IND,
};

/*
 * Connection Subrate Update Procedure Helpers
 */

static bool sr_check_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const uint16_t subrate_factor = ctx->data.subrate.subrate_factor;
	const uint16_t continuation_number = ctx->data.subrate.continuation_number;
	const uint16_t latency = ctx->data.subrate.latency;
	const uint16_t timeout = ctx->data.subrate.timeout;
	const uint16_t interval = conn->lll.interval;

	/* Valid LL_SUBRATE_IND parameters (Core 5.4, Vol 6, Part B, 5.1.19.2):
	 *  - 1 <= subrate factor <= 500
	 *  - 0 <= continuation number < subrate factor
	 *  - peripheral latency and supervision timeout in valid ranges
	 *  - the supervision timeout shall be larger than the maximum time
	 *    between two consecutive subrate receive windows:
	 *      timeout(ms) > 2 * subrate_factor * (latency + 1) * interval(ms)
	 *    In connection event units (*4U re. conn events == *2U re. ms).
	 */
	return (subrate_factor >= SUBRATE_FACTOR_MIN) &&
	       (subrate_factor <= SUBRATE_FACTOR_MAX) &&
	       (continuation_number < subrate_factor) &&
	       (latency <= SUBRATE_LATENCY_MAX) &&
	       (timeout >= SUBRATE_TIMEOUT_100MS) &&
	       (timeout <= SUBRATE_TIMEOUT_32SEC) &&
	       (((uint32_t)timeout * 4U) >
		((uint32_t)subrate_factor * ((uint32_t)latency + 1U) * interval));
}

static void sr_apply(struct ll_conn *conn, struct proc_ctx *ctx)
{
	ull_conn_subrate_apply(conn,
			       ctx->data.subrate.subrate_factor,
			       ctx->data.subrate.subrate_base_event,
			       ctx->data.subrate.latency,
			       ctx->data.subrate.continuation_number,
			       ctx->data.subrate.timeout);
}

static void sr_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint8_t piggy_back = 1U;

	/* Re-use the received RX node for the host notification, falling back
	 * to a freshly allocated node only if none was handed over. A piggy-
	 * backed node is enqueued towards the LL on the ull_cp_rx return path,
	 * so it must not be enqueued here as well (that double-appends it).
	 */
	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	if (!ntf) {
		ntf = llcp_ntf_alloc();
		LL_ASSERT_DBG(ntf);
		piggy_back = 0U;
	}

	ntf->hdr.type = NODE_RX_TYPE_DC_PDU;
	ntf->hdr.handle = conn->lll.handle;
	pdu = (struct pdu_data *)ntf->pdu;

	switch (ctx->data.subrate.error) {
	case BT_HCI_ERR_SUCCESS:
		llcp_ntf_encode_subrate_change(ctx, pdu);
		break;
	case BT_HCI_ERR_UNSUPP_REMOTE_FEATURE:
		llcp_ntf_encode_unknown_rsp(ctx, pdu);
		break;
	default:
		llcp_ntf_encode_reject_ext_ind(ctx, pdu);
		break;
	}

	if (!piggy_back) {
		/* Enqueue notification towards LL, unless piggy-backing,
		 * in which case this is done on the rx return path
		 */
		ll_rx_put_sched(ntf->hdr.link, ntf);
	}
}

/*
 * LLCP Local Procedure Connection Subrate Update FSM
 */

static void lp_sr_tx(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	llcp_pdu_encode_subrate_req(ctx, pdu);
	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_SR_STATE_IDLE;
}

static void lp_sr_send_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_SR_STATE_WAIT_TX_SUBRATE_REQ;
	} else {
		lp_sr_tx(conn, ctx);

		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_SUBRATE_IND;
		ctx->state = LP_SR_STATE_WAIT_RX_SUBRATE_IND;
	}
}

static void lp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		lp_sr_send_subrate_req(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_wait_tx_subrate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		lp_sr_send_subrate_req(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_wait_rx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case LP_SR_EVT_SUBRATE_IND:
		llcp_pdu_decode_subrate_ind(ctx, pdu);

		/* Valid PDU? */
		if (sr_check_ind_parameters(conn, ctx)) {
			/* Apply new subrate parameters immediately (non-instant) */
			sr_apply(conn, ctx);

			/* Notify host of the applied subrate parameters */
			ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
			sr_ntf(conn, ctx);
			lp_sr_complete(conn, ctx);
		} else {
			/* Invalid parameters - terminate the connection */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_INVALID_LL_PARAM;
			lp_sr_complete(conn, ctx);
		}
		break;
	case LP_SR_EVT_REJECT:
		llcp_pdu_decode_reject_ext_ind(ctx, pdu);
		ctx->data.subrate.error = ctx->reject_ext_ind.error_code;
		sr_ntf(conn, ctx);
		lp_sr_complete(conn, ctx);
		break;
	case LP_SR_EVT_UNKNOWN:
		llcp_pdu_decode_unknown_rsp(ctx, pdu);
		/* Unsupported in peer, so disable locally for this connection */
		feature_unmask_features(conn, BIT64(BT_LE_FEAT_BIT_CONN_SUBRATING));
		ctx->data.subrate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
		sr_ntf(conn, ctx);
		lp_sr_complete(conn, ctx);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_SR_STATE_IDLE:
		lp_sr_st_idle(conn, ctx, evt, param);
		break;
	case LP_SR_STATE_WAIT_TX_SUBRATE_REQ:
		lp_sr_st_wait_tx_subrate_req(conn, ctx, evt, param);
		break;
	case LP_SR_STATE_WAIT_RX_SUBRATE_IND:
		lp_sr_st_wait_rx_subrate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_lp_sr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_SUBRATE_IND, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_UNKNOWN, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_REJECT, pdu);
		break;
	default:
		/* Invalid behaviour */
		/* Invalid PDU received so terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		lp_sr_complete(conn, ctx);
		break;
	}
}

void llcp_lp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_RUN, param);
}

/*
 * LLCP Remote Procedure Connection Subrate Update FSM
 */

static void rp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_SR_STATE_IDLE;
}

static void rp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case RP_SR_EVT_SUBRATE_IND:
		llcp_pdu_decode_subrate_ind(ctx, pdu);

		/* Valid PDU? */
		if (sr_check_ind_parameters(conn, ctx)) {
			/* Apply new subrate parameters immediately (non-instant) */
			sr_apply(conn, ctx);

			/* Notify host of the applied subrate parameters */
			ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
			sr_ntf(conn, ctx);
			rp_sr_complete(conn, ctx);
		} else {
			/* Invalid parameters - terminate the connection */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_INVALID_LL_PARAM;
			rp_sr_complete(conn, ctx);
		}
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case RP_SR_STATE_IDLE:
		rp_sr_st_idle(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_rp_sr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_SUBRATE_IND, pdu);
		break;
	default:
		/* Invalid behaviour */
		/* Invalid PDU received so terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		rp_sr_complete(conn, ctx);
		break;
	}
}

void llcp_rp_sr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_RUN, param);
}
