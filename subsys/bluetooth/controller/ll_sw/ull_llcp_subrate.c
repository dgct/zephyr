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
#if defined(CONFIG_BT_CENTRAL)
	/* Central origination (Core 5.4, Vol 6, Part B, 5.1.19): the Central
	 * transmits LL_SUBRATE_IND and applies on the Link Layer ACK.
	 */
	LP_SR_STATE_WAIT_TX_SUBRATE_IND,
	LP_SR_STATE_WAIT_TX_ACK,
#endif /* CONFIG_BT_CENTRAL */
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
#if defined(CONFIG_BT_CENTRAL)
	/* Link Layer acknowledgment of a transmitted LL_SUBRATE_IND */
	LP_SR_EVT_ACK,
#endif /* CONFIG_BT_CENTRAL */
};

/* LLCP Remote Procedure Connection Subrate Update FSM states */
enum {
	RP_SR_STATE_IDLE = LLCP_STATE_IDLE,
#if defined(CONFIG_BT_CENTRAL)
	/* Central responder (Core 5.4, Vol 6, Part B, 5.1.20): answer a peer
	 * Peripheral's LL_SUBRATE_REQ with LL_SUBRATE_IND or LL_REJECT_EXT_IND.
	 */
	RP_SR_STATE_WAIT_TX_SUBRATE_IND,
	RP_SR_STATE_WAIT_TX_REJECT_EXT_IND,
	RP_SR_STATE_WAIT_TX_ACK,
#endif /* CONFIG_BT_CENTRAL */
};

/* LLCP Remote Procedure Connection Subrate Update FSM events */
enum {
	/* Procedure run */
	RP_SR_EVT_RUN,

	/* Indication received */
	RP_SR_EVT_SUBRATE_IND,
#if defined(CONFIG_BT_CENTRAL)
	/* Request received (Peripheral -> Central) */
	RP_SR_EVT_SUBRATE_REQ,

	/* Link Layer acknowledgment of a transmitted LL_SUBRATE_IND */
	RP_SR_EVT_ACK,
#endif /* CONFIG_BT_CENTRAL */
};

/*
 * Connection Subrate Update Procedure Helpers
 */

/* lll->interval is in 1.25 ms units, or in 125 us units once Shorter
 * Connection Intervals is active: the spec relationship
 * timeout > 2 * factor * (latency + 1) * interval is evaluated in
 * microseconds so a subrated SCI link is not held to a check ten times too
 * strict (a failed check terminates the connection).
 */
static uint32_t sr_interval_us(const struct ll_conn *conn)
{
#if defined(CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS)
	if (conn->lll.sci_active) {
		return (uint32_t)conn->lll.interval * CONN_SCI_INT_UNIT_US;
	}
#endif /* CONFIG_BT_CTLR_SHORTER_CONNECTION_INTERVALS */
	return (uint32_t)conn->lll.interval * CONN_INT_UNIT_US;
}

static bool sr_check_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const uint16_t subrate_factor = ctx->data.subrate.subrate_factor;
	const uint16_t continuation_number = ctx->data.subrate.continuation_number;
	const uint16_t latency = ctx->data.subrate.latency;
	const uint16_t timeout = ctx->data.subrate.timeout;
	const uint32_t interval_us = sr_interval_us(conn);

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
	       (((uint64_t)timeout * 10000U) >
		(2U * (uint64_t)subrate_factor * ((uint64_t)latency + 1U) * interval_us));
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

#if defined(CONFIG_BT_CENTRAL)
/* Map the host-requested ranges of a Central-initiated procedure
 * (Core 5.4, Vol 6, Part B, 5.1.19) onto the concrete LL_SUBRATE_IND fields the
 * Central is about to transmit. The subrate base event is the connection event
 * counter at the time the indication is first queued, which trivially satisfies
 * the spec constraint that its two MSBs equal those of the event counter.
 *
 * The request inputs (continuation number, timeout) alias the indication output
 * fields in the shared subrate context, so all inputs are captured into locals
 * before any output is written.
 */
static void sr_prepare_loc_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	uint16_t factor = ctx->data.subrate.subrate_factor_max;
	uint16_t cont = ctx->data.subrate.continuation_number;
	const uint16_t max_latency = ctx->data.subrate.max_latency;

	if (factor < ctx->data.subrate.subrate_factor_min) {
		factor = ctx->data.subrate.subrate_factor_min;
	}
	if (factor < SUBRATE_FACTOR_MIN) {
		factor = SUBRATE_FACTOR_MIN;
	}

	/* Continuation number must be strictly less than the subrate factor */
	if (cont > (uint16_t)(factor - 1U)) {
		cont = factor - 1U;
	}

	ctx->data.subrate.subrate_factor = factor;
	ctx->data.subrate.continuation_number = cont;
	ctx->data.subrate.latency = max_latency;
	ctx->data.subrate.subrate_base_event = ull_conn_event_counter(conn);
	/* timeout is unchanged: the request timeout is the indication timeout */
}

/* Negotiate a peer Peripheral's LL_SUBRATE_REQ against the Central's acceptable
 * defaults (Core 5.4, Vol 6, Part B, 5.1.20). On acceptance, populate the
 * LL_SUBRATE_IND fields and return true; otherwise return false so the caller
 * rejects with LL_REJECT_EXT_IND.
 *
 * The request inputs are captured into locals before any output field is
 * written because they share storage with the indication outputs.
 */
static bool sr_select_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const struct llcp_subrate_defaults *acc = llcp_subrate_defaults_get();
	const uint16_t factor_min_req = ctx->data.subrate.subrate_factor_min;
	const uint16_t factor_max_req = ctx->data.subrate.subrate_factor_max;
	const uint16_t max_latency_req = ctx->data.subrate.max_latency;
	const uint16_t cont_req = ctx->data.subrate.continuation_number;
	const uint16_t timeout_req = ctx->data.subrate.timeout;
	const uint32_t interval_us = sr_interval_us(conn);
	uint16_t factor;
	uint16_t cont;

	/* Reject if the request cannot be honoured within the acceptable
	 * defaults, or if the requested timeout is too small to cover the
	 * maximum time between subrate receive windows:
	 *   timeout(ms) > 2 * factor_min * (max_latency + 1) * interval(ms)
	 * expressed in connection-event units (*4U re. events == *2U re. ms).
	 */
	if ((max_latency_req > acc->max_latency) ||
	    (timeout_req > acc->timeout) ||
	    (factor_max_req < acc->factor_min) ||
	    (factor_min_req > acc->factor_max) ||
	    ((2U * (uint64_t)factor_min_req * ((uint64_t)max_latency_req + 1U) * interval_us) >=
	     ((uint64_t)timeout_req * 10000U))) {
		return false;
	}

	/* Select the negotiated values within both peers' constraints */
	factor = MIN(acc->factor_max, factor_max_req);
	if (factor < SUBRATE_FACTOR_MIN) {
		factor = SUBRATE_FACTOR_MIN;
	}

	cont = MAX(acc->continuation_number, cont_req);
	if (cont > (uint16_t)(factor - 1U)) {
		cont = factor - 1U;
	}

	ctx->data.subrate.subrate_factor = factor;
	ctx->data.subrate.continuation_number = cont;
	ctx->data.subrate.latency = MIN(max_latency_req, acc->max_latency);
	ctx->data.subrate.timeout = MIN(timeout_req, acc->timeout);
	ctx->data.subrate.subrate_base_event = ull_conn_event_counter(conn);

	/* Defensive: the chosen indication must itself be receive-side valid */
	return sr_check_ind_parameters(conn, ctx);
}
#endif /* CONFIG_BT_CENTRAL */

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

#if defined(CONFIG_BT_CENTRAL)
static void lp_sr_tx_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	llcp_pdu_encode_subrate_ind(ctx, pdu);
	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Apply the negotiated parameters on the Link Layer acknowledgment of
	 * this indication (Core 5.4, Vol 6, Part B, 5.1.19).
	 */
	ctx->node_ref.tx_ack = tx;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_sr_send_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_SR_STATE_WAIT_TX_SUBRATE_IND;
	} else {
		/* Derive the indication fields (incl. the subrate base event) at
		 * the time the PDU is actually queued.
		 */
		sr_prepare_loc_ind_parameters(conn, ctx);
		lp_sr_tx_ind(conn, ctx);

		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
		ctx->state = LP_SR_STATE_WAIT_TX_ACK;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void lp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
#if defined(CONFIG_BT_CENTRAL)
		if (conn->lll.role == BT_HCI_ROLE_CENTRAL) {
			/* A Central originates the subrate change directly with
			 * an LL_SUBRATE_IND (Core 5.4, Vol 6, Part B, 5.1.19).
			 */
			lp_sr_send_subrate_ind(conn, ctx, evt, param);
			break;
		}
#endif /* CONFIG_BT_CENTRAL */
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

#if defined(CONFIG_BT_CENTRAL)
static void lp_sr_st_wait_tx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case LP_SR_EVT_RUN:
		lp_sr_send_subrate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_sr_st_wait_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case LP_SR_EVT_ACK:
		/* The peer acknowledged the LL_SUBRATE_IND; apply the negotiated
		 * parameters now and notify the host (Core 5.4, Vol 6, Part B,
		 * 5.1.19).
		 */
		sr_apply(conn, ctx);
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		sr_ntf(conn, ctx);
		lp_sr_complete(conn, ctx);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

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
#if defined(CONFIG_BT_CENTRAL)
	case LP_SR_STATE_WAIT_TX_SUBRATE_IND:
		lp_sr_st_wait_tx_subrate_ind(conn, ctx, evt, param);
		break;
	case LP_SR_STATE_WAIT_TX_ACK:
		lp_sr_st_wait_tx_ack(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
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

#if defined(CONFIG_BT_CENTRAL)
void llcp_lp_sr_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	lp_sr_execute_fsm(conn, ctx, LP_SR_EVT_ACK, tx->pdu);
}
#endif /* CONFIG_BT_CENTRAL */

/*
 * LLCP Remote Procedure Connection Subrate Update FSM
 */

static void rp_sr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_SR_STATE_IDLE;
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_sr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_IND:
		llcp_pdu_encode_subrate_ind(ctx, pdu);
		/* Apply the negotiated parameters on the Link Layer
		 * acknowledgment of this indication (Core 5.4, Vol 6,
		 * Part B, 5.1.19 / 5.1.20).
		 */
		ctx->node_ref.tx_ack = tx;
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		llcp_pdu_encode_reject_ext_ind(pdu, ctx->reject_ext_ind.reject_opcode,
					       ctx->reject_ext_ind.error_code);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_rr_prt_restart(conn);
}

static void rp_sr_send_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_SR_STATE_WAIT_TX_SUBRATE_IND;
	} else {
		rp_sr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_SUBRATE_IND);
		ctx->state = RP_SR_STATE_WAIT_TX_ACK;
	}
}

static void rp_sr_send_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				      void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_SR_STATE_WAIT_TX_REJECT_EXT_IND;
	} else {
		rp_sr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);

		/* A rejected, peer-initiated procedure makes no parameter change
		 * and was not started by the host, so no host notification is
		 * generated (Core 5.4, Vol 4, Part E, 7.7.65.30).
		 */
		rp_sr_complete(conn, ctx);
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void rp_sr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case RP_SR_EVT_SUBRATE_IND:
#if defined(CONFIG_BT_CENTRAL)
		if (conn->lll.role == BT_HCI_ROLE_CENTRAL) {
			/* Only the Central may originate LL_SUBRATE_IND; one
			 * received as a Central is a protocol violation
			 * (Core 5.4, Vol 6, Part B, 5.1.19).
			 */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
			rp_sr_complete(conn, ctx);
			break;
		}
#endif /* CONFIG_BT_CENTRAL */
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
#if defined(CONFIG_BT_CENTRAL)
	case RP_SR_EVT_SUBRATE_REQ:
		llcp_pdu_decode_subrate_req(ctx, pdu);

		if (!feature_subrating(conn)) {
			/* The Connection Subrating feature is not usable on this
			 * connection: reject the peer's request.
			 */
			ctx->reject_ext_ind.reject_opcode =
				PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ;
			ctx->reject_ext_ind.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
			rp_sr_send_reject_ext_ind(conn, ctx, evt, param);
		} else if (sr_select_ind_parameters(conn, ctx)) {
			/* Acceptable request: answer with an LL_SUBRATE_IND */
			rp_sr_send_subrate_ind(conn, ctx, evt, param);
		} else {
			/* Unacceptable parameters: reject the request */
			ctx->reject_ext_ind.reject_opcode =
				PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ;
			ctx->reject_ext_ind.error_code = BT_HCI_ERR_UNSUPP_LL_PARAM_VAL;
			rp_sr_send_reject_ext_ind(conn, ctx, evt, param);
		}
		break;
#endif /* CONFIG_BT_CENTRAL */
	default:
		/* Ignore other evts */
		break;
	}
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_sr_st_wait_tx_subrate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case RP_SR_EVT_RUN:
		rp_sr_send_subrate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_sr_st_wait_tx_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					    uint8_t evt, void *param)
{
	switch (evt) {
	case RP_SR_EVT_RUN:
		rp_sr_send_reject_ext_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_sr_st_wait_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				 void *param)
{
	switch (evt) {
	case RP_SR_EVT_ACK:
		/* The peer acknowledged the LL_SUBRATE_IND; apply the negotiated
		 * parameters now and notify the host (Core 5.4, Vol 6, Part B,
		 * 5.1.20).
		 */
		sr_apply(conn, ctx);
		ctx->data.subrate.error = BT_HCI_ERR_SUCCESS;
		sr_ntf(conn, ctx);
		rp_sr_complete(conn, ctx);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void rp_sr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case RP_SR_STATE_IDLE:
		rp_sr_st_idle(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_CENTRAL)
	case RP_SR_STATE_WAIT_TX_SUBRATE_IND:
		rp_sr_st_wait_tx_subrate_ind(conn, ctx, evt, param);
		break;
	case RP_SR_STATE_WAIT_TX_REJECT_EXT_IND:
		rp_sr_st_wait_tx_reject_ext_ind(conn, ctx, evt, param);
		break;
	case RP_SR_STATE_WAIT_TX_ACK:
		rp_sr_st_wait_tx_ack(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
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
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_SUBRATE_REQ:
		rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_SUBRATE_REQ, pdu);
		break;
#endif /* CONFIG_BT_CENTRAL */
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

#if defined(CONFIG_BT_CENTRAL)
void llcp_rp_sr_tx_ack(struct ll_conn *conn, struct proc_ctx *ctx, struct node_tx *tx)
{
	rp_sr_execute_fsm(conn, ctx, RP_SR_EVT_ACK, tx->pdu);
}
#endif /* CONFIG_BT_CENTRAL */
