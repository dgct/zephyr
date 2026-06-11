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

/* Connection Rate parameter ranges (Shorter Connection Intervals; Core 6.2,
 * Vol 6, Part B, 5.1.32 / 5.1.33). The connInterval of an LL_CONNECTION_RATE_IND
 * is expressed in 125 us units, in contrast to the 1.25 ms units used by the
 * Connection Update procedure.
 */
#define CONN_RATE_INSTANT_DELTA		6U
#define CONN_RATE_SUBRATE_FACTOR_MIN	1U
#define CONN_RATE_SUBRATE_FACTOR_MAX	500U
#define CONN_RATE_LATENCY_MAX		499U
#define CONN_RATE_TIMEOUT_100MS		10U	/* 10 ms units */
#define CONN_RATE_TIMEOUT_32SEC		3200U	/* 10 ms units */
/* connInterval bounds in 125 us units: the absolute Link Layer floor of 750 us
 * (below the legacy 7.5 ms minimum, which is the whole point of the feature)
 * up to 4 s. The effective minimum on a given connection is further constrained
 * by the active frame space (Frame Space Update procedure).
 */
#define CONN_RATE_INTERVAL_MIN		6U	/* 750 us */
#define CONN_RATE_INTERVAL_MAX		32000U	/* 4 s */

/* LLCP Local Procedure Connection Rate FSM states */
enum {
	LP_CR_STATE_IDLE = LLCP_STATE_IDLE,
	LP_CR_STATE_WAIT_INSTANT,
#if defined(CONFIG_BT_PERIPHERAL)
	/* Peripheral origination (Core 6.2, Vol 6, Part B, 5.1.33): the
	 * Peripheral requests a rate change with LL_CONNECTION_RATE_REQ and
	 * waits for the Central's LL_CONNECTION_RATE_IND (or a rejection).
	 */
	LP_CR_STATE_WAIT_TX_CONN_RATE_REQ,
	LP_CR_STATE_WAIT_RX_CONN_RATE_IND,
	LP_CR_STATE_WAIT_TX_UNKNOWN_RSP,
#endif /* CONFIG_BT_PERIPHERAL */
#if defined(CONFIG_BT_CENTRAL)
	/* Central origination (Core 6.2, Vol 6, Part B, 5.1.32): the Central
	 * transmits LL_CONNECTION_RATE_IND and applies at the instant.
	 */
	LP_CR_STATE_WAIT_TX_CONN_RATE_IND,
	LP_CR_STATE_WAIT_NTF_AVAIL,
#endif /* CONFIG_BT_CENTRAL */
};

/* LLCP Local Procedure Connection Rate FSM events */
enum {
	/* Procedure run */
	LP_CR_EVT_RUN,

	/* Indication received */
	LP_CR_EVT_CONN_RATE_IND,

	/* Reject response received */
	LP_CR_EVT_REJECT,

	/* Unknown response received */
	LP_CR_EVT_UNKNOWN,
};

/* LLCP Remote Procedure Connection Rate FSM states */
enum {
	RP_CR_STATE_IDLE = LLCP_STATE_IDLE,
	RP_CR_STATE_WAIT_INSTANT,
	RP_CR_STATE_WAIT_TX_UNKNOWN_RSP,
#if defined(CONFIG_BT_CENTRAL)
	/* Central responder (Core 6.2, Vol 6, Part B, 5.1.33): answer a peer
	 * Peripheral's LL_CONNECTION_RATE_REQ with LL_CONNECTION_RATE_IND or
	 * LL_REJECT_EXT_IND.
	 */
	RP_CR_STATE_WAIT_TX_CONN_RATE_IND,
	RP_CR_STATE_WAIT_TX_REJECT_EXT_IND,
#endif /* CONFIG_BT_CENTRAL */
};

/* LLCP Remote Procedure Connection Rate FSM events */
enum {
	/* Procedure run */
	RP_CR_EVT_RUN,

	/* Indication received (Central -> Peripheral) */
	RP_CR_EVT_CONN_RATE_IND,
#if defined(CONFIG_BT_CENTRAL)
	/* Request received (Peripheral -> Central) */
	RP_CR_EVT_CONN_RATE_REQ,
#endif /* CONFIG_BT_CENTRAL */
};

/*
 * Connection Rate Procedure Helpers
 */

static bool cr_check_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const uint16_t interval = ctx->data.conn_rate.interval; /* 125 us units */
	const uint16_t subrate_factor = ctx->data.conn_rate.subrate_factor;
	const uint16_t continuation_number = ctx->data.conn_rate.continuation_number;
	const uint16_t latency = ctx->data.conn_rate.latency;
	const uint16_t timeout = ctx->data.conn_rate.timeout; /* 10 ms units */

	/* Valid LL_CONNECTION_RATE_IND parameters (Core 6.2, Vol 6, Part B,
	 * 5.1.32):
	 *  - connInterval within [Link Layer floor .. 4 s], in 125 us units
	 *  - 1 <= subrate factor <= 500, 0 <= continuation number < factor
	 *  - peripheral latency and supervision timeout within range
	 *  - the supervision timeout exceeds the maximum time between two
	 *    consecutive subrated receive windows:
	 *       timeout(ms) > 2 * factor * (latency + 1) * interval(ms)
	 *    With interval in 125 us units and timeout in 10 ms units this is
	 *    equivalent to: timeout * 40 > factor * (latency + 1) * interval.
	 */
	return (interval >= CONN_RATE_INTERVAL_MIN) &&
	       (interval <= CONN_RATE_INTERVAL_MAX) &&
	       (subrate_factor >= CONN_RATE_SUBRATE_FACTOR_MIN) &&
	       (subrate_factor <= CONN_RATE_SUBRATE_FACTOR_MAX) &&
	       (continuation_number < subrate_factor) &&
	       (latency <= CONN_RATE_LATENCY_MAX) &&
	       (timeout >= CONN_RATE_TIMEOUT_100MS) &&
	       (timeout <= CONN_RATE_TIMEOUT_32SEC) &&
	       (((uint32_t)timeout * 40U) >
		((uint32_t)subrate_factor * ((uint32_t)latency + 1U) * interval));
}

static void cr_apply(struct ll_conn *conn, struct proc_ctx *ctx)
{
	/* Apply the new connection rate at the instant. The interval (in 125 us
	 * units) and the connection timing are applied first; this sets the
	 * SCI-active flag so subsequent interval-to-microseconds conversions use
	 * the 125 us unit. The subrate cadence, whose base event is the instant,
	 * is applied afterwards (Core 6.2, Vol 6, Part B, 5.1.32).
	 */
	ull_conn_update_parameters(conn, 1U, 1U, ctx->data.conn_rate.win_offset_us,
				   ctx->data.conn_rate.interval, ctx->data.conn_rate.latency,
				   ctx->data.conn_rate.timeout, ctx->data.conn_rate.instant, 1U);

	ull_conn_subrate_apply(conn, ctx->data.conn_rate.subrate_factor,
			       ctx->data.conn_rate.instant, ctx->data.conn_rate.latency,
			       ctx->data.conn_rate.continuation_number,
			       ctx->data.conn_rate.timeout);
}

static void cr_ntf(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_rx_pdu *ntf;
	struct pdu_data *pdu;
	uint8_t piggy_back;

	/* Re-use a node handed over for the host notification, falling back to a
	 * freshly allocated node only if none is available. A node explicitly
	 * retained for notification (NODE_RX_TYPE_RETAIN, e.g. a Central's own
	 * origination) is scheduled towards the LL here; a piggy-backed received
	 * node is scheduled on the ull_cp_rx return path instead, so it must not
	 * be enqueued here as well (that would double-append it).
	 */
	ntf = ctx->node_ref.rx;
	ctx->node_ref.rx = NULL;
	if (!ntf) {
		ntf = llcp_ntf_alloc();
		LL_ASSERT_DBG(ntf);
		piggy_back = 0U;
	} else {
		piggy_back = (ntf->hdr.type != NODE_RX_TYPE_RETAIN);
	}

	ntf->hdr.type = NODE_RX_TYPE_DC_PDU;
	ntf->hdr.handle = conn->lll.handle;
	pdu = (struct pdu_data *)ntf->pdu;

	switch (ctx->data.conn_rate.error) {
	case BT_HCI_ERR_SUCCESS:
		llcp_ntf_encode_conn_rate_change(ctx, pdu);
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
/* Set the local instant and transmit window offset for a Central-transmitted
 * LL_CONNECTION_RATE_IND (Core 6.2, Vol 6, Part B, 5.1.32). The instant is
 * computed at the time the indication is queued so it remains valid even if the
 * transmission had to be deferred. A zero transmit window offset (no anchor
 * point move) is always within the permitted [0 .. connIntervalNEW] range.
 */
static void cr_set_loc_instant(struct ll_conn *conn, struct proc_ctx *ctx)
{
	ctx->data.conn_rate.win_offset_us = 0U;
	ctx->data.conn_rate.instant = ull_conn_event_counter(conn) + conn->lll.latency +
				      CONN_RATE_INSTANT_DELTA;
}

/* Map the host-requested ranges of a Central-initiated procedure (Core 6.2,
 * Vol 6, Part B, 5.1.32) onto the concrete LL_CONNECTION_RATE_IND fields the
 * Central is about to transmit.
 *
 * The request inputs (continuation number, timeout) alias the indication output
 * fields in the shared connection-rate context, so all inputs are captured into
 * locals before any output is written.
 */
static void cr_prepare_loc_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	uint16_t interval = ctx->data.conn_rate.interval_max;
	uint16_t factor = ctx->data.conn_rate.subrate_factor_max;
	uint16_t cont = ctx->data.conn_rate.continuation_number;
	const uint16_t interval_min = ctx->data.conn_rate.interval_min;
	const uint16_t factor_min = ctx->data.conn_rate.subrate_factor_min;
	const uint16_t max_latency = ctx->data.conn_rate.max_latency;

	if (interval < interval_min) {
		interval = interval_min;
	}
	if (interval < CONN_RATE_INTERVAL_MIN) {
		interval = CONN_RATE_INTERVAL_MIN;
	}

	if (factor < factor_min) {
		factor = factor_min;
	}
	if (factor < CONN_RATE_SUBRATE_FACTOR_MIN) {
		factor = CONN_RATE_SUBRATE_FACTOR_MIN;
	}

	/* Continuation number must be strictly less than the subrate factor */
	if (cont > (uint16_t)(factor - 1U)) {
		cont = factor - 1U;
	}

	ctx->data.conn_rate.interval = interval;
	ctx->data.conn_rate.subrate_factor = factor;
	ctx->data.conn_rate.continuation_number = cont;
	ctx->data.conn_rate.latency = max_latency;
	/* timeout is unchanged: the request timeout is the indication timeout */

	cr_set_loc_instant(conn, ctx);
}

/* Negotiate a peer Peripheral's LL_CONNECTION_RATE_REQ against the Central's
 * acceptable defaults (Core 6.2, Vol 6, Part B, 5.1.33). On acceptance, populate
 * the LL_CONNECTION_RATE_IND rate fields and return BT_HCI_ERR_SUCCESS; otherwise
 * return the LL_REJECT_EXT_IND error code mandated by 5.1.33.
 *
 * The request inputs are captured into locals before any output field is written
 * because they share storage with the indication outputs. The instant is set
 * separately, at indication transmit time.
 */
static uint8_t cr_select_ind_parameters(struct ll_conn *conn, struct proc_ctx *ctx)
{
	const struct llcp_conn_rate_defaults *acc = llcp_conn_rate_defaults_get();
	const uint16_t interval_min_req = ctx->data.conn_rate.interval_min;
	const uint16_t interval_max_req = ctx->data.conn_rate.interval_max;
	const uint16_t factor_min_req = ctx->data.conn_rate.subrate_factor_min;
	const uint16_t factor_max_req = ctx->data.conn_rate.subrate_factor_max;
	const uint16_t max_latency_req = ctx->data.conn_rate.max_latency;
	const uint16_t cont_req = ctx->data.conn_rate.continuation_number;
	const uint16_t timeout_req = ctx->data.conn_rate.timeout;
	uint16_t interval;
	uint16_t factor;
	uint16_t cont;

	/* Out-of-range fields are rejected with Invalid LL Parameters (0x1E) */
	if ((interval_min_req < CONN_RATE_INTERVAL_MIN) ||
	    (interval_max_req > CONN_RATE_INTERVAL_MAX) ||
	    (interval_min_req > interval_max_req) ||
	    (factor_min_req < CONN_RATE_SUBRATE_FACTOR_MIN) ||
	    (factor_max_req > CONN_RATE_SUBRATE_FACTOR_MAX) ||
	    (factor_max_req < factor_min_req) ||
	    (max_latency_req > CONN_RATE_LATENCY_MAX) ||
	    (timeout_req < CONN_RATE_TIMEOUT_100MS) ||
	    (timeout_req > CONN_RATE_TIMEOUT_32SEC) ||
	    (cont_req >= factor_max_req)) {
		return BT_HCI_ERR_INVALID_LL_PARAM;
	}

	/* An Interval_Max below the Central's required minimum interval is
	 * rejected with Unsupported Feature or Parameter Value (0x11).
	 */
	if (interval_max_req < acc->interval_min) {
		return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
	}

	/* Select the negotiated values within both peers' constraints */
	interval = MIN(interval_max_req, acc->interval_max);
	if (interval < acc->interval_min) {
		interval = acc->interval_min;
	}
	if (interval < interval_min_req) {
		interval = interval_min_req;
	}

	factor = MIN(acc->factor_max, factor_max_req);
	if (factor < CONN_RATE_SUBRATE_FACTOR_MIN) {
		factor = CONN_RATE_SUBRATE_FACTOR_MIN;
	}

	cont = MAX(acc->continuation_number, cont_req);
	if (cont > (uint16_t)(factor - 1U)) {
		cont = factor - 1U;
	}

	ctx->data.conn_rate.interval = interval;
	ctx->data.conn_rate.subrate_factor = factor;
	ctx->data.conn_rate.continuation_number = cont;
	ctx->data.conn_rate.latency = MIN(max_latency_req, acc->max_latency);
	ctx->data.conn_rate.timeout = MIN(timeout_req, acc->timeout);

	/* Any otherwise unacceptable parameter set is rejected with Unsupported
	 * LL Parameter Value (0x20). The chosen indication must itself be
	 * receive-side valid.
	 */
	if (!cr_check_ind_parameters(conn, ctx)) {
		return BT_HCI_ERR_UNSUPP_LL_PARAM_VAL;
	}

	return BT_HCI_ERR_SUCCESS;
}
#endif /* CONFIG_BT_CENTRAL */

/*
 * LLCP Local Procedure Connection Rate FSM
 */

static void lp_cr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_lr_complete(conn);
	ctx->state = LP_CR_STATE_IDLE;
}

static void lp_cr_check_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				void *param)
{
	uint16_t event_counter = ull_conn_event_counter_at_prepare(conn);

	if (is_instant_reached_or_passed(ctx->data.conn_rate.instant, event_counter)) {
		/* The procedure is complete once the instant has passed and the
		 * new connection event parameters have been applied.
		 */
		llcp_rr_set_incompat(conn, INCOMPAT_NO_COLLISION);
		cr_apply(conn, ctx);

		ctx->data.conn_rate.error = BT_HCI_ERR_SUCCESS;
		cr_ntf(conn, ctx);
		lp_cr_complete(conn, ctx);
	}
}

#if defined(CONFIG_BT_PERIPHERAL)
static void lp_cr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	switch (opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ:
		llcp_pdu_encode_conn_rate_req(ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		llcp_pdu_encode_unknown_rsp(ctx, pdu);
		break;
	default:
		LL_ASSERT_DBG(0);
		break;
	}
	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_cr_send_conn_rate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				     void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CR_STATE_WAIT_TX_CONN_RATE_REQ;
	} else {
		lp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ);

		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND;
		ctx->state = LP_CR_STATE_WAIT_RX_CONN_RATE_IND;
	}
}

static void lp_cr_send_unknown_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = LP_CR_STATE_WAIT_TX_UNKNOWN_RSP;
	} else {
		lp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP);

		/* The host-initiated procedure failed; notify the host that the
		 * Central's indication could not be honoured.
		 */
		ctx->data.conn_rate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
		cr_ntf(conn, ctx);
		lp_cr_complete(conn, ctx);
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

#if defined(CONFIG_BT_CENTRAL)
static void lp_cr_tx_ind(struct ll_conn *conn, struct proc_ctx *ctx)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Get pre-allocated tx node */
	tx = ctx->node_ref.tx;
	ctx->node_ref.tx = NULL;
	if (!tx) {
		tx = llcp_tx_alloc(conn, ctx);
		LL_ASSERT_DBG(tx);
	}

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU. The connection rate is applied at the instant
	 * (not on the Link Layer acknowledgment), so no tx_ack is registered.
	 */
	llcp_pdu_encode_conn_rate_ind(ctx, pdu);
	ctx->tx_opcode = pdu->llctrl.opcode;

	/* Enqueue LL Control PDU towards LLL */
	llcp_tx_enqueue(conn, tx);

	/* Restart procedure response timeout timer */
	llcp_lr_prt_restart(conn);
}

static void lp_cr_send_conn_rate_ind_finalize(struct ll_conn *conn, struct proc_ctx *ctx,
					      uint8_t evt, void *param)
{
	if (ctx->node_ref.rx == NULL) {
		/* If we get here without RX node we know one is avail to be
		 * allocated, so pre-alloc the NTF node used at the instant.
		 */
		ctx->node_ref.rx = llcp_ntf_alloc();
	}

	/* Signal put/sched on NTF - ie non-RX node piggy */
	ctx->node_ref.rx->hdr.type = NODE_RX_TYPE_RETAIN;

	cr_prepare_loc_ind_parameters(conn, ctx);
	lp_cr_tx_ind(conn, ctx);
	ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
	ctx->state = LP_CR_STATE_WAIT_INSTANT;
}

static void lp_cr_send_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				     void *param)
{
	if (llcp_lr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx) ||
	    (ull_tx_q_peek(&conn->tx_q) != NULL) || !ull_conn_lll_tx_queue_is_empty(conn)) {
		llcp_tx_pause_data(conn, LLCP_TX_QUEUE_PAUSE_DATA_CONN_UPD);
		ctx->state = LP_CR_STATE_WAIT_TX_CONN_RATE_IND;
	} else {
		/* ensure alloc of TX node, before possibly waiting for NTF node */
		ctx->node_ref.tx = llcp_tx_alloc(conn, ctx);
		if (ctx->node_ref.rx == NULL && !llcp_ntf_alloc_is_available()) {
			/* No RX node piggy, and no NTF avail, so go wait for one,
			 * before TX'ing
			 */
			ctx->state = LP_CR_STATE_WAIT_NTF_AVAIL;
		} else {
			lp_cr_send_conn_rate_ind_finalize(conn, ctx, evt, param);
			llcp_tx_resume_data(conn, LLCP_TX_QUEUE_PAUSE_DATA_CONN_UPD);
		}
	}
}

static void lp_cr_st_wait_tx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					   void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_send_conn_rate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_cr_st_wait_ntf_avail(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				    void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
		if (llcp_ntf_alloc_is_available()) {
			lp_cr_send_conn_rate_ind_finalize(conn, ctx, evt, param);
			llcp_tx_resume_data(conn, LLCP_TX_QUEUE_PAUSE_DATA_CONN_UPD);
		}
		break;
	default:
		/* Ignore other evts */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void lp_cr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
#if defined(CONFIG_BT_CENTRAL)
		if (conn->lll.role == BT_HCI_ROLE_CENTRAL) {
			/* A Central originates the rate change directly with an
			 * LL_CONNECTION_RATE_IND (Core 6.2, Vol 6, Part B, 5.1.32).
			 */
			ctx->node_ref.rx = NULL;
			lp_cr_send_conn_rate_ind(conn, ctx, evt, param);
			break;
		}
#endif /* CONFIG_BT_CENTRAL */
#if defined(CONFIG_BT_PERIPHERAL)
		lp_cr_send_conn_rate_req(conn, ctx, evt, param);
#endif /* CONFIG_BT_PERIPHERAL */
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

#if defined(CONFIG_BT_PERIPHERAL)
static void lp_cr_st_wait_tx_conn_rate_req(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					   void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_send_conn_rate_req(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_cr_st_wait_tx_unknown_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_send_unknown_rsp(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void lp_cr_st_wait_rx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					   void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case LP_CR_EVT_CONN_RATE_IND:
		llcp_pdu_decode_conn_rate_ind(ctx, pdu);

		/* Valid PDU? */
		if (cr_check_ind_parameters(conn, ctx)) {
			uint16_t event_counter = ull_conn_event_counter(conn);

			if (is_instant_not_passed(ctx->data.conn_rate.instant, event_counter)) {
				llcp_rr_set_incompat(conn, INCOMPAT_RESERVED);

				/* Keep RX node to use for NTF */
				llcp_rx_node_retain(ctx);

				ctx->state = LP_CR_STATE_WAIT_INSTANT;
				break;
			}

			/* Instant already passed - terminate the connection */
			llcp_rr_set_incompat(conn, INCOMPAT_NO_COLLISION);
			conn->llcp_terminate.reason_final = BT_HCI_ERR_INSTANT_PASSED;
			lp_cr_complete(conn, ctx);
		} else {
			/* Out-of-range or unsupported fields: signal back with
			 * LL_UNKNOWN_RSP without terminating the link (Core 6.2,
			 * Vol 6, Part B, 5.1.32).
			 */
			ctx->unknown_response.type = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND;
			lp_cr_send_unknown_rsp(conn, ctx, evt, param);
		}
		break;
	case LP_CR_EVT_REJECT:
		llcp_pdu_decode_reject_ext_ind(ctx, pdu);
		ctx->data.conn_rate.error = ctx->reject_ext_ind.error_code;
		cr_ntf(conn, ctx);
		lp_cr_complete(conn, ctx);
		break;
	case LP_CR_EVT_UNKNOWN:
		llcp_pdu_decode_unknown_rsp(ctx, pdu);
		/* Unsupported in peer, so disable locally for this connection */
		feature_unmask_features_page1(conn, LL_FEAT_P1_BIT_SHORTER_CI);
		ctx->data.conn_rate.error = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
		cr_ntf(conn, ctx);
		lp_cr_complete(conn, ctx);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}
#endif /* CONFIG_BT_PERIPHERAL */

static void lp_cr_st_wait_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				  void *param)
{
	switch (evt) {
	case LP_CR_EVT_RUN:
		lp_cr_check_instant(conn, ctx, evt, param);
		break;
	default:
		/* Any LL Control PDU received while waiting for the instant is
		 * ignored: a node is already retained for the completion
		 * notification, and the RX-node retention mechanism does not
		 * support consuming a second received node here without
		 * discarding the retained one. The locally-initiated procedure is
		 * allowed to complete at the instant (this mirrors the documented
		 * collision handling in lp_cu_st_wait_instant(); see also the
		 * comment in ull_llcp_local.c::llcp_lr_rx()).
		 */
		break;
	}
}

static void lp_cr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case LP_CR_STATE_IDLE:
		lp_cr_st_idle(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_PERIPHERAL)
	case LP_CR_STATE_WAIT_TX_CONN_RATE_REQ:
		lp_cr_st_wait_tx_conn_rate_req(conn, ctx, evt, param);
		break;
	case LP_CR_STATE_WAIT_RX_CONN_RATE_IND:
		lp_cr_st_wait_rx_conn_rate_ind(conn, ctx, evt, param);
		break;
	case LP_CR_STATE_WAIT_TX_UNKNOWN_RSP:
		lp_cr_st_wait_tx_unknown_rsp(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_PERIPHERAL */
#if defined(CONFIG_BT_CENTRAL)
	case LP_CR_STATE_WAIT_TX_CONN_RATE_IND:
		lp_cr_st_wait_tx_conn_rate_ind(conn, ctx, evt, param);
		break;
	case LP_CR_STATE_WAIT_NTF_AVAIL:
		lp_cr_st_wait_ntf_avail(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
	case LP_CR_STATE_WAIT_INSTANT:
		lp_cr_st_wait_instant(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_lp_cr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_CONN_RATE_IND, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_UNKNOWN, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_REJECT, pdu);
		break;
	default:
		/* Invalid behaviour */
		/* Invalid PDU received so terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		lp_cr_complete(conn, ctx);
		break;
	}
}

void llcp_lp_cr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	lp_cr_execute_fsm(conn, ctx, LP_CR_EVT_RUN, param);
}

/*
 * LLCP Remote Procedure Connection Rate FSM
 */

static void rp_cr_complete(struct ll_conn *conn, struct proc_ctx *ctx)
{
	llcp_rr_complete(conn);
	ctx->state = RP_CR_STATE_IDLE;
}

static void rp_cr_tx(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t opcode)
{
	struct node_tx *tx;
	struct pdu_data *pdu;

	/* Allocate tx node */
	tx = llcp_tx_alloc(conn, ctx);
	LL_ASSERT_DBG(tx);

	pdu = (struct pdu_data *)tx->pdu;

	/* Encode LL Control PDU */
	switch (opcode) {
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND:
		llcp_pdu_encode_conn_rate_ind(ctx, pdu);
		break;
	case PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND:
		llcp_pdu_encode_reject_ext_ind(pdu, ctx->reject_ext_ind.reject_opcode,
					       ctx->reject_ext_ind.error_code);
		break;
#endif /* CONFIG_BT_CENTRAL */
	case PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP:
		llcp_pdu_encode_unknown_rsp(ctx, pdu);
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

static void rp_cr_check_instant_by_counter(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					   uint16_t event_counter, void *param)
{
	if (is_instant_reached_or_passed(ctx->data.conn_rate.instant, event_counter)) {
		/* The procedure is complete once the instant has passed and the
		 * new connection event parameters have been applied.
		 */
		cr_apply(conn, ctx);

		ctx->data.conn_rate.error = BT_HCI_ERR_SUCCESS;
		cr_ntf(conn, ctx);
		rp_cr_complete(conn, ctx);
	}
}

static void rp_cr_check_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				void *param)
{
	uint16_t event_counter = ull_conn_event_counter_at_prepare(conn);

	rp_cr_check_instant_by_counter(conn, ctx, evt, event_counter, param);
}

static void rp_cr_check_instant_rx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx,
						 uint8_t evt, void *param)
{
	uint16_t event_counter = ull_conn_event_counter(conn);

	rp_cr_check_instant_by_counter(conn, ctx, evt, event_counter, param);
}

static void rp_cr_send_unknown_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				   void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_CR_STATE_WAIT_TX_UNKNOWN_RSP;
	} else {
		rp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_UNKNOWN_RSP);

		/* A remote-initiated change that we decline was not requested by
		 * our host, so no host notification is generated.
		 */
		rp_cr_complete(conn, ctx);
	}
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_cr_send_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				     void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx) ||
	    (ull_tx_q_peek(&conn->tx_q) != NULL) || !ull_conn_lll_tx_queue_is_empty(conn)) {
		llcp_tx_pause_data(conn, LLCP_TX_QUEUE_PAUSE_DATA_CONN_UPD);
		ctx->state = RP_CR_STATE_WAIT_TX_CONN_RATE_IND;
	} else {
		/* Derive the instant at the time the indication is actually
		 * queued. The retained RX node (the request) is re-used for the
		 * host notification at the instant.
		 */
		cr_set_loc_instant(conn, ctx);
		rp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND);

		ctx->rx_opcode = PDU_DATA_LLCTRL_TYPE_UNUSED;
		ctx->state = RP_CR_STATE_WAIT_INSTANT;
		llcp_tx_resume_data(conn, LLCP_TX_QUEUE_PAUSE_DATA_CONN_UPD);
	}
}

static void rp_cr_send_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				      void *param)
{
	if (llcp_rr_ispaused(conn) || !llcp_tx_alloc_peek(conn, ctx)) {
		ctx->state = RP_CR_STATE_WAIT_TX_REJECT_EXT_IND;
	} else {
		rp_cr_tx(conn, ctx, PDU_DATA_LLCTRL_TYPE_REJECT_EXT_IND);

		/* A rejected, peer-initiated procedure makes no parameter change
		 * and was not started by the host, so no host notification is
		 * generated.
		 */
		rp_cr_complete(conn, ctx);
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void rp_cr_st_idle(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	struct pdu_data *pdu = (struct pdu_data *)param;

	switch (evt) {
	case RP_CR_EVT_CONN_RATE_IND:
#if defined(CONFIG_BT_CENTRAL)
		if (conn->lll.role == BT_HCI_ROLE_CENTRAL) {
			/* Only the Central may originate LL_CONNECTION_RATE_IND;
			 * one received as a Central is a protocol violation
			 * (Core 6.2, Vol 6, Part B, 5.1.32).
			 */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
			rp_cr_complete(conn, ctx);
			break;
		}
#endif /* CONFIG_BT_CENTRAL */
		llcp_pdu_decode_conn_rate_ind(ctx, pdu);

		/* Valid PDU? */
		if (cr_check_ind_parameters(conn, ctx)) {
			uint16_t event_counter = ull_conn_event_counter(conn);

			if (is_instant_not_passed(ctx->data.conn_rate.instant, event_counter)) {
				/* Keep RX node to use for NTF */
				llcp_rx_node_retain(ctx);

				ctx->state = RP_CR_STATE_WAIT_INSTANT;

				/* In case we only just received it in time */
				rp_cr_check_instant_rx_conn_rate_ind(conn, ctx, evt, param);
				break;
			}

			/* Instant already passed - terminate the connection */
			conn->llcp_terminate.reason_final = BT_HCI_ERR_INSTANT_PASSED;
			rp_cr_complete(conn, ctx);
		} else {
			/* Out-of-range or unsupported fields: signal back with
			 * LL_UNKNOWN_RSP without terminating the link (Core 6.2,
			 * Vol 6, Part B, 5.1.32).
			 */
			ctx->unknown_response.type = PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND;
			rp_cr_send_unknown_rsp(conn, ctx, evt, param);
		}
		break;
#if defined(CONFIG_BT_CENTRAL)
	case RP_CR_EVT_CONN_RATE_REQ:
		llcp_pdu_decode_conn_rate_req(ctx, pdu);

		if (!feature_shorter_conn_intervals(conn)) {
			/* The Shorter Connection Intervals feature is not usable on
			 * this connection: reject the peer's request.
			 */
			ctx->reject_ext_ind.reject_opcode =
				PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ;
			ctx->reject_ext_ind.error_code = BT_HCI_ERR_UNSUPP_REMOTE_FEATURE;
			rp_cr_send_reject_ext_ind(conn, ctx, evt, param);
		} else {
			uint8_t err = cr_select_ind_parameters(conn, ctx);

			if (err == BT_HCI_ERR_SUCCESS) {
				/* Acceptable request: keep the RX node for the
				 * host NTF and answer with LL_CONNECTION_RATE_IND.
				 */
				llcp_rx_node_retain(ctx);
				rp_cr_send_conn_rate_ind(conn, ctx, evt, param);
			} else {
				/* Unacceptable parameters: reject with the error
				 * code mandated by 5.1.33.
				 */
				ctx->reject_ext_ind.reject_opcode =
					PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ;
				ctx->reject_ext_ind.error_code = err;
				rp_cr_send_reject_ext_ind(conn, ctx, evt, param);
			}
		}
		break;
#endif /* CONFIG_BT_CENTRAL */
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_cr_st_wait_tx_unknown_rsp(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					 void *param)
{
	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_send_unknown_rsp(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

#if defined(CONFIG_BT_CENTRAL)
static void rp_cr_st_wait_tx_conn_rate_ind(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
					   void *param)
{
	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_send_conn_rate_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_cr_st_wait_tx_reject_ext_ind(struct ll_conn *conn, struct proc_ctx *ctx,
					    uint8_t evt, void *param)
{
	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_send_reject_ext_ind(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}
#endif /* CONFIG_BT_CENTRAL */

static void rp_cr_st_wait_instant(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt,
				  void *param)
{
	switch (evt) {
	case RP_CR_EVT_RUN:
		rp_cr_check_instant(conn, ctx, evt, param);
		break;
	default:
		/* Ignore other evts */
		break;
	}
}

static void rp_cr_execute_fsm(struct ll_conn *conn, struct proc_ctx *ctx, uint8_t evt, void *param)
{
	switch (ctx->state) {
	case RP_CR_STATE_IDLE:
		rp_cr_st_idle(conn, ctx, evt, param);
		break;
	case RP_CR_STATE_WAIT_TX_UNKNOWN_RSP:
		rp_cr_st_wait_tx_unknown_rsp(conn, ctx, evt, param);
		break;
#if defined(CONFIG_BT_CENTRAL)
	case RP_CR_STATE_WAIT_TX_CONN_RATE_IND:
		rp_cr_st_wait_tx_conn_rate_ind(conn, ctx, evt, param);
		break;
	case RP_CR_STATE_WAIT_TX_REJECT_EXT_IND:
		rp_cr_st_wait_tx_reject_ext_ind(conn, ctx, evt, param);
		break;
#endif /* CONFIG_BT_CENTRAL */
	case RP_CR_STATE_WAIT_INSTANT:
		rp_cr_st_wait_instant(conn, ctx, evt, param);
		break;
	default:
		/* Unknown state */
		LL_ASSERT_DBG(0);
		break;
	}
}

void llcp_rp_cr_rx(struct ll_conn *conn, struct proc_ctx *ctx, struct node_rx_pdu *rx)
{
	struct pdu_data *pdu = (struct pdu_data *)rx->pdu;

	switch (pdu->llctrl.opcode) {
	case PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_IND:
		rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_CONN_RATE_IND, pdu);
		break;
#if defined(CONFIG_BT_CENTRAL)
	case PDU_DATA_LLCTRL_TYPE_CONNECTION_RATE_REQ:
		rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_CONN_RATE_REQ, pdu);
		break;
#endif /* CONFIG_BT_CENTRAL */
	default:
		/* Invalid behaviour */
		/* Invalid PDU received so terminate connection */
		conn->llcp_terminate.reason_final = BT_HCI_ERR_LMP_PDU_NOT_ALLOWED;
		rp_cr_complete(conn, ctx);
		break;
	}
}

void llcp_rp_cr_run(struct ll_conn *conn, struct proc_ctx *ctx, void *param)
{
	rp_cr_execute_fsm(conn, ctx, RP_CR_EVT_RUN, param);
}
