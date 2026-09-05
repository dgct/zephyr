/*
 * Copyright (c) 2018-2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int lll_prepare_done(void *param);
int lll_done(void *param);
bool lll_is_done(void *param, bool *is_resume);
int lll_is_abort_cb(void *next, void *curr, lll_prepare_cb_t *resume_cb);
#if defined(CONFIG_BT_CTLR_USER_EXT)
/* True when the prepare with this LLL context belongs to a user event that
 * must never preempt a Bluetooth role; the preemptor is cancelled instead.
 */
bool lll_user_prepare_yields(void *param);
#endif /* CONFIG_BT_CTLR_USER_EXT */
void lll_abort_cb(struct lll_prepare_param *prepare_param, void *param);

uint32_t lll_event_offset_get(struct ull_hdr *ull);
uint32_t lll_preempt_calc(struct ull_hdr *ull, uint8_t ticker_id,
			  uint32_t ticks_at_event);

void lll_chan_set(uint32_t chan);

void lll_isr_tx_status_reset(void);
void lll_isr_rx_status_reset(void);
void lll_isr_tx_sub_status_reset(void);
void lll_isr_rx_sub_status_reset(void);
void lll_isr_status_reset(void);
void lll_isr_abort(void *param);
void lll_isr_done(void *param);
void lll_isr_cleanup(void *param);
void lll_isr_early_abort(void *param);
