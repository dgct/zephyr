/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * Copyright (c) 2020 Demant
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/dbuf.h"

#include "hal/ccm.h"

#include "pdu_df.h"
#include "lll/pdu_vendor.h"
#include "pdu.h"

#include "lll.h"
#include "lll/lll_df_types.h"
#include "lll_conn.h"

#include "ull_conn_internal.h"

#include "ll_feat.h"
#include "ll_settings.h"

#include <zephyr/bluetooth/hci_types.h>

#include "hal/debug.h"

#if defined(CONFIG_BT_CTLR_SET_HOST_FEATURE)
static uint64_t host_features;
static uint64_t host_features_page1;

uint8_t ll_set_host_feature(uint8_t bit_number, uint8_t bit_value)
{
	uint64_t feature;

	if (bit_number < 64) {
		/* Page 0 host-controllable feature bit */
		feature = BIT64(bit_number);
		if (!(feature & LL_FEAT_HOST_BIT_MASK)) {
			return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
		}
	} else {
		/* Page 1 host-controllable feature bit. Computing BIT64() with
		 * the absolute bit number would be undefined behaviour for
		 * bit_number >= 64, so store it relative to the page base.
		 */
		feature = BIT64(bit_number - 64);
		if (!(feature & LL_FEAT_PAGE1_HOST_BIT_MASK)) {
			return BT_HCI_ERR_UNSUPP_FEATURE_PARAM_VAL;
		}
	}

#if defined(CONFIG_BT_CONN)
	/* Check if the Controller has an established ACL */
	uint16_t conn_free_count = ll_conn_free_count_get();

	/* Check if any connection contexts where allocated */
	if (conn_free_count != CONFIG_BT_MAX_CONN) {
		uint16_t handle;

		/* Check if there are established connections */
		for (handle = 0U; handle < CONFIG_BT_MAX_CONN; handle++) {
			if (ll_connected_get(handle)) {
				return BT_HCI_ERR_CMD_DISALLOWED;
			}
		}
	}
#endif /* CONFIG_BT_CONN */

	/* Set or Clear the Host feature bit */
	if (bit_number < 64) {
		if (bit_value) {
			host_features |= feature;
		} else {
			host_features &= ~feature;
		}
	} else {
		if (bit_value) {
			host_features_page1 |= feature;
		} else {
			host_features_page1 &= ~feature;
		}
	}

	return BT_HCI_ERR_SUCCESS;
}

void ll_feat_reset(void)
{
	host_features = 0U;
	host_features_page1 = 0U;
}

uint64_t ll_feat_get(void)
{
	return LL_FEAT | (host_features & LL_FEAT_HOST_BIT_MASK);
}

uint64_t ll_feat_get_page1(void)
{
	return LL_FEAT_PAGE1 | (host_features_page1 & LL_FEAT_PAGE1_HOST_BIT_MASK);
}

#else /* !CONFIG_BT_CTLR_SET_HOST_FEATURE */
uint64_t ll_feat_get(void)
{
	return LL_FEAT;
}

uint64_t ll_feat_get_page1(void)
{
	return LL_FEAT_PAGE1;
}

#endif /* !CONFIG_BT_CTLR_SET_HOST_FEATURE */
