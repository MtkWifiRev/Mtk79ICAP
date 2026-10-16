/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * Out-of-tree build shims for compiling upstream mt76 against older kernels.
 * Injected via KCFLAGS="-include ..." so the upstream sources stay untouched
 * and remain trivially rebaseable.
 *
 * Target: Ubuntu 24.04 HWE kernel 7.0.0-28-generic.
 */

#ifndef __MT76_COMPAT_MT7921_H
#define __MT76_COMPAT_MT7921_H

#include <linux/version.h>
#include <net/mac80211.h>

/*
 * ieee80211_txq_aql_pending() is a mac80211 helper newer than 7.0.0-28; the
 * kernel here only has ieee80211_txq_airtime_check(). mt76 gained the call in
 * commit 9a46d8d2 ("wifi: mt76: add PS buffering support for HW-managed TIM
 * drivers").
 *
 * The only caller (tx.c, mt76_txq_send_burst) is gated on
 * MT_DRV_HW_PS_BUFFERING, which is set by mt7915 and mt7996 only -- mt7921
 * (pci/sdio/usb) never sets it, so the call is unreachable in an mt7921-only
 * build and returning false is behaviourally identical.
 *
 * WARNING: this stub is NOT correct for an mt7915 or mt7996 build. If those
 * are ever enabled here, implement it properly or move to a kernel that has
 * the real helper.
 */
#ifndef MT76_HAVE_IEEE80211_TXQ_AQL_PENDING
static inline bool
ieee80211_txq_aql_pending(struct ieee80211_hw *hw, struct ieee80211_txq *txq)
{
	return false;
}
#endif

#endif /* __MT76_COMPAT_MT7921_H */
