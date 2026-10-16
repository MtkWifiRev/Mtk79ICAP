// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * mt7921 ICAP (RBIST internal capture) — raw I/Q capture over
 * MCU_EXT_CMD(RF_TEST).
 *
 * See spectrum.h for the firmware trace this is built from. Two places
 * where mt7961 deliberately diverges from the mt7915 implementation:
 *
 *   1. No host-side dump polling loop. mt7915's firmware emits one event per
 *      DUMP_DATA command, so its driver re-issues 0x11 until the capture
 *      drains. mt7961's icapSpectrumDumpDataEvent loops internally until
 *      data_len == 0 (0x0095c88e: bnez s0,0x0095c7c4), so we issue 0x11 once
 *      and consume the resulting event stream.
 *
 *   2. cap_src and cap_stop_cycle were long thought dead (spectrum.h) - they
 *      are in fact read by icapSpectrumSetParam_rbist @ 0x0095c58a, missed
 *      for a long time because Ghidra truncated that function. The knobs
 *      are fully live.
 */

#include <linux/debugfs.h>
#include "mt7921.h"
#include "mcu.h"
#include "spectrum.h"
#include "iqstream.h"

static struct mt7921_spectrum *mt7921_spectrum_get(struct mt792x_dev *dev)
{
	return dev->spectrum;
}

/* Full power for the duration of a capture: runtime PM will otherwise sleep
 * the chip mid-sequence. Mirrors mt7921_tm_set() (testmode.c).
 */
static void mt7921_spectrum_pm_hold(struct mt792x_dev *dev)
{
	struct mt76_connac_pm *pm = &dev->pm;

	pm->enable = false;
	cancel_delayed_work_sync(&pm->ps_work);
	cancel_work_sync(&pm->wake_work);

	/* __mt792x_mcu_drv_pmctrl() is a bare macro - ((dev)->hif_ops->drv_own(dev))
	 * with no NULL check - and mt7921u's hif_ops supplies only mcu_init,
	 * init_reset and reset. drv_own/fw_own are PCIe/SDIO only, which is also
	 * why the runtime-pm and deep-sleep debugfs files return -EOPNOTSUPP on
	 * USB. Calling it unguarded jumps to NULL and oopses.
	 *
	 * mt7921_tm_set() in testmode.c has the same latent bug; it survives
	 * only because mt7921 testmode is never exercised over USB.
	 *
	 * The portable wake already happened in the caller via
	 * mt792x_mutex_acquire() -> mt76_connac_pm_wake().
	 */
	if (dev->hif_ops && dev->hif_ops->drv_own)
		__mt792x_mcu_drv_pmctrl(dev);
}

static void mt7921_spectrum_pm_release(struct mt792x_dev *dev)
{
	/* USB has no firmware/driver ownership callbacks.  As in device init,
	 * keep runtime PM disabled when stopping capture or leaving testmode.
	 */
	dev->pm.enable = !mt76_is_usb(&dev->mt76);
}

static void
mt7921_spectrum_fill_start(struct mt7921_spectrum *spec,
			   struct mt7921_rbist_cap_start *req, bool trigger)
{
	req->trigger		= cpu_to_le32(trigger);
	req->ring_cap_en	= cpu_to_le32(spec->ring_cap_en);
	req->trigger_event	= cpu_to_le32(spec->trigger_event);
	req->capture_node	= cpu_to_le32(spec->capture_node);
	req->capture_len	= cpu_to_le32(spec->capture_len);
	req->band_idx		= cpu_to_le32(spec->band_idx);
	req->bw			= cpu_to_le32(spec->bw);
	req->arch		= cpu_to_le32(spec->arch);
	req->phy_idx		= cpu_to_le32(spec->phy_idx);
	req->emi_start_addr	= cpu_to_le32(spec->emi_start_addr);
	req->emi_end_addr	= cpu_to_le32(spec->emi_end_addr);
	/* All fields are read by mt7961 firmware except pd_enable, fix_rx_gain,
	 * wifi_path and emi_msb_addr (spectrum.h). Values still follow MediaTek's
	 * semantics so a firmware that does read them gets something legal -
	 * bit_width 0 in particular is not a valid u4EnBitWidth (only
	 * CAP_32BITS/CAP_96BITS are).
	 */
	req->cap_stop_cycle	= cpu_to_le32(spec->cap_stop_cycle);
	req->bit_width		= cpu_to_le32(spec->bit_width);
	req->cap_source		= cpu_to_le32(spec->cap_src);
	req->pd_enable		= cpu_to_le32(spec->pd_enable);
	req->fix_rx_gain	= cpu_to_le32(spec->fix_rx_gain);
	req->wifi_path		= cpu_to_le32(spec->wifi_path);
}

static void
mt7921_spectrum_fill_dump(struct mt7921_spectrum *spec,
			  struct mt7921_rbist_dump_req *req)
{
	req->bank	= cpu_to_le32(spec->bank);
	req->bank_size	= cpu_to_le32(spec->bank_size);
	req->wf_num	= cpu_to_le32(spec->wf_num);
	req->iq_type	= cpu_to_le32(spec->iq_type);
}

/* Build and send one RF_TEST ICAP command.
 *
 * action must be 1 or the firmware drops the command at 0x00933dee. Only one
 * arm of the payload union is ever filled: on mt7915, filling both corrupted
 * capture_len and cap_stop_cycle (commit a6e9bc8).
 */
static int
mt7921_icap_send(struct mt792x_dev *dev, u32 func_idx, bool trigger)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_icap_ctrl req = {
		.action = 1,
		.icap_len = 120,
		.func_idx = cpu_to_le32(func_idx),
	};
	int ret;

	/* Every RF_TEST command needs FWOpMode == ICAP or the firmware drops it
	 * at 0x00933dd8 and the only trace is an RFTEST-E line in the fw_debug
	 * log - which is off by default, so the failure looks like silence.
	 * A module reload resets fw_mode, and forgetting to re-enter afterwards
	 * is the mistake this catches. Warn rather than refuse: the other modes
	 * (ICAP_OVERLAP, WIFISPECTRUM) are still worth experimenting with.
	 */
	if (spec->fw_mode != MT7921_TM_ICAP)
		dev_warn(dev->mt76.dev,
			 "icap: last requested mode is %u, not ICAP(%u) - firmware will likely reject this; run 'enter' first\n",
			 spec->fw_mode, MT7921_TM_ICAP);

	switch (func_idx) {
	case MT7921_ICAP_SET_PARAMETER:
		mt7921_spectrum_fill_start(spec, &req.param.start, trigger);
		break;
	case MT7921_ICAP_DUMP_DATA:
		mt7921_spectrum_fill_dump(spec, &req.param.dump);
		break;
	default:
		break;
	}

	/* Replies arrive as unsolicited ext events, so never wait here. */
	ret = mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(RF_TEST), &req,
				sizeof(req), false);

	/* Bring-up instrumentation: without hardware confirmation that the
	 * transport works at all, a silent failure is indistinguishable from
	 * a firmware that simply declines to answer. Drop once proven.
	 */
	dev_info(dev->mt76.dev,
		 "icap: RF_TEST subcmd 0x%02x action=%u len=%zu -> %d\n",
		 func_idx, req.action, sizeof(req), ret);

	return ret;
}

/* Build and send one SPECTRUM_CTRL (EXT 0x56) command.
 *
 * The ungated twin of mt7921_icap_send(). wifiSpectrumExtCmdHandler needs no
 * mode entry at all, so this deliberately does not consult spec->fw_mode and
 * does not warn about it - the whole point is to arm a capture with the radio
 * still up in NORMAL mode. See spectrum.h for why that matters.
 *
 * Both payload structures are sent every time. They are adjacent in the command
 * (+0x44 and +0x94) rather than alternatives, and the firmware reads only the
 * one its control word selects, so there is no mt7915-style union hazard here.
 */
static int
mt7921_spectrum_ctrl_send(struct mt792x_dev *dev, u32 control, bool trigger)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_spectrum_ctrl_req req = {
		.control = cpu_to_le32(control),
	};
	int ret;

	mt7921_spectrum_fill_start(spec, &req.start, trigger);
	mt7921_spectrum_fill_dump(spec, &req.dump);

	ret = mt76_mcu_send_msg(&dev->mt76, MT7921_MCU_CMD_SPECTRUM_CTRL, &req,
				sizeof(req), false);

	dev_info(dev->mt76.dev,
		 "icap: SPECTRUM_CTRL control=%u len=%zu -> %d (queued)\n",
		 control, sizeof(req), ret);

	return ret;
}

/* start/stop/status/dump go through whichever path spec->path selects, so the
 * two can be compared without reloading the driver.
 */
static int
mt7921_spectrum_set_param(struct mt792x_dev *dev, bool trigger)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);

	if (spec->path == MT7921_SPECTRUM_PATH_CTRL)
		return mt7921_spectrum_ctrl_send(dev,
						 MT7921_SPECTRUM_CTRL_SET_PARAM,
						 trigger);

	return mt7921_icap_send(dev, MT7921_ICAP_SET_PARAMETER, trigger);
}

static int mt7921_spectrum_send_dump(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);

	if (spec->path == MT7921_SPECTRUM_PATH_CTRL)
		return mt7921_spectrum_ctrl_send(dev,
						 MT7921_SPECTRUM_CTRL_DUMP,
						 false);

	return mt7921_icap_send(dev, MT7921_ICAP_DUMP_DATA, false);
}

static void mt7921_spectrum_reset_packets(struct mt7921_spectrum *spec)
{
	int i;

	spin_lock_bh(&spec->lock);
	spec->pkt_count = 0;
	spec->truncated = false;
	spec->end_reason = NULL;
	for (i = 0; i < ARRAY_SIZE(spec->pkt); i++)
		spec->pkt[i].valid = false;
	spin_unlock_bh(&spec->lock);
}

static int mt7921_spectrum_start(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);

	mt7921_spectrum_pm_hold(dev);
	mt7921_spectrum_reset_packets(spec);

	spin_lock_bh(&spec->lock);
	spec->running = true;
	spec->status.valid = false;
	spin_unlock_bh(&spec->lock);

	return mt7921_spectrum_set_param(dev, true);
}

static int mt7921_spectrum_stop(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	int ret;

	/* trigger == 0 takes icapSpectrumSetParam's stop branch (0x0095c622). */
	ret = mt7921_spectrum_set_param(dev, false);

	spin_lock_bh(&spec->lock);
	spec->running = false;
	spin_unlock_bh(&spec->lock);

	mt7921_spectrum_pm_release(dev);

	return ret;
}

/*
 * Issue DUMP_DATA once and wait for the firmware to drain the capture.
 *
 * Deliberately not a request/response pump: mt7961's firmware emits events
 * back to back until data_len == 0. We wait for that terminal packet, for the
 * ring to fill, or for the stream to go quiet.
 */
static int mt7921_spectrum_dump(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	u32 last_count, first_count, events, packets;
	bool ended;
	int ret;

	spin_lock_bh(&spec->lock);
	spec->running = true;
	/* event_count is cumulative across the driver's life, so remember where
	 * this dump started; only the delta says anything about this capture.
	 */
	first_count = spec->event_count;
	spin_unlock_bh(&spec->lock);

	ret = mt7921_spectrum_send_dump(dev);
	if (ret)
		goto out;

	for (;;) {
		spin_lock_bh(&spec->lock);
		last_count = spec->event_count;
		spin_unlock_bh(&spec->lock);

		/* Woken by the event handler; times out if the stream stalls. */
		wait_event_timeout(dev->mt76.mcu.wait,
				   !READ_ONCE(spec->running) ||
				   READ_ONCE(spec->event_count) != last_count,
				   MT7921_SPECTRUM_DUMP_TIMEOUT);

		spin_lock_bh(&spec->lock);
		if (!spec->running || spec->event_count == last_count ||
		    spec->pkt_count >= ARRAY_SIZE(spec->pkt)) {
			spin_unlock_bh(&spec->lock);
			break;
		}
		spin_unlock_bh(&spec->lock);
	}

	/* The firmware drives the dump loop itself and will keep emitting
	 * 0x43c-byte events until data_len hits 0. On the RF_TEST path a capture
	 * that never completed meant that never happened, and the resulting event
	 * flood saturated the USB transport until the bus was marked hung.
	 *
	 * More events than committed packets normally means the stream is
	 * repeating itself, and disarming is the only lever the host has - we
	 * cannot cancel a dump already in flight, because the MCU is inside its
	 * own loop and will not service another command until it ends.
	 *
	 * But a clean end marker is not a flood: SPECTRUM_CTRL terminates after a
	 * single data_len == 0 event, which is one event for zero packets and
	 * would trip the naive comparison every time. Only chase a stream that
	 * did not end on its own.
	 */
	spin_lock_bh(&spec->lock);
	events = spec->event_count - first_count;
	packets = spec->pkt_count;
	ended = spec->end_reason == MT7921_SPECTRUM_END_MARKER;
	spin_unlock_bh(&spec->lock);

	if (!ended && events > packets) {
		dev_warn(dev->mt76.dev,
			 "icap: %u events for %u packets, disarming capture\n",
			 events, packets);
		mt7921_spectrum_set_param(dev, false);
	}

out:
	spin_lock_bh(&spec->lock);
	spec->running = false;
	spin_unlock_bh(&spec->lock);

	return ret;
}

static int mt7921_spectrum_status(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);

	if (spec->path == MT7921_SPECTRUM_PATH_CTRL)
		return mt7921_spectrum_ctrl_send(dev,
						 MT7921_SPECTRUM_CTRL_STATUS,
						 false);

	return mt7921_icap_send(dev, MT7921_ICAP_GET_CAPTURE_STATUS, false);
}

/*
 * Switch the firmware RF-test op mode.
 *
 * rlmTestExtCmdFsmEventTestCtrl gates the ICAP dispatch behind two checks
 * before it ever looks at the action byte:
 *
 *   0x00933dc4: state == 4        else "Current state isn't RF_TEST mode"
 *   0x00933dd8: fw opmode == 2    else "Current FWOpMode isn't ICAP"
 *
 * opmode 2 is MT7921_TM_ICAP, so the sequence is TESTMODE then ICAP - the
 * two-step the firmware's own "Test: then same mode (%d), return for re-entry"
 * string describes. Sent over the CE testmode pipe, which is a separate
 * transport from the RF_TEST EXT command that carries ICAP itself.
 *
 * Deliberately not routed through mt7921_testmode_cmd(): that requires
 * nl80211 testmode plus monitor mode, neither of which the firmware cares
 * about here.
 */
static int mt7921_spectrum_set_mode(struct mt792x_dev *dev, u32 mode)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_rftest_cmd cmd = {
		.action = TM_SWITCH_MODE,
		.param0 = cpu_to_le32(mode),
	};
	int ret;

	if (mode != MT7921_TM_NORMAL)
		mt7921_spectrum_pm_hold(dev);

	ret = mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(TEST_CTRL), &cmd,
				sizeof(cmd), false);

	/* -> is the USB queueing result only; the firmware's verdict shows up in
	 * the fw_debug log, or later as an RF_TEST gate rejection.
	 */
	dev_info(dev->mt76.dev, "icap: TM_SWITCH_MODE %u -> %d (queued)\n",
		 mode, ret);

	if (!ret) {
		spin_lock_bh(&spec->lock);
		spec->fw_mode = mode;
		spin_unlock_bh(&spec->lock);
	}

	if (mode == MT7921_TM_NORMAL)
		mt7921_spectrum_pm_release(dev);

	return ret;
}

/* Snapshot the raw event so a misparse is debuggable from userspace. */
static void
mt7921_spectrum_record_event(struct mt7921_spectrum *spec, struct sk_buff *skb)
{
	struct mt76_connac2_mcu_rxd *rxd =
		(struct mt76_connac2_mcu_rxd *)skb->data;
	const __le32 *payload;
	int i, words;

	memset(&spec->last_event, 0, sizeof(spec->last_event));
	spec->last_event.valid		= true;
	spec->last_event.skb_len	= skb->len;
	spec->last_event.len		= le16_to_cpu(rxd->len);
	spec->last_event.pkt_type_id	= le16_to_cpu(rxd->pkt_type_id);
	spec->last_event.eid		= rxd->eid;
	spec->last_event.ext_eid	= rxd->ext_eid;
	spec->last_event.seq		= rxd->seq;
	spec->last_event.option		= rxd->option;

	if (skb->len <= sizeof(*rxd))
		return;

	spec->last_event.payload_len = skb->len - sizeof(*rxd);
	payload = (const __le32 *)(skb->data + sizeof(*rxd));
	words = min_t(int, spec->last_event.payload_len / sizeof(*payload),
		      ARRAY_SIZE(spec->last_event.words));
	for (i = 0; i < words; i++)
		spec->last_event.words[i] = le32_to_cpu(payload[i]);
}

/*
 * Terminate on a repeated payload.
 *
 * Ported from mt7915 (commit b48eded): when a capture never completes the
 * hardware stop-pointer stays zero and the firmware re-emits the same words
 * forever. A duplicated packet means the stream is going nowhere.
 */
static bool
mt7921_spectrum_is_stale(struct mt7921_spectrum *spec,
			 const struct mt7921_spectrum_pkt *new_pkt)
{
	const struct mt7921_spectrum_pkt *prev;

	if (!spec->pkt_count)
		return false;

	prev = &spec->pkt[(spec->pkt_count - 1) % ARRAY_SIZE(spec->pkt)];
	if (!prev->valid || prev->data_len != new_pkt->data_len)
		return false;

	return !memcmp(prev->data, new_pkt->data,
		       new_pkt->data_len * sizeof(new_pkt->data[0]));
}

/*
 * Record why the event stream stopped, and say so exactly once.
 *
 * The firmware keeps sending after we stop accepting, so this path is reached
 * thousands of times per stuck dump; logging on every call is bring-up bug 5
 * all over again (an unthrottled printk in the MCU RX path is itself enough to
 * stall the USB transport). Only the first, state-changing call prints.
 *
 * Which of the three reasons fires is the main diagnostic for a capture that
 * never completes, so it is also kept for debugfs.
 *
 * Caller holds spec->lock.
 */
static void
mt7921_spectrum_end_stream(struct mt792x_dev *dev, struct mt7921_spectrum *spec,
			   const char *reason)
{
	if (!spec->running)
		return;

	spec->running = false;
	spec->end_reason = reason;

	dev_info(dev->mt76.dev, "icap: dump stream ended: %s (%u packets)\n",
		 reason, spec->pkt_count);

	/* Put a sample of the samples in the kernel log.
	 *
	 * spectrum/dump is the real interface, but the window to read it is
	 * short and unforgiving: a capture that floods the bus gets the chip
	 * reset by usb_queue_reset_device(), and the re-probe tears down the
	 * debugfs directory and this ring with it. Roughly five seconds, which
	 * is less than it takes to type the command. dmesg survives that.
	 *
	 * One line, once per stream, so it cannot become bring-up bug 5.
	 */
	if (spec->pkt_count) {
		const struct mt7921_spectrum_pkt *p = &spec->pkt[0];
		u32 n = min_t(u32, p->data_len, 8);
		char line[160];
		int len = 0, i;

		for (i = 0; i < n; i++)
			len += scnprintf(line + len, sizeof(line) - len, " %d",
					 (s32)le32_to_cpu(p->data[i]));

		dev_info(dev->mt76.dev,
			 "icap: pkt0 bank=%u wf_cnt=%u smpl_cnt=%u data_len=%u:%s\n",
			 p->bank, p->wf_cnt, p->smpl_cnt, p->data_len, line);
	}
}

static void
mt7921_spectrum_dump_event(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_icap_evt_dump *evt;
	struct mt7921_spectrum_pkt *slot;
	u32 avail, data_len;

	if (skb->len < sizeof(struct mt76_connac2_mcu_rxd) + sizeof(*evt))
		return;

	evt = (struct mt7921_icap_evt_dump *)(skb->data +
					      sizeof(struct mt76_connac2_mcu_rxd));

	avail = (skb->len - sizeof(struct mt76_connac2_mcu_rxd) - sizeof(*evt)) /
		sizeof(__le32);
	data_len = le32_to_cpu(evt->data_len);
	if (data_len > avail)
		data_len = avail;
	if (data_len > MT7921_SPECTRUM_DATA_MAX)
		data_len = MT7921_SPECTRUM_DATA_MAX;

	spin_lock_bh(&spec->lock);

	/* data_len == 0 is the firmware's end-of-capture marker. */
	if (!data_len) {
		spec->drain_end_markers++;
		spec->drain_active = false;
		mt7921_spectrum_end_stream(dev, spec, MT7921_SPECTRUM_END_MARKER);
		goto out;
	}

	if (spec->pkt_count >= ARRAY_SIZE(spec->pkt)) {
		spec->drain_dropped++;
		spec->truncated = true;
		if (!spec->drain_active)
			mt7921_spectrum_end_stream(dev, spec, "packet ring full");
		goto out;
	}

	/* Fill the ring slot in place: a struct mt7921_spectrum_pkt is over 1 KB,
	 * far too large to stage on the kernel stack.
	 */
	slot = &spec->pkt[spec->pkt_count];
	slot->func_index	= le32_to_cpu(evt->func_index);
	slot->pkt_num		= le32_to_cpu(evt->pkt_num);
	slot->bank		= le32_to_cpu(evt->bank);
	slot->data_len		= data_len;
	slot->wf_cnt		= le32_to_cpu(evt->wf_cnt);
	slot->smpl_cnt		= le32_to_cpu(evt->smpl_cnt);
	memcpy(slot->data, evt->data, data_len * sizeof(__le32));

	/* Compare only after the slot is populated; on a duplicate we simply
	 * do not commit it by leaving pkt_count alone.
	 */
	if (!spec->drain_active && mt7921_spectrum_is_stale(spec, slot)) {
		mt7921_spectrum_end_stream(dev, spec, "duplicate packet");
		goto out;
	}

	slot->valid = true;
	spec->pkt_count++;

out:
	spin_unlock_bh(&spec->lock);
}

static void
mt7921_spectrum_status_event(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_icap_evt_status *evt;

	if (skb->len < sizeof(struct mt76_connac2_mcu_rxd) + sizeof(*evt))
		return;

	evt = (struct mt7921_icap_evt_status *)(skb->data +
						sizeof(struct mt76_connac2_mcu_rxd));

	spin_lock_bh(&spec->lock);
	spec->status.valid	= true;
	spec->status.func_index	= le32_to_cpu(evt->func_index);
	spec->status.cap_done	= le32_to_cpu(evt->cap_done);
	spin_unlock_bh(&spec->lock);
}

void mt7921_mcu_rx_spectrum_event(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	const __le32 *payload;
	u32 func_index;

	if (!spec)
		return;

	spin_lock_bh(&spec->lock);
	spec->event_count++;
	mt7921_spectrum_record_event(spec, skb);
	spin_unlock_bh(&spec->lock);

	if (skb->len < sizeof(struct mt76_connac2_mcu_rxd) + sizeof(__le32))
		goto wake;

	payload = (const __le32 *)(skb->data +
				   sizeof(struct mt76_connac2_mcu_rxd));
	func_index = le32_to_cpu(payload[0]);

	switch (func_index) {
	/* 0x11 is the RF_TEST path's subcmd echo; 0x02 is what the SPECTRUM_CTRL
	 * path emits, because icapSpectrumDumpDataEvent was called with mode 1.
	 * Same event body either way.
	 */
	case MT7921_ICAP_DUMP_DATA:
	case MT7921_SPECTRUM_FUNC_DUMP:
		mt7921_spectrum_dump_event(dev, skb);
		break;
	case MT7921_ICAP_GET_CAPTURE_STATUS:
		mt7921_spectrum_status_event(dev, skb);
		break;
	default:
		dev_dbg(dev->mt76.dev,
			"spectrum: unhandled func_index 0x%x\n", func_index);
		break;
	}

wake:
	wake_up(&dev->mt76.mcu.wait);
}

/* debugfs ------------------------------------------------------------- */

/* ---- ATE interface: the mt6769 icaptool / mt7915 prepare_rx sequence ----
 *
 * The mt6769 MtkICAPtool (the one implementation that ever produced real I/Q)
 * always runs this sequence inside testmode before arming a capture:
 *   set channel (RF_AT_FUNCID_CHNL_FREQ), SET_CHANNEL_BANDWIDTH,
 *   SET_RXOK_MATCH_RULE, SET_RX_PATH, STARTRX (RF_AT_COMMAND_STARTRX).
 * The connac2 equivalents are ATE_CTRL (EXT 0x3d, MCU_ATE_SET_TRX=0x1 /
 * MCU_ATE_SET_PHY_COUNT=0x11) and SET_RX_PATH (EXT 0x4e, which this driver
 * already uses for normal channel changes). Ported from the mt7915 ICAP
 * driver (mt7915_mcu_icap_set_trx/_set_phy_count/_refresh_channel).
 */
enum {
	TM_MAC_TX = 1,
	TM_MAC_RX,
	TM_MAC_TXRX,
	TM_MAC_TXRX_RXV,
	TM_MAC_RXV,
	TM_MAC_RX_RXV,
};

#define MT7921_SPECTRUM_ATE_SET_TRX	0x1
#define MT7921_SPECTRUM_ATE_SET_PHY_COUNT	0x11

struct mt7921_spectrum_tm_trx {
	u8 type;
	u8 enable;
	u8 band;
	u8 rsv;
} __packed;

struct mt7921_spectrum_tm_cfg {
	u8 enable;
	u8 band;
	u8 _rsv[2];
} __packed;

struct mt7921_spectrum_tm_cmd {
	u8 testmode_en;
	u8 param_idx;
	u8 _rsv[2];
	union {
		__le32 data;
		struct mt7921_spectrum_tm_trx trx;
		struct mt7921_spectrum_tm_cfg cfg;
		u8 test[72];
	} param;
} __packed;

static int
mt7921_spectrum_ate_trx(struct mt792x_dev *dev, int type, bool en)
{
	struct mt7921_spectrum_tm_cmd cmd = {
		.testmode_en = 1,
		.param_idx = MT7921_SPECTRUM_ATE_SET_TRX,
		.param.trx.type = type,
		.param.trx.enable = en,
		.param.trx.band = 0,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(ATE_CTRL), &cmd,
				 sizeof(cmd), false);
}

static int
mt7921_spectrum_ate_phycnt(struct mt792x_dev *dev, u8 control)
{
	struct mt7921_spectrum_tm_cmd cmd = {
		.testmode_en = 1,
		.param_idx = MT7921_SPECTRUM_ATE_SET_PHY_COUNT,
		.param.cfg.enable = control,
		.param.cfg.band = 0,
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(ATE_CTRL), &cmd,
				 sizeof(cmd), false);
}

static int
mt7921_spectrum_ate_chan(struct mt792x_dev *dev)
{
	/* chandef is current from the association; do NOT call
	 * mt76_update_channel here - it needs locks the debugfs writer
	 * does not hold (deadlocked the VM on 2026-09-06). */
	return mt7921_mcu_set_chan_info(&dev->phy, MCU_EXT_CMD(SET_RX_PATH));
}

/* MT7961 exposes AT functions through CE TEST_CTRL, not EXT ATE_CTRL.
 * A set is only queued; a query returns the firmware's index/data pair.
 */
static int
mt7921_spectrum_at(struct mt792x_dev *dev, bool query, u32 index, u32 value)
{
	struct mt7921_rftest_cmd cmd = {
		.action = query ? TM_QUERY_AT_CMD : TM_SET_AT_CMD,
		.param0 = cpu_to_le32(index),
		.param1 = cpu_to_le32(value),
	};
	struct mt7921_rftest_evt *evt;
	struct sk_buff *skb = NULL;
	int ret;

	if (!query)
		return mt76_mcu_send_msg(&dev->mt76, MCU_CE_CMD(TEST_CTRL),
					&cmd, sizeof(cmd), false);

	ret = mt76_mcu_send_and_get_msg(&dev->mt76, MCU_CE_CMD(TEST_CTRL),
				      &cmd, sizeof(cmd), true, &skb);
	if (!ret) {
		if (!skb || skb->len < sizeof(*evt)) {
			ret = -EPROTO;
		} else {
			evt = (void *)skb->data;
			dev_info(dev->mt76.dev,
				 "icap: AT query %u: index=0x%08x data=0x%08x\n",
				 index, le32_to_cpu(evt->param0),
				 le32_to_cpu(evt->param1));
			if (le32_to_cpu(evt->param0) != index)
				ret = -EPROTO;
		}
	}
	dev_kfree_skb(skb);
	return ret;
}

static void mt7921_spectrum_clear(struct mt7921_spectrum *spec)
{
	mt7921_spectrum_reset_packets(spec);
	spin_lock_bh(&spec->lock);
	spec->event_count = 0;
	spec->status.valid = false;
	spec->last_event.valid = false;
	/* A dump_begin whose end marker never came (e.g. the firmware
	 * did not enter the builder) would otherwise block the next
	 * dump_begin with -EBUSY until the module is reloaded.
	 */
	spec->drain_active = false;
	spec->running = false;
	spin_unlock_bh(&spec->lock);
}

/* Caller holds the mt792x mutex. */
static int mt7921_spectrum_dump_begin(struct mt792x_dev *dev)
{
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	int ret = 0;

	/* Asynchronous streaming must not queue the legacy dump waiter's
	 * automatic stop when the small debug history fills. */
	spin_lock_bh(&spec->lock);
	if (spec->drain_active || spec->pkt_count) {
		ret = -EBUSY;
	} else {
		spec->drain_active = true;
		spec->running = true;
		spec->end_reason = NULL;
	}
	spin_unlock_bh(&spec->lock);
	if (ret)
		return ret;

	ret = mt7921_spectrum_send_dump(dev);
	if (ret) {
		spin_lock_bh(&spec->lock);
		spec->drain_active = false;
		spec->running = false;
		spin_unlock_bh(&spec->lock);
	}

	return ret;
}

/* Entry points for the IQB2 stream control (iqstream.c): the same MCU
 * commands as the ctrl file's clear+start, dump_begin and stop.
 */
int mt7921_spectrum_iq_arm(struct mt792x_dev *dev)
{
	int ret;

	mt792x_mutex_acquire(dev);
	mt7921_spectrum_clear(dev->spectrum);
	ret = mt7921_spectrum_start(dev);
	mt792x_mutex_release(dev);

	return ret;
}

int mt7921_spectrum_iq_dump_begin(struct mt792x_dev *dev)
{
	int ret;

	mt792x_mutex_acquire(dev);
	ret = mt7921_spectrum_dump_begin(dev);
	mt792x_mutex_release(dev);

	return ret;
}

int mt7921_spectrum_iq_disarm(struct mt792x_dev *dev)
{
	int ret;

	mt792x_mutex_acquire(dev);
	ret = mt7921_spectrum_stop(dev);
	mt7921_spectrum_clear(dev->spectrum);
	mt792x_mutex_release(dev);

	return ret;
}

static ssize_t
mt7921_spectrum_ctrl_set(struct file *file, const char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	struct mt792x_dev *dev = file->private_data;
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	char buf[32];
	int ret = 0;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, user_buf, count))
		return -EFAULT;
	buf[count] = '\0';
	strim(buf);

	/* mt792x_mutex_acquire(), not a bare mutex_lock(): it also runs
	 * mt76_connac_pm_wake(), without which a runtime-suspended chip never
	 * sees the command. Every other mt7921 debugfs entry does this.
	 */
	mt792x_mutex_acquire(dev);

	if (!strncmp(buf, "at_set ", 7) || !strncmp(buf, "at_query ", 9)) {
		bool query = !strncmp(buf, "at_query ", 9);
		u32 index, value;
		char extra;

		if (sscanf(buf + (query ? 9 : 7), "%u %u %c",
			   &index, &value, &extra) != 2)
			ret = -EINVAL;
		else
			ret = mt7921_spectrum_at(dev, query, index, value);
	} else if (!strncmp(buf, "mode ", 5)) {
		u32 mode;

		if (kstrtou32(buf + 5, 0, &mode))
			ret = -EINVAL;
		else
			ret = mt7921_spectrum_set_mode(dev, mode);
	} else if (!strcmp(buf, "enter")) {
		/* The full two-step entry: TESTMODE then ICAP.
		 *
		 * The switches must not be issued back to back. Sent from one
		 * write they land ~250 us apart, and the second one is dropped:
		 * state becomes RF_TEST but FWOpMode stays TESTMODE, so every
		 * later RF_TEST command dies at the 0x00933dd8 gate with
		 * "Current FWOpMode isn't ICAP". Two shell echoes hundreds of
		 * ms apart is what was verified working; this reproduces that.
		 *
		 * A settle delay rather than a completion check because
		 * TM_SWITCH_MODE goes out with wait=false - see set_mode().
		 */
		ret = mt7921_spectrum_set_mode(dev, MT7921_TM_TESTMODE);
		if (!ret) {
			msleep(MT7921_SPECTRUM_MODE_SETTLE_MS);
			ret = mt7921_spectrum_set_mode(dev, MT7921_TM_ICAP);
			if (!ret)
				msleep(MT7921_SPECTRUM_MODE_SETTLE_MS);
		}
	} else if (!strcmp(buf, "leave")) {
		ret = mt7921_spectrum_set_mode(dev, MT7921_TM_NORMAL);
	} else if (!strcmp(buf, "start"))
		ret = mt7921_spectrum_start(dev);
	else if (!strcmp(buf, "stop"))
		ret = mt7921_spectrum_stop(dev);
	else if (!strcmp(buf, "status"))
		ret = mt7921_spectrum_status(dev);
	else if (!strcmp(buf, "dump_begin"))
		ret = mt7921_spectrum_dump_begin(dev);
	else if (!strcmp(buf, "dump"))
		ret = mt7921_spectrum_dump(dev);
	else if (!strcmp(buf, "ate_rx_on"))
		ret = mt7921_spectrum_ate_trx(dev, TM_MAC_RX_RXV, true);
	else if (!strcmp(buf, "ate_rx_off"))
		ret = mt7921_spectrum_ate_trx(dev, TM_MAC_RX_RXV, false);
	else if (!strcmp(buf, "ate_chan"))
		ret = mt7921_spectrum_ate_chan(dev);
	else if (!strcmp(buf, "ate_phycnt")) {
		ret = mt7921_spectrum_ate_phycnt(dev, 0);
		if (!ret)
			ret = mt7921_spectrum_ate_phycnt(dev, 1);
	} else if (!strcmp(buf, "clear")) {
		mt7921_spectrum_clear(spec);
	} else {
		ret = -EINVAL;
	}

	mt792x_mutex_release(dev);

	return ret ? ret : count;
}

static ssize_t
mt7921_spectrum_ctrl_get(struct file *file, char __user *user_buf,
			 size_t count, loff_t *ppos)
{
	struct mt792x_dev *dev = file->private_data;
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	char buf[768];
	int len;

	spin_lock_bh(&spec->lock);
	len = scnprintf(buf, sizeof(buf),
			"commands: enter leave mode <n> start stop status dump dump_begin clear ate_rx_on ate_rx_off ate_chan ate_phycnt\n"
			"AT: at_set <decimal index> <decimal value>; at_query <decimal index> <decimal value> (result in kernel log)\n"
			"modes: 0=normal 1=testmode 2=icap 3=icap_overlap 4=wifispectrum\n"
			"path=%u (0=rf_test needs 'enter', 1=spectrum_ctrl ungated)\n"
			"fw_mode=%u (2=icap required for rf_test; requested, not confirmed)\n"
			"running=%d packets=%u truncated=%d events=%u end_reason=%s\n"
			"capture_node=0x%08x capture_len=0x%x trigger_event=0x%x\n"
			"ring_cap_en=%u band_idx=%u bw=%u phy_idx=%u arch=%u\n"
			"wf_num=%u iq_type=%u bank=%u bank_size=%u\n"
			"cap_src=%u cap_stop_cycle=%u bit_width=%u\n",
			spec->path, spec->fw_mode,
			spec->running, spec->pkt_count, spec->truncated,
			spec->event_count, spec->end_reason ?: "none",
			spec->capture_node, spec->capture_len,
			spec->trigger_event, spec->ring_cap_en, spec->band_idx,
			spec->bw, spec->phy_idx, spec->arch,
			spec->wf_num, spec->iq_type, spec->bank,
			spec->bank_size,
			spec->cap_src, spec->cap_stop_cycle, spec->bit_width);
	spin_unlock_bh(&spec->lock);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static const struct file_operations mt7921_spectrum_ctrl_fops = {
	.write = mt7921_spectrum_ctrl_set,
	.read = mt7921_spectrum_ctrl_get,
	.open = simple_open,
	.llseek = default_llseek,
	.owner = THIS_MODULE,
};

static int mt7921_spectrum_dump_show(struct seq_file *s, void *data)
{
	struct mt792x_dev *dev = dev_get_drvdata(s->private);
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	int i, j;

	spin_lock_bh(&spec->lock);

	seq_printf(s, "packets=%u truncated=%d events=%u end_reason=%s\n",
		   spec->pkt_count, spec->truncated, spec->event_count,
		   spec->end_reason ?: "none");

	for (i = 0; i < spec->pkt_count && i < ARRAY_SIZE(spec->pkt); i++) {
		struct mt7921_spectrum_pkt *p = &spec->pkt[i];

		if (!p->valid)
			continue;

		seq_printf(s,
			   "packet=%d func_index=0x%x pkt_num=%u bank=%u data_len=%u wf_cnt=%u smpl_cnt=%u\n",
			   i, p->func_index, p->pkt_num, p->bank, p->data_len,
			   p->wf_cnt, p->smpl_cnt);

		for (j = 0; j < p->data_len; j++)
			seq_printf(s, "%d%c", (s32)le32_to_cpu(p->data[j]),
				   (j % 8 == 7) ? '\n' : ' ');
		if (p->data_len % 8)
			seq_puts(s, "\n");
	}

	spin_unlock_bh(&spec->lock);

	return 0;
}

static int mt7921_spectrum_status_show(struct seq_file *s, void *data)
{
	struct mt792x_dev *dev = dev_get_drvdata(s->private);
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);

	spin_lock_bh(&spec->lock);
	seq_printf(s, "running=%d events=%u packets=%u end_reason=%s\n",
		   spec->running, spec->event_count, spec->pkt_count,
		   spec->end_reason ?: "none");
	if (spec->status.valid)
		seq_printf(s, "func_index=0x%x cap_done=%u\n",
			   spec->status.func_index, spec->status.cap_done);
	else
		seq_puts(s, "no status event received\n");
	/* mt7915 lesson: cap_done is not a reliable success indicator. */
	seq_puts(s, "note: trust the dump path, not cap_done\n");
	spin_unlock_bh(&spec->lock);

	return 0;
}

static int mt7921_spectrum_event_show(struct seq_file *s, void *data)
{
	struct mt792x_dev *dev = dev_get_drvdata(s->private);
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	int i;

	spin_lock_bh(&spec->lock);

	if (!spec->last_event.valid) {
		seq_puts(s, "no event received\n");
		goto out;
	}

	seq_printf(s,
		   "skb_len=%u rxd_len=%u pkt_type_id=0x%x eid=0x%x ext_eid=0x%x seq=%u option=0x%x\n",
		   spec->last_event.skb_len, spec->last_event.len,
		   spec->last_event.pkt_type_id, spec->last_event.eid,
		   spec->last_event.ext_eid, spec->last_event.seq,
		   spec->last_event.option);
	seq_printf(s, "payload_len=%u\n", spec->last_event.payload_len);
	for (i = 0; i < ARRAY_SIZE(spec->last_event.words); i++)
		seq_printf(s, "  [%d] 0x%08x\n", i, spec->last_event.words[i]);

out:
	spin_unlock_bh(&spec->lock);

	return 0;
}

/* Binary, consuming snapshot: IQD1, packet count, cumulative drops, cumulative
 * end markers (all LE32), then six LE32 header fields + data for each packet.
 * Copy and dequeue under one lock. Events arriving after that point remain
 * queued, unlike userspace's racy read(dump); write(clear) sequence.
 * Each open owns its immutable buffer, including across short reads/unplug.
 */
struct mt7921_iq_drain {
	size_t len;
	u8 data[];
};

static int mt7921_iq_drain_open(struct inode *inode, struct file *file)
{
	struct mt792x_dev *dev = inode->i_private;
	struct mt7921_spectrum *spec = mt7921_spectrum_get(dev);
	struct mt7921_iq_drain *snap;
	__le32 *p;
	unsigned int i, count;
	size_t capacity = 16 + ARRAY_SIZE(spec->pkt) *
		(24 + MT7921_SPECTRUM_DATA_MAX * sizeof(__le32));

	snap = kvzalloc(sizeof(*snap) + capacity, GFP_KERNEL);
	if (!snap)
		return -ENOMEM;
	p = (__le32 *)snap->data;
	spin_lock_bh(&spec->lock);
	count = spec->pkt_count;
	*p++ = cpu_to_le32(0x31445149); /* IQD1 */
	*p++ = cpu_to_le32(count);
	*p++ = cpu_to_le32(spec->drain_dropped);
	*p++ = cpu_to_le32(spec->drain_end_markers);
	for (i = 0; i < count; i++) {
		struct mt7921_spectrum_pkt *pkt = &spec->pkt[i];

		*p++ = cpu_to_le32(pkt->func_index);
		*p++ = cpu_to_le32(pkt->pkt_num);
		*p++ = cpu_to_le32(pkt->bank);
		*p++ = cpu_to_le32(pkt->data_len);
		*p++ = cpu_to_le32(pkt->wf_cnt);
		*p++ = cpu_to_le32(pkt->smpl_cnt);
		memcpy(p, pkt->data, pkt->data_len * sizeof(*p));
		p += pkt->data_len;
		pkt->valid = false;
	}
	spec->pkt_count = 0;
	spin_unlock_bh(&spec->lock);
	snap->len = (u8 *)p - snap->data;
	file->private_data = snap;
	return 0;
}

static ssize_t mt7921_iq_drain_read(struct file *file, char __user *buf,
				  size_t count, loff_t *ppos)
{
	struct mt7921_iq_drain *snap = file->private_data;

	return simple_read_from_buffer(buf, count, ppos, snap->data, snap->len);
}

static int mt7921_iq_drain_release(struct inode *inode, struct file *file)
{
	kvfree(file->private_data);
	return 0;
}

static loff_t mt7921_iq_drain_llseek(struct file *file, loff_t off, int whence)
{
	struct mt7921_iq_drain *snap = file->private_data;

	return fixed_size_llseek(file, off, whence, snap->len);
}

static const struct file_operations mt7921_iq_drain_fops = {
	.owner = THIS_MODULE,
	.open = mt7921_iq_drain_open,
	.read = mt7921_iq_drain_read,
	.release = mt7921_iq_drain_release,
	.llseek = mt7921_iq_drain_llseek,
};

int mt7921_spectrum_init(struct mt792x_dev *dev, struct dentry *dir)
{
	struct mt7921_spectrum *spec;
	struct dentry *sdir;

	spec = devm_kzalloc(dev->mt76.dev, sizeof(*spec), GFP_KERNEL);
	if (!spec)
		return -ENOMEM;

	spin_lock_init(&spec->lock);
	init_waitqueue_head(&spec->wait);

	spec->capture_node = MT7921_SPECTRUM_DEF_CAPTURE_NODE;
	spec->capture_len = MT7921_SPECTRUM_DEF_CAPTURE_LEN;
	spec->trigger_event = MT7921_SPECTRUM_DEF_TRIGGER_EVENT;
	spec->bit_width = MT7921_SPECTRUM_DEF_BIT_WIDTH;
	spec->fw_mode = MT7921_TM_NORMAL;

	dev->spectrum = spec;

	sdir = debugfs_create_dir("spectrum", dir);
	if (IS_ERR(sdir))
		return PTR_ERR(sdir);

	/* 0 = RF_TEST (needs 'enter'), 1 = SPECTRUM_CTRL (ungated, radio stays up) */
	debugfs_create_u32("path", 0600, sdir, &spec->path);
	debugfs_create_u32("trigger_event", 0600, sdir, &spec->trigger_event);
	debugfs_create_u32("ring_cap_en", 0600, sdir, &spec->ring_cap_en);
	debugfs_create_u32("capture_node", 0600, sdir, &spec->capture_node);
	debugfs_create_u32("capture_len", 0600, sdir, &spec->capture_len);
	debugfs_create_u32("band_idx", 0600, sdir, &spec->band_idx);
	debugfs_create_u32("bw", 0600, sdir, &spec->bw);
	debugfs_create_u32("arch", 0600, sdir, &spec->arch);
	debugfs_create_u32("phy_idx", 0600, sdir, &spec->phy_idx);
	debugfs_create_u32("emi_start_addr", 0600, sdir, &spec->emi_start_addr);
	debugfs_create_u32("emi_end_addr", 0600, sdir, &spec->emi_end_addr);
	debugfs_create_u32("wf_num", 0600, sdir, &spec->wf_num);
	debugfs_create_u32("iq_type", 0600, sdir, &spec->iq_type);
	debugfs_create_u32("bank", 0600, sdir, &spec->bank);
	debugfs_create_u32("bank_size", 0600, sdir, &spec->bank_size);
	/* Present for parity with the vendor struct; mt7961 firmware ignores
	 * all of these - see the DEAD markings in spectrum.h.
	 */
	debugfs_create_u32("cap_src", 0600, sdir, &spec->cap_src);
	debugfs_create_u32("cap_stop_cycle", 0600, sdir, &spec->cap_stop_cycle);
	debugfs_create_u32("bit_width", 0600, sdir, &spec->bit_width);
	debugfs_create_u32("pd_enable", 0600, sdir, &spec->pd_enable);
	debugfs_create_u32("fix_rx_gain", 0600, sdir, &spec->fix_rx_gain);
	debugfs_create_u32("wifi_path", 0600, sdir, &spec->wifi_path);

	debugfs_create_file("ctrl", 0600, sdir, dev,
			    &mt7921_spectrum_ctrl_fops);
	debugfs_create_file("drain", 0400, sdir, dev, &mt7921_iq_drain_fops);
	debugfs_create_devm_seqfile(dev->mt76.dev, "dump", sdir,
				    mt7921_spectrum_dump_show);
	debugfs_create_devm_seqfile(dev->mt76.dev, "status", sdir,
				    mt7921_spectrum_status_show);
	debugfs_create_devm_seqfile(dev->mt76.dev, "event_debug", sdir,
				    mt7921_spectrum_event_show);

	return mt7921_iqs_init(dev, sdir);
}
