// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * IQB2 block stream: receive path, ring, and stream control. See iqstream.h
 * for the wire format.
 *
 * Producer: NAPI (mt7921_queue_rx_skb -> mt7921_iqs_rx), one block at a time.
 * Consumer: a single reader of /dev/mt7921_iq_<phy> (or, for the bring-up
 * harness, debugfs spectrum/iq_stream).
 *
 * The ring stores each block as header + payload, byte for byte as received,
 * so userspace can check sequence, source cursor and payload independently.
 * head and tail are free-running byte counts; the producer only writes into
 * [head, tail + size) and the consumer only reads [tail, head), so the
 * reader's copy_to_user runs without the spinlock.
 *
 * Stream control (firmware v62 mode 6) follows iqdma_run.py, which proved it
 * on hardware (experiments/mt7921-mcu-wfdma-tx0-20260929): open() arms the
 * DLPF capture and starts the firmware scheduler, release() stops it and
 * restores every register it touched. Any other MCU command (retune,
 * interface changes) first stops the builder, then the stream restarts with
 * a new generation, so the reader sees the discontinuity; if it cannot
 * restart (monitor mode gone), the stream ends. A chip reset or disconnect
 * ends the stream too: read() returns 0 once the ring is drained.
 */

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/random.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include "mt7921.h"
#include "spectrum.h"
#include "iqstream.h"
#include "iqstream_ioctl.h"

static unsigned int iq_stream_mb = 64;
module_param(iq_stream_mb, uint, 0444);
MODULE_PARM_DESC(iq_stream_mb, "IQB2 stream ring size in MiB (power of two, 0 disables)");

static unsigned int iq_block = 4096;
module_param(iq_block, uint, 0644);
MODULE_PARM_DESC(iq_block, "IQB2 payload bytes per block: 4096, 8192 or 14336");

static unsigned int iq_snap_ms;
module_param(iq_snap_ms, uint, 0644);
MODULE_PARM_DESC(iq_snap_ms, "80 MHz snapshot mode: pause between snapshots in ms (0: back to back)");

static unsigned int iq_ring_kb = 64;
module_param(iq_ring_kb, uint, 0644);
MODULE_PARM_DESC(iq_ring_kb, "ICAP capture ring in KiB: 64 (default; streaming needs firmware v63+), 16, or 256 (needs firmware v65, whose SYSRAM layout gives ICAP 0xe0200000-0xe023ffff; falls back to 64 otherwise)");

static bool iq_tap80;
module_param(iq_tap80, bool, 0644);
MODULE_PARM_DESC(iq_tap80, "40 MHz streams, experimental: 80 MS/s tap (u4BW 2) decimated by bit 17, 320 MB/s on the ring (laps it); default is dense bit-19 packing at 240 MB/s");

static bool iq_dense;
module_param(iq_dense, bool, 0644);
MODULE_PARM_DESC(iq_dense, "40/80 MHz: dense single-chain capture (DFE MODE 0x13, tap 12, or tap 13 for RX chain 1 via the tune flag CHAIN1): 40 MHz as DENSE40 at 160 MB/s instead of PACK96, 80 MHz as a continuous DENSE80 stream at 320 MB/s instead of snapshots (needs a reader that knows formats 5-8)");

static bool iq_pack32 = true;
module_param(iq_pack32, bool, 0644);
MODULE_PARM_DESC(iq_pack32, "ICAP En32b64b packing: one 20 MS/s stream at 160 MB/s (default); off = raw two-lane records at ~320 MB/s, which laps the ring");

static bool iq_fw_check = true;
module_param(iq_fw_check, bool, 0644);
MODULE_PARM_DESC(iq_fw_check, "Stream only on a patched I/Q WM firmware, version label iq_dma_vNN (default); 0 = on any firmware, for developing new builds: the stream writes fixed MCU addresses that are only valid in the I/Q builds");

#define MT7921_MCU_RXD_LEN	0x24
#define MT7921_RXD_EID_OFS	0x1c
#define MT7921_RXD_EXT_EID_OFS	0x20

/* Firmware v62 interface in MCU DLM (firmware/iq_live.c). */
#define IQ_MODE			0x02017340	/* 6 = TX0 stream, 3 = stop */
#define IQ_MODE_STREAM		6
#define IQ_MODE_STOP		3
#define IQ_GEN			0x020173a8
#define IQ_ACK			0x020173ac	/* = IQ_GEN once the builder exited */
#define IQ_TRIGGER_ORDER	0x02017310
#define IQ_TRIGGER_MARKER	0x02017314
#define IQ_CUSTOM_RANGE		0x020173c8
#define IQ_CTL			0x02109a00	/* mode-6 control block */
#define IQ_CTL_MAGIC		0x36445149	/* "IQD6" */
#define IQ_CTL_LATE		(IQ_CTL + 0x40)
#define IQ_CTL_MAX_LEAD		(IQ_CTL + 0x44)
#define IQ_CTL_STATE		(IQ_CTL + 0x30)	/* 1 running, 2 stopped */
#define IQ_CTL_STATE_BADCTL	0xf01		/* control block rejected */
/* SYSRAM layout (ROM gp 0x02003000): ICAP pool position (byte) and banks
 * (half-word, 2 KiB each). v64+ give ICAP position 0 with 128 banks, i.e.
 * 0xe0200000-0xe023ffff; stock firmware gives it no banks, so a ring there
 * overlaps PLE/PSE buffers.
 */
#define IQ_SYSRAM_ICAP_POS	0x020142d0
#define IQ_SYSRAM_ICAP_BANKS	0x02014cac
/* g_ucEnableSER: sustained DMA reads of the ICAP window trip a firmware SER
 * check whose recovery resets the MCU WFDMA. Masked only while streaming.
 */
#define IQ_SER_ENABLE		0x0202e08c
#define IQ_ICAP_SRC		0xe0200000
/* The firmware arms the capture with the range 0..0xffc (16 KiB, word
 * units); the end register is raised afterwards, live, for a larger ring.
 * The bank at IQ_ICAP_SRC is 256 KiB, but only its first 64 KiB stayed free
 * of foreign data (2026-10-09). Firmware v62 masks the producer pointer to
 * 12 bits in follow mode, so a 64 KiB stream needs v63.
 */
#define IQ_ICAP_RING_16K	0x4000
#define IQ_ICAP_END		0x8002109c
#define IQ_ICAP_END_16K		0xffc
#define IQ_ICAP_PACK		0x800210b8
#define IQ_ICAP_EN32B64B	BIT(17)	/* lane 0, every other sample, padded */
#define IQ_ICAP_EN96B128B	BIT(19)	/* lane 0 full rate + lane 1 half rate */
/* DFE capture mux (MT7996 dfe.h names; same addresses on mt7961). The dense
 * single-chain mode writes one sample per 32-bit word, four consecutive
 * samples per 16-byte record, no packer (BirriaSDR's setting, with tap 12
 * instead of their tap 4: tap 12 carries the channel filter, tap 4 aliases;
 * experiments/mt7921-dense-mode-20261014). Rate field m: 40 MHz << (3 - m),
 * so m = 3 for 40 MS/s and m = 2 for 80 MS/s.
 */
#define IQ_DFE_MODE		0x830ad440
#define IQ_DFE_MODE_CODE	GENMASK(7, 0)
#define IQ_DFE_MODE_DENSE	0x13
#define IQ_DFE_MUX		0x830ad448
#define IQ_DFE_MUX_KEEP		GENMASK(31, 28)
#define IQ_DFE_MUX_DENSE(m, tap) (BIT(23) | (m) << 18 | (tap) << 8)
/* Live dense taps come in pairs, even = chain 0, odd = chain 1; 0-9 have no
 * digital channel filter (alias), 12/13 do.
 */
#define IQ_DFE_TAP_CHAIN0	12
#define IQ_DFE_TAP_CHAIN1	13
/* UDMA RX aggregation for DENSE80 (8 KiB blocks at 320 MB/s): 32 KiB per
 * bulk transfer, the setting validated with iqdma_run.py --usb-agg 32.
 */
#define IQ_AGG_KIB		32
#define IQ_AGG_TO		0x40
#define IQ_AGG_PKTS		16
/* Receive gain hold (MT7921_IQ_IOC_SET_GAIN; MT7996 phy.h names, same
 * addresses here): per RX chain the PHY force word RXGAIN (sets A and B
 * drive the capture path; set C and the enable bits [2] are left alone) and
 * the A-die gain force on the chain's RF SPI port (1 = chain 0, 0 = chain 1).
 * Both are needed: the PHY force alone leaves the A-die gain free (-34 dB on
 * 5 GHz), the A-die force alone leaves the AGC acting on frames. Only the
 * chains the stream carries are held: holding the other one too nearly
 * doubles the blanking runs on the streamed chain (247 -> 480 per second).
 */
#define IQ_GAIN_CHAINS		2
#define IQ_PHY_RXGAIN(c)	(0x830000c4 + ((c) << 16))
#define IQ_PHY_RXGAIN_AB	0x00ffff03
#define IQ_PHY_RXGAIN_HOLD(lna, lpf) \
	(BIT(0) | BIT(1) | (lna) << 8 | (lpf) << 12 | (lna) << 16 | (lpf) << 20)
#define IQ_PHY_RXGAIN_FORCED	GENMASK(2, 0)
#define IQ_RF_GAIN_PORT(c)	((c) ? 0 : 1)
#define IQ_RF_GAIN		0x768
#define IQ_RF_GAIN_MASK		GENMASK(31, 23)
#define IQ_RF_GAIN_HOLD(lna, lpf) (BIT(31) | (lna) << 27 | (lpf) << 23)
/* 80 MHz on 2.4 GHz (experiments/mt7921-24g-80mhz-20261014). The vendor
 * firmware has no 2.4 GHz BW80 channel, so the driver sets a 40 MHz channel
 * and then writes the words in which a real 5 GHz BW80 channel differs from
 * it (BirriaSDR issue #1, vendor FUN_0093dbd4; measured on both): per chain
 * bank (RF SPI port 1 = chain 0, port 0 = chain 1) the analog filter class
 * 0x160 [31:30] and 0xb00/0xb04, the digital rate class 0x83081000 [27:24]
 * (latched by a pulse of 0x8300c008/0x8301c008 [31]), 0x830a0080 [7:4] = 6
 * (2.4 GHz, BW80) and the BW80 AGC table. Without them the 80 MS/s tap
 * repeats 40 MS/s samples and passes only +-20 MHz.
 */
#define IQ_W24_RF_FILTER	0x160
#define IQ_W24_RF_FILTER_MASK	GENMASK(31, 30)
#define IQ_W24_RF_FILTER_BW80	(2u << 30)
#define IQ_W24_RF_B00		0xb00
#define IQ_W24_RF_B00_MASK	GENMASK(29, 24)
#define IQ_W24_RF_B00_BW80	(0x20u << 24)
#define IQ_W24_RF_B04		0xb04
#define IQ_W24_RF_B04_MASK	(GENMASK(28, 24) | GENMASK(7, 6))
#define IQ_W24_RF_B04_BW80	(0x1fu << 24 | 2u << 6)
#define IQ_W24_RATE		0x83081000
#define IQ_W24_RATE_MASK	GENMASK(27, 24)
#define IQ_W24_RATE_BW80	(2u << 24)
#define IQ_W24_LATCH(c)		(0x8300c008 + ((c) << 16))
#define IQ_W24_A0080		0x830a0080
#define IQ_W24_A0080_MASK	GENMASK(7, 4)
#define IQ_W24_A0080_BW80	(6u << 4)
#define IQ_W24_AGC_TBL(c)	(0x830000dc + ((c) << 16))
#define IQ_W24_AGC_BW40		0x30	/* BW20 base + 0x30 / + 0x60 */
#define IQ_W24_AGC_BW80		0x60
#define IQ_W24_NRF		3
static const u16 iq_w24_agc_bw20[48] = {
	0x4d2, 0x4d2, 0x4d3, 0x4d4, 0x4d5, 0x4d7, 0x4da, 0x4de,
	0x4d2, 0x4d2, 0x4d3, 0x4d4, 0x4d5, 0x4d7, 0x4da, 0x4de,
	0x4d2, 0x4d3, 0x4d3, 0x4d5, 0x4d7, 0x4da, 0x4de, 0x4e4,
	0x4d3, 0x4d5, 0x4d7, 0x4da, 0x4df, 0x4e5, 0x4ee, 0x4fb,
	0x4d5, 0x4d8, 0x4dc, 0x4e1, 0x4e9, 0x4f4, 0x502, 0x514,
	0x4d8, 0x4db, 0x4e1, 0x4e8, 0x4f4, 0x502, 0x513, 0x528,
};
/* MCU WFDMA TX0 ring: idle boot state the stream must leave behind. */
#define IQ_TX0_BASE		0x54000300
#define IQ_TX0_BOOT_BASE	0x02109900

/*
 * The stream drives the patched WM firmware through fixed MCU memory
 * addresses (IQ_MODE, IQ_CTL, IQ_SER_ENABLE, ...) that hold unrelated data in
 * any other firmware, so it only starts on an I/Q build. Those carry the
 * label "iq_dma_vNN" in the image's version field (set by the build and the
 * patcher), which mt76 copies into wiphy->fw_version at every firmware load
 * ("iq_dma_v65-20260224110949"). Builds from IQ_FW_MIN on keep the interface
 * this driver uses; an incompatible one has to change the label prefix.
 */
#define IQ_FW_LABEL		"iq_dma_v"
#define IQ_FW_MIN		65

#define IQ_ARM_TIMEOUT_MS	2000
#define IQ_STOP_TIMEOUT_MS	5000
#define IQ_START_TIMEOUT_MS	100	/* builder accepts or rejects its control block */
/* Lets back-to-back MCU commands of one mac80211 reconfiguration (e.g. the
 * two sniffer commands of a retune) pass before the stream restarts.
 */
#define IQ_RESTART_DELAY_MS	30

struct mt7921_iqs_saved {
	u32 path, cap_src, trigger_event, capture_len, cap_stop_cycle, arch;
	u32 bw, band_idx, phy_idx, ring_cap_en, bit_width, capture_node;
	u32 trigger_order, custom_range, ser, pack, end;
	u32 dfe_mode, dfe_mux, wlcfg0, wlcfg1;
};

struct mt7921_iqs {
	spinlock_t lock;	/* head, tail and counters */
	struct mutex read_lock;	/* one reader at a time */
	wait_queue_head_t wait;
	struct kref ref;	/* device + open char device */

	/* Stream control; ctl_lock nests outside the mt792x mutex. */
	struct mutex ctl_lock;
	struct mt792x_dev *dev;	/* NULL once the device is gone */
	struct miscdevice misc;
	char name[32];
	bool misc_registered;
	bool opened;		/* char device held: stream wanted */
	bool fw_running;	/* stream set up, registers saved */
	/* The mode-6 builder owns the MCU: every MCU command sent meanwhile
	 * times out and forces a chip reset. hw_lock is innermost (taken
	 * from the MCU send path) and covers builder_live.
	 */
	struct mutex hw_lock;
	bool builder_live;
	bool builder_acked;
	struct delayed_work restart_work;
	u32 interrupts;		/* builder stopped for a foreign MCU command */
	u32 freq_khz;		/* set by MT7921_IQ_IOC_TUNE; 0 = mac80211's */
	u32 bw_mhz;		/* stream width: 20 or 40 (tune ioctl) */
	u32 chain;		/* dense formats: RX chain (tune flag CHAIN1) */
	u32 ring;		/* ICAP ring bytes of the running stream */
	u32 fmt;		/* MT7921_IQ_FMT_* of the running stream */
	/* A stream whose first cursor is 32 mod 128 bytes slips about half of
	 * its 128-byte DMA lines; such a generation is dropped and restarted.
	 */
	u32 align_gen;		/* last generation whose alignment was checked */
	u32 bad_gen;
	u32 realigns;
	u64 realign_drops;
	struct work_struct realign_work;
	/* 80 MHz: repeated frozen-ring snapshots instead of a stream */
	bool snap_mode;
	struct delayed_work snap_work;
	u32 snaps, snap_errors, snap_fail_run;
	u32 tunes;
	u32 last_cmd;
	bool ended;		/* no more blocks will come: read() drains, then EOF */
	atomic_t chip_reset;	/* set by the reset path, lock-free */
	u32 fw_gen;
	u32 starts;
	u32 fw_errors;
	u32 last_late;
	u32 last_max_lead;
	struct mt7921_iqs_saved saved;
	u32 fw_ver;		/* I/Q build of the last checked firmware load, 0 = none */
	/* Receive gain: what the opener asked for, and what the hardware has.
	 * gain_held (a mask of RX chains) stays set if a release could not
	 * reach the hardware (chip reset); the forces can survive that, so the
	 * next open releases them.
	 */
	struct mt7921_iq_gain gain;
	u32 gain_held;
	u32 gain_lna, gain_lpf;	/* code last forced */
	u32 gain_phy[IQ_GAIN_CHAINS], gain_rf[IQ_GAIN_CHAINS];	/* unforced words */
	u32 gain_errors;
	/* 80 MHz on 2.4 GHz: graft applied, and the words it replaced (as for
	 * the gain, kept across a release that could not reach the hardware).
	 */
	bool w24_applied;
	u32 w24_rf[IQ_GAIN_CHAINS][IQ_W24_NRF];
	u32 w24_rate, w24_a0080;
	u32 w24_errors;

	u8 *buf;
	u64 size;		/* power of two */
	u64 head;		/* bytes produced */
	u64 tail;		/* bytes consumed */

	bool enabled;
	u32 generation;
	u32 last_seq;
	bool have_seq;

	u64 blocks;
	u64 bytes;		/* payload bytes accepted */
	u64 drops;		/* whole blocks dropped: ring full */
	u64 malformed;
	u64 seq_gaps;		/* blocks missing according to seq */
	u64 seq_repeats;
	u64 gen_changes;
	u64 high_water;
	u64 first_ns;
	u64 last_ns;
};

static mt7921_rf_xfer_t mt7921_iqs_rf_xfer;

void mt7921_iqs_set_rf_xfer(mt7921_rf_xfer_t xfer)
{
	WRITE_ONCE(mt7921_iqs_rf_xfer, xfer);
}
EXPORT_SYMBOL_GPL(mt7921_iqs_set_rf_xfer);

static int mt7921_iqs_rf(struct mt792x_dev *dev, u32 port, u32 addr, u32 *val,
			 bool read)
{
	mt7921_rf_xfer_t xfer = READ_ONCE(mt7921_iqs_rf_xfer);

	return xfer ? xfer(dev, port, addr, val, read) : -EOPNOTSUPP;
}

static struct mt7921_iqs *mt7921_iqs_get(struct mt792x_dev *dev)
{
	return dev->spectrum ? dev->spectrum->iqs : NULL;
}

static void mt7921_iqs_copy_in(struct mt7921_iqs *iqs, struct sk_buff *skb,
			       int offset, u64 pos, u32 len)
{
	u32 at = pos & (iqs->size - 1);
	u32 first = min_t(u64, len, iqs->size - at);

	skb_copy_bits(skb, offset, iqs->buf + at, first);
	if (first < len)
		skb_copy_bits(skb, offset + first, iqs->buf, len - first);
}

bool mt7921_iqs_rx(struct mt792x_dev *dev, struct sk_buff *skb)
{
	struct mt7921_iqs *iqs = mt7921_iqs_get(dev);
	u8 tmp[MT7921_MCU_RXD_LEN + sizeof(struct mt7921_iqb2_hdr)];
	const struct mt7921_iqb2_hdr *hdr;
	u32 hdr_len, payload_len, rec, seq, gen;
	const u8 *p;
	u64 used, now;

	if (!iqs || !iqs->enabled || skb->len < sizeof(tmp))
		return false;

	p = skb_header_pointer(skb, 0, sizeof(tmp), tmp);
	if (!p || p[MT7921_RXD_EID_OFS] != MCU_EVENT_EXT ||
	    p[MT7921_RXD_EXT_EID_OFS] != MT7921_IQB2_EXT_EID)
		return false;

	hdr = (const struct mt7921_iqb2_hdr *)(p + MT7921_MCU_RXD_LEN);
	if (le32_to_cpu(hdr->magic) != MT7921_IQB2_MAGIC)
		return false;

	/* From here on the packet is ours, valid or not. */
	hdr_len = le16_to_cpu(hdr->hdr_len);
	payload_len = le32_to_cpu(hdr->payload_len);
	seq = le32_to_cpu(hdr->seq);
	gen = le32_to_cpu(hdr->generation);
	rec = ALIGN(hdr_len + payload_len, 4);
	now = ktime_get_ns();

	spin_lock_bh(&iqs->lock);

	if (le16_to_cpu(hdr->version) != MT7921_IQB2_VERSION ||
	    hdr_len < sizeof(*hdr) || (hdr_len & 3) ||
	    MT7921_MCU_RXD_LEN + hdr_len + payload_len > skb->len) {
		iqs->malformed++;
		goto out;
	}

	/* First block of a generation: a start 32 bytes into a 128-byte DMA
	 * line slips about half of all lines (in PACK96 each slip loses data),
	 * so such a generation is dropped and restarted. Kept for DENSE40/80
	 * too: their tone tests saw no slips, but none started at that
	 * alignment.
	 */
	if (payload_len && gen != iqs->align_gen) {
		iqs->align_gen = gen;
		if ((le32_to_cpu(hdr->cursor) & 127) == 32) {
			iqs->bad_gen = gen;
			iqs->realigns++;
			schedule_work(&iqs->realign_work);
		}
	}
	if (gen == iqs->bad_gen) {
		iqs->realign_drops++;
		goto out;
	}

	if (iqs->have_seq && gen != iqs->generation) {
		iqs->gen_changes++;
		iqs->have_seq = false;
	}
	if (iqs->have_seq) {
		u32 delta = seq - iqs->last_seq;

		if (!delta || delta > 0x80000000)
			iqs->seq_repeats++;
		else
			iqs->seq_gaps += delta - 1;
	}
	iqs->have_seq = true;
	iqs->last_seq = seq;
	iqs->generation = gen;

	used = iqs->head - iqs->tail;
	if (rec > iqs->size - used) {
		iqs->drops++;
		goto out;
	}

	/* Copied under the lock: NAPI and the MCU-queue worker can both
	 * deliver. The reader never touches [head, ...), so it only waits
	 * for the head/tail snapshot, never for this copy to finish.
	 */
	mt7921_iqs_copy_in(iqs, skb, MT7921_MCU_RXD_LEN, iqs->head,
			   hdr_len + payload_len);

	iqs->head += rec;
	iqs->blocks++;
	iqs->bytes += payload_len;
	if (!iqs->first_ns)
		iqs->first_ns = now;
	iqs->last_ns = now;
	used += rec;
	if (used > iqs->high_water)
		iqs->high_water = used;
	spin_unlock_bh(&iqs->lock);

	wake_up_interruptible(&iqs->wait);
	dev_kfree_skb(skb);
	return true;

out:
	spin_unlock_bh(&iqs->lock);
	dev_kfree_skb(skb);
	return true;
}

static ssize_t mt7921_iqs_read(struct file *file, char __user *ubuf,
			       size_t count, loff_t *ppos)
{
	struct mt7921_iqs *iqs = file->private_data;
	u64 head, tail;
	size_t n, at, first;
	int ret;

	if (mutex_lock_interruptible(&iqs->read_lock))
		return -ERESTARTSYS;

	for (;;) {
		spin_lock_bh(&iqs->lock);
		head = iqs->head;
		tail = iqs->tail;
		spin_unlock_bh(&iqs->lock);
		if (head != tail)
			break;
		if (READ_ONCE(iqs->ended)) {
			ret = 0;
			goto unlock;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto unlock;
		}
		ret = wait_event_interruptible(iqs->wait,
					       READ_ONCE(iqs->head) != tail ||
					       READ_ONCE(iqs->ended));
		if (ret)
			goto unlock;
	}

	n = min_t(u64, count, head - tail);
	at = tail & (iqs->size - 1);
	first = min_t(size_t, n, iqs->size - at);
	if (copy_to_user(ubuf, iqs->buf + at, first) ||
	    (first < n && copy_to_user(ubuf + first, iqs->buf, n - first))) {
		ret = -EFAULT;
		goto unlock;
	}

	spin_lock_bh(&iqs->lock);
	iqs->tail += n;
	spin_unlock_bh(&iqs->lock);
	ret = n;

unlock:
	mutex_unlock(&iqs->read_lock);
	return ret;
}

static __poll_t mt7921_iqs_poll(struct file *file, poll_table *wait)
{
	struct mt7921_iqs *iqs = file->private_data;

	poll_wait(file, &iqs->wait, wait);
	if (READ_ONCE(iqs->head) != READ_ONCE(iqs->tail))
		return EPOLLIN | EPOLLRDNORM;
	return READ_ONCE(iqs->ended) ? EPOLLHUP : 0;
}

static int mt7921_iqs_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return nonseekable_open(inode, file);
}

static const struct file_operations mt7921_iqs_fops = {
	.owner = THIS_MODULE,
	.open = mt7921_iqs_open,
	.read = mt7921_iqs_read,
	.poll = mt7921_iqs_poll,
};

static int mt7921_iqs_stats_show(struct seq_file *s, void *data)
{
	struct mt7921_iqs *iqs = s->private;
	u64 dt;

	spin_lock_bh(&iqs->lock);
	dt = iqs->last_ns - iqs->first_ns;
	seq_printf(s, "enabled=%d ring=%llu used=%llu high_water=%llu\n",
		   iqs->enabled, iqs->size, iqs->head - iqs->tail,
		   iqs->high_water);
	seq_printf(s, "blocks=%llu payload_bytes=%llu drops=%llu malformed=%llu\n",
		   iqs->blocks, iqs->bytes, iqs->drops, iqs->malformed);
	seq_printf(s, "seq_gaps=%llu seq_repeats=%llu gen_changes=%llu last_seq=%u generation=%u\n",
		   iqs->seq_gaps, iqs->seq_repeats, iqs->gen_changes,
		   iqs->last_seq, iqs->generation);
	seq_printf(s, "first_to_last_ns=%llu payload_kBps=%llu\n", dt,
		   dt ? div64_u64(iqs->bytes * 1000000ULL, dt) : 0);
	spin_unlock_bh(&iqs->lock);
	seq_printf(s, "dev=/dev/%s opened=%d fw_running=%d builder_live=%d ended=%d fw_gen=%u starts=%u fw_errors=%u\n",
		   iqs->name, READ_ONCE(iqs->opened), READ_ONCE(iqs->fw_running),
		   READ_ONCE(iqs->builder_live), READ_ONCE(iqs->ended),
		   READ_ONCE(iqs->fw_gen), READ_ONCE(iqs->starts),
		   READ_ONCE(iqs->fw_errors));
	seq_printf(s, "bw_mhz=%u chain=%u fmt=%u icap_ring=%u realigns=%u realign_drops=%llu snaps=%u snap_errors=%u\n",
		   READ_ONCE(iqs->bw_mhz), READ_ONCE(iqs->chain), READ_ONCE(iqs->fmt),
		   READ_ONCE(iqs->ring),
		   READ_ONCE(iqs->realigns),
		   READ_ONCE(iqs->realign_drops), READ_ONCE(iqs->snaps),
		   READ_ONCE(iqs->snap_errors));
	seq_printf(s, "freq_khz=%u tunes=%u interrupts=%u last_cmd=0x%08x last_late=%u last_max_lead=%u\n",
		   READ_ONCE(iqs->freq_khz), READ_ONCE(iqs->tunes),
		   READ_ONCE(iqs->interrupts), READ_ONCE(iqs->last_cmd),
		   READ_ONCE(iqs->last_late), READ_ONCE(iqs->last_max_lead));
	seq_printf(s, "gain_mode=%u gain_lna=%u gain_lpf=%u gain_held=0x%x held_lna=%u held_lpf=%u gain_errors=%u\n",
		   READ_ONCE(iqs->gain.mode), READ_ONCE(iqs->gain.lna),
		   READ_ONCE(iqs->gain.lpf), READ_ONCE(iqs->gain_held),
		   READ_ONCE(iqs->gain_lna), READ_ONCE(iqs->gain_lpf),
		   READ_ONCE(iqs->gain_errors));
	seq_printf(s, "w24_applied=%d w24_errors=%u\n", READ_ONCE(iqs->w24_applied),
		   READ_ONCE(iqs->w24_errors));
	seq_printf(s, "fw=%s iq_build=%u\n",
		   iqs->dev ? mt76_hw(iqs->dev)->wiphy->fw_version : "-",
		   READ_ONCE(iqs->fw_ver));

	return 0;
}

static int mt7921_iqs_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mt7921_iqs_stats_show, inode->i_private);
}

/* Empty the ring and zero the counters; only while no reader holds it. */
static int mt7921_iqs_reset_ring(struct mt7921_iqs *iqs)
{
	if (!mutex_trylock(&iqs->read_lock))
		return -EBUSY;
	spin_lock_bh(&iqs->lock);
	iqs->head = iqs->tail = 0;
	iqs->have_seq = false;
	iqs->blocks = iqs->bytes = iqs->drops = iqs->malformed = 0;
	iqs->seq_gaps = iqs->seq_repeats = iqs->gen_changes = 0;
	iqs->high_water = iqs->first_ns = iqs->last_ns = 0;
	spin_unlock_bh(&iqs->lock);
	mutex_unlock(&iqs->read_lock);

	return 0;
}

/* "1"/"0" enables/disables interception; "reset" empties the ring and zeroes
 * the counters (only while no reader holds the ring).
 */
static ssize_t mt7921_iqs_ctrl_write(struct file *file, const char __user *ubuf,
				     size_t count, loff_t *ppos)
{
	struct mt7921_iqs *iqs = file->private_data;
	char buf[16] = {};

	if (count >= sizeof(buf) || copy_from_user(buf, ubuf, count))
		return -EINVAL;
	strim(buf);

	if (!strcmp(buf, "1") || !strcmp(buf, "0")) {
		WRITE_ONCE(iqs->enabled, buf[0] == '1');
		return count;
	}
	if (strcmp(buf, "reset"))
		return -EINVAL;

	return mt7921_iqs_reset_ring(iqs) ?: count;
}

static const struct file_operations mt7921_iqs_ctrl_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mt7921_iqs_ctrl_write,
	.llseek = default_llseek,
};

static const struct file_operations mt7921_iqs_stats_fops = {
	.owner = THIS_MODULE,
	.open = mt7921_iqs_stats_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* debugfs spectrum/rf_spi (experiments, plan item G3): write "r PORT ADDR"
 * or "w PORT ADDR VALUE"; read shows the last transaction.
 */
static u32 rf_spi_last[4];	/* port, addr, value, -errno */

static ssize_t mt7921_iqs_rf_spi_write(struct file *file,
				       const char __user *ubuf,
				       size_t count, loff_t *ppos)
{
	struct mt7921_iqs *iqs = file->private_data;
	char buf[64] = {}, op;
	u32 port, addr, val = 0;
	int n, ret;

	if (count >= sizeof(buf) || copy_from_user(buf, ubuf, count))
		return -EINVAL;
	n = sscanf(buf, " %c %i %i %i", &op, &port, &addr, &val);
	if (n < 3 || (op != 'r' && op != 'w') || (op == 'w' && n < 4))
		return -EINVAL;
	mutex_lock(&iqs->ctl_lock);
	ret = iqs->dev ? mt7921_iqs_rf(iqs->dev, port, addr, &val, op == 'r')
		       : -ENODEV;
	mutex_unlock(&iqs->ctl_lock);
	rf_spi_last[0] = port;
	rf_spi_last[1] = addr;
	rf_spi_last[2] = val;
	rf_spi_last[3] = -ret;
	return ret ?: count;
}

static ssize_t mt7921_iqs_rf_spi_read(struct file *file, char __user *ubuf,
				      size_t count, loff_t *ppos)
{
	char buf[64];
	int len = scnprintf(buf, sizeof(buf), "%u 0x%03x 0x%08x %d\n",
			    rf_spi_last[0], rf_spi_last[1], rf_spi_last[2],
			    -(int)rf_spi_last[3]);

	return simple_read_from_buffer(ubuf, count, ppos, buf, len);
}

static const struct file_operations mt7921_iqs_rf_spi_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = mt7921_iqs_rf_spi_read,
	.write = mt7921_iqs_rf_spi_write,
	.llseek = default_llseek,
};

/* ---- stream control ---------------------------------------------------- */

static bool mt7921_iqs_hw_ok(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;

	return dev && !atomic_read(&iqs->chip_reset) &&
	       !atomic_read(&dev->mt76.bus_hung) &&
	       !test_bit(MT76_RESET, &dev->mphy.state) &&
	       !test_bit(MT76_REMOVED, &dev->mphy.state);
}

/* I/Q build number of the loaded firmware, from its version label; 0 if it
 * is not an I/Q build.
 */
static u32 mt7921_iqs_fw_version(struct mt792x_dev *dev)
{
	const char *v = mt76_hw(dev)->wiphy->fw_version;
	char num[8] = {};
	u32 ver;

	if (strncmp(v, IQ_FW_LABEL, strlen(IQ_FW_LABEL)))
		return 0;
	v += strlen(IQ_FW_LABEL);
	strscpy(num, v, min_t(size_t, sizeof(num), strcspn(v, "-") + 1));
	return kstrtou32(num, 10, &ver) ? 0 : ver;
}

/* Before anything touches the patch's MCU addresses: is the loaded firmware
 * an I/Q build? A chip reset reloads the firmware and its label, so this
 * runs at every stream start. Caller holds ctl_lock.
 */
static int mt7921_iqs_fw_check(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;

	iqs->fw_ver = mt7921_iqs_fw_version(dev);
	if (iqs->fw_ver >= IQ_FW_MIN)
		return 0;
	if (!iq_fw_check) {
		dev_warn_ratelimited(dev->mt76.dev,
				     "iq: firmware %s is not an I/Q build, streaming anyway (iq_fw_check=0)\n",
				     mt76_hw(dev)->wiphy->fw_version);
		return 0;
	}
	dev_warn_ratelimited(dev->mt76.dev,
			     "iq: firmware %s is not an I/Q build (needs %s%u or later); not streaming\n",
			     mt76_hw(dev)->wiphy->fw_version, IQ_FW_LABEL, IQ_FW_MIN);
	return -EOPNOTSUPP;
}

static bool mt7921_iqs_tx0_idle(struct mt792x_dev *dev)
{
	return mt76_rr(dev, IQ_TX0_BASE) == IQ_TX0_BOOT_BASE &&
	       mt76_rr(dev, IQ_TX0_BASE + 4) == 4 &&
	       mt76_rr(dev, IQ_TX0_BASE + 8) == 2 &&
	       mt76_rr(dev, IQ_TX0_BASE + 12) == 2;
}

static void mt7921_iqs_save(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	struct mt7921_spectrum *spec = dev->spectrum;
	struct mt7921_iqs_saved *sv = &iqs->saved;

	sv->path = spec->path;
	sv->cap_src = spec->cap_src;
	sv->trigger_event = spec->trigger_event;
	sv->capture_len = spec->capture_len;
	sv->cap_stop_cycle = spec->cap_stop_cycle;
	sv->arch = spec->arch;
	sv->bw = spec->bw;
	sv->band_idx = spec->band_idx;
	sv->phy_idx = spec->phy_idx;
	sv->ring_cap_en = spec->ring_cap_en;
	sv->bit_width = spec->bit_width;
	sv->capture_node = spec->capture_node;
	sv->trigger_order = mt76_rr(dev, IQ_TRIGGER_ORDER);
	sv->custom_range = mt76_rr(dev, IQ_CUSTOM_RANGE);
	sv->ser = mt76_rr(dev, IQ_SER_ENABLE);
	sv->pack = mt76_rr(dev, IQ_ICAP_PACK);
	sv->end = mt76_rr(dev, IQ_ICAP_END);
	sv->dfe_mode = mt76_rr(dev, IQ_DFE_MODE);
	sv->dfe_mux = mt76_rr(dev, IQ_DFE_MUX);
	sv->wlcfg0 = mt76_rr(dev, MT_UDMA_WLCFG_0);
	sv->wlcfg1 = mt76_rr(dev, MT_UDMA_WLCFG_1);
}

static void mt7921_iqs_restore(struct mt7921_iqs *iqs, bool hw)
{
	struct mt792x_dev *dev = iqs->dev;
	struct mt7921_spectrum *spec = dev->spectrum;
	struct mt7921_iqs_saved *sv = &iqs->saved;

	spec->path = sv->path;
	spec->cap_src = sv->cap_src;
	spec->trigger_event = sv->trigger_event;
	spec->capture_len = sv->capture_len;
	spec->cap_stop_cycle = sv->cap_stop_cycle;
	spec->arch = sv->arch;
	spec->bw = sv->bw;
	spec->band_idx = sv->band_idx;
	spec->phy_idx = sv->phy_idx;
	spec->ring_cap_en = sv->ring_cap_en;
	spec->bit_width = sv->bit_width;
	spec->capture_node = sv->capture_node;
	if (!hw)
		return;
	mt76_wr(dev, IQ_SER_ENABLE, sv->ser);
	mt76_wr(dev, IQ_ICAP_PACK, sv->pack);
	mt76_wr(dev, IQ_ICAP_END, sv->end);
	mt76_wr(dev, IQ_DFE_MODE, sv->dfe_mode);
	mt76_wr(dev, IQ_DFE_MUX, sv->dfe_mux);
	mt76_wr(dev, MT_UDMA_WLCFG_0, sv->wlcfg0);
	mt76_wr(dev, MT_UDMA_WLCFG_1, sv->wlcfg1);
	mt76_wr(dev, IQ_TRIGGER_ORDER, sv->trigger_order);
	mt76_wr(dev, IQ_CUSTOM_RANGE, sv->custom_range);
}

/* Phase 1 DLPF capture: ring at IQ_ICAP_SRC, free-running. The dump
 * builder dereferences the ICAP context this creates, so it is armed even
 * though mode 6 reads the ring by DMA.
 */
static void mt7921_iqs_set_dlpf(struct mt7921_spectrum *spec, u32 tap_bw)
{
	spec->path = MT7921_SPECTRUM_PATH_CTRL;
	spec->cap_src = 0;
	spec->trigger_event = 0xffffffff;
	spec->capture_len = IQ_ICAP_RING_16K;
	spec->cap_stop_cycle = 1024;
	spec->arch = 0;
	/* u4BW selects the tap rate, independent of the channel filter:
	 * 0/1 -> 40 MS/s per lane, 2 -> 80 MS/s (measured 2026-10-07).
	 */
	spec->bw = tap_bw;
	spec->band_idx = 0;
	spec->phy_idx = 0;
	spec->ring_cap_en = 1;
	spec->bit_width = 0;
	spec->capture_node = 0x17141006;
}

static int mt7921_iqs_wait_armed(struct mt792x_dev *dev)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(IQ_ARM_TIMEOUT_MS);

	do {
		if (mt76_rr(dev, 0x80021098) == 0 &&
		    mt76_rr(dev, IQ_ICAP_END) == IQ_ICAP_END_16K &&
		    (mt76_rr(dev, 0x80021090) & 0x4f1) == 0x4f1 &&
		    mt76_rr(dev, IQ_TRIGGER_MARKER) == 1)
			return 0;
		usleep_range(5000, 6000);
	} while (time_before(jiffies, timeout));

	return -ETIMEDOUT;
}

static u32 mt7921_iqs_ring_bytes(struct mt792x_dev *dev)
{
	if (iq_ring_kb == 16)
		return IQ_ICAP_RING_16K;
	if (iq_ring_kb == 256) {
		/* Only where ICAP owns the memory (firmware v64+ layout). */
		if ((mt76_rr(dev, IQ_SYSRAM_ICAP_POS) & 0xff) == 0 &&
		    (mt76_rr(dev, IQ_SYSRAM_ICAP_BANKS) & 0xffff) >= 128)
			return 0x40000;
		dev_warn_once(dev->mt76.dev,
			      "iq: iq_ring_kb=256 needs the v64+ SYSRAM layout; using 64 KiB\n");
	}
	return 0x10000;
}

/* Widen the armed capture range to the stream's ring (live; the producer
 * pointer continues past the old end on its next lap).
 */
static int mt7921_iqs_set_ring(struct mt792x_dev *dev, u32 ring)
{
	u32 end = ring / 4 - 4;

	mt76_wr(dev, IQ_ICAP_END, end);
	return mt76_rr(dev, IQ_ICAP_END) == end ? 0 : -EIO;
}

/* Switch the armed capture to the dense single-chain layout at rate m (live,
 * like the packing bits). The next arm rewrites both registers.
 */
static int mt7921_iqs_set_dense(struct mt792x_dev *dev, u32 m, u32 tap)
{
	u32 mode = (mt76_rr(dev, IQ_DFE_MODE) & ~IQ_DFE_MODE_CODE) |
		   BIT(31) | IQ_DFE_MODE_DENSE;
	u32 mux = (mt76_rr(dev, IQ_DFE_MUX) & IQ_DFE_MUX_KEEP) |
		  IQ_DFE_MUX_DENSE(m, tap);

	mt76_wr(dev, IQ_DFE_MODE, mode);
	mt76_wr(dev, IQ_DFE_MUX, mux);
	if (mt76_rr(dev, IQ_DFE_MODE) != mode || mt76_rr(dev, IQ_DFE_MUX) != mux)
		return -EIO;
	/* Let the writer overwrite the ring once in the new layout (64 KiB
	 * at 160 MB/s is 410 us) before the builder follows it.
	 */
	usleep_range(1000, 2000);
	return 0;
}

/* UDMA RX aggregation on (DENSE80) or back to the saved value. Register
 * writes only: also used from the MCU send path.
 */
static void mt7921_iqs_set_agg(struct mt7921_iqs *iqs, bool on)
{
	struct mt792x_dev *dev = iqs->dev;
	struct mt7921_iqs_saved *sv = &iqs->saved;

	if (!on) {
		mt76_wr(dev, MT_UDMA_WLCFG_0, sv->wlcfg0);
		mt76_wr(dev, MT_UDMA_WLCFG_1, sv->wlcfg1);
		return;
	}
	mt76_wr(dev, MT_UDMA_WLCFG_0,
		(sv->wlcfg0 & ~(MT_WL_RX_AGG_LMT | MT_WL_RX_AGG_TO)) |
		FIELD_PREP(MT_WL_RX_AGG_LMT, IQ_AGG_KIB) |
		FIELD_PREP(MT_WL_RX_AGG_TO, IQ_AGG_TO));
	mt76_wr(dev, MT_UDMA_WLCFG_1,
		(sv->wlcfg1 & ~MT_WL_RX_AGG_PKT_LMT) |
		FIELD_PREP(MT_WL_RX_AGG_PKT_LMT, IQ_AGG_PKTS));
}

/* ---- receive gain -------------------------------------------------------- */

/* Hand both chains back to the AGC: the PHY forces first, the A-die last.
 * Register and RF SPI accesses only, no MCU command. On failure gain_held
 * stays set, so a later release retries. Caller holds ctl_lock.
 */
static int mt7921_iqs_gain_release(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	int c, err, ret = 0;
	u32 val;

	if (!iqs->gain_held)
		return 0;
	for (c = 0; c < IQ_GAIN_CHAINS; c++) {
		if (!(iqs->gain_held & BIT(c)))
			continue;
		mt76_wr(dev, IQ_PHY_RXGAIN(c), iqs->gain_phy[c]);
		if (mt76_rr(dev, IQ_PHY_RXGAIN(c)) != iqs->gain_phy[c])
			ret = -EIO;
	}
	for (c = 0; c < IQ_GAIN_CHAINS; c++) {
		if (!(iqs->gain_held & BIT(c)))
			continue;
		val = iqs->gain_rf[c];
		err = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c), IQ_RF_GAIN, &val,
				    false);
		ret = ret ?: err;
	}
	if (ret) {
		iqs->gain_errors++;
		dev_warn(dev->mt76.dev, "iq: gain release failed: %d\n", ret);
		return ret;
	}
	iqs->gain_held = 0;
	return 0;
}

/* RX chains the running stream carries: the dense formats one, PACK32 lane
 * 0 (chain 0), the others both lanes.
 */
static u32 mt7921_iqs_gain_chains(struct mt7921_iqs *iqs)
{
	switch (iqs->fmt) {
	case MT7921_IQ_FMT_DENSE40:
	case MT7921_IQ_FMT_DENSE80:
	case MT7921_IQ_FMT_PACK32:
	case MT7921_IQ_FMT_PACK32_40:
		return BIT(0);
	case MT7921_IQ_FMT_DENSE40_C1:
	case MT7921_IQ_FMT_DENSE80_C1:
		return BIT(1);
	default:
		return BIT(0) | BIT(1);
	}
}

/* Hold the chains in mask at (lna, lpf): the A-die forces first, then the
 * PHY force sets A and B, each read back. The unforced words are saved on
 * the first hold; a later hold of the same chains only changes the code.
 * Caller holds ctl_lock.
 */
static int mt7921_iqs_gain_hold(struct mt7921_iqs *iqs, u32 mask, u32 lna,
				u32 lpf)
{
	struct mt792x_dev *dev = iqs->dev;
	u32 phy[IQ_GAIN_CHAINS] = {}, rf[IQ_GAIN_CHAINS] = {}, val, chk;
	int c, ret;

	if (iqs->gain_held && iqs->gain_held != mask) {
		ret = mt7921_iqs_gain_release(iqs);
		if (ret)
			return ret;
	}
	if (!iqs->gain_held) {
		for (c = 0; c < IQ_GAIN_CHAINS; c++) {
			if (!(mask & BIT(c)))
				continue;
			phy[c] = mt76_rr(dev, IQ_PHY_RXGAIN(c));
			ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c), IQ_RF_GAIN,
					    &rf[c], true);
			if (ret)
				return ret;
			/* Somebody else forces the gain (debugfs, gain.py). */
			if ((phy[c] & IQ_PHY_RXGAIN_FORCED) || (rf[c] & BIT(31)))
				return -EBUSY;
		}
		for (c = 0; c < IQ_GAIN_CHAINS; c++) {
			iqs->gain_phy[c] = phy[c];
			iqs->gain_rf[c] = rf[c];
		}
		iqs->gain_held = mask;	/* release restores these from now on */
	}
	for (c = 0; c < IQ_GAIN_CHAINS; c++) {
		if (!(mask & BIT(c)))
			continue;
		val = (iqs->gain_rf[c] & ~IQ_RF_GAIN_MASK) |
		      IQ_RF_GAIN_HOLD(lna, lpf);
		ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c), IQ_RF_GAIN, &val,
				    false);
		if (!ret)
			ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c), IQ_RF_GAIN,
					    &chk, true);
		if (!ret && chk != val)
			ret = -EIO;
		if (ret)
			goto fail;
	}
	for (c = 0; c < IQ_GAIN_CHAINS; c++) {
		if (!(mask & BIT(c)))
			continue;
		val = (iqs->gain_phy[c] & ~IQ_PHY_RXGAIN_AB) |
		      IQ_PHY_RXGAIN_HOLD(lna, lpf);
		mt76_wr(dev, IQ_PHY_RXGAIN(c), val);
		if (mt76_rr(dev, IQ_PHY_RXGAIN(c)) != val) {
			ret = -EIO;
			goto fail;
		}
	}
	iqs->gain_lna = lna;
	iqs->gain_lpf = lpf;
	return 0;

fail:
	iqs->gain_errors++;
	dev_warn(dev->mt76.dev, "iq: gain hold LNA %u LPF %u failed: %d\n",
		 lna, lpf, ret);
	mt7921_iqs_gain_release(iqs);
	return ret;
}

/* ---- 80 MHz on 2.4 GHz ---------------------------------------------------- */

static const u32 iq_w24_rf_addr[IQ_W24_NRF] = {
	IQ_W24_RF_FILTER, IQ_W24_RF_B00, IQ_W24_RF_B04,
};
static const u32 iq_w24_rf_mask[IQ_W24_NRF] = {
	IQ_W24_RF_FILTER_MASK, IQ_W24_RF_B00_MASK, IQ_W24_RF_B04_MASK,
};
static const u32 iq_w24_rf_bw80[IQ_W24_NRF] = {
	IQ_W24_RF_FILTER_BW80, IQ_W24_RF_B00_BW80, IQ_W24_RF_B04_BW80,
};

/* Is the stream an 80 MHz one on 2.4 GHz (a 40 MHz channel plus graft)? */
static bool mt7921_iqs_wide24(struct mt7921_iqs *iqs)
{
	return iqs->bw_mhz == 80 && iqs->freq_khz && iqs->freq_khz < 3000000;
}

static void mt7921_iqs_w24_agc(struct mt792x_dev *dev, u32 add)
{
	int c, i;

	/* The firmware waits 1 us between entries. */
	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		for (i = 0; i < ARRAY_SIZE(iq_w24_agc_bw20); i++) {
			mt76_wr(dev, IQ_W24_AGC_TBL(c),
				BIT(31) | i << 16 | (iq_w24_agc_bw20[i] + add));
			udelay(1);
		}
}

/* Latch the rate class: pulse [31] of both chains' word, as the firmware. */
static void mt7921_iqs_w24_latch(struct mt792x_dev *dev)
{
	int c;

	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		mt76_set(dev, IQ_W24_LATCH(c), BIT(31));
	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		mt76_clear(dev, IQ_W24_LATCH(c), BIT(31));
}

/* Put back the 40 MHz channel's words. Register and RF SPI accesses only.
 * On failure w24_applied stays set and a later restore retries. Caller
 * holds ctl_lock.
 */
static int mt7921_iqs_w24_restore(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	int c, k, err, ret = 0;
	u32 val;

	if (!iqs->w24_applied)
		return 0;
	mt7921_iqs_w24_agc(dev, IQ_W24_AGC_BW40);
	mt76_wr(dev, IQ_W24_A0080, iqs->w24_a0080);
	mt76_wr(dev, IQ_W24_RATE, iqs->w24_rate);
	mt7921_iqs_w24_latch(dev);
	if (mt76_rr(dev, IQ_W24_RATE) != iqs->w24_rate)
		ret = -EIO;
	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		for (k = IQ_W24_NRF - 1; k >= 0; k--) {
			val = iqs->w24_rf[c][k];
			err = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c),
					    iq_w24_rf_addr[k], &val, false);
			ret = ret ?: err;
		}
	if (ret) {
		iqs->w24_errors++;
		dev_warn(dev->mt76.dev, "iq: 2.4 GHz 80 MHz restore failed: %d\n", ret);
		return ret;
	}
	iqs->w24_applied = false;
	return 0;
}

/* Turn the tuned 2.4 GHz 40 MHz channel into an 80 MHz one: the analog
 * filter class first, then the digital rate class, the 2.4 GHz BW word and
 * the AGC table. Before the stream starts, after the arm. Caller holds
 * ctl_lock.
 */
static int mt7921_iqs_w24_apply(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	u32 val, chk;
	int c, k, ret;

	if (iqs->w24_applied)
		return -EBUSY;		/* a stop without restore: caller bug */
	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		for (k = 0; k < IQ_W24_NRF; k++) {
			ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c),
					    iq_w24_rf_addr[k], &iqs->w24_rf[c][k],
					    true);
			if (ret)
				return ret;
		}
	iqs->w24_rate = mt76_rr(dev, IQ_W24_RATE);
	iqs->w24_a0080 = mt76_rr(dev, IQ_W24_A0080);
	iqs->w24_applied = true;	/* restore puts these back from now on */

	for (c = 0; c < IQ_GAIN_CHAINS; c++)
		for (k = 0; k < IQ_W24_NRF; k++) {
			val = (iqs->w24_rf[c][k] & ~iq_w24_rf_mask[k]) |
			      iq_w24_rf_bw80[k];
			ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c),
					    iq_w24_rf_addr[k], &val, false);
			if (!ret)
				ret = mt7921_iqs_rf(dev, IQ_RF_GAIN_PORT(c),
						    iq_w24_rf_addr[k], &chk, true);
			if (!ret && chk != val)
				ret = -EIO;
			if (ret)
				goto fail;
		}
	val = (iqs->w24_rate & ~IQ_W24_RATE_MASK) | IQ_W24_RATE_BW80;
	mt76_wr(dev, IQ_W24_RATE, val);
	if (mt76_rr(dev, IQ_W24_RATE) != val) {
		ret = -EIO;
		goto fail;
	}
	mt7921_iqs_w24_latch(dev);
	mt76_wr(dev, IQ_W24_A0080,
		(iqs->w24_a0080 & ~IQ_W24_A0080_MASK) | IQ_W24_A0080_BW80);
	mt7921_iqs_w24_agc(dev, IQ_W24_AGC_BW80);
	return 0;

fail:
	iqs->w24_errors++;
	dev_warn(dev->mt76.dev, "iq: 2.4 GHz 80 MHz setup failed: %d\n", ret);
	mt7921_iqs_w24_restore(iqs);
	return ret;
}

/* Bring the hardware to the requested gain (iqs->gain). On failure the
 * request falls back to AGC, so MT7921_IQ_IOC_GET_GAIN tells the truth.
 * Caller holds ctl_lock.
 */
static int mt7921_iqs_gain_sync(struct mt7921_iqs *iqs)
{
	int ret;

	if (!mt7921_iqs_hw_ok(iqs))
		return -ENODEV;
	if (iqs->gain.mode != MT7921_IQ_GAIN_HOLD)
		return mt7921_iqs_gain_release(iqs);
	ret = mt7921_iqs_gain_hold(iqs, mt7921_iqs_gain_chains(iqs),
				   iqs->gain.lna, iqs->gain.lpf);
	if (ret)
		memset(&iqs->gain, 0, sizeof(iqs->gain));
	return ret;
}

/* ---- 80 MHz snapshots ---------------------------------------------------- */

static void mt7921_iqs_end(struct mt7921_iqs *iqs);

#define IQ_STOP_PTR		0x800210b4
#define IQ_SNAP_BLOCK		4096
#define IQ_SNAP_TIMEOUT_MS	500
#define IQ_FREEZE_TIMEOUT_MS	200

/* Append a driver-made record (IQB2 header + payload) to the ring. */
static void mt7921_iqs_put_record(struct mt7921_iqs *iqs, const void *rec,
				  u32 len)
{
	u64 used;
	u32 at, first;

	spin_lock_bh(&iqs->lock);
	used = iqs->head - iqs->tail;
	if (ALIGN(len, 4) > iqs->size - used) {
		iqs->drops++;
		spin_unlock_bh(&iqs->lock);
		return;
	}
	at = iqs->head & (iqs->size - 1);
	first = min_t(u64, len, iqs->size - at);
	memcpy(iqs->buf + at, rec, first);
	if (first < len)
		memcpy(iqs->buf, (const u8 *)rec + first, len - first);
	iqs->head += ALIGN(len, 4);
	spin_unlock_bh(&iqs->lock);
	wake_up_interruptible(&iqs->wait);
}

static void mt7921_iqs_put_meta(struct mt7921_iqs *iqs, u32 stop)
{
	struct {
		struct mt7921_iqb2_hdr hdr;
		struct mt7921_iq_snap_meta meta;
	} __packed r = {
		.hdr = {
			.magic = cpu_to_le32(MT7921_IQB2_MAGIC),
			.version = cpu_to_le16(MT7921_IQB2_VERSION),
			.hdr_len = cpu_to_le16(sizeof(r.hdr)),
			.payload_len = cpu_to_le32(sizeof(r.meta)),
			.seq = cpu_to_le32(iqs->snaps),
			.cursor = cpu_to_le32(stop),
			.flags = cpu_to_le32(MT7921_IQ_FMT_META << 4 |
					     MT7921_IQ_RING_CODE(iqs->ring) <<
					     MT7921_IQ_RING_SHIFT),
			.generation = cpu_to_le32(iqs->fw_gen),
		},
		.meta = {
			.magic = cpu_to_le32(MT7921_IQ_SNAP_MAGIC),
			.stop_word = cpu_to_le32(stop),
			.freq_khz = cpu_to_le32(iqs->freq_khz),
			.bw_mhz = cpu_to_le32(iqs->bw_mhz),
			.ring_bytes = cpu_to_le32(iqs->ring),
		},
	};

	mt7921_iqs_put_record(iqs, &r, sizeof(r));
}

/*
 * One 80 MHz snapshot. Arm the DLPF tap at 80 MS/s (u4BW 2, raw two-lane
 * records), let the ring fill (64 KiB in ~102 us), freeze it with the
 * capture stop, then have firmware mode 6 DMA the frozen ring once
 * (non-follow, ring / 4 KiB blocks from cursor 0) and wait for its
 * acknowledgement. A META record with the stop pointer precedes the blocks.
 * Caller holds
 * ctl_lock; registers were saved by mt7921_iqs_fw_start().
 */
static int mt7921_iqs_snap_once(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	unsigned long timeout;
	u32 ctl[8], stop, s2;
	bool acked = false;
	int i, ret;

	if (!mt7921_iqs_hw_ok(iqs))
		return -ENODEV;
	if (mt76_rr(dev, IQ_MODE) != 0 || !mt7921_iqs_tx0_idle(dev))
		return -EBUSY;

	mt7921_iqs_set_dlpf(dev->spectrum, 2);
	mt76_wr(dev, IQ_TRIGGER_ORDER, 1);
	mt76_wr(dev, IQ_CUSTOM_RANGE, 1);
	ret = mt7921_spectrum_iq_arm(dev);
	if (ret)
		goto out;
	ret = mt7921_iqs_wait_armed(dev);
	if (ret) {
		mt7921_spectrum_iq_disarm(dev);
		goto out;
	}
	ret = mt7921_iqs_set_ring(dev, iqs->ring);
	if (ret) {
		mt7921_spectrum_iq_disarm(dev);
		goto out;
	}
	mt76_wr(dev, IQ_ICAP_PACK, mt76_rr(dev, IQ_ICAP_PACK) & ~GENMASK(19, 16));
	usleep_range(400, 600);		/* at least a few full ring laps */

	/* Freeze: the capture stop (a queued MCU command) leaves the last
	 * ring in place; wait until the write pointer stands still.
	 */
	mt7921_spectrum_iq_disarm(dev);
	timeout = jiffies + msecs_to_jiffies(IQ_FREEZE_TIMEOUT_MS);
	stop = mt76_rr(dev, IQ_STOP_PTR);
	for (;;) {
		usleep_range(300, 500);
		s2 = mt76_rr(dev, IQ_STOP_PTR);
		if (s2 == stop)
			break;
		stop = s2;
		if (time_after(jiffies, timeout)) {
			ret = -EAGAIN;		/* still writing */
			goto out;
		}
	}
	stop &= iqs->ring / 4 - 1;

	for (i = 8; i < 18; i++)
		mt76_wr(dev, IQ_CTL + 4 * i, 0);
	ctl[0] = IQ_CTL_MAGIC;
	ctl[1] = IQ_ICAP_SRC;
	ctl[2] = iqs->ring;
	ctl[3] = IQ_SNAP_BLOCK;
	/* bit 0 clear: no follow */
	ctl[4] = MT7921_IQ_FMT_RAW80 << 4 |
		 MT7921_IQ_RING_CODE(iqs->ring) << MT7921_IQ_RING_SHIFT;
	ctl[5] = iqs->ring / IQ_SNAP_BLOCK;	/* exactly one ring */
	ctl[6] = iqs->fw_gen;
	ctl[7] = 0;
	for (i = 0; i < ARRAY_SIZE(ctl); i++) {
		mt76_wr(dev, IQ_CTL + 4 * i, ctl[i]);
		if (mt76_rr(dev, IQ_CTL + 4 * i) != ctl[i]) {
			ret = -EIO;
			goto out;
		}
	}
	mt7921_iqs_put_meta(iqs, stop);
	mt76_wr(dev, IQ_ACK, 0);
	mt76_wr(dev, IQ_GEN, iqs->fw_gen);
	mt76_wr(dev, IQ_SER_ENABLE, iqs->saved.ser & ~0xff);
	mt76_wr(dev, IQ_MODE, IQ_MODE_STREAM);
	ret = mt7921_spectrum_iq_dump_begin(dev);
	if (ret) {
		mt76_wr(dev, IQ_MODE, 0);
		mt76_wr(dev, IQ_SER_ENABLE, iqs->saved.ser);
		goto out;
	}
	mutex_lock(&iqs->hw_lock);
	iqs->builder_live = true;
	mutex_unlock(&iqs->hw_lock);

	timeout = jiffies + msecs_to_jiffies(IQ_SNAP_TIMEOUT_MS);
	do {
		if (mt76_rr(dev, IQ_ACK) == iqs->fw_gen) {
			acked = true;
			break;
		}
		usleep_range(300, 600);
	} while (time_before(jiffies, timeout) && READ_ONCE(iqs->builder_live));

	mutex_lock(&iqs->hw_lock);
	if (iqs->builder_live) {
		iqs->builder_live = false;
		mt76_wr(dev, IQ_MODE, 0);
		mt76_wr(dev, IQ_SER_ENABLE, iqs->saved.ser);
	}
	mutex_unlock(&iqs->hw_lock);
	ret = acked ? 0 : -ETIMEDOUT;
	if (!ret && mt76_rr(dev, IQ_CTL_STATE) == IQ_CTL_STATE_BADCTL)
		ret = -EINVAL;		/* control block rejected (ring size) */
out:
	mt76_wr(dev, IQ_ICAP_PACK, iqs->saved.pack);
	if (ret)
		iqs->snap_errors++;
	else
		iqs->snaps++;
	return ret;
}

static void mt7921_iqs_snap_work(struct work_struct *work)
{
	struct mt7921_iqs *iqs = container_of(work, struct mt7921_iqs,
					      snap_work.work);
	int ret;

	mutex_lock(&iqs->ctl_lock);
	if (!iqs->dev || !iqs->snap_mode || !iqs->fw_running || iqs->ended) {
		mutex_unlock(&iqs->ctl_lock);
		return;
	}
	ret = mt7921_iqs_snap_once(iqs);
	if (ret)
		iqs->snap_fail_run++;
	else
		iqs->snap_fail_run = 0;
	if (ret == -ENODEV || iqs->snap_fail_run >= 20) {
		dev_warn(iqs->dev->mt76.dev, "iq: snapshots stopped: %d\n", ret);
		iqs->snap_mode = false;
		iqs->fw_running = false;
		mt7921_iqs_restore(iqs, ret != -ENODEV);
		mt7921_iqs_end(iqs);
		mutex_unlock(&iqs->ctl_lock);
		return;
	}
	mutex_unlock(&iqs->ctl_lock);
	queue_delayed_work(system_unbound_wq, &iqs->snap_work,
			   msecs_to_jiffies(iq_snap_ms));
}

/* Arm the capture and start firmware mode 6. Caller holds ctl_lock. */
static int mt7921_iqs_fw_start(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	u32 ctl[8], gen, pack, fmt, block = iq_block, state = 0;
	unsigned long timeout;
	bool dense;
	int i, ret;

	if (!mt7921_iqs_hw_ok(iqs))
		return -ENODEV;
	if (!(mt76_hw(dev)->conf.flags & IEEE80211_CONF_MONITOR))
		return -ENETDOWN;
	if (iq_block != 4096 && iq_block != 8192 && iq_block != 14336)
		return -EINVAL;
	ret = mt7921_iqs_fw_check(iqs);
	if (ret)
		return ret;
	/* Somebody else (the debugfs harness) owns the stream. */
	if (mt76_rr(dev, IQ_MODE) != 0 || !mt7921_iqs_tx0_idle(dev))
		return -EBUSY;

	mt7921_iqs_save(iqs);
	iqs->ring = mt7921_iqs_ring_bytes(dev);
	dense = iq_dense && iqs->bw_mhz >= 40;
	if (mt7921_iqs_wide24(iqs) && !dense)
		return -EOPNOTSUPP;	/* iq_dense switched off since the tune */
	if (iqs->bw_mhz == 80 && !dense) {
		/* 80 MHz: snapshots of the 80 MS/s tap; one generation for
		 * all snapshots of this start.
		 */
		do {
			gen = get_random_u32() & 0x7fffffff;
		} while (!gen || gen == iqs->fw_gen);
		iqs->fw_gen = gen;
		iqs->starts++;
		iqs->snap_fail_run = 0;
		iqs->snap_mode = true;
		iqs->fmt = MT7921_IQ_FMT_RAW80;
		iqs->fw_running = true;
		if (iqs->gain.mode == MT7921_IQ_GAIN_HOLD &&
		    mt7921_iqs_gain_sync(iqs))
			dev_warn(dev->mt76.dev, "iq: gain hold dropped, AGC\n");
		queue_delayed_work(system_unbound_wq, &iqs->snap_work, 0);
		dev_info(dev->mt76.dev, "iq: 80 MHz snapshots started, generation %u\n",
			 gen);
		return 0;
	}
	/* iq_tap80 (40 MHz, experimental): run the tap at 80 MS/s (u4BW 2)
	 * and let bit 17 keep every other lane-0 sample, alias-free behind
	 * the 40 MHz channel filter. The padded layout costs 8 B per sample,
	 * 320 MB/s, which the transport cannot sustain.
	 */
	mt7921_iqs_set_dlpf(dev->spectrum,
			    !dense && iqs->bw_mhz == 40 && iq_tap80 ? 2 : 0);
	mt76_wr(dev, IQ_TRIGGER_ORDER, 1);
	mt76_wr(dev, IQ_CUSTOM_RANGE, 1);

	ret = mt7921_spectrum_iq_arm(dev);
	if (ret)
		goto restore;
	ret = mt7921_iqs_wait_armed(dev);
	if (ret)
		goto disarm;
	ret = mt7921_iqs_set_ring(dev, iqs->ring);
	if (ret)
		goto disarm;

	/* Arming rewrites the packing register (0x200 -> 0x9c000200), so the
	 * format bits go on top of the armed value; bits 19:16 are cleared
	 * first (the firmware's bit_width parameter can leave bit 19 set).
	 * Stop restores the value from before the start.
	 */
	pack = mt76_rr(dev, IQ_ICAP_PACK) & ~GENMASK(19, 16);
	if (dense) {
		/* The DLPF arm above only sets up the capture; the dense mode
		 * then replaces its tap, layout and rate (no packer).
		 */
		if (iqs->bw_mhz == 80)
			fmt = iqs->chain ? MT7921_IQ_FMT_DENSE80_C1 :
					   MT7921_IQ_FMT_DENSE80;
		else
			fmt = iqs->chain ? MT7921_IQ_FMT_DENSE40_C1 :
					   MT7921_IQ_FMT_DENSE40;
	} else if (iqs->bw_mhz == 40 && iq_tap80) {
		pack |= IQ_ICAP_EN32B64B;
		fmt = MT7921_IQ_FMT_PACK32_40;
	} else if (iqs->bw_mhz == 40) {
		pack |= IQ_ICAP_EN96B128B;
		fmt = MT7921_IQ_FMT_PACK96;
	} else if (iq_pack32) {
		pack |= IQ_ICAP_EN32B64B;
		fmt = MT7921_IQ_FMT_PACK32;
	} else {
		fmt = MT7921_IQ_FMT_RAW;
	}
	mt76_wr(dev, IQ_ICAP_PACK, pack);
	if (dense && mt7921_iqs_wide24(iqs)) {
		ret = mt7921_iqs_w24_apply(iqs);
		if (ret)
			goto disarm;
	}
	if (dense) {
		ret = mt7921_iqs_set_dense(dev, iqs->bw_mhz == 80 ? 2 : 3,
					   iqs->chain ? IQ_DFE_TAP_CHAIN1 :
							IQ_DFE_TAP_CHAIN0);
		if (ret)
			goto disarm;
	}
	/* DENSE80 runs at 320 MB/s, the TX0 ceiling: it was validated with
	 * 8 KiB blocks and 32 KiB USB aggregation (0 late blocks in 3 x 30 s).
	 */
	if (fmt == MT7921_IQ_FMT_DENSE80 || fmt == MT7921_IQ_FMT_DENSE80_C1) {
		block = max_t(u32, block, 8192);
		mt7921_iqs_set_agg(iqs, true);
	}

	do {
		gen = get_random_u32() & 0x7fffffff;
	} while (!gen || gen == iqs->fw_gen);
	/* Status words 8..17: a stream stopped before the firmware's first
	 * loop pass would otherwise report the previous run's (or random)
	 * late/max-lead values.
	 */
	for (i = 8; i < 18; i++)
		mt76_wr(dev, IQ_CTL + 4 * i, 0);
	ctl[0] = IQ_CTL_MAGIC;
	ctl[1] = IQ_ICAP_SRC;
	ctl[2] = iqs->ring;
	ctl[3] = block;
	/* bit 0: follow the capture write pointer; bits 7:1 are echoed in
	 * every IQB2 header (flags): 3:1 ring size code, 7:4 sample format.
	 */
	ctl[4] = 1 | MT7921_IQ_RING_CODE(iqs->ring) << MT7921_IQ_RING_SHIFT |
		 fmt << 4;
	ctl[5] = 0;		/* no block limit */
	ctl[6] = gen;
	ctl[7] = 0;
	for (i = 0; i < ARRAY_SIZE(ctl); i++) {
		mt76_wr(dev, IQ_CTL + 4 * i, ctl[i]);
		if (mt76_rr(dev, IQ_CTL + 4 * i) != ctl[i]) {
			ret = -EIO;
			goto disarm;
		}
	}
	mt76_wr(dev, IQ_GEN, gen);
	mt76_wr(dev, IQ_SER_ENABLE, iqs->saved.ser & ~0xff);
	mt76_wr(dev, IQ_MODE, IQ_MODE_STREAM);

	ret = mt7921_spectrum_iq_dump_begin(dev);
	if (ret) {
		/* The builder never ran, so nothing will acknowledge a stop. */
		mt76_wr(dev, IQ_MODE, 0);
		goto disarm;
	}
	/* The builder validates the control block first; older firmware
	 * rejects a ring it does not support (v64 and earlier: > 64 KiB) and
	 * returns at once, acknowledging like a finished stream.
	 */
	timeout = jiffies + msecs_to_jiffies(IQ_START_TIMEOUT_MS);
	do {
		state = mt76_rr(dev, IQ_CTL_STATE);
		if (state)
			break;
		usleep_range(500, 1000);
	} while (time_before(jiffies, timeout));
	if (state == IQ_CTL_STATE_BADCTL) {
		mt76_wr(dev, IQ_MODE, 0);
		ret = -EINVAL;
		dev_warn(dev->mt76.dev,
			 "iq: firmware rejected the stream (%u KiB ring, %u-byte blocks); %u KiB rings need firmware v65\n",
			 iqs->ring / 1024, block, iqs->ring / 1024);
		goto disarm;
	}

	mutex_lock(&iqs->hw_lock);
	iqs->builder_live = true;
	mutex_unlock(&iqs->hw_lock);
	iqs->fw_gen = gen;
	iqs->fmt = fmt;
	iqs->starts++;
	iqs->fw_running = true;
	dev_info(dev->mt76.dev, "iq: stream started, generation %u, %u-byte blocks, %u KiB ring, %u MHz, format %u\n",
		 gen, block, iqs->ring / 1024, iqs->bw_mhz, fmt);
	/* After the channel set and the arm: both run under AGC. */
	if (iqs->gain.mode == MT7921_IQ_GAIN_HOLD && mt7921_iqs_gain_sync(iqs))
		dev_warn(dev->mt76.dev, "iq: gain hold dropped, AGC\n");
	return 0;

disarm:
	mt7921_spectrum_iq_disarm(dev);
restore:
	mt7921_iqs_w24_restore(iqs);
	mt7921_iqs_restore(iqs, true);
	iqs->fw_errors++;
	dev_warn(dev->mt76.dev, "iq: stream start failed: %d\n", ret);
	return ret;
}

/* Take the builder out of mode 6 with register writes only: no MCU
 * command, so this is safe from the MCU send path. Caller holds hw_lock.
 */
static void mt7921_iqs_builder_stop(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;
	unsigned long timeout;

	if (!iqs->builder_live)
		return;
	iqs->builder_live = false;
	iqs->builder_acked = false;

	mt76_wr(dev, IQ_MODE, IQ_MODE_STOP);
	timeout = jiffies + msecs_to_jiffies(IQ_STOP_TIMEOUT_MS);
	do {
		if (mt76_rr(dev, IQ_ACK) == iqs->fw_gen) {
			iqs->builder_acked = true;
			break;
		}
		usleep_range(1000, 2000);
	} while (time_before(jiffies, timeout));

	iqs->last_late = mt76_rr(dev, IQ_CTL_LATE);
	iqs->last_max_lead = mt76_rr(dev, IQ_CTL_MAX_LEAD);
	mt76_wr(dev, IQ_MODE, 0);
	/* SER, packing and USB aggregation back before any MCU command. */
	mt76_wr(dev, IQ_SER_ENABLE, iqs->saved.ser);
	mt76_wr(dev, IQ_ICAP_PACK, iqs->saved.pack);
	mt7921_iqs_set_agg(iqs, false);
}

/* Stop the stream and undo mt7921_iqs_fw_start(). Caller holds ctl_lock.
 * After a chip reset the firmware state is gone, so only the bookkeeping
 * remains.
 */
static void mt7921_iqs_fw_stop(struct mt7921_iqs *iqs)
{
	struct mt792x_dev *dev = iqs->dev;

	if (!iqs->fw_running)
		return;
	iqs->fw_running = false;
	/* Every MCU command that follows (channel set, close) runs under AGC;
	 * the next start applies the hold again.
	 */
	if (mt7921_iqs_hw_ok(iqs)) {
		mt7921_iqs_gain_release(iqs);
		mt7921_iqs_w24_restore(iqs);
	}

	if (iqs->snap_mode) {
		/* The worker runs one snapshot at a time under ctl_lock, so
		 * none is in progress here; it sees snap_mode clear and quits.
		 */
		iqs->snap_mode = false;
		cancel_delayed_work(&iqs->snap_work);
		mt7921_iqs_restore(iqs, mt7921_iqs_hw_ok(iqs));
		dev_info(dev->mt76.dev, "iq: 80 MHz snapshots stopped, %u taken, %u errors\n",
			 iqs->snaps, iqs->snap_errors);
		return;
	}

	mutex_lock(&iqs->hw_lock);
	if (!mt7921_iqs_hw_ok(iqs)) {
		iqs->builder_live = false;
		mutex_unlock(&iqs->hw_lock);
		mt7921_iqs_restore(iqs, false);
		return;
	}
	mt7921_iqs_builder_stop(iqs);
	mutex_unlock(&iqs->hw_lock);

	if (!mt7921_iqs_tx0_idle(dev)) {
		iqs->fw_errors++;
		dev_warn(dev->mt76.dev, "iq: TX0 ring not back in boot state\n");
	}
	if (iqs->builder_acked) {
		/* Builder finished: MCU commands are safe again. */
		mt7921_spectrum_iq_disarm(dev);
	} else {
		iqs->fw_errors++;
		dev_warn(dev->mt76.dev,
			 "iq: firmware did not acknowledge stop; capture left armed\n");
	}
	mt7921_iqs_restore(iqs, true);
	dev_info(dev->mt76.dev, "iq: stream stopped, generation %u, late %u, max lead %u\n",
		 iqs->fw_gen, iqs->last_late, iqs->last_max_lead);
}

static void mt7921_iqs_end(struct mt7921_iqs *iqs)
{
	WRITE_ONCE(iqs->ended, true);
	wake_up_interruptible(&iqs->wait);
}

/* Called for every MCU command before it is sent (mt7921u). mac80211
 * reconfigures the chip through MCU commands (retune: sniffer config;
 * interface down: BSS and MAC commands), and each would time out against
 * a running builder. Stop the builder first; the restart work finishes the
 * stop and starts a new generation once the reconfiguration has settled.
 */
void mt7921_iqs_mcu_guard(struct mt792x_dev *dev, int cmd)
{
	struct mt7921_iqs *iqs = mt7921_iqs_get(dev);

	if (!iqs || !READ_ONCE(iqs->builder_live))
		return;

	mutex_lock(&iqs->hw_lock);
	if (iqs->builder_live) {
		if (mt7921_iqs_hw_ok(iqs))
			mt7921_iqs_builder_stop(iqs);
		else
			iqs->builder_live = false;
		iqs->interrupts++;
		iqs->last_cmd = cmd;
		dev_info(dev->mt76.dev,
			 "iq: MCU command 0x%08x interrupts generation %u\n",
			 cmd, iqs->fw_gen);
		schedule_delayed_work(&iqs->restart_work,
				      msecs_to_jiffies(IQ_RESTART_DELAY_MS));
	}
	mutex_unlock(&iqs->hw_lock);
}
EXPORT_SYMBOL_GPL(mt7921_iqs_mcu_guard);

static void mt7921_iqs_restart_work(struct work_struct *work)
{
	struct mt7921_iqs *iqs = container_of(work, struct mt7921_iqs,
					      restart_work.work);

	mutex_lock(&iqs->ctl_lock);
	if (iqs->dev && !iqs->builder_live) {
		mt7921_iqs_fw_stop(iqs);
		if (iqs->opened && !iqs->ended && mt7921_iqs_fw_start(iqs))
			mt7921_iqs_end(iqs);
	}
	mutex_unlock(&iqs->ctl_lock);
}

/* Restart a generation that started at the slip-prone alignment. */
static void mt7921_iqs_realign_work(struct work_struct *work)
{
	struct mt7921_iqs *iqs = container_of(work, struct mt7921_iqs,
					      realign_work);

	mutex_lock(&iqs->ctl_lock);
	if (iqs->dev && iqs->opened && iqs->fw_running &&
	    iqs->fw_gen == READ_ONCE(iqs->bad_gen)) {
		mt7921_iqs_fw_stop(iqs);
		if (!iqs->ended && mt7921_iqs_fw_start(iqs))
			mt7921_iqs_end(iqs);
	}
	mutex_unlock(&iqs->ctl_lock);
}

/* Chip reset in progress: lock-free, may run while ctl_lock holders wait
 * for the device. The stream ends; the reader has to reopen.
 */
void mt7921_iqs_chip_reset(struct mt792x_dev *dev)
{
	struct mt7921_iqs *iqs = mt7921_iqs_get(dev);

	if (!iqs)
		return;
	atomic_set(&iqs->chip_reset, 1);
	mt7921_iqs_end(iqs);
}
EXPORT_SYMBOL_GPL(mt7921_iqs_chip_reset);

/* ---- retune ------------------------------------------------------------ */

static void mt7921_iqs_find_monitor(void *priv, u8 *mac,
				    struct ieee80211_vif *vif)
{
	struct ieee80211_vif **mon = priv;

	if (vif->type == NL80211_IFTYPE_MONITOR && !*mon)
		*mon = vif;
}

/* Caller holds the mt792x mutex; the vif is valid while it is held. */
static struct ieee80211_vif *mt7921_iqs_monitor_vif(struct mt792x_dev *dev)
{
	struct ieee80211_vif *mon = NULL;

	ieee80211_iterate_active_interfaces(mt76_hw(dev),
					    IEEE80211_IFACE_ITER_RESUME_ALL,
					    mt7921_iqs_find_monitor, &mon);
	return mon;
}

/*
 * Raster: 5 MHz channel numbers, as the sniffer command takes them. *ch is
 * the center channel; a 40 MHz channel has its control channel 10 MHz below
 * (SCA), so its center needs channel numbers 2 apart in range.
 */
static int mt7921_iqs_freq_to_chan(u32 khz, u32 bw, enum nl80211_band *band,
				   u8 *ch)
{
	u32 mhz = DIV_ROUND_CLOSEST(khz, 1000);

	if (bw == 80) {
		/* 2.4 GHz: a 40 MHz channel (control channel 10 MHz below the
		 * center) widened by the driver; only the dense stream does it.
		 */
		if (mhz >= 2420 && mhz <= 2464) {
			if (!iq_dense)
				return -EOPNOTSUPP;
			*band = NL80211_BAND_2GHZ;
			*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 2407, 5), 3, 11);
			return 0;
		}
		/* 5 GHz: control channel 30 MHz below the center */
		if (mhz < 5033 || mhz > 5967)
			return -ERANGE;
		*band = NL80211_BAND_5GHZ;
		*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 5000, 5), 7, 193);
		return 0;
	}
	if (bw == 40) {
		if (mhz >= 2420 && mhz <= 2464) {
			*band = NL80211_BAND_2GHZ;
			*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 2407, 5), 3, 11);
		} else if (mhz >= 5013 && mhz <= 5987) {
			*band = NL80211_BAND_5GHZ;
			*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 5000, 5), 3, 197);
		} else {
			return -ERANGE;
		}
		return 0;
	}
	if (mhz >= 2410 && mhz <= 2474) {
		*band = NL80211_BAND_2GHZ;
		*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 2407, 5), 1, 13);
	} else if (mhz >= 2482 && mhz <= 2486) {
		*band = NL80211_BAND_2GHZ;
		*ch = 14;
	} else if (mhz >= 5003 && mhz <= 5997) {
		*band = NL80211_BAND_5GHZ;
		*ch = clamp_t(u32, DIV_ROUND_CLOSEST(mhz - 5000, 5), 1, 199);
	} else {
		return -ERANGE;
	}
	return 0;
}

static u32 mt7921_iqs_chan_to_khz(enum nl80211_band band, u8 ch)
{
	if (band == NL80211_BAND_2GHZ)
		return ch == 14 ? 2484000 : (2407 + 5 * ch) * 1000;
	return (5000 + 5 * ch) * 1000;
}

/*
 * Point the monitor receiver at (band, center channel, width), bypassing
 * mac80211: the same two sniffer commands mt7921_change_chanctx() sends, with
 * a synthetic chandef so channel numbers off the regulatory list work too.
 * At 40 MHz the control channel is the lower half (center - 10 MHz).
 * restore puts back mac80211's own chanctx. Caller holds ctl_lock, the
 * builder is stopped.
 */
static int mt7921_iqs_set_sniffer(struct mt792x_dev *dev,
				  enum nl80211_band band, u8 ch, u32 bw,
				  bool restore)
{
	/* 80 MHz on 2.4 GHz: the channel is set as 40 MHz, fw_start widens it */
	u32 cbw = band == NL80211_BAND_2GHZ && bw == 80 ? 40 : bw;
	u8 ctrl = cbw == 80 ? ch - 6 : cbw == 40 ? ch - 2 : ch;
	struct ieee80211_channel chan = {
		.band = band,
		.hw_value = ctrl,
		.center_freq = mt7921_iqs_chan_to_khz(band, ctrl) / 1000,
	};
	struct ieee80211_chanctx_conf ctx = {
		.def = {
			.chan = &chan,
			.width = cbw == 80 ? NL80211_CHAN_WIDTH_80 :
				 cbw == 40 ? NL80211_CHAN_WIDTH_40 :
					     NL80211_CHAN_WIDTH_20,
			.center_freq1 = mt7921_iqs_chan_to_khz(band, ch) / 1000,
		},
	};
	struct ieee80211_chanctx_conf *use = &ctx;
	struct ieee80211_vif *vif;
	struct mt792x_vif *mvif;
	int ret;

	mt792x_mutex_acquire(dev);
	vif = mt7921_iqs_monitor_vif(dev);
	if (!vif) {
		ret = -ENETDOWN;
		goto out;
	}
	mvif = (struct mt792x_vif *)vif->drv_priv;
	if (restore) {
		use = mvif->bss_conf.mt76.ctx;
		if (!use) {
			ret = 0;
			goto out;
		}
	}
	ret = mt7921_mcu_set_sniffer(dev, vif, true);
	if (!ret)
		ret = mt7921_mcu_config_sniffer(mvif, use);
out:
	mt792x_mutex_release(dev);
	return ret;
}

static long mt7921_iqs_tune(struct mt7921_iqs *iqs, struct mt7921_iq_tune *t)
{
	enum nl80211_band band;
	u32 bw;
	u8 ch;
	int ret;

	if (t->flags & ~MT7921_IQ_TUNE_FLAGS)
		return -EINVAL;
	mutex_lock(&iqs->ctl_lock);
	bw = t->bw_mhz ?: iqs->bw_mhz;
	if (bw != 20 && bw != 40 && bw != 80) {
		ret = -EINVAL;
		goto out;
	}
	ret = mt7921_iqs_freq_to_chan(t->freq_khz, bw, &band, &ch);
	if (ret)
		goto out;
	if (!iqs->dev || iqs->ended) {
		ret = -ENODEV;
		goto out;
	}
	mt7921_iqs_fw_stop(iqs);
	ret = mt7921_iqs_set_sniffer(iqs->dev, band, ch, bw, false);
	if (!ret) {
		iqs->freq_khz = mt7921_iqs_chan_to_khz(band, ch);
		iqs->bw_mhz = bw;
	}
	iqs->chain = !!(t->flags & MT7921_IQ_TUNE_CHAIN1);
	/* Restart even after a failed retune: the reader keeps a stream. */
	if (mt7921_iqs_fw_start(iqs)) {
		mt7921_iqs_end(iqs);
		ret = ret ?: -EIO;
	}
	iqs->tunes++;
	t->freq_khz = iqs->freq_khz;
	t->generation = iqs->fw_gen;
	t->bw_mhz = iqs->bw_mhz;
	t->flags = iqs->chain ? MT7921_IQ_TUNE_CHAIN1 : 0;
out:
	mutex_unlock(&iqs->ctl_lock);
	return ret;
}

static long mt7921_iqs_set_gain(struct mt7921_iqs *iqs,
				struct mt7921_iq_gain *g)
{
	long ret = 0;

	if (g->reserved || g->mode > MT7921_IQ_GAIN_HOLD ||
	    (g->mode == MT7921_IQ_GAIN_HOLD &&
	     (g->lna > MT7921_IQ_GAIN_LNA_MAX || g->lpf > MT7921_IQ_GAIN_LPF_MAX)))
		return -EINVAL;
	if (g->mode == MT7921_IQ_GAIN_AGC)
		g->lna = g->lpf = 0;
	mutex_lock(&iqs->ctl_lock);
	if (!iqs->dev || iqs->ended) {
		ret = -ENODEV;
	} else {
		iqs->gain = *g;
		/* Live while streaming; otherwise the next start applies it. */
		if (iqs->fw_running)
			ret = mt7921_iqs_gain_sync(iqs);
	}
	*g = iqs->gain;
	mutex_unlock(&iqs->ctl_lock);
	return ret;
}

static long mt7921_iqs_gain_ioctl(struct mt7921_iqs *iqs, unsigned int cmd,
				  unsigned long arg)
{
	struct mt7921_iq_gain g;
	long ret = 0;

	if (cmd == MT7921_IQ_IOC_SET_GAIN) {
		if (copy_from_user(&g, (void __user *)arg, sizeof(g)))
			return -EFAULT;
		ret = mt7921_iqs_set_gain(iqs, &g);
	} else {
		mutex_lock(&iqs->ctl_lock);
		g = iqs->gain;
		mutex_unlock(&iqs->ctl_lock);
	}
	if (copy_to_user((void __user *)arg, &g, sizeof(g)))
		return -EFAULT;
	return ret;
}

static long mt7921_iqs_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	struct mt7921_iqs *iqs = file->private_data;
	struct mt7921_iq_tune t;
	long ret;

	switch (cmd) {
	case MT7921_IQ_IOC_SET_GAIN:
	case MT7921_IQ_IOC_GET_GAIN:
		return mt7921_iqs_gain_ioctl(iqs, cmd, arg);
	case MT7921_IQ_IOC_TUNE:
		if (copy_from_user(&t, (void __user *)arg, sizeof(t)))
			return -EFAULT;
		ret = mt7921_iqs_tune(iqs, &t);
		break;
	case MT7921_IQ_IOC_GET:
		memset(&t, 0, sizeof(t));
		mutex_lock(&iqs->ctl_lock);
		t.freq_khz = iqs->freq_khz;
		t.generation = iqs->fw_gen;
		t.bw_mhz = iqs->bw_mhz;
		t.flags = iqs->chain ? MT7921_IQ_TUNE_CHAIN1 : 0;
		mutex_unlock(&iqs->ctl_lock);
		ret = 0;
		break;
	default:
		return -ENOTTY;
	}
	if (copy_to_user((void __user *)arg, &t, sizeof(t)))
		return -EFAULT;
	return ret;
}

static int mt7921_iqs_misc_open(struct inode *inode, struct file *file)
{
	struct mt7921_iqs *iqs = container_of(file->private_data,
					      struct mt7921_iqs, misc);
	int ret;

	mutex_lock(&iqs->ctl_lock);
	if (!iqs->dev) {
		ret = -ENODEV;
		goto out;
	}
	if (iqs->opened) {
		ret = -EBUSY;
		goto out;
	}
	ret = mt7921_iqs_reset_ring(iqs);
	if (ret)
		goto out;
	/* A reset that finished before this open no longer matters. */
	atomic_set(&iqs->chip_reset, 0);
	/* Forces left by a release that could not reach the hardware. */
	if (mt7921_iqs_hw_ok(iqs)) {
		if (iqs->gain_held)
			mt7921_iqs_gain_release(iqs);
		mt7921_iqs_w24_restore(iqs);
	}
	memset(&iqs->gain, 0, sizeof(iqs->gain));
	WRITE_ONCE(iqs->ended, false);
	WRITE_ONCE(iqs->enabled, true);
	iqs->freq_khz = 0;
	iqs->bw_mhz = 20;	/* mac80211's channel, as a 20 MHz stream */
	iqs->chain = 0;
	iqs->align_gen = 0;
	iqs->bad_gen = 0;
	ret = mt7921_iqs_fw_start(iqs);
	if (ret)
		goto out;

	iqs->opened = true;
	kref_get(&iqs->ref);
	file->private_data = iqs;
	ret = nonseekable_open(inode, file);
out:
	mutex_unlock(&iqs->ctl_lock);
	return ret;
}

static void mt7921_iqs_kref_release(struct kref *ref)
{
	struct mt7921_iqs *iqs = container_of(ref, struct mt7921_iqs, ref);

	vfree(iqs->buf);
	kfree(iqs);
}

static int mt7921_iqs_misc_release(struct inode *inode, struct file *file)
{
	struct mt7921_iqs *iqs = file->private_data;

	mutex_lock(&iqs->ctl_lock);
	if (iqs->dev) {
		mt7921_iqs_fw_stop(iqs);
		/* Hand the receiver back to mac80211's channel. */
		if (iqs->freq_khz && mt7921_iqs_hw_ok(iqs))
			mt7921_iqs_set_sniffer(iqs->dev, 0, 0, 0, true);
	}
	iqs->freq_khz = 0;
	iqs->bw_mhz = 20;
	memset(&iqs->gain, 0, sizeof(iqs->gain));
	iqs->opened = false;
	mutex_unlock(&iqs->ctl_lock);
	kref_put(&iqs->ref, mt7921_iqs_kref_release);

	return 0;
}

static const struct file_operations mt7921_iqs_misc_fops = {
	.owner = THIS_MODULE,
	.open = mt7921_iqs_misc_open,
	.release = mt7921_iqs_misc_release,
	.read = mt7921_iqs_read,
	.poll = mt7921_iqs_poll,
	.unlocked_ioctl = mt7921_iqs_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

/* Detach from the device and remove the char device; idempotent. With hw,
 * stop the stream first (USB disconnect, before mt76 unregisters). Without,
 * the device may already be freed and only bookkeeping is done.
 */
static void mt7921_iqs_detach(struct mt7921_iqs *iqs, bool hw)
{
	bool misc;

	mutex_lock(&iqs->ctl_lock);
	if (hw && iqs->dev)
		mt7921_iqs_fw_stop(iqs);
	iqs->fw_running = false;
	iqs->builder_live = false;
	iqs->dev = NULL;
	misc = iqs->misc_registered;
	iqs->misc_registered = false;
	mutex_unlock(&iqs->ctl_lock);
	cancel_delayed_work_sync(&iqs->restart_work);
	cancel_work_sync(&iqs->realign_work);
	cancel_delayed_work_sync(&iqs->snap_work);
	mt7921_iqs_end(iqs);

	/* Outside ctl_lock: misc_open() runs our open under misc_mtx. */
	if (misc)
		misc_deregister(&iqs->misc);
}

void mt7921_iqs_shutdown(struct mt792x_dev *dev)
{
	struct mt7921_iqs *iqs = mt7921_iqs_get(dev);

	if (iqs)
		mt7921_iqs_detach(iqs, true);
}
EXPORT_SYMBOL_GPL(mt7921_iqs_shutdown);

static void mt7921_iqs_free(void *data)
{
	struct mt7921_iqs *iqs = data;

	mt7921_iqs_detach(iqs, false);
	kref_put(&iqs->ref, mt7921_iqs_kref_release);
}

int mt7921_iqs_init(struct mt792x_dev *dev, struct dentry *dir)
{
	struct mt7921_iqs *iqs;
	u64 size;

	if (!iq_stream_mb || !is_power_of_2(iq_stream_mb))
		return 0;

	/* Not devm: an open char device outlives the driver binding. */
	iqs = kzalloc(sizeof(*iqs), GFP_KERNEL);
	if (!iqs)
		return -ENOMEM;

	size = (u64)iq_stream_mb << 20;
	iqs->buf = vmalloc(size);
	if (!iqs->buf) {
		kfree(iqs);
		return -ENOMEM;
	}

	iqs->size = size;
	iqs->enabled = true;
	iqs->dev = dev;
	kref_init(&iqs->ref);
	spin_lock_init(&iqs->lock);
	mutex_init(&iqs->read_lock);
	mutex_init(&iqs->ctl_lock);
	mutex_init(&iqs->hw_lock);
	INIT_DELAYED_WORK(&iqs->restart_work, mt7921_iqs_restart_work);
	INIT_WORK(&iqs->realign_work, mt7921_iqs_realign_work);
	INIT_DELAYED_WORK(&iqs->snap_work, mt7921_iqs_snap_work);
	iqs->bw_mhz = 20;
	init_waitqueue_head(&iqs->wait);
	if (devm_add_action_or_reset(dev->mt76.dev, mt7921_iqs_free, iqs))
		return -ENOMEM;
	dev->spectrum->iqs = iqs;

	debugfs_create_file("iq_stream", 0400, dir, iqs, &mt7921_iqs_fops);
	debugfs_create_file("iq_stream_ctrl", 0200, dir, iqs,
			    &mt7921_iqs_ctrl_fops);
	debugfs_create_file("iq_stream_stats", 0400, dir, iqs,
			    &mt7921_iqs_stats_fops);
	debugfs_create_file("rf_spi", 0600, dir, iqs, &mt7921_iqs_rf_spi_fops);

	snprintf(iqs->name, sizeof(iqs->name), "mt7921_iq_%s",
		 wiphy_name(mt76_hw(dev)->wiphy));
	iqs->fw_ver = mt7921_iqs_fw_version(dev);
	if (iqs->fw_ver >= IQ_FW_MIN)
		dev_info(dev->mt76.dev, "iq: I/Q firmware %s\n",
			 mt76_hw(dev)->wiphy->fw_version);
	else
		dev_info(dev->mt76.dev, "iq: firmware %s has no I/Q stream (needs %s%u or later)\n",
			 mt76_hw(dev)->wiphy->fw_version, IQ_FW_LABEL, IQ_FW_MIN);
	iqs->misc.minor = MISC_DYNAMIC_MINOR;
	iqs->misc.name = iqs->name;
	iqs->misc.fops = &mt7921_iqs_misc_fops;
	if (misc_register(&iqs->misc))
		dev_warn(dev->mt76.dev, "iq: %s not registered\n", iqs->name);
	else
		iqs->misc_registered = true;

	return 0;
}
