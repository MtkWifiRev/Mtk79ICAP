/* SPDX-License-Identifier: BSD-3-Clause-Clear */
/*
 * ioctls of /dev/mt7921_iq_<phy>, shared with userspace
 * (userspace/libmt7921iq includes this file).
 *
 * The stream needs a patched WM firmware (WIFI_RAM_CODE_MT7961_1 with the
 * version label iq_dma_v65 or later, shown as "WM Firmware Version" in the
 * kernel log). On any other firmware open() and MT7921_IQ_IOC_TUNE fail with
 * EOPNOTSUPP (module parameter iq_fw_check).
 */
#ifndef __MT7921_IQSTREAM_IOCTL_H
#define __MT7921_IQSTREAM_IOCTL_H

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
typedef uint32_t __u32;
#endif

struct mt7921_iq_tune {
	__u32 freq_khz;		/* in: requested; out: tuned center */
	__u32 generation;	/* out: first generation at the new frequency */
	__u32 flags;		/* in/out: MT7921_IQ_TUNE_* */
	__u32 bw_mhz;		/* in: 20, 40 or 80 (0: keep); out: tuned width */
};

/*
 * Sample format of a stream, in IQB2 header flags bits 7:4
 * (MT7921_IQ_FMT(flags)). 32-bit sample words: I bits 29:16, Q bits 13:0,
 * signed 14 bits; the two RX chains are lane 0 and lane 1.
 */
#define MT7921_IQ_FMT(flags)	(((flags) >> 4) & 0xf)
/*
 * ICAP ring size of a stream, in IQB2 header flags bits 3:1 (bit 0 is the
 * firmware's follow flag): 16 KiB << code, codes 0-4 (16-256 KiB). Code 0
 * (16 KiB) is what streams recorded before the field existed carry. Header
 * cursors are byte offsets in this ring.
 */
#define MT7921_IQ_RING_SHIFT	1
#define MT7921_IQ_RING_CODE(bytes)	((bytes) == 262144 ? 4 : (bytes) == 131072 ? 3 : \
					 (bytes) == 65536 ? 2 : (bytes) == 32768 ? 1 : 0)
#define MT7921_IQ_RING(flags)	(16384u << (((flags) >> MT7921_IQ_RING_SHIFT) & 7))
#define MT7921_IQ_FMT_RAW	0	/* [l0(t) l1(t) l0(t+1) l1(t+1)], 40 MS/s */
#define MT7921_IQ_FMT_PACK32	1	/* [l0(2k) l0(2k+2) 0 0]: 20 MS/s, 20 MHz */
#define MT7921_IQ_FMT_PACK96	2	/* groups [l0(t) l0(t+1) l1(t)]: lane 0
					 * at 40 MS/s, 40 MHz channel */
#define MT7921_IQ_FMT_PACK32_40	3	/* PACK32 layout from the 80 MS/s tap:
					 * 40 MS/s at 320 MB/s (experimental,
					 * module parameter iq_tap80) */
#define MT7921_IQ_FMT_RAW80	4	/* 80 MHz snapshots: one frozen ring
					 * (16 or 64 KiB) per snapshot, raw records
					 * [l0(t) l1(t) l0(t+1) l1(t+1)], lanes at
					 * 80 MS/s; preceded by a META record */
#define MT7921_IQ_FMT_DENSE40	5	/* dense single chain (DFE MODE 0x13,
					 * tap 12): one complex sample per word,
					 * consecutive, 40 MS/s, 40 MHz channel,
					 * 160 MB/s (module parameter iq_dense) */
#define MT7921_IQ_FMT_DENSE80	6	/* the same at 80 MS/s on an 80 MHz
					 * channel: continuous, 320 MB/s */
#define MT7921_IQ_FMT_DENSE40_C1 7	/* DENSE40 from RX chain 1 (tap 13) */
#define MT7921_IQ_FMT_DENSE80_C1 8	/* DENSE80 from RX chain 1 (tap 13) */
#define MT7921_IQ_FMT_META	0xf	/* driver record: 16-byte payload
					 * struct mt7921_iq_snap_meta */

/* Payload of a META record (IQB2 header: seq = snapshot number,
 * generation = that of the following RAW80 blocks, tick = 0). Drivers before
 * ring_bytes existed sent 16 bytes: then the ring is 16 KiB.
 */
struct mt7921_iq_snap_meta {
	__u32 magic;		/* MT7921_IQ_SNAP_MAGIC */
	__u32 stop_word;	/* capture stop pointer (word index in the
				 * ring); the ring is chronological from
				 * record ((stop_word + 4) % (ring_bytes / 4)) / 4 */
	__u32 freq_khz;		/* tuned center */
	__u32 bw_mhz;
	__u32 ring_bytes;	/* snapshot ring size: 16384, 65536 or 262144 */
};
#define MT7921_IQ_SNAP_MAGIC	0x31504e53	/* "SNP1" */

/*
 * Retune the receiver on the opener's stream: stop the firmware stream,
 * send the sniffer channel command for the nearest raster point, restart.
 * 20 MHz (20.0008 MS/s, PACK32): 5 MHz raster, 2412-2472, 2484 and
 * 5005-5995 MHz. 40 MHz (40.0016 MS/s, PACK96, ~36 MHz usable): 5 MHz
 * raster, centers 2422-2462 and 5015-5985 MHz; PACK96 at 240 MB/s
 * (PACK32_40 at 320 MB/s with the experimental module parameter iq_tap80).
 * 80 MHz (RAW80 snapshots, 80.0032 MS/s per lane, 8192 samples = 102.4 us
 * each with the default 64 KiB ring, 2048 with 16 KiB; not continuous): 5 MHz raster, centers 5035-5965 MHz.
 * 80 MHz on 2.4 GHz (centers 2422-2462 MHz, iq_dense only, else
 * -EOPNOTSUPP): the vendor firmware has no such channel, so the driver sets
 * the 40 MHz channel and widens it to the BW80 analog filter and digital
 * rate (experiments/mt7921-24g-80mhz-20261014): DENSE80, tones exact from
 * -35 to +39 MHz at center 2442; the low edge rolls off sooner (-39 MHz
 * about 20 dB down), +41..45 MHz fold in about 15 dB down.
 * With the module parameter iq_dense, 40 MHz streams DENSE40 (40.0016 MS/s,
 * 160 MB/s) and 80 MHz streams DENSE80 continuously (80.0032 MS/s,
 * 320 MB/s) instead of PACK96 and snapshots. Samples of every later
 * generation are at the new frequency. mac80211 is bypassed; its channel is
 * restored when the device is closed.
 */
#define MT7921_IQ_IOC_TUNE	_IOWR('q', 1, struct mt7921_iq_tune)
/* Tune flags. CHAIN1: dense formats (driver iq_dense) stream RX chain 1
 * (DFE tap 13) instead of chain 0 (tap 12); other formats ignore it (PACK96
 * carries lane 1 anyway). Each tune sets the chain; opening the device
 * resets it to 0. Unknown flags are rejected with -EINVAL.
 */
#define MT7921_IQ_TUNE_CHAIN1	(1u << 0)
#define MT7921_IQ_TUNE_FLAGS	MT7921_IQ_TUNE_CHAIN1
/* Current center (0: mac80211's channel), generation and width. */
#define MT7921_IQ_IOC_GET	_IOR('q', 2, struct mt7921_iq_tune)

/*
 * Receive gain of the opener's stream (experiments/mt7921-g3-agc-20261014).
 * AGC (the default; every open starts with it) lets the chip's AGC follow
 * the traffic on the primary 20 MHz: each detected frame dips the gain
 * (about -4 dB for 16-19 us) and can step the phase. HOLD fixes the RX
 * chains the stream carries (the dense formats: the selected chain; PACK32:
 * chain 0; the others: both) at the analog code (lna, lpf): the A-die gain
 * force plus the PHY gain force sets A and B at the same code; the digital
 * gain is left alone. Holding a chain that is not streamed would double the
 * blanking on the streamed one.
 * - Measured on 5 GHz: no gain dips, the same SNR as the AGC.
 * - Output is about 13 dB below the AGC's level (the AGC's digital gain);
 *   that is scaling only.
 * - Usable codes on 5 GHz: LNA 2-5 (about 6 dB per step; LNA sets the noise
 *   figure, 5 is the AGC's idle choice) and LPF 0-8 (about 2 dB per step,
 *   same SNR). LNA 6/7 repeat 3/4; LNA 0/1 and LPF 9-15 (except 11) give
 *   zero words.
 * - 2.4 GHz codes are not mapped (they differ: LNA 4 is louder than 5).
 * The driver takes the full fields (LNA 0-7, LPF 0-15).
 * The hold is live: no restart, no new generation. It is released before
 * every retune and stop (the channel set runs under AGC), re-applied after
 * the restart, and released on close.
 * Frame-synchronous blanking (exact-zero runs of 21 and 104 samples at
 * 80 MS/s) remains with either mode.
 * reserved must be 0. SET returns the gain in force; GET reads it.
 */
struct mt7921_iq_gain {
	__u32 mode;		/* MT7921_IQ_GAIN_* */
	__u32 lna;		/* HOLD: LNA code 0-7 */
	__u32 lpf;		/* HOLD: LPF code 0-15 */
	__u32 reserved;
};
#define MT7921_IQ_GAIN_AGC	0
#define MT7921_IQ_GAIN_HOLD	1
#define MT7921_IQ_GAIN_LNA_MAX	7
#define MT7921_IQ_GAIN_LPF_MAX	15
#define MT7921_IQ_IOC_SET_GAIN	_IOWR('q', 3, struct mt7921_iq_gain)
#define MT7921_IQ_IOC_GET_GAIN	_IOR('q', 4, struct mt7921_iq_gain)

#endif /* __MT7921_IQSTREAM_IOCTL_H */
