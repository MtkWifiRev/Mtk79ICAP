/* SPDX-License-Identifier: MIT */
/*
 * libmt7921iq: decode the IQB2 stream of /dev/mt7921_iq_<phy> into complex
 * samples.
 *
 * Wire format (driver/mt7921/iqstream.h): records of a 32-byte IQB2 header
 * followed by payload_len bytes, padded to 4 bytes. The payload is a copy of
 * the ICAP ring (64 KiB by default; 16 KiB with the driver's iq_ring_kb=16 or
 * from older drivers; 256 KiB with iq_ring_kb=256 and firmware v65; header
 * flags bits 3:1), starting at header.cursor.
 *
 * Sample formats (IQB2 header flags bits 7:4, see iqstream_ioctl.h); a
 * sample word carries I in bits 29:16 and Q in bits 13:0, both signed 14
 * bits:
 *  - PACK32 (20 MHz channel, driver default): each 16-byte record holds two
 *    samples as [s(n), s(n+1), 0, 0]. One stream at 20.0008 MS/s.
 *  - PACK32_40 (40 MHz channel, experimental driver option iq_tap80): the
 *    PACK32 layout from the tap running at 80 MS/s (capture u4BW 2), i.e.
 *    40.0016 MS/s, but 320 MB/s on the ring.
 *  - RAW80 (80 MHz channel): not a stream but snapshots. A META record
 *    (stop pointer, ring size) precedes one frozen ring of raw records
 *    [l0(t), l1(t), l0(t+1), l1(t+1)] at 80.0032 MS/s per lane; the decoder
 *    rotates it to chronological order and outputs lane 0 as one chunk,
 *    flagged MT7921IQ_F_START | MT7921IQ_F_SNAPSHOT: 8192 samples
 *    (102.4 us) from a 64 KiB ring, 2048 (25.6 us) from a 16 KiB one.
 *  - DENSE40 / DENSE80 (40 / 80 MHz channel, driver option iq_dense): one
 *    chain-0 sample per 32-bit word, consecutive, no padding, from the
 *    channel-filtered tap 12 of the dense DFE mode; DENSE40_C1 / DENSE80_C1
 *    the same from RX chain 1 (tap 13, mt7921iq_tune_chain()).
 *    40.0016 / 80.0032 MS/s,
 *    continuous (160 / 320 MB/s). Words with spare bits set are not samples
 *    (packet-buffer data in the ring with firmware before v64): they are
 *    output as 0 and counted (garbage_words).
 *  - PACK96 (40 MHz channel, driver default): dense 3-word groups
 *    [l0(t), l0(t+1), l1(t)].
 *    The decoder outputs lane 0 at 40.0016 MS/s, or lane 1 at 20.0008 MS/s
 *    (mt7921iq_dec_set_chain()). The group phase is not
 *    marked in the stream; it is found statistically from the first ~98 k
 *    words of each continuous run (lane-0 neighbours correlate the same way
 *    in both directions, wrong phases do not). A slip there loses one known
 *    8-byte unit; its lane-0 samples are filled with the previous sample and
 *    counted (repaired_samples).
 *
 * DMA line slip: a 128-byte line (aligned to the ring address), or the part
 * of it inside one block, may arrive as D0 D0 0 D1 0 ... instead of
 * D0 0 D1 0 ...: its first 8-byte unit repeats and its last (padding) unit
 * is lost. Rates vary widely between runs (from 1e-5 to over 30 % of lines). The decoder
 * recognises both patterns by position, so no data is lost and zero-valued
 * samples are kept. Lines matching neither pattern are decoded by the
 * fallback rule (drop zero units and exact repeats) and counted.
 *
 * Discontinuities split the output into chunks; a chunk never spans one:
 *   - generation change: the driver restarted the stream (e.g. retune);
 *   - sequence gap: the driver dropped whole blocks (reader too slow);
 *   - cursor break: payload does not continue the previous block;
 *   - ring overrun: the firmware DMA read ring lines after the producer
 *     refilled them. The newer data then appears twice, one ring lap
 *     (PACK32: ring / 8 samples, 8192 with 64 KiB) apart, bit-identical. Detected when the second copy
 *     arrives, so it is reported up to one lap late; info.overrun_start
 *     gives the sample index where the bad copy began.
 */
#ifndef MT7921IQ_H
#define MT7921IQ_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MT7921IQ_IQB2_MAGIC	0x32425149u
#define MT7921IQ_HDR_LEN	32
/* DLPF lanes run at 40.0016 MS/s. PACK32 (20 MHz channel) keeps every other
 * lane-0 sample; PACK96 (40 MHz channel) keeps all of lane 0.
 */
#define MT7921IQ_SAMPLE_RATE	20000800.0
#define MT7921IQ_SAMPLE_RATE_40	40001600.0
#define MT7921IQ_SAMPLE_RATE_80	80003200.0	/* DENSE80 stream, RAW80 snapshots */
#define MT7921IQ_SNAP_SAMPLES	8192		/* per 80 MHz snapshot (64 KiB
						 * ring; 2048 with 16 KiB) */
#define MT7921IQ_FULL_SCALE	8192

/* Chunk flags: why the chunk starts a new continuous run. */
#define MT7921IQ_F_START	(1u << 0)	/* first chunk of a generation */
#define MT7921IQ_F_SEQ_GAP	(1u << 1)
#define MT7921IQ_F_CURSOR	(1u << 2)
#define MT7921IQ_F_OVERRUN	(1u << 3)	/* see overrun_start */
#define MT7921IQ_F_SNAPSHOT	(1u << 4)	/* one 80 MHz snapshot (RAW80):
						 * contiguous samples, not
						 * continuous with the previous */

struct mt7921iq_chunk {
	uint32_t flags;
	uint32_t generation;
	uint64_t sample_index;	/* since the start of this generation */
	uint64_t time_ns;	/* MCU tick at generation start + samples/rate */
	uint64_t overrun_start;	/* with MT7921IQ_F_OVERRUN */
	uint64_t overrun_len;	/* samples */
	uint32_t lost_blocks;	/* with MT7921IQ_F_SEQ_GAP */
	uint32_t format;	/* MT7921_IQ_FMT_* of the generation */
	double sample_rate;	/* of this chunk's samples */
};

struct mt7921iq_stats {
	uint64_t records, blocks, bytes, samples;
	uint64_t lines, slip_lines, fallback_lines;
	uint64_t generations, seq_gaps, lost_blocks, cursor_breaks;
	uint64_t overruns, overrun_samples;
	uint64_t malformed;
	/* PACK96 (40 MHz) */
	uint64_t repaired_samples;	/* lane-0 samples lost to slips, held */
	uint64_t phase_detections;
	uint64_t detect_dropped_words;	/* too few words to find the phase */
	uint32_t phase_score_best;	/* last detection, x1000 (0 = clear) */
	uint32_t phase_score_next;
	/* RAW80 (80 MHz snapshots) */
	uint64_t snapshots, snap_incomplete;
	/* DENSE40/80: non-sample words output as 0 */
	uint64_t garbage_words;
};

enum mt7921iq_format { MT7921IQ_CS16, MT7921IQ_CF32 };

struct mt7921iq_dec;

struct mt7921iq_dec *mt7921iq_dec_new(void);
void mt7921iq_dec_free(struct mt7921iq_dec *d);
/* Forget all state (after reopening the device); the chain setting stays. */
void mt7921iq_dec_reset(struct mt7921iq_dec *d);

/*
 * PACK96 (40 MHz) only: output RX chain 1 (lane 1) instead of chain 0. Lane 1
 * comes at 20.0008 MS/s, every other sample of a 40 MS/s lane without a
 * filter, so the channel's content beyond +-10 MHz folds into the band.
 * Set it before the first push. Other formats carry chain 0 only.
 */
void mt7921iq_dec_set_chain(struct mt7921iq_dec *d, int chain);

/*
 * Append raw bytes read from the device. Returns 0, or -1 on a framing error
 * (bad magic; the decoder then needs a reset).
 */
int mt7921iq_dec_push(struct mt7921iq_dec *d, const void *buf, size_t len);

/*
 * End of stream (EOF from the device): decode a trailing partial line that
 * would otherwise wait for the next block.
 */
void mt7921iq_dec_flush(struct mt7921iq_dec *d);

/*
 * Take up to max samples of one continuous chunk. out holds max complex
 * samples (CS16: 2*max int16_t, CF32: 2*max float). Returns the number of
 * samples; *info describes the chunk (flags only on its first call). Returns
 * 0 when more input is needed.
 */
size_t mt7921iq_dec_read(struct mt7921iq_dec *d, enum mt7921iq_format fmt,
			 void *out, size_t max, struct mt7921iq_chunk *info);

void mt7921iq_dec_stats(const struct mt7921iq_dec *d,
			struct mt7921iq_stats *st);

/*
 * Retune the stream open on fd (MT7921_IQ_IOC_TUNE). The receiver tunes in
 * 5 MHz steps (2412-2472, 2484, 5005-5995 MHz); *actual_hz gets the tuned
 * center and *generation the first generation at it (samples of earlier
 * generations are from the old frequency). Takes ~15-20 ms, during which
 * the stream pauses. Returns 0 or -errno (-ERANGE outside the bands).
 */
int mt7921iq_tune(int fd, double hz, double *actual_hz, uint32_t *generation);

/*
 * As mt7921iq_tune(), also choosing the channel width: bw_mhz 20
 * (20.0008 MS/s, centers 2412-2472, 2484, 5005-5995 MHz) or 40
 * (40.0016 MS/s, ~36 MHz usable, centers 2422-2462, 5015-5985 MHz) or 80
 * (80.0032 MS/s, centers 5035-5965 MHz: snapshots, or DENSE80 with the
 * driver's iq_dense; with iq_dense also 2422-2462 MHz, a 2.4 GHz 40 MHz
 * channel the driver widens to 80 MHz, -EOPNOTSUPP without it); 0 keeps the
 * current width. *actual_bw gets the tuned width.
 */
int mt7921iq_tune_bw(int fd, double hz, unsigned bw_mhz, double *actual_hz,
		     unsigned *actual_bw, uint32_t *generation);

/*
 * Like mt7921iq_tune_bw(), and select the RX chain of dense streams (driver
 * iq_dense): chain 1 streams DFE tap 13 (formats DENSE40_C1 / DENSE80_C1)
 * at the full rate. PACK32/PACK96/RAW80 streams ignore it (PACK96 lane 1:
 * mt7921iq_dec_set_chain()). Every tune sets the chain; opening the device
 * resets it to 0.
 */
int mt7921iq_tune_chain(int fd, double hz, unsigned bw_mhz, int chain,
			double *actual_hz, unsigned *actual_bw, uint32_t *generation);

/*
 * Receive gain of the stream open on fd (MT7921_IQ_IOC_SET_GAIN): hold = 0
 * gives the gain to the chip's AGC (the default after every open); hold = 1
 * fixes the streamed RX chain(s) at the analog code (lna, lpf), live, with
 * no gap in the stream. The driver keeps the hold across retunes.
 * Measured on 5 GHz:
 *  - LNA 2-5: about 6 dB per step; it sets the noise figure, 5 is the
 *    AGC's idle code;
 *  - LPF 0-8: about 2 dB per step at the same SNR;
 *  - under hold the AGC's gain dips on other stations' frames (about -4 dB
 *    for 16-19 us, hundreds per second on a busy primary) are gone;
 *  - the output is about 13 dB below the AGC's level (scaling only);
 *  - the hardware's blanking runs (exact zeros) remain.
 * 2.4 GHz codes are not mapped yet. The driver takes LNA 0-7, LPF 0-15.
 * Returns 0 or -errno (-EINVAL out of range, -EBUSY gain forced by
 * somebody else).
 */
#define MT7921IQ_LNA_MIN	2	/* usable range on 5 GHz */
#define MT7921IQ_LNA_MAX	5
#define MT7921IQ_LPF_MIN	0
#define MT7921IQ_LPF_MAX	8
#define MT7921IQ_LNA_STEP_DB	6.0	/* measured, approximate */
#define MT7921IQ_LPF_STEP_DB	2.0
int mt7921iq_set_gain(int fd, int hold, unsigned lna, unsigned lpf);
/* The gain in force: *hold 0 (AGC) or 1, and the held code. */
int mt7921iq_get_gain(int fd, int *hold, unsigned *lna, unsigned *lpf);

#ifdef __cplusplus
}
#endif

#endif /* MT7921IQ_H */
