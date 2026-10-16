// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * libmt7921iq: IQB2 stream decoder. See mt7921iq.h.
 *
 * Pipeline: raw bytes -> records -> payload units in ring-address order ->
 *  PACK32: 128-byte lines -> positional line decode -> data units (2 samples
 *          each) -> lap-overrun check -> sample FIFO
 *  PACK96: per-block line segments -> slip repair -> lap-overrun check ->
 *          32-bit words -> group phase (detected) -> lane 0 -> sample FIFO
 *  DENSE40/80: 32-bit words, one sample each -> garbage words zeroed ->
 *          lap-overrun check over 8-byte units -> sample FIFO
 * -> chunk marks -> read().
 */
#include <complex.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include "mt7921iq.h"
#include "iqstream_ioctl.h"

#define RING_MAX	262144u		/* largest ICAP ring (header flags 3:1) */
#define LINE_UNITS	16		/* 128-byte DMA line */
#define OVERRUN_MINRUN	8		/* identical units that prove a lap */
#define DET_WORDS	(3 * 32768)	/* PACK96 words used to find the phase */
#define DET_MIN_WORDS	(3 * 4096)	/* fewer are dropped, not guessed */

struct mark {
	uint64_t pos;			/* absolute FIFO sample position */
	struct mt7921iq_chunk info;
};

struct mt7921iq_dec {
	/* raw input */
	uint8_t *in;
	size_t in_len, in_cap;
	int broken;

	/* stream state */
	int have_gen;
	uint32_t gen, last_seq, next_cursor;
	uint64_t tick64, gen_tick_us;
	uint32_t last_tick;
	uint64_t gen_samples;		/* sample index within the generation */
	uint32_t pending_flags, pending_lost;
	uint32_t fmt;			/* MT7921_IQ_FMT_* of the generation */
	uint32_t ring;			/* ICAP ring bytes of the generation */
	double rate;

	/* PACK96: words since the run start (incl. placeholders and words
	 * of lost blocks), group phase (-1: still detecting), held sample of
	 * the output chain, output chain (0: lane 0, 1: lane 1)
	 */
	uint64_t widx;
	int phase;
	int chain;
	uint32_t *det_w;
	uint8_t *det_miss;
	size_t det_n, det_cap;
	uint32_t last_l0;
	uint64_t hist96[RING_MAX / 8];	/* one lap: ring / 8 units (also DENSE) */
	uint64_t hist96_n;

	/* RAW80 snapshot assembly */
	uint8_t snap_buf[RING_MAX];
	uint32_t snap_have, snap_stop, snap_tick, snap_ring;
	uint64_t snap_samples;		/* gen sample index of the next one */
	int snap_active;

	/* line assembly */
	uint64_t line[LINE_UNITS];
	int line_j0, line_n;
	uint64_t last_unit;
	int have_last;

	/* lap-overrun detection over data units */
	uint64_t hist[RING_MAX / 16];	/* one lap: ring / 16 units */
	uint64_t hist_n;		/* data units since the last reset */
	uint64_t run_start;		/* gen sample index of the good copy */
	uint64_t run_len;		/* units */

	/* output */
	uint32_t *fifo;
	uint64_t fifo_base;		/* absolute position of fifo[0] */
	size_t fifo_rd, fifo_len, fifo_cap;
	struct mark *marks;
	size_t marks_rd, marks_len, marks_cap;

	/* read side: chunk the next read() continues */
	uint32_t rd_gen, rd_fmt;
	uint64_t rd_idx, rd_t0_ns;	/* sample index; time of index 0 */
	double rd_rate;

	struct mt7921iq_stats st;
};

static int is_dense(uint32_t fmt)
{
	return fmt == MT7921_IQ_FMT_DENSE40 || fmt == MT7921_IQ_FMT_DENSE80 ||
	       fmt == MT7921_IQ_FMT_DENSE40_C1 || fmt == MT7921_IQ_FMT_DENSE80_C1;
}

static int is_dense80(uint32_t fmt)
{
	return fmt == MT7921_IQ_FMT_DENSE80 || fmt == MT7921_IQ_FMT_DENSE80_C1;
}

static uint32_t le32(const uint8_t *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24;
}

static uint16_t le16(const uint8_t *p)
{
	return p[0] | p[1] << 8;
}

static uint64_t le64(const uint8_t *p)
{
	return le32(p) | (uint64_t)le32(p + 4) << 32;
}

static int grow(void **buf, size_t *cap, size_t need, size_t elem)
{
	size_t n = *cap ? *cap : 4096;
	void *p;

	if (need <= *cap)
		return 0;
	while (n < need)
		n *= 2;
	p = realloc(*buf, n * elem);
	if (!p)
		return -1;
	*buf = p;
	*cap = n;
	return 0;
}

struct mt7921iq_dec *mt7921iq_dec_new(void)
{
	return calloc(1, sizeof(struct mt7921iq_dec));
}

void mt7921iq_dec_set_chain(struct mt7921iq_dec *d, int chain)
{
	d->chain = !!chain;
}

void mt7921iq_dec_free(struct mt7921iq_dec *d)
{
	if (!d)
		return;
	free(d->in);
	free(d->fifo);
	free(d->marks);
	free(d->det_w);
	free(d->det_miss);
	free(d);
}

void mt7921iq_dec_reset(struct mt7921iq_dec *d)
{
	uint8_t *in = d->in, *det_miss = d->det_miss;
	uint32_t *fifo = d->fifo, *det_w = d->det_w;
	struct mark *marks = d->marks;
	size_t in_cap = d->in_cap, fifo_cap = d->fifo_cap;
	size_t marks_cap = d->marks_cap, det_cap = d->det_cap;
	int chain = d->chain;

	memset(d, 0, sizeof(*d));
	d->chain = chain;
	d->in = in;
	d->in_cap = in_cap;
	d->fifo = fifo;
	d->fifo_cap = fifo_cap;
	d->marks = marks;
	d->marks_cap = marks_cap;
	d->det_w = det_w;
	d->det_miss = det_miss;
	d->det_cap = det_cap;
}

void mt7921iq_dec_stats(const struct mt7921iq_dec *d,
			struct mt7921iq_stats *st)
{
	*st = d->st;
}

/* ---- output ------------------------------------------------------------ */

static void add_mark(struct mt7921iq_dec *d, const struct mt7921iq_chunk *c)
{
	uint64_t pos = d->fifo_base + d->fifo_len;
	struct mark *m;

	/* Merge with a mark at the same position (no samples in between). */
	if (d->marks_len > d->marks_rd &&
	    d->marks[d->marks_len - 1].pos == pos) {
		m = &d->marks[d->marks_len - 1];
		m->info.flags |= c->flags;
		if (c->flags & MT7921IQ_F_OVERRUN) {
			m->info.overrun_start = c->overrun_start;
			m->info.overrun_len = c->overrun_len;
		}
		m->info.lost_blocks += c->lost_blocks;
		m->info.generation = c->generation;
		m->info.sample_index = c->sample_index;
		m->info.time_ns = c->time_ns;
		m->info.format = c->format;
		m->info.sample_rate = c->sample_rate;
		return;
	}
	if (grow((void **)&d->marks, &d->marks_cap, d->marks_len + 1,
		 sizeof(*d->marks)))
		return;
	d->marks[d->marks_len].pos = pos;
	d->marks[d->marks_len].info = *c;
	d->marks_len++;
}

static uint64_t gen_time_ns(const struct mt7921iq_dec *d, uint64_t idx)
{
	return d->gen_tick_us * 1000 + (uint64_t)((double)idx * 1e9 / d->rate);
}

static void chunk_here(struct mt7921iq_dec *d, uint32_t flags)
{
	struct mt7921iq_chunk c = {
		.flags = flags,
		.generation = d->gen,
		.sample_index = d->gen_samples,
		.time_ns = gen_time_ns(d, d->gen_samples),
		.format = d->fmt,
		.sample_rate = d->rate,
	};

	if (flags & MT7921IQ_F_SEQ_GAP)
		c.lost_blocks = d->pending_lost;
	add_mark(d, &c);
}

static void overrun_mark(struct mt7921iq_dec *d)
{
	struct mt7921iq_chunk c = {
		.flags = MT7921IQ_F_OVERRUN,
		.generation = d->gen,
		.sample_index = d->gen_samples,
		.time_ns = gen_time_ns(d, d->gen_samples),
		.format = d->fmt,
		.sample_rate = d->rate,
	};

	/* The bad copy is one lap before the good one. PACK32: a lap is
	 * ring / 16 units of 2 samples; PACK96: ring / 4 words, 2 of 3 are
	 * lane 0 (1 of 3 lane 1).
	 */
	if (is_dense(d->fmt)) {
		uint64_t lap = d->ring / 4;	/* one sample per word */

		c.overrun_start = d->run_start > lap ? d->run_start - lap : 0;
		c.overrun_len = 2 * d->run_len;
	} else if (d->fmt == MT7921_IQ_FMT_PACK96) {
		uint64_t lap = ((uint64_t)d->ring / 4 * (d->chain ? 1 : 2) + 1) / 3;

		c.overrun_start = d->run_start > lap ? d->run_start - lap : 0;
		c.overrun_len = d->run_len * (d->chain ? 2 : 4) / 3;
	} else {
		uint64_t lap = d->ring / 8;

		c.overrun_start = d->run_start > lap ? d->run_start - lap : 0;
		c.overrun_len = 2 * d->run_len;
	}

	d->st.overruns++;
	d->st.overrun_samples += c.overrun_len;
	add_mark(d, &c);
}

static void emit_unit(struct mt7921iq_dec *d, uint64_t u)
{
	uint64_t n = d->hist_n, lap = d->ring / 16;

	/* Lap check: identical to the unit one ring lap earlier. Zero units
	 * are excluded (silence repeats legitimately).
	 */
	if (n >= lap && u && d->hist[n % lap] == u) {
		if (!d->run_len)
			d->run_start = d->gen_samples;
		d->run_len++;
	} else if (d->run_len) {
		if (d->run_len >= OVERRUN_MINRUN)
			overrun_mark(d);
		d->run_len = 0;
	}
	d->hist[n % lap] = u;
	d->hist_n++;

	if (grow((void **)&d->fifo, &d->fifo_cap, d->fifo_len + 2,
		 sizeof(*d->fifo)))
		return;
	d->fifo[d->fifo_len++] = (uint32_t)u;
	d->fifo[d->fifo_len++] = (uint32_t)(u >> 32);
	d->gen_samples += 2;
	d->st.samples += 2;
	d->last_unit = u;
	d->have_last = 1;
}

/* ---- line decode ------------------------------------------------------- */

static void fallback_unit(struct mt7921iq_dec *d, uint64_t u)
{
	if (!u || (d->have_last && u == d->last_unit))
		return;
	emit_unit(d, u);
}

static void flush_line(struct mt7921iq_dec *d)
{
	const uint64_t *L = d->line;
	int j0 = d->line_j0, n = d->line_n, k, ok;

	if (!n)
		return;
	d->line_n = 0;
	d->st.lines++;

	/* normal: data at even line positions, zero at odd */
	ok = 1;
	for (k = 0; k < n; k++)
		if (((j0 + k) & 1) && L[k])
			ok = 0;
	if (ok) {
		for (k = 0; k < n; k++)
			if (!((j0 + k) & 1))
				emit_unit(d, L[k]);
		return;
	}

	/* slip: D0 D0 0 D1 0 D2 ... (last padding unit lost). It hits each
	 * DMA segment on its own: a line split across two blocks can slip in
	 * one half only, so segments are decoded separately (see block()).
	 */
	if (!(j0 & 1) && n >= 2 && L[0] == L[1]) {
		ok = 1;
		for (k = 2; k < n; k += 2)
			if (L[k])
				ok = 0;
		if (ok) {
			d->st.slip_lines++;
			emit_unit(d, L[0]);
			for (k = 3; k < n; k += 2)
				emit_unit(d, L[k]);
			return;
		}
	}

	d->st.fallback_lines++;
	for (k = 0; k < n; k++)
		fallback_unit(d, L[k]);
}

static void push_unit(struct mt7921iq_dec *d, uint32_t addr, uint64_t u)
{
	int j = (addr % 128) / 8;

	if (d->line_n && (j == 0 || j != d->line_j0 + d->line_n))
		flush_line(d);
	if (!d->line_n)
		d->line_j0 = j;
	d->line[d->line_n++] = u;
	if (j == LINE_UNITS - 1)
		flush_line(d);
}

/* End of a continuous run: flush and forget cross-block state. */
static void break_run(struct mt7921iq_dec *d)
{
	flush_line(d);
	if (d->run_len >= OVERRUN_MINRUN)
		overrun_mark(d);
	d->run_len = 0;
	d->hist_n = 0;
	d->have_last = 0;
}

/* ---- PACK96 (40 MHz): dense groups [l0(t) l0(t+1) l1(t)] ------------- */

static void emit_word(struct mt7921iq_dec *d, uint32_t w)
{
	if (grow((void **)&d->fifo, &d->fifo_cap, d->fifo_len + 1,
		 sizeof(*d->fifo)))
		return;
	d->fifo[d->fifo_len++] = w;
	d->gen_samples++;
	d->st.samples++;
}

/* Group position of word k: 0, 1 = lane 0 (t, t+1), 2 = lane 1 (t). */
static int gpos(const struct mt7921iq_dec *d, uint64_t k)
{
	return (int)((k + 3 - (uint64_t)d->phase % 3) % 3);
}

/* Is group position g part of the output chain? */
static int out_pos(const struct mt7921iq_dec *d, int g)
{
	return d->chain ? g == 2 : g < 2;
}

/* Output-chain words among word indices [0, n) for the current phase. */
static uint64_t out_before(const struct mt7921iq_dec *d, uint64_t n)
{
	uint64_t c = n / 3 * (d->chain ? 1 : 2), k;

	for (k = n / 3 * 3; k < n; k++)
		c += out_pos(d, gpos(d, k));
	return c;
}

static void word96(struct mt7921iq_dec *d, uint32_t w, int missing)
{
	if (out_pos(d, gpos(d, d->widx))) {
		if (missing) {
			w = d->last_l0;		/* hold: a 1-2 sample hole */
			d->st.repaired_samples++;
		}
		d->last_l0 = w;
		emit_word(d, w);
	}
	d->widx++;
}

static double complex wiq(uint32_t w)
{
	int i = (int16_t)((w >> 16 & 0x3fff) << 2) >> 2;
	int q = (int16_t)((w & 0x3fff) << 2) >> 2;

	return i + I * q;
}

/*
 * Find the group phase of the buffered words, from two cues:
 *  - lag-1 symmetry: for the right phase, positions (0, 1) and (1, next 0)
 *    are both lane-0 neighbours, so the two normalised correlations are
 *    equal; wrong phases pair the chains at different lags.
 *  - power: positions 0 and 1 are the same chain, so their powers match;
 *    the other chain has a different gain (0.6-0.7x measured).
 * A strong pure tone defeats the first cue (all lags look alike, margin
 * 0.004), noise alone weakens the second; score = sym + 2 * power separated
 * every measured case (tone, weak tone, noise only, Wi-Fi traffic).
 * Means are removed per position (DC).
 */
static int detect_phase(struct mt7921iq_dec *d)
{
	double score[3], best = 9, next = 9;
	int p, bp = 0;

	for (p = 0; p < 3; p++) {
		double complex mean[3] = { 0 }, r1 = 0, r2 = 0;
		double pa = 0, pb = 0;
		size_t k, cnt[3] = { 0 }, g = (d->det_n - p) / 3;

		for (k = 0; k + 1 < g; k++) {
			size_t b = p + 3 * k;
			int j;

			for (j = 0; j < 3; j++)
				if (!d->det_miss[b + j]) {
					mean[j] += wiq(d->det_w[b + j]);
					cnt[j]++;
				}
		}
		for (k = 0; k < 3; k++)
			mean[k] = cnt[k] ? mean[k] / cnt[k] : 0;
		for (k = 0; k + 1 < g; k++) {
			size_t b = p + 3 * k;
			double complex A, B, An;

			if (d->det_miss[b] || d->det_miss[b + 1] ||
			    d->det_miss[b + 3])
				continue;
			A = wiq(d->det_w[b]) - mean[0];
			B = wiq(d->det_w[b + 1]) - mean[1];
			An = wiq(d->det_w[b + 3]) - mean[0];
			r1 += A * conj(B);
			r2 += B * conj(An);
			pa += creal(A * conj(A));
			pb += creal(B * conj(B));
		}
		if (pa <= 0 || pb <= 0) {
			score[p] = 9;
		} else {
			r1 /= sqrt(pa * pb);
			r2 /= sqrt(pa * pb);
			score[p] = cabs(r1 - r2) / (cabs(r1) + cabs(r2) + 1e-12) +
				   2 * fabs(pa - pb) / (pa + pb);
		}
	}
	for (p = 0; p < 3; p++)
		if (score[p] < best) {
			next = best;
			best = score[p];
			bp = p;
		} else if (score[p] < next) {
			next = score[p];
		}
	d->st.phase_detections++;
	d->st.phase_score_best = (uint32_t)(best * 1000);
	d->st.phase_score_next = (uint32_t)(next * 1000);
	return bp;
}

/* Resolve the phase from the buffer (or drop a too-short buffer). */
static void det_finish(struct mt7921iq_dec *d)
{
	size_t k, n = d->det_n;

	if (d->phase >= 0 || !n)
		return;
	d->det_n = 0;
	if (n < DET_MIN_WORDS) {
		d->st.detect_dropped_words += n;
		d->gen_samples += n * (d->chain ? 1 : 2) / 3;
		d->widx = 0;
		return;
	}
	d->det_n = n;
	d->phase = detect_phase(d);
	d->det_n = 0;
	/* word indices restart at the buffer start */
	d->widx = 0;
	for (k = 0; k < n; k++)
		word96(d, d->det_w[k], d->det_miss[k]);
}

static void push_word96(struct mt7921iq_dec *d, uint32_t w, int missing)
{
	if (d->phase >= 0) {
		word96(d, w, missing);
		return;
	}
	if (!d->det_cap) {
		d->det_w = malloc(DET_WORDS * sizeof(*d->det_w));
		d->det_miss = malloc(DET_WORDS);
		if (!d->det_w || !d->det_miss)
			return;
		d->det_cap = DET_WORDS;
	}
	d->det_w[d->det_n] = w;
	d->det_miss[d->det_n++] = missing;
	if (d->det_n >= DET_WORDS)
		det_finish(d);
}

static void unit96(struct mt7921iq_dec *d, uint64_t u, int missing)
{
	uint64_t n = d->hist96_n, lap = d->ring / 8;

	if (!missing) {
		if (n >= lap && u && d->hist96[n % lap] == u) {
			if (!d->run_len)
				d->run_start = d->gen_samples;
			d->run_len++;
		} else if (d->run_len) {
			if (d->run_len >= OVERRUN_MINRUN)
				overrun_mark(d);
			d->run_len = 0;
		}
	}
	d->hist96[n % lap] = missing ? 0 : u;
	d->hist96_n++;
	push_word96(d, (uint32_t)u, missing);
	push_word96(d, (uint32_t)(u >> 32), missing);
}

/* One block: split at 128-byte line boundaries; a segment whose first two
 * units are equal (and not silence) slipped: drop the copy, the segment's
 * last unit is lost and becomes a placeholder, so word counts stay exact.
 */
static void block96(struct mt7921iq_dec *d, const uint8_t *p, uint32_t cur,
		    uint32_t len)
{
	uint32_t n = len / 8, s = 0;

	while (s < n) {
		uint32_t e = s + 1, k;

		while (e < n && (cur + 8 * e) % 128)
			e++;
		d->st.lines++;
		if (e - s >= 2 && le64(p + 8 * s) == le64(p + 8 * s + 8) &&
		    le64(p + 8 * s)) {
			d->st.slip_lines++;
			unit96(d, le64(p + 8 * s), 0);
			for (k = s + 2; k < e; k++)
				unit96(d, le64(p + 8 * k), 0);
			unit96(d, 0, 1);
		} else {
			for (k = s; k < e; k++)
				unit96(d, le64(p + 8 * k), 0);
		}
		s = e;
	}
}

/* ---- DENSE40/80: one chain-0 sample per 32-bit word ------------------- */

/* Words with spare bits 31:30 / 15:14 set are not samples (packet-buffer
 * data that landed in the ICAP ring before firmware v64 gave ICAP its own
 * SYSRAM): they are output as 0 and counted. Lap check as for PACK32, over
 * 8-byte units, one lap = ring / 8 units.
 */
static void block_dense(struct mt7921iq_dec *d, const uint8_t *p, uint32_t len)
{
	uint64_t lap = d->ring / 8;
	uint32_t k;

	if (grow((void **)&d->fifo, &d->fifo_cap, d->fifo_len + len / 4,
		 sizeof(*d->fifo)))
		return;
	for (k = 0; k + 8 <= len; k += 8) {
		uint32_t w0 = le32(p + k), w1 = le32(p + k + 4);
		uint64_t u, n = d->hist96_n;

		if (w0 & 0xc000c000u) {
			w0 = 0;
			d->st.garbage_words++;
		}
		if (w1 & 0xc000c000u) {
			w1 = 0;
			d->st.garbage_words++;
		}
		u = w0 | (uint64_t)w1 << 32;
		if (n >= lap && u && d->hist96[n % lap] == u) {
			if (!d->run_len)
				d->run_start = d->gen_samples;
			d->run_len++;
		} else if (d->run_len) {
			if (d->run_len >= OVERRUN_MINRUN)
				overrun_mark(d);
			d->run_len = 0;
		}
		d->hist96[n % lap] = u;
		d->hist96_n++;
		d->fifo[d->fifo_len++] = w0;
		d->fifo[d->fifo_len++] = w1;
		d->gen_samples += 2;
		d->st.samples += 2;
	}
}

static void break_dense(struct mt7921iq_dec *d)
{
	if (d->run_len >= OVERRUN_MINRUN)
		overrun_mark(d);
	d->run_len = 0;
	d->hist96_n = 0;
}

/* Start of a continuous PACK96 run: forget phase and history. */
static void reset96(struct mt7921iq_dec *d)
{
	det_finish(d);
	if (d->run_len >= OVERRUN_MINRUN)
		overrun_mark(d);
	d->run_len = 0;
	d->hist96_n = 0;
	d->phase = -1;
	d->widx = 0;
	d->det_n = 0;
}

/* End the current continuous run in whatever format it is. */
static void end_run(struct mt7921iq_dec *d)
{
	if (d->fmt == MT7921_IQ_FMT_PACK96)
		reset96(d);
	else if (is_dense(d->fmt))
		break_dense(d);
	else
		break_run(d);
}

void mt7921iq_dec_flush(struct mt7921iq_dec *d)
{
	if (is_dense(d->fmt)) {
		break_dense(d);
		return;
	}
	break_run(d);
	if (d->fmt == MT7921_IQ_FMT_PACK96)
		det_finish(d);
}

/* ---- RAW80: 80 MHz snapshots --------------------------------------------- */

static void snap_meta(struct mt7921iq_dec *d, const uint8_t *h,
		      const uint8_t *p, uint32_t len)
{
	/* ring_bytes (offset 16) is absent from older 16-byte META records */
	uint32_t ring = len >= 20 ? le32(p + 16) : 16384;

	if (len < 16 || le32(p) != MT7921_IQ_SNAP_MAGIC)
		return;
	if (ring != 16384 && ring != 32768 && ring != 65536 && ring != 131072 &&
	    ring != 262144) {
		d->st.malformed++;
		return;
	}
	if (d->snap_active)
		d->st.snap_incomplete++;
	/* leaving a stream format: flush its state */
	if (d->have_gen && d->fmt != MT7921_IQ_FMT_RAW80)
		end_run(d);
	if (!d->have_gen || d->fmt != MT7921_IQ_FMT_RAW80 ||
	    le32(h + 24) != d->gen)
		d->snap_samples = 0;
	d->have_gen = 1;
	d->gen = le32(h + 24);
	d->fmt = MT7921_IQ_FMT_RAW80;
	d->rate = MT7921IQ_SAMPLE_RATE_80;
	d->snap_ring = ring;
	d->snap_stop = le32(p + 4) % (ring / 4);
	d->snap_have = 0;
	d->snap_tick = 0;
	d->snap_active = 1;
}

static void snap_block(struct mt7921iq_dec *d, const uint8_t *h,
		       const uint8_t *p, uint32_t len)
{
	uint32_t ring = d->snap_ring, cur = le32(h + 16) % ring, k, start;

	if (!d->snap_active || le32(h + 24) != d->gen)
		return;
	if (!d->snap_have)
		d->snap_tick = le32(h + 28);
	for (k = 0; k < len; k++)
		d->snap_buf[(cur + k) % ring] = p[k];
	d->snap_have += len;
	if (d->snap_have < ring)
		return;

	/* Complete ring: chronological from the record after the stop
	 * pointer (word index), lane 0 = words 0 and 2 of each record.
	 */
	d->snap_active = 0;
	d->gen_samples = d->snap_samples;
	d->gen_tick_us = d->snap_tick;
	chunk_here(d, MT7921IQ_F_START | MT7921IQ_F_SNAPSHOT);
	start = (((d->snap_stop + 4) & ~3u) % (ring / 4)) / 4;
	for (k = 0; k < ring / 16; k++) {
		const uint8_t *r = d->snap_buf + ((start + k) % (ring / 16)) * 16;

		emit_word(d, le32(r));
		emit_word(d, le32(r + 8));
	}
	d->snap_samples += ring / 8;
	d->st.snapshots++;
}

/* ---- records ----------------------------------------------------------- */

static void block(struct mt7921iq_dec *d, const uint8_t *h, const uint8_t *p,
		  uint32_t len)
{
	uint32_t seq = le32(h + 12), cur = le32(h + 16);
	uint32_t gen = le32(h + 24), tick = le32(h + 28);
	uint32_t fmt = MT7921_IQ_FMT(le32(h + 20));
	uint32_t ring = MT7921_IQ_RING(le32(h + 20));
	uint32_t flags = 0, k;

	if (fmt == MT7921_IQ_FMT_META) {
		snap_meta(d, h, p, len);
		return;
	}
	len &= ~7u;		/* sample payload: whole 8-byte units */
	d->st.blocks++;
	d->st.bytes += len;
	if (fmt == MT7921_IQ_FMT_RAW80) {
		snap_block(d, h, p, len);
		return;
	}
	if (ring > RING_MAX) {
		d->st.malformed++;
		return;
	}

	if (!d->have_gen || gen != d->gen) {
		if (d->have_gen)
			end_run(d);
		d->have_gen = 1;
		d->gen = gen;
		d->gen_samples = 0;
		d->ring = ring;
		/* PACK96 and DENSE have their own decoders; everything else
		 * uses the padded PACK32 layout (flags 0/1: 20 MS/s, 3: 40 MS/s).
		 */
		d->fmt = fmt == MT7921_IQ_FMT_PACK96 || fmt == MT7921_IQ_FMT_PACK32_40 ||
			 is_dense(fmt) ? fmt : MT7921_IQ_FMT_PACK32;
		if (is_dense80(d->fmt))
			d->rate = MT7921IQ_SAMPLE_RATE_80;
		else if (is_dense(d->fmt))
			d->rate = MT7921IQ_SAMPLE_RATE_40;
		else
			d->rate = d->fmt == MT7921_IQ_FMT_PACK32 ||
				  (d->fmt == MT7921_IQ_FMT_PACK96 && d->chain) ?
				  MT7921IQ_SAMPLE_RATE : MT7921IQ_SAMPLE_RATE_40;
		if (is_dense(d->fmt))
			break_dense(d);
		if (d->fmt == MT7921_IQ_FMT_PACK96)
			reset96(d);
		d->tick64 = d->st.generations ?
			    d->tick64 + (uint32_t)(tick - d->last_tick) : tick;
		d->gen_tick_us = d->tick64;
		d->st.generations++;
		flags = MT7921IQ_F_START;
	} else {
		d->tick64 += (uint32_t)(tick - d->last_tick);
		if (seq != d->last_seq + 1) {
			uint32_t lost = seq - d->last_seq - 1;

			d->st.seq_gaps++;
			d->st.lost_blocks += lost;
			d->pending_lost = lost;
			if (d->fmt == MT7921_IQ_FMT_PACK96) {
				/* the group phase carries on over the lost
				 * words; only the lap history restarts
				 */
				uint64_t w = (uint64_t)lost * len / 4;

				det_finish(d);
				if (d->run_len >= OVERRUN_MINRUN)
					overrun_mark(d);
				d->run_len = 0;
				d->hist96_n = 0;
				if (d->phase >= 0) {
					d->gen_samples += out_before(d, d->widx + w) -
							  out_before(d, d->widx);
					d->widx += w;
				}
			} else if (is_dense(d->fmt)) {
				break_dense(d);
				/* one sample per word */
				d->gen_samples += (uint64_t)lost * (len / 4);
			} else {
				break_run(d);
				/* each lost block held len/16 records of 2
				 * samples
				 */
				d->gen_samples += (uint64_t)lost * (len / 8);
			}
			flags = MT7921IQ_F_SEQ_GAP;
		} else if (cur != d->next_cursor) {
			end_run(d);
			d->st.cursor_breaks++;
			flags = MT7921IQ_F_CURSOR;
		}
	}
	d->last_tick = tick;
	d->last_seq = seq;
	d->next_cursor = (cur + len) % d->ring;
	if (flags)
		chunk_here(d, flags);

	if (d->fmt == MT7921_IQ_FMT_PACK96) {
		block96(d, p, cur, len);
		return;
	}
	if (is_dense(d->fmt)) {
		block_dense(d, p, len);
		return;
	}
	for (k = 0; k + 8 <= len; k += 8)
		push_unit(d, (cur + k) % d->ring, le64(p + k));
	/* A block is one DMA transfer: never join its last partial line
	 * with the next block's first one.
	 */
	flush_line(d);
}

int mt7921iq_dec_push(struct mt7921iq_dec *d, const void *buf, size_t len)
{
	size_t pos = 0;

	if (d->broken)
		return -1;
	if (grow((void **)&d->in, &d->in_cap, d->in_len + len, 1))
		return -1;
	memcpy(d->in + d->in_len, buf, len);
	d->in_len += len;

	while (d->in_len - pos >= MT7921IQ_HDR_LEN) {
		const uint8_t *h = d->in + pos;
		uint32_t hl = le16(h + 6), pl = le32(h + 8);
		size_t rec = (hl + pl + 3) & ~(size_t)3;

		if (le32(h) != MT7921IQ_IQB2_MAGIC || hl < MT7921IQ_HDR_LEN ||
		    (hl & 3)) {
			d->st.malformed++;
			d->broken = 1;
			return -1;
		}
		if (d->in_len - pos < rec)
			break;
		d->st.records++;
		if (pl)
			block(d, h, h + hl, pl);
		pos += rec;
	}
	memmove(d->in, d->in + pos, d->in_len - pos);
	d->in_len -= pos;
	return 0;
}

/* ---- read -------------------------------------------------------------- */

static void convert(const uint32_t *w, size_t n, enum mt7921iq_format fmt,
		    void *out)
{
	int16_t *o16 = out;
	float *of = out;
	size_t k;

	for (k = 0; k < n; k++) {
		int16_t i = (int16_t)((w[k] >> 16 & 0x3fff) << 2) >> 2;
		int16_t q = (int16_t)((w[k] & 0x3fff) << 2) >> 2;

		if (fmt == MT7921IQ_CS16) {
			o16[2 * k] = i;
			o16[2 * k + 1] = q;
		} else {
			of[2 * k] = i / (float)MT7921IQ_FULL_SCALE;
			of[2 * k + 1] = q / (float)MT7921IQ_FULL_SCALE;
		}
	}
}

size_t mt7921iq_dec_read(struct mt7921iq_dec *d, enum mt7921iq_format fmt,
			 void *out, size_t max, struct mt7921iq_chunk *info)
{
	uint64_t pos = d->fifo_base + d->fifo_rd;
	uint64_t end = d->fifo_base + d->fifo_len;
	size_t n;

	memset(info, 0, sizeof(*info));
	if (end == pos)
		return 0;	/* keep a pending mark until its samples exist */

	/* A mark at the read position opens the chunk. */
	if (d->marks_rd < d->marks_len && d->marks[d->marks_rd].pos <= pos) {
		*info = d->marks[d->marks_rd].info;
		d->marks_rd++;
		d->rd_gen = info->generation;
		d->rd_fmt = info->format;
		d->rd_rate = info->sample_rate;
		d->rd_idx = info->sample_index;
		d->rd_t0_ns = info->time_ns -
			(uint64_t)((double)d->rd_idx * 1e9 / d->rd_rate);
	} else {
		info->generation = d->rd_gen;
		info->format = d->rd_fmt;
		info->sample_rate = d->rd_rate;
		info->sample_index = d->rd_idx;
		info->time_ns = d->rd_t0_ns + (uint64_t)((double)d->rd_idx *
					1e9 / d->rd_rate);
	}
	/* Stop before the next mark. */
	if (d->marks_rd < d->marks_len && d->marks[d->marks_rd].pos < end)
		end = d->marks[d->marks_rd].pos;

	n = end - pos;
	if (n > max)
		n = max;
	if (n) {
		convert(d->fifo + d->fifo_rd, n, fmt, out);
		d->fifo_rd += n;
		d->rd_idx += n;
	}

	/* compact */
	if (d->fifo_rd == d->fifo_len) {
		d->fifo_base += d->fifo_len;
		d->fifo_rd = d->fifo_len = 0;
	} else if (d->fifo_rd > (1u << 20)) {
		memmove(d->fifo, d->fifo + d->fifo_rd,
			(d->fifo_len - d->fifo_rd) * sizeof(*d->fifo));
		d->fifo_base += d->fifo_rd;
		d->fifo_len -= d->fifo_rd;
		d->fifo_rd = 0;
	}
	if (d->marks_rd == d->marks_len)
		d->marks_rd = d->marks_len = 0;

	return n;
}

int mt7921iq_tune(int fd, double hz, double *actual_hz, uint32_t *generation)
{
	struct mt7921_iq_tune t = { .freq_khz = (uint32_t)(hz / 1e3 + 0.5) };

	if (ioctl(fd, MT7921_IQ_IOC_TUNE, &t) < 0)
		return -errno;
	if (actual_hz)
		*actual_hz = t.freq_khz * 1e3;
	if (generation)
		*generation = t.generation;
	return 0;
}

int mt7921iq_tune_chain(int fd, double hz, unsigned bw_mhz, int chain,
			double *actual_hz, unsigned *actual_bw, uint32_t *generation)
{
	struct mt7921_iq_tune t = {
		.freq_khz = (uint32_t)(hz / 1e3 + 0.5),
		.bw_mhz = bw_mhz,
		.flags = chain ? MT7921_IQ_TUNE_CHAIN1 : 0,
	};

	if (ioctl(fd, MT7921_IQ_IOC_TUNE, &t) < 0)
		return -errno;
	if (actual_hz)
		*actual_hz = t.freq_khz * 1e3;
	if (actual_bw)
		*actual_bw = t.bw_mhz;
	if (generation)
		*generation = t.generation;
	return 0;
}

int mt7921iq_set_gain(int fd, int hold, unsigned lna, unsigned lpf)
{
	struct mt7921_iq_gain g = {
		.mode = hold ? MT7921_IQ_GAIN_HOLD : MT7921_IQ_GAIN_AGC,
		.lna = hold ? lna : 0,
		.lpf = hold ? lpf : 0,
	};

	return ioctl(fd, MT7921_IQ_IOC_SET_GAIN, &g) < 0 ? -errno : 0;
}

int mt7921iq_get_gain(int fd, int *hold, unsigned *lna, unsigned *lpf)
{
	struct mt7921_iq_gain g;

	if (ioctl(fd, MT7921_IQ_IOC_GET_GAIN, &g) < 0)
		return -errno;
	if (hold)
		*hold = g.mode == MT7921_IQ_GAIN_HOLD;
	if (lna)
		*lna = g.lna;
	if (lpf)
		*lpf = g.lpf;
	return 0;
}

int mt7921iq_tune_bw(int fd, double hz, unsigned bw_mhz, double *actual_hz,
		     unsigned *actual_bw, uint32_t *generation)
{
	return mt7921iq_tune_chain(fd, hz, bw_mhz, 0, actual_hz, actual_bw,
				   generation);
}
