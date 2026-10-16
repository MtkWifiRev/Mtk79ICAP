// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * Synthetic tests for the IQB2 decoder: a model of the firmware stream
 * (En32b64b records in a 16 or 64 KiB ring, 4 KiB blocks starting mid-line)
 * with injected line slips, zero-valued samples, sequence gaps, generation
 * changes, lap overruns and garbage lines. The stream tests run once per
 * ring size.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mt7921iq.h"
#include "iqstream_ioctl.h"
#include <math.h>

#define RING_MAX	262144
#define BLOCK	4096
#ifndef M_PI
#define M_PI	3.14159265358979323846
#endif

static int failures;
static uint32_t RING = 16384;	/* ring of the emitted streams */

/* IQB2 header flags bits 3:1 for RING */
static uint32_t ring_flags(void)
{
	return MT7921_IQ_RING_CODE(RING) << MT7921_IQ_RING_SHIFT;
}

#define CHECK(c) do { if (!(c)) { \
	fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #c); \
	failures++; } } while (0)

/* Distinct, nonzero sample word for sample n (I and Q 14 bits each). */
static uint32_t word(uint64_t n)
{
	return (uint32_t)((n * 7 + 1) & 0x3fff) << 16 |
	       ((uint32_t)((n >> 11) + 3) & 0x3fff);
}

static int16_t sx14(uint32_t v)
{
	return (int16_t)((v & 0x3fff) << 2) >> 2;
}

/* Unit stream in ring-address order: pair k -> [w(2k)|w(2k+1)<<32, 0]. */
struct stream {
	uint64_t *u;
	size_t n;
	uint32_t cursor0;	/* ring address of u[0] */
};

static struct stream make_stream(uint64_t pairs, uint32_t cursor0,
				 uint64_t first_sample)
{
	struct stream s = { calloc(pairs * 2, 8), pairs * 2, cursor0 };
	uint64_t k;

	for (k = 0; k < pairs; k++) {
		uint64_t n = first_sample + 2 * k;

		s.u[2 * k] = word(n) | (uint64_t)word(n + 1) << 32;
		s.u[2 * k + 1] = 0;
	}
	return s;
}

/* Index of the first unit of the i-th full 128-byte line in the stream. */
static size_t line_at(const struct stream *s, size_t i)
{
	size_t first = ((128 - s->cursor0 % 128) % 128) / 8;

	return first + 16 * i;
}

/* Slip n units starting at unit b: D0 0 D1 0 ... -> D0 D0 0 D1 0 ... */
static void slip_units(struct stream *s, size_t b, size_t n)
{
	uint64_t L[16];
	size_t k;

	memcpy(L, s->u + b, n * 8);
	s->u[b] = s->u[b + 1] = L[0];
	for (k = 2; k < n; k++)
		s->u[b + k] = (k & 1) ? L[k - 1] : 0;
}

static void slip_line(struct stream *s, size_t i)
{
	slip_units(s, line_at(s, i), 16);
}

/* Serialize as IQB2 records into buf; returns bytes. */
static size_t emit(uint8_t *buf, const struct stream *s, uint32_t gen,
		   uint32_t seq0, const uint32_t *skip, int nskip)
{
	size_t pos = 0, off;
	uint32_t seq = seq0, tick = 1000;

	for (off = 0; off < s->n * 8; off += BLOCK, seq++, tick += 26) {
		uint32_t hdr[8], len = BLOCK;
		int k, skipped = 0;

		if (off + len > s->n * 8)
			len = s->n * 8 - off;
		for (k = 0; k < nskip; k++)
			if (skip[k] == seq)
				skipped = 1;
		if (skipped)
			continue;
		hdr[0] = MT7921IQ_IQB2_MAGIC;
		hdr[1] = 1 | 32 << 16;
		hdr[2] = len;
		hdr[3] = seq;
		hdr[4] = (s->cursor0 + off) % RING;
		hdr[5] = ring_flags();
		hdr[6] = gen;
		hdr[7] = tick;
		memcpy(buf + pos, hdr, 32);
		memcpy(buf + pos + 32, (uint8_t *)s->u + off, len);
		pos += 32 + len;
	}
	return pos;
}

/* Decode buf fed in odd-sized pieces; collect samples and chunk infos. */
struct result {
	int16_t *iq;
	size_t n;
	struct mt7921iq_chunk ch[64];
	size_t ch_at[64];
	int nch;
	struct mt7921iq_stats st;
};

static int run_chain;		/* chain run() decodes (PACK96) */

static struct result run(const uint8_t *buf, size_t len)
{
	struct mt7921iq_dec *d = mt7921iq_dec_new();
	struct result r;
	size_t off = 0;

	mt7921iq_dec_set_chain(d, run_chain);
	memset(&r, 0, sizeof(r));
	r.iq = malloc(len * 2);

	while (off < len) {
		size_t piece = len - off < 3001 ? len - off : 3001;
		struct mt7921iq_chunk c;
		size_t got;

		CHECK(mt7921iq_dec_push(d, buf + off, piece) == 0);
		off += piece;
		while ((got = mt7921iq_dec_read(d, MT7921IQ_CS16,
						r.iq + 2 * r.n, 777, &c))) {
			if (c.flags && r.nch < 64) {
				r.ch[r.nch] = c;
				r.ch_at[r.nch++] = r.n;
			}
			r.n += got;
		}
	}
	mt7921iq_dec_flush(d);
	for (;;) {
		struct mt7921iq_chunk c;
		size_t got = mt7921iq_dec_read(d, MT7921IQ_CS16,
					       r.iq + 2 * r.n, 777, &c);

		if (!got)
			break;
		if (c.flags && r.nch < 64) {
			r.ch[r.nch] = c;
			r.ch_at[r.nch++] = r.n;
		}
		r.n += got;
	}
	mt7921iq_dec_stats(d, &r.st);
	mt7921iq_dec_free(d);
	return r;
}

static int matches(const struct result *r, size_t at, uint64_t first,
		   size_t n)
{
	size_t k;

	for (k = 0; k < n; k++) {
		uint32_t w = word(first + k);

		if (r->iq[2 * (at + k)] != sx14(w >> 16) ||
		    r->iq[2 * (at + k) + 1] != sx14(w))
			return 0;
	}
	return 1;
}

static void test_plain_and_slips(void)
{
	struct stream s = make_stream(20000, 1600, 0);
	uint8_t *buf = malloc(s.n * 8 * 2);
	struct result r;
	size_t len;

	slip_line(&s, 0);
	slip_line(&s, 5);
	slip_line(&s, 1234);
	/* The line split by the first block boundary (cursor 1600 starts at
	 * line unit 8): slip only its tail half in block 0, and the head half
	 * of the next split line in block 1.
	 */
	slip_units(&s, BLOCK / 8 - 8, 8);
	slip_units(&s, 2 * BLOCK / 8, 8);
	len = emit(buf, &s, 77, 10, NULL, 0);
	r = run(buf, len);
	CHECK(r.n == 40000);
	CHECK(matches(&r, 0, 0, 40000));
	CHECK(r.st.slip_lines == 5);
	CHECK(r.st.fallback_lines == 0);
	CHECK(r.nch == 1 && r.ch[0].flags == MT7921IQ_F_START &&
	      r.ch[0].generation == 77 && r.ch[0].sample_index == 0);
	CHECK(r.st.overruns == 0);
	free(buf);
	free(r.iq);
	free(s.u);
}

static void test_zero_samples_kept(void)
{
	struct stream s = make_stream(4000, 1600, 0);
	uint8_t *buf = malloc(s.n * 8 * 2);
	struct result r;
	size_t len, b = line_at(&s, 9);

	s.u[b + 2] = 0;		/* data unit: two zero samples */
	s.u[b + 4] = 0;
	slip_line(&s, 20);
	s.u[line_at(&s, 20)] = s.u[line_at(&s, 20) + 1] = 0; /* slipped D0 = 0 */
	len = emit(buf, &s, 1, 0, NULL, 0);
	r = run(buf, len);
	CHECK(r.n == 8000);
	CHECK(r.st.fallback_lines == 0);
	CHECK(r.iq[2 * (b + 2)] == 0);	/* unit b+2 holds samples b+2, b+3 */
	free(buf);
	free(r.iq);
	free(s.u);
}

static void test_seq_gap_and_generation(void)
{
	struct stream a = make_stream(8192, 1600, 0);
	struct stream b = make_stream(4096, 64, 0);
	uint8_t *buf = malloc((a.n + b.n) * 8 * 2);
	uint32_t skip[] = { 3 };
	struct result r;
	size_t len;

	len = emit(buf, &a, 5, 0, skip, 1);
	len += emit(buf + len, &b, 6, 0, NULL, 0);
	r = run(buf, len);
	/* a: 32 blocks of 512 samples each, block seq 3 missing */
	CHECK(r.nch == 3);
	CHECK(r.ch[1].flags & MT7921IQ_F_SEQ_GAP);
	CHECK(r.ch[1].lost_blocks == 1);
	CHECK(r.ch[1].sample_index == 4 * 512);	/* after the lost block */
	CHECK(r.ch_at[1] == 3 * 512);
	CHECK(matches(&r, 0, 0, 3 * 512));
	CHECK(matches(&r, 3 * 512, 4 * 512, 16384 - 4 * 512));
	CHECK(r.ch[2].flags == MT7921IQ_F_START && r.ch[2].generation == 6 &&
	      r.ch[2].sample_index == 0);
	CHECK(r.ch_at[2] == 16384 - 512);
	CHECK(matches(&r, r.ch_at[2], 0, 8192));
	free(buf);
	free(r.iq);
	free(a.u);
	free(b.u);
}

static void test_overrun(void)
{
	struct stream s = make_stream(40000, 1600, 0);
	uint8_t *buf = malloc(s.n * 8 * 2);
	struct result r;
	size_t len, b = line_at(&s, 300), k;

	/* 5 lines (40 pairs) read one lap late: they carry the pairs one
	 * lap (RING / 16 pairs) later, which then arrive again in their own
	 * position. */
	for (k = 0; k < 80; k++)
		s.u[b + k] = s.u[b + k + RING / 8];
	len = emit(buf, &s, 9, 0, NULL, 0);
	r = run(buf, len);
	CHECK(r.n == 80000);
	CHECK(r.st.overruns == 1);
	CHECK(r.nch == 2 && (r.ch[1].flags & MT7921IQ_F_OVERRUN));
	CHECK(r.ch[1].overrun_start == (b / 2) * 2);
	CHECK(r.ch[1].overrun_len == 80);
	free(buf);
	free(r.iq);
	free(s.u);
}

static void test_fallback(void)
{
	struct stream s = make_stream(4000, 0, 0);
	uint8_t *buf = malloc(s.n * 8 * 2);
	struct result r;
	size_t len, b = line_at(&s, 7);

	s.u[b + 5] = 0x1234;	/* garbage in a padding slot */
	len = emit(buf, &s, 1, 0, NULL, 0);
	r = run(buf, len);
	CHECK(r.st.fallback_lines == 1);
	CHECK(r.n == 8002);	/* 8 data units + the garbage unit */
	CHECK(matches(&r, 0, 0, b));
	free(buf);
	free(r.iq);
	free(s.u);
}

static void test_framing_error(void)
{
	struct mt7921iq_dec *d = mt7921iq_dec_new();
	uint8_t junk[64] = { 1, 2, 3 };

	CHECK(mt7921iq_dec_push(d, junk, sizeof(junk)) == -1);
	mt7921iq_dec_free(d);
}


/* ---- PACK96 (40 MHz) ---------------------------------------------------- */

static uint32_t pack_iq(double i, double q)
{
	int ii = (int)lrint(i), qq = (int)lrint(q);

	if (ii > 8191) ii = 8191;
	if (ii < -8192) ii = -8192;
	if (qq > 8191) qq = 8191;
	if (qq < -8192) qq = -8192;
	return (uint32_t)(ii & 0x3fff) << 16 | (uint32_t)(qq & 0x3fff);
}

static double rnd(uint64_t *st)
{
	*st = *st * 6364136223846793005ULL + 1442695040888963407ULL;
	return ((*st >> 11) * (1.0 / 9007199254740992.0)) - 0.5;
}

/*
 * Two RX chains at 40 MS/s: lane 0 = tone + low-passed noise, lane 1 =
 * 0.6 * rotated lane 0 + its own noise. Group words [l0(t) l0(t+1) l1(t)].
 * Returns the word stream starting `skip` words into the first group.
 */
static uint32_t *make_groups(size_t groups, int skip, uint32_t **lane0,
			     size_t *nwords)
{
	uint32_t *w = malloc(groups * 3 * sizeof(*w));
	uint32_t *l0 = malloc(groups * 2 * sizeof(*l0));
	uint64_t st = 12345;
	double pi = 0, pq = 0, ph = 0;
	size_t g;

	for (g = 0; g < groups; g++) {
		double i0[2], q0[2], n1i, n1q;
		int k;

		for (k = 0; k < 2; k++) {
			double ni = 600 * rnd(&st), nq = 600 * rnd(&st);

			ph += 2 * M_PI * 0.0731;
			i0[k] = 900 * cos(ph) + 0.5 * (ni + pi);
			q0[k] = 900 * sin(ph) + 0.5 * (nq + pq);
			pi = ni;
			pq = nq;
		}
		n1i = 300 * rnd(&st);
		n1q = 300 * rnd(&st);
		w[3 * g] = pack_iq(i0[0], q0[0]);
		w[3 * g + 1] = pack_iq(i0[1], q0[1]);
		/* 0.6 * exp(j 2.4) * l0(t) + noise */
		w[3 * g + 2] = pack_iq(0.6 * (cos(2.4) * i0[0] - sin(2.4) * q0[0]) + n1i,
				       0.6 * (sin(2.4) * i0[0] + cos(2.4) * q0[0]) + n1q);
		l0[2 * g] = w[3 * g];
		l0[2 * g + 1] = w[3 * g + 1];
	}
	*lane0 = l0;
	*nwords = groups * 3 - skip;
	memmove(w, w + skip, *nwords * sizeof(*w));
	return w;
}

/* Serialize words as PACK96 blocks (4 KiB) from ring address cursor0. */
static size_t emit96(uint8_t *buf, const uint32_t *w, size_t nwords,
		     uint32_t cursor0, uint32_t gen, const uint32_t *skip,
		     int nskip)
{
	size_t pos = 0, off, bytes = nwords * 4 / 8 * 8;
	uint32_t seq = 0;

	for (off = 0; off < bytes; off += BLOCK, seq++) {
		uint32_t hdr[8], len = BLOCK;
		int k, skipped = 0;

		if (off + len > bytes)
			len = bytes - off;
		for (k = 0; k < nskip; k++)
			skipped |= skip[k] == seq;
		if (skipped)
			continue;
		hdr[0] = MT7921IQ_IQB2_MAGIC;
		hdr[1] = 1 | 32 << 16;
		hdr[2] = len;
		hdr[3] = seq;
		hdr[4] = (cursor0 + off) % RING;
		hdr[5] = 1 | MT7921_IQ_FMT_PACK96 << 4 | ring_flags();
		hdr[6] = gen;
		hdr[7] = 1000 + seq * 17;
		memcpy(buf + pos, hdr, 32);
		memcpy(buf + pos + 32, (const uint8_t *)w + off, len);
		pos += 32 + len;
	}
	return pos;
}

/* Lane-0 words of a word stream that starts `skip` words into a group. */
static size_t expect_lane0(const uint32_t *w, size_t n, int skip, uint32_t *out)
{
	size_t k, m = 0;

	for (k = 0; k < n; k++)
		if ((k + skip) % 3 < 2)
			out[m++] = w[k];
	return m;
}

static int same_samples(const struct result *r, size_t at, const uint32_t *e,
			size_t n)
{
	size_t k;

	for (k = 0; k < n; k++)
		if (r->iq[2 * (at + k)] != sx14(e[k] >> 16) ||
		    r->iq[2 * (at + k) + 1] != sx14(e[k]))
			return 0;
	return 1;
}

static void test96_phases(void)
{
	int skip;

	for (skip = 0; skip < 3; skip++) {
		uint32_t *l0, *w, *e;
		size_t nw, ne, len;
		uint8_t *buf;
		struct result r;

		w = make_groups(60000, skip, &l0, &nw);
		e = malloc(nw * sizeof(*e));
		ne = expect_lane0(w, nw / 2 * 2, skip, e);
		buf = malloc(nw * 4 * 2);
		len = emit96(buf, w, nw, 1600, 42, NULL, 0);
		r = run(buf, len);
		CHECK(r.st.phase_detections == 1);
		CHECK(r.n == ne);
		CHECK(same_samples(&r, 0, e, ne));
		CHECK(r.nch == 1 && r.ch[0].format == MT7921_IQ_FMT_PACK96 &&
		      r.ch[0].sample_rate == MT7921IQ_SAMPLE_RATE_40);
		if (r.n != ne || !same_samples(&r, 0, e, ne))
			fprintf(stderr, "  skip %d: n %zu want %zu, scores %u/%u\n",
				skip, r.n, ne, r.st.phase_score_best,
				r.st.phase_score_next);
		free(w); free(l0); free(e); free(buf); free(r.iq);
	}
}

/* Chain 1: lane-1 words (group position 2) at 20.0008 MS/s, any phase. */
static void test96_chain1(void)
{
	int skip;

	run_chain = 1;
	for (skip = 0; skip < 3; skip++) {
		uint32_t *l0, *w, *e;
		size_t nw, ne = 0, len, k;
		uint8_t *buf;
		struct result r;

		w = make_groups(60000, skip, &l0, &nw);
		e = malloc(nw * sizeof(*e));
		for (k = 0; k < nw / 2 * 2; k++)	/* whole 8-byte units */
			if ((k + skip) % 3 == 2)
				e[ne++] = w[k];
		buf = malloc(nw * 4 * 2);
		len = emit96(buf, w, nw, 1600, 42, NULL, 0);
		r = run(buf, len);
		CHECK(r.st.phase_detections == 1);
		CHECK(r.n == ne);
		CHECK(same_samples(&r, 0, e, r.n < ne ? r.n : ne));
		CHECK(r.nch == 1 && r.ch[0].sample_rate == MT7921IQ_SAMPLE_RATE);
		if (r.n != ne)
			fprintf(stderr, "  chain 1, skip %d: n %zu want %zu\n",
				skip, r.n, ne);
		free(w); free(l0); free(e); free(buf); free(r.iq);
	}
	run_chain = 0;
}

/* Slips: inside a block and in the tail segment before a block boundary. */
static void test96_slips(void)
{
	uint32_t *l0, *w, *e, *ws;
	size_t nw, ne, len, k;
	uint64_t *u;
	uint8_t *buf;
	struct result r;
	/* unit index of a line start (cursor 1600 -> lines start at unit 8) */
	size_t lines[] = { 8 + 16 * 2000, 8 + 16 * 5000 };
	size_t tail = 512 * 20 - 8;	/* last 8-unit segment of block 19 */

	w = make_groups(60000, 0, &l0, &nw);
	nw = nw / 2 * 2;
	ws = malloc(nw * sizeof(*ws));
	memcpy(ws, w, nw * sizeof(*ws));
	u = (uint64_t *)ws;
	/* expected: the lost last unit of each slipped segment is held */
	for (k = 0; k < 2; k++) {
		size_t b = lines[k];

		memmove(u + b + 1, u + b, 15 * 8);	/* D0 D0 D1 ... D14 */
	}
	memmove(u + tail + 1, u + tail, 7 * 8);
	e = malloc(nw * sizeof(*e));
	{
		/* build the expected lane-0 output with holds */
		uint32_t last = 0;
		size_t m = 0;

		for (k = 0; k < nw; k++) {
			size_t unit = k / 2;
			int lost = unit == lines[0] + 15 || unit == lines[1] + 15 ||
				   unit == tail + 7;

			if (k % 3 < 2) {
				e[m++] = lost ? last : w[k];
				last = e[m - 1];
			}
		}
		ne = m;
	}
	buf = malloc(nw * 4 * 2);
	len = emit96(buf, ws, nw, 1600, 7, NULL, 0);
	r = run(buf, len);
	CHECK(r.st.slip_lines == 3);
	CHECK(r.n == ne);
	CHECK(same_samples(&r, 0, e, ne));
	CHECK(r.st.repaired_samples >= 3 && r.st.repaired_samples <= 6);
	free(w); free(ws); free(l0); free(e); free(buf); free(r.iq);
}

static void test96_gap_and_generation(void)
{
	uint32_t *l0, *w, *e;
	size_t nw, ne, len, len2, k;
	uint8_t *buf;
	struct result r;
	uint32_t skip[] = { 40 };

	w = make_groups(60000, 1, &l0, &nw);
	nw = nw / 2 * 2;
	e = malloc(nw * sizeof(*e));
	ne = expect_lane0(w, nw, 1, e);
	buf = malloc(nw * 4 * 3);
	len = emit96(buf, w, nw, 0, 9, skip, 1);
	len2 = emit96(buf + len, w, nw, 4096, 10, NULL, 0);
	r = run(buf, len + len2);
	CHECK(r.nch == 3);
	CHECK(r.ch[1].flags & MT7921IQ_F_SEQ_GAP);
	{
		/* block 40 held words [40*1024, 41*1024) */
		size_t before = 0, after;

		for (k = 0; k < 40 * 1024; k++)
			before += (k + 1) % 3 < 2;
		after = before;
		for (; k < 41 * 1024; k++)
			after += (k + 1) % 3 < 2;
		CHECK(r.ch_at[1] == before);
		CHECK(r.ch[1].sample_index == after);
		CHECK(same_samples(&r, 0, e, before));
		CHECK(same_samples(&r, before, e + after, ne - after));
		CHECK(r.ch[2].flags == MT7921IQ_F_START &&
		      r.ch[2].generation == 10);
		CHECK(same_samples(&r, r.ch_at[2], e, ne));
	}
	free(w); free(l0); free(e); free(buf); free(r.iq);
}

static void test96_overrun(void)
{
	uint32_t *l0, *w;
	size_t nw, len, k;
	uint8_t *buf;
	struct result r;
	uint64_t *u;

	w = make_groups(60000, 0, &l0, &nw);
	nw = nw / 2 * 2;
	u = (uint64_t *)w;
	/* 40 units read one lap (RING / 8 units) late */
	for (k = 0; k < 40; k++)
		u[70000 + k] = u[70000 + k + RING / 8];
	buf = malloc(nw * 4 * 2);
	len = emit96(buf, w, nw, 1600, 3, NULL, 0);
	r = run(buf, len);
	CHECK(r.st.overruns == 1);
	CHECK(r.nch == 2 && (r.ch[1].flags & MT7921IQ_F_OVERRUN));
	free(w); free(l0); free(buf); free(r.iq);
}


/* ---- RAW80 (80 MHz snapshots) ------------------------------------------- */

/* One snapshot: META + RING / 4 KiB blocks of a ring whose chronological
 * order starts at record (stop + 4) / 4; lane-0 sample n = word(base + n),
 * lane 1 = 0x5555. legacy: the 16-byte META of older drivers (16 KiB only).
 */
static size_t emit_snap(uint8_t *buf, uint32_t gen, uint32_t snap,
			uint32_t stop, uint64_t base, int legacy)
{
	static uint32_t ring[RING_MAX / 4];
	uint32_t hdr[8], meta[5], k, mlen = legacy ? 16 : 20;
	uint32_t start = (((stop + 4) & ~3u) % (RING / 4)) / 4;
	size_t pos = 0;

	for (k = 0; k < RING / 16; k++) {
		uint32_t r = (start + k) % (RING / 16);

		ring[4 * r] = word(base + 2 * k);
		ring[4 * r + 1] = 0x5555;
		ring[4 * r + 2] = word(base + 2 * k + 1);
		ring[4 * r + 3] = 0x5555;
	}
	hdr[0] = MT7921IQ_IQB2_MAGIC;
	hdr[1] = 1 | 32 << 16;
	hdr[2] = mlen;
	hdr[3] = snap;
	hdr[4] = stop;
	hdr[5] = MT7921_IQ_FMT_META << 4 | (legacy ? 0 : ring_flags());
	hdr[6] = gen;
	hdr[7] = 0;
	meta[0] = MT7921_IQ_SNAP_MAGIC;
	meta[1] = stop;
	meta[2] = 5775000;
	meta[3] = 80;
	meta[4] = RING;
	memcpy(buf + pos, hdr, 32);
	memcpy(buf + pos + 32, meta, mlen);
	pos += 32 + mlen;
	for (k = 0; k < RING / BLOCK; k++) {
		hdr[2] = BLOCK;
		hdr[3] = k;
		hdr[4] = k * BLOCK;
		hdr[5] = MT7921_IQ_FMT_RAW80 << 4 | (legacy ? 0 : ring_flags());
		hdr[7] = 5000 + 1000 * snap + k;
		memcpy(buf + pos, hdr, 32);
		memcpy(buf + pos + 32, (uint8_t *)ring + k * BLOCK, BLOCK);
		pos += 32 + BLOCK;
	}
	return pos;
}

static void test80_snapshots(int legacy)
{
	uint8_t *buf = malloc(3 * (52 + RING_MAX / BLOCK * (32 + BLOCK)));
	uint32_t n = RING / 8;		/* lane-0 samples per snapshot */
	struct result r;
	size_t len = 0;

	len += emit_snap(buf + len, 77, 0, 0x3fc, 0, legacy);
	/* stop pointer at the last record of the ring: wraps to record 0 */
	len += emit_snap(buf + len, 77, 1, RING / 4 - 4, 100000, legacy);
	r = run(buf, len);
	CHECK(r.n == 2 * n);
	CHECK(r.st.snapshots == 2 && r.st.snap_incomplete == 0);
	CHECK(r.st.malformed == 0);
	CHECK(r.nch == 2);
	CHECK(r.ch[0].flags == (MT7921IQ_F_START | MT7921IQ_F_SNAPSHOT) &&
	      r.ch[0].format == MT7921_IQ_FMT_RAW80 &&
	      r.ch[0].sample_rate == MT7921IQ_SAMPLE_RATE_80);
	CHECK(r.ch_at[1] == n && r.ch[1].sample_index == n);
	CHECK(matches(&r, 0, 0, n));
	CHECK(matches(&r, n, 100000, n));
	free(buf);
	free(r.iq);
}

/* ---- DENSE40/80 ----------------------------------------------------------- */

/* Serialize one word per sample as IQB2 records of format fmt. */
static size_t emit_dense(uint8_t *buf, const uint32_t *w, size_t nwords,
			 uint32_t fmt, uint32_t gen, uint32_t seq0,
			 uint32_t cursor0, uint32_t skip_seq)
{
	size_t pos = 0, off, bytes = nwords * 4;
	uint32_t seq = seq0, tick = 1000;

	for (off = 0; off < bytes; off += BLOCK, seq++, tick += 13) {
		uint32_t hdr[8], len = BLOCK;

		if (off + len > bytes)
			len = bytes - off;
		if (seq == skip_seq)
			continue;
		hdr[0] = MT7921IQ_IQB2_MAGIC;
		hdr[1] = 1 | 32 << 16;
		hdr[2] = len;
		hdr[3] = seq;
		hdr[4] = (cursor0 + off) % RING;
		hdr[5] = 1 | ring_flags() | fmt << 4;
		hdr[6] = gen;
		hdr[7] = tick;
		memcpy(buf + pos, hdr, 32);
		memcpy(buf + pos + 32, (const uint8_t *)w + off, len);
		pos += 32 + len;
	}
	return pos;
}

static void test_dense(uint32_t fmt)
{
	size_t n = RING / 4 + 6000, b = RING / 4 + 1000, k, len;
	uint32_t *w = malloc(n * 4);
	uint8_t *buf = malloc((n * 4 / BLOCK + 2) * (32 + BLOCK));
	double rate = fmt == MT7921_IQ_FMT_DENSE80 || fmt == MT7921_IQ_FMT_DENSE80_C1 ?
		      MT7921IQ_SAMPLE_RATE_80 : MT7921IQ_SAMPLE_RATE_40;
	struct result r;

	/* plain, continuous */
	for (k = 0; k < n; k++)
		w[k] = word(k);
	len = emit_dense(buf, w, n, fmt, 9, 0, 0x340, ~0u);
	r = run(buf, len);
	CHECK(r.n == n);
	CHECK(matches(&r, 0, 0, n));
	CHECK(r.nch == 1 && r.ch[0].flags == MT7921IQ_F_START &&
	      r.ch[0].format == fmt && r.ch[0].sample_rate == rate);
	CHECK(r.st.overruns == 0 && r.st.garbage_words == 0);
	free(r.iq);

	/* a lost block: 1024 samples skipped in the index */
	len = emit_dense(buf, w, n, fmt, 9, 0, 0x340, 3);
	r = run(buf, len);
	CHECK(r.n == n - BLOCK / 4);
	CHECK(r.nch == 2 && (r.ch[1].flags & MT7921IQ_F_SEQ_GAP) &&
	      r.ch[1].lost_blocks == 1 &&
	      r.ch[1].sample_index == 4 * (BLOCK / 4));
	CHECK(matches(&r, 0, 0, 3 * (BLOCK / 4)));
	CHECK(matches(&r, 3 * (BLOCK / 4), 4 * (BLOCK / 4), 2000));
	free(r.iq);

	/* garbage words (spare bits set) come out as 0 and are counted */
	w[500] = 0x40000c4f;
	w[501] = 0x80003bff;
	w[777] = 0x0000c000;
	len = emit_dense(buf, w, n, fmt, 9, 0, 0x340, ~0u);
	r = run(buf, len);
	CHECK(r.n == n && r.st.garbage_words == 3);
	CHECK(r.iq[2 * 500] == 0 && r.iq[2 * 500 + 1] == 0 &&
	      r.iq[2 * 501] == 0 && r.iq[2 * 777 + 1] == 0);
	CHECK(matches(&r, 0, 0, 500) && matches(&r, 502, 502, 275) &&
	      matches(&r, 778, 778, n - 778));
	free(r.iq);
	for (k = 0; k < n; k++)
		w[k] = word(k);

	/* 80 samples read one lap (RING / 4 samples) late: a repeat of the
	 * data one lap earlier
	 */
	for (k = 0; k < 80; k++)
		w[b + k] = w[b + k - RING / 4];
	len = emit_dense(buf, w, n, fmt, 9, 0, 0, ~0u);
	r = run(buf, len);
	CHECK(r.n == n && r.st.overruns == 1);
	CHECK(r.nch == 2 && (r.ch[1].flags & MT7921IQ_F_OVERRUN) &&
	      r.ch[1].overrun_start == b - RING / 4 && r.ch[1].overrun_len == 80);
	free(r.iq);

	free(w);
	free(buf);
}

int main(void)
{
	static const uint32_t rings[] = { 16384, 65536 };
	static const uint32_t rings_dense[] = { 16384, 65536, 262144 };
	size_t i;

	for (i = 0; i < 3; i++) {
		RING = rings_dense[i];
		test_dense(MT7921_IQ_FMT_DENSE40);
		test_dense(MT7921_IQ_FMT_DENSE80);
		test_dense(MT7921_IQ_FMT_DENSE40_C1);
		test_dense(MT7921_IQ_FMT_DENSE80_C1);
		if (failures)
			fprintf(stderr, "(dense, ring %u)\n", RING);
	}
	RING = 262144;
	test80_snapshots(0);
	if (failures)
		fprintf(stderr, "(snapshots, ring %u)\n", RING);

	for (i = 0; i < 2; i++) {
		RING = rings[i];
		test_plain_and_slips();
		test_zero_samples_kept();
		test_seq_gap_and_generation();
		test_overrun();
		test_fallback();
		test96_phases();
		test96_chain1();
		test96_slips();
		test96_gap_and_generation();
		test96_overrun();
		test80_snapshots(0);
		if (failures)
			fprintf(stderr, "(ring %u)\n", RING);
	}
	RING = 16384;
	test80_snapshots(1);		/* 16-byte META of older drivers */
	test_framing_error();
	if (failures) {
		fprintf(stderr, "%d check(s) failed\n", failures);
		return 1;
	}
	printf("all tests passed\n");
	return 0;
}
