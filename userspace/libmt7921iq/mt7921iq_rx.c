// SPDX-License-Identifier: BSD-3-Clause-Clear
/*
 * mt7921iq_rx: record decoded I/Q from /dev/mt7921_iq_<phy> (like rtl_sdr).
 *
 *   mt7921iq_rx [-d DEV] [-F MHZ [-w 20|40|80]] [-c 0|1] [-g LNA:LPF]
 *               [-t SECONDS] [-f cs16|cf32] [-R RAW] OUT|-
 *   mt7921iq_rx -D RAW [-f cs16|cf32] OUT|-     decode a raw capture
 *
 * The device must be on a monitor interface; opening it starts the stream.
 * Chunk events (generation changes, gaps, overruns) and final counters go to
 * stderr. -R also keeps the raw IQB2 stream for offline analysis. -g holds
 * the receive gain at an analog code (5 GHz: LNA 2-5, LPF 0-8; LNA 5, LPF 7
 * is the AGC's idle gain) instead of the AGC.
 */
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "mt7921iq.h"
#include "iqstream_ioctl.h"

#define RAW_READ	(4 << 20)
#define OUT_SAMPLES	(1 << 18)

static volatile sig_atomic_t stop;

static void on_signal(int sig)
{
	(void)sig;
	stop = 1;
}

static double now(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static void usage(void)
{
	fprintf(stderr,
		"usage: mt7921iq_rx [-d DEV] [-F MHZ [-w 20|40|80]] [-c 0|1] [-g LNA:LPF] [-t SECONDS] [-f cs16|cf32] [-R RAW] OUT|-\n"
		"       mt7921iq_rx -D RAW [-f cs16|cf32] OUT|-\n");
	exit(2);
}

static int want_chain;

static void report(const struct mt7921iq_chunk *c)
{
	static int warned;

	if ((c->flags & MT7921IQ_F_START) && !(c->flags & MT7921IQ_F_SNAPSHOT))
		fprintf(stderr, "chunk: generation %u starts, %.4f MS/s\n",
			c->generation, c->sample_rate / 1e6);
	if ((c->flags & MT7921IQ_F_START) && want_chain && !warned &&
	    (c->format == MT7921_IQ_FMT_DENSE40 || c->format == MT7921_IQ_FMT_DENSE80)) {
		fprintf(stderr, "warning: -c 1 but the DENSE stream carries chain 0; "
			"chain 1 is selected when tuning (-F)\n");
		warned = 1;
	}
	if (c->flags & MT7921IQ_F_SEQ_GAP)
		fprintf(stderr, "chunk: %u block(s) lost before sample %llu\n",
			c->lost_blocks, (unsigned long long)c->sample_index);
	if (c->flags & MT7921IQ_F_CURSOR)
		fprintf(stderr, "chunk: cursor break at sample %llu\n",
			(unsigned long long)c->sample_index);
	if (c->flags & MT7921IQ_F_OVERRUN)
		fprintf(stderr, "chunk: ring overrun, samples %llu..%llu invalid\n",
			(unsigned long long)c->overrun_start,
			(unsigned long long)(c->overrun_start + c->overrun_len));
}

/* Write all decoded samples; returns -1 on a write error. */
static int drain(struct mt7921iq_dec *d, enum mt7921iq_format fmt,
		 void *out, FILE *fo)
{
	size_t elem = fmt == MT7921IQ_CS16 ? 4 : 8;
	struct mt7921iq_chunk c;
	size_t n;

	while ((n = mt7921iq_dec_read(d, fmt, out, OUT_SAMPLES, &c))) {
		if (c.flags)
			report(&c);
		if (fwrite(out, elem, n, fo) != n)
			return -1;
	}
	return 0;
}

int main(int argc, char **argv)
{
	const char *dev = NULL, *raw_in = NULL, *raw_out = NULL;
	enum mt7921iq_format fmt = MT7921IQ_CS16;
	double seconds = 0, tune_mhz = 0, t0, dt;
	unsigned width = 0;
	int chain = 0, hold = 0;
	unsigned lna = 0, lpf = 0;
	struct mt7921iq_stats st;
	struct mt7921iq_dec *d;
	FILE *fo, *fr = NULL;
	uint8_t *raw;
	void *out;
	int opt, fd, ret = 0;
	glob_t g;

	while ((opt = getopt(argc, argv, "d:F:w:c:g:t:f:R:D:h")) != -1) {
		switch (opt) {
		case 'g':
			if (sscanf(optarg, "%u:%u", &lna, &lpf) != 2)
				usage();
			hold = 1;
			break;
		case 'w': width = atoi(optarg); break;
		case 'c': chain = atoi(optarg); break;
		case 'F': tune_mhz = atof(optarg); break;
		case 'd': dev = optarg; break;
		case 't': seconds = atof(optarg); break;
		case 'R': raw_out = optarg; break;
		case 'D': raw_in = optarg; break;
		case 'f':
			if (!strcmp(optarg, "cs16"))
				fmt = MT7921IQ_CS16;
			else if (!strcmp(optarg, "cf32"))
				fmt = MT7921IQ_CF32;
			else
				usage();
			break;
		default: usage();
		}
	}
	if (optind + 1 != argc)
		usage();

	fo = strcmp(argv[optind], "-") ? fopen(argv[optind], "wb") : stdout;
	if (!fo) {
		perror(argv[optind]);
		return 1;
	}
	if (raw_in) {
		fd = open(raw_in, O_RDONLY);
	} else {
		if (!dev) {
			if (glob("/dev/mt7921_iq_*", 0, NULL, &g) || !g.gl_pathc) {
				fprintf(stderr, "no /dev/mt7921_iq_* device\n");
				return 1;
			}
			dev = strdup(g.gl_pathv[0]);
			globfree(&g);
		}
		fd = open(dev, O_RDONLY);
	}
	if (fd < 0) {
		perror(raw_in ? raw_in : dev);
		if (errno == ENETDOWN)
			fprintf(stderr, "the interface must be up in monitor mode\n");
		return 1;
	}
	if (tune_mhz && !raw_in) {
		double hz;
		uint32_t gen;
		unsigned bw;
		/* -c 1: dense streams switch to chain 1 at the tune; PACK96
		 * takes lane 1 in the decoder (mt7921iq_dec_set_chain()).
		 */
		int err = mt7921iq_tune_chain(fd, tune_mhz * 1e6, width, chain,
					      &hz, &bw, &gen);

		if (err) {
			fprintf(stderr, "tune %.3f MHz: %s\n", tune_mhz,
				strerror(-err));
			return 1;
		}
		fprintf(stderr, "tuned to %.3f MHz, %u MHz wide, generation %u\n",
			hz / 1e6, bw, gen);
	}
	if (hold && !raw_in) {
		int err = mt7921iq_set_gain(fd, 1, lna, lpf);

		if (err) {
			fprintf(stderr, "gain hold LNA %u LPF %u: %s\n", lna, lpf,
				strerror(-err));
			return 1;
		}
		fprintf(stderr, "gain held at LNA %u, LPF %u\n", lna, lpf);
	}
	if (raw_out && !(fr = fopen(raw_out, "wb"))) {
		perror(raw_out);
		return 1;
	}

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	d = mt7921iq_dec_new();
	mt7921iq_dec_set_chain(d, chain);
	want_chain = chain;
	raw = malloc(RAW_READ);
	out = malloc(OUT_SAMPLES * 8);
	t0 = now();

	while (!stop && (!seconds || now() - t0 < seconds)) {
		struct pollfd p = { .fd = fd, .events = POLLIN };
		ssize_t n;

		if (!raw_in && poll(&p, 1, 100) <= 0)
			continue;
		n = read(fd, raw, RAW_READ);
		if (n < 0) {
			if (errno == EINTR)
				continue;
			perror("read");
			ret = 1;
			break;
		}
		if (!n) {
			if (!raw_in)
				fprintf(stderr, "stream ended by the driver\n");
			break;
		}
		if (fr && fwrite(raw, 1, n, fr) != (size_t)n) {
			perror(raw_out);
			ret = 1;
			break;
		}
		if (mt7921iq_dec_push(d, raw, n)) {
			fprintf(stderr, "IQB2 framing error\n");
			ret = 1;
			break;
		}
		if (drain(d, fmt, out, fo)) {
			perror("write");
			ret = 1;
			break;
		}
	}
	dt = now() - t0;
	close(fd);
	mt7921iq_dec_flush(d);
	drain(d, fmt, out, fo);
	mt7921iq_dec_stats(d, &st);

	fprintf(stderr,
		"%.2f s: %llu samples (%.3f MS/s), %llu blocks, %llu generation(s)\n"
		"lines %llu, slips %llu, fallback %llu; seq gaps %llu (%llu blocks), "
		"cursor breaks %llu, overruns %llu (%llu samples), malformed %llu\n"
		"40 MHz: repaired samples %llu, phase detections %llu (score %.3f, next %.3f), "
		"dropped words %llu\n"
		"80 MHz: snapshots %llu, incomplete %llu; dense: garbage words %llu\n",
		dt, (unsigned long long)st.samples, st.samples / dt / 1e6,
		(unsigned long long)st.blocks,
		(unsigned long long)st.generations,
		(unsigned long long)st.lines, (unsigned long long)st.slip_lines,
		(unsigned long long)st.fallback_lines,
		(unsigned long long)st.seq_gaps,
		(unsigned long long)st.lost_blocks,
		(unsigned long long)st.cursor_breaks,
		(unsigned long long)st.overruns,
		(unsigned long long)st.overrun_samples,
		(unsigned long long)st.malformed,
		(unsigned long long)st.repaired_samples,
		(unsigned long long)st.phase_detections,
		st.phase_score_best / 1000.0, st.phase_score_next / 1000.0,
		(unsigned long long)st.detect_dropped_words,
		(unsigned long long)st.snapshots,
		(unsigned long long)st.snap_incomplete,
		(unsigned long long)st.garbage_words);

	if (fr)
		fclose(fr);
	if (fo != stdout)
		fclose(fo);
	mt7921iq_dec_free(d);
	free(raw);
	free(out);
	return ret;
}
