# libmt7921iq — IQB2 stream decoder

Decodes the stream of `/dev/mt7921_iq_<phy>` (driver `driver/mt7921/iqstream.c`)
into complex samples, for the SoapySDR module and for recording.

- `mt7921iq.h` / `mt7921iq.c`: C99 decoder, no dependencies. It takes raw
  bytes in (`mt7921iq_dec_push`) and gives out CS16 or CF32 chunks
  (`mt7921iq_dec_read`). A chunk never spans a discontinuity: generation
  change, sequence gap, cursor break or ring overrun.
- `mt7921iq_rx`: recorder in the style of `rtl_sdr`.
  `mt7921iq_rx [-d DEV] [-F MHZ [-w 20|40|80]] [-c 0|1] [-g LNA:LPF] [-t S] [-f cs16|cf32] [-R RAW] OUT|-`; with
  `-D RAW` it decodes a saved raw stream instead of reading the device.
  `-g` holds the receive gain (below).
- `test_mt7921iq`: synthetic stream tests (`ctest`).

Build: `cmake -S . -B build && cmake --build build && (cd build && ctest)`.

## Tuning (for gqrx / SoapySDR)

`mt7921iq_tune(fd, hz, &actual, &generation)` wraps the driver ioctl
`MT7921_IQ_IOC_TUNE` (`driver/mt7921/iqstream_ioctl.h`).
- **What the driver does:** it stops the firmware stream, sends the
  sniffer channel command itself (bypassing mac80211), and restarts.
- **Gap:** 15–23 ms per retune, against ~212 ms through `iw set freq` and
  mac80211.
- **Raster:** 5 MHz, including channel numbers outside the regulatory list
  (for example 5790 or 5795 MHz). The ranges are 2412–2472, 2484 and
  5005–5995 MHz; outside them the call fails with `ERANGE`.
- **Frequency tagging:** the returned generation is the first one at the new
  frequency. Samples of older generations still in the pipeline belong to
  the old frequency.
- **Closing:** closing the device hands the receiver back to mac80211's
  channel.

Verified with a CW tone: the tone lands exactly 5 MHz further on each step,
both at 2.4 GHz and at non-regulatory 5 GHz raster points.

**Access without root:** install `../udev/99-mt7921-iq.rules` (group
`plugdev`), or run `sudo chmod 666 /dev/mt7921_iq_*` (lost on a driver
reload or replug), and run `../mt7921iq-setup.sh [MHz]` once as root, which puts the
interface into monitor mode. After that, streaming and retuning need only
the device node.

## Receive gain

`mt7921iq_set_gain(fd, hold, lna, lpf)` wraps the driver ioctl
`MT7921_IQ_IOC_SET_GAIN`.
- `hold = 0` is the chip's AGC, the default after every open.
- `hold = 1` fixes the streamed RX chain(s) at the analog code. It applies
  live, with no gap, survives retunes, and is released on close.
- **5 GHz codes** (`MT7921IQ_LNA_*`, `MT7921IQ_LPF_*`):
  - LNA 2–5 at about 6 dB per step; it sets the noise figure;
  - LPF 0–8 at about 2 dB per step, same SNR;
  - LNA 5 / LPF 7 is the AGC's idle gain.
- **Under the hold:** the AGC's gain dips on other stations' frames are
  gone, and the output is about 13 dB below the AGC's level.
- **Remains:** blanking runs (exact zeros).
- 2.4 GHz codes are not mapped yet.

## 40 MHz (PACK96)

`mt7921iq_rx -F <center MHz> -w 40` (or the Soapy rate 40.0016 MS/s) streams
lane 0 of a 40 MHz channel at 40.0016 MS/s; about 36 MHz is usable (flat to
±17 MHz, −6 dB at ±19.5). The wire carries dense groups
`[l0(t), l0(t+1), l1(t)]` at 240 MB/s.
- **Group phase:** detected per continuous run from the first 98 k words
  (`phase_score_best/next` in the stats).
- **Slips:** in this dense format each slip loses one 8-byte unit. Those
  lane-0 samples are held and counted as `repaired_samples` (2.3·10⁻⁶ of
  samples in a 60 s run).
- **Format detection:** the format comes from the IQB2 header flags (bits
  7:4), so old captures (flags 1) decode as PACK32.
- **Chain 1:** `mt7921iq_rx -c 1` (`mt7921iq_dec_set_chain()`) outputs
  lane 1 instead, the second RX chain. It runs at 20.0008 MS/s: every
  other sample of a 40 MS/s lane, with no filter, so the channel's content
  beyond ±10 MHz folds into the band. Useful when a stick's antenna sits
  on chain 1.

## DENSE40 / DENSE80 (driver `iq_dense=1`)

The driver switches the capture to the dense single-chain DFE mode with
the channel-filtered tap 12.
- **Format:** each 32-bit word is one chain-0 sample, consecutive, no
  padding and no line structure.
  - 40 MHz channel: DENSE40, 40.0016 MS/s, 160 MB/s.
  - 80 MHz channel: DENSE80, 80.0032 MS/s, 320 MB/s, continuous (replaces
    the snapshots).
- **Decoder:** words go straight to the output. Sequence gaps advance the
  sample index by one sample per word. The lap-overrun check works over
  8-byte units (one lap = ring / 4 samples).
- **Garbage words:** words with spare bits 31:30 or 15:14 set are not
  samples. They were packet-buffer data landing in the ring before firmware
  v64 gave ICAP its own SYSRAM. They are output as 0 and counted
  (`garbage_words`).
- **Rings up to 256 KiB** (header ring code 4: driver `iq_ring_kb=256`,
  firmware v65).

## 80 MHz (RAW80 snapshots, without `iq_dense`)

`mt7921iq_rx -F <center MHz> -w 80` (5 GHz, 5035–5965 MHz), or the Soapy rate
80.0032 MS/s, delivers separate captures of an 80 MHz channel.
- **Each snapshot:** one frozen ring of the 80 MS/s tap, rotated to
  chronological order. Lane 0 is output, flagged
  `MT7921IQ_F_START | MT7921IQ_F_SNAPSHOT`:
  - 8192 samples (102.4 µs) per chunk from the default 64 KiB ring;
  - 2048 samples (25.6 µs) with the driver parameter `iq_ring_kb=16`.

  The ring size comes from the META record.
- **Rate:** about 42 snapshots/s.
- **Alias-free:** a +45 MHz tone is rejected, while ±35 MHz is exact.

## Format facts the decoder relies on

- **Samples.** 20.0008 MS/s (`En32b64b`). Each sample is a 32-bit word
  with I in bits 29:16 and Q in bits 13:0, signed 14-bit; full scale is
  8192. Each 16-byte ring record holds `[s(n), s(n+1), 0, 0]`.
- **Positional decode.** Data sits at even 8-byte units of every 128-byte
  line (aligned to the ring address), padding at the odd ones.
  **All-zero data units are real samples.** The receiver blanks the ADC to
  exact zeros during gain changes, about 150 times a second. The earlier
  experiment decoders ("drop zero units") deleted them: about 39 k samples
  per 10 s. That deletion caused the phase steps previously attributed to
  AGC. With positional decoding, a CW capture had 1 phase step > 0.3 rad in
  5 s, against 1428 per 10 s for the old decoder on the same data.
- **DMA line slip.** The part of a line inside one block (one DMA
  transfer) can arrive as `D0 D0 0 D1 0 …`: the first unit repeats and the
  last padding unit is lost. Slip rates vary from about 1e-5 to over 50 %
  of lines between runs. The decoder recognises slips per segment and
  never joins a line across blocks. Lines matching neither pattern go
  through a counted fallback; there were 0 in 10 s of real data.
- **Exact sample count.** Every 4 KiB block yields exactly 512 samples
  (390,590 blocks → 199,982,080 samples in 10 s).
- **Ring overruns.** Newer data appears twice, one lap apart
  (bit-identical; 8192 samples at 20 MHz with the 64 KiB ring, 2048 with
  16 KiB). The ring size is in IQB2 header flags bits 3:1. They are reported as a chunk flag when the second
  copy arrives, with the start and length of the invalid first copy.
- **Timestamps.** `time_ns` is the MCU tick of the generation's first block
  plus the sample index divided by the rate. The tick runs about 190 ppm off
  the sample clock, so it labels generations; it does not time samples.

## Verified (2026-09-30)

On a 10 s real capture, the output equals the old Python decoder's output
once the 19,396 all-zero data units are removed. Offline decoding runs at
about 106–127 MS/s on one core (5–6× real time).
