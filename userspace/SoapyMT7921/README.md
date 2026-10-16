# SoapyMT7921 — SoapySDR module for the MT7921AU I/Q stream

Exposes `/dev/mt7921_iq_<phy>` (fork driver) as SoapySDR device
`driver=mt7921`, for gqrx, GNU Radio and others.

| Property | Value |
|---|---|
| Channels | 1 RX |
| Sample rate | 20.0008 MS/s (20 MHz channel, ~18 MHz usable), 40.0016 MS/s (40 MHz channel, ~36 MHz usable) or 80.0032 MS/s (80 MHz channel, see below) |
| Formats | CS16 (native, full scale 8192), CF32 |
| Tuning (20 MHz) | 2409.5–2474.5, 2481.5–2486.5, 5002.5–5997.5 MHz, any frequency |
| Tuning (40 MHz) | 2419.5–2464.5, 5012.5–5987.5 MHz, any frequency |
| 80 MHz (rate 80.0032 MS/s) | 5032.5–5967.5 MHz, and 2419.5–2464.5 MHz with `iq_dense=1` (center 2442 covers 2402–2482 MHz: tones exact from −39 to +39 MHz, the low edge rolls off from about −35 MHz). On 2.4 GHz the driver sets a 40 MHz channel and widens it itself. **Continuous** with the driver parameter `iq_dense=1` (DENSE80, 320 MB/s; use `iq_ring_kb=256`). Without it, 5 GHz only: separate 8192-sample (102.4 µs) snapshots, not continuous, for spectrum and waterfall views only |
| 40 MHz with `iq_dense=1` | DENSE40, 160 MB/s instead of PACK96 at 240 MB/s; same rate and tuning |
| RX chain | setting / device arg `chain` = 0 or 1: with `iq_dense`, 1 streams RX chain 1 (tap 13) at the full rate; changing it retunes |
| Gain | AGC (default) or manual: `setGainMode(false)` holds the streamed RX chain at a fixed analog gain. Elements `LNA` 0–18 dB (6 dB steps) and `LPF` 0–16 dB (2 dB steps); overall 0–34 dB, filled LNA first; default 32 dB |

- **Tuning.** The receiver tunes in 5 MHz steps through the driver ioctl,
  with a gap of about 15–20 ms. The remainder is shifted in software, so
  every requested frequency is exact.
  - `setFrequency()` returns at once; a tuner thread applies the latest
    request.
  - Samples from before a hardware retune are dropped. A frequency change
    that stays within `max_offset` of the current center causes no gap.
  - `max_offset` is a setting or device arg, default 2.5 MHz. Larger values
    mean fewer retunes, but the band edges shown in gqrx then extend past
    the channel the receiver actually covers.
- **Gain.** By default the receiver's AGC acts on every frame it detects
  on the primary 20 MHz: about −4 dB for 16–19 µs per frame, hundreds of
  times per second on a busy channel. That breaks coherent processing.
  - **Manual mode** (`setGainMode(false)`, gqrx: untick "Hardware AGC")
    holds the analog gain (driver ioctl `MT7921_IQ_IOC_SET_GAIN`). It
    applies live, with no gap, and survives retunes.
  - **Default 32 dB** = LNA 5 / LPF 7, the AGC's own idle gain: the same
    SNR, and the output is about 13 dB lower (the AGC's digital gain is
    missing; scaling only).
  - **Elements:** `LNA` sets the noise figure (each 6 dB step down costs
    SNR); `LPF` comes after it (same SNR at every step).
  - **Measured on 5 GHz;** 2.4 GHz steps are not mapped yet.
  - **What remains:** the hardware's frame-synchronous blanking (runs of
    exact-zero samples, 0.02 % at 32 dB, fewer at lower gains).
- **Discontinuities.** Blocks dropped by the driver (slow consumer) and ring
  overruns come back from `readStream` as `SOAPY_SDR_OVERFLOW`.
- **Timestamps.** `timeNs` is the MCU tick at the generation start plus the
  sample count. It labels samples but is not sample-accurate (see the
  libmt7921iq README).

## Setup

1. Install the patched firmware (`firmware/` at the top of this repository)
   and load the fork driver. On stock firmware the device opens with
   "Operation not supported".
2. Put the interface into monitor mode once, as root:
   `../mt7921iq-setup.sh [MHz]`.
3. For access without root, either:
   - install `../udev/99-mt7921-iq.rules` and join the group `plugdev`
     (permanent); or
   - make the device node readable once, as root:
     `sudo chmod 666 /dev/mt7921_iq_*`. This is lost on a driver reload or
     replug.

## Build

```sh
cmake -S . -B build && cmake --build build
# without installing:
SOAPY_SDR_PLUGIN_PATH=$PWD/build SoapySDRUtil --find=driver=mt7921
# or install into the SoapySDR module directory:
sudo cmake --install build
```

In gqrx, choose "Other…" as the device and enter the device string
`soapy=0,driver=mt7921` with an input rate of 20000800, 40001600 or
80003200 (20, 40 or 80 MHz channel). Changing the rate while streaming
retunes, with a gap of about 20–30 ms. gqrx has to see the module: either
set `SOAPY_SDR_PLUGIN_PATH=<build dir>` or install it. It also needs read
access to `/dev/mt7921_iq_*` (setup step 3). In SDR++, pick the "SoapySDR"
source and the device "MT7921AU I/Q (phyN)".

## Tested (2026-10-01, CW tone from a HackRF at 2441.639 MHz)

- **Discovery:** `SoapySDRUtil --find` and `--probe` list the device, its
  formats, the frequency ranges and the `max_offset` setting.
- **Throughput:** `SoapySDRUtil --rate=20000800` gives 19.9 MS/s.
- **Tuning** (`soapy_tune_test.py`): every request puts the tone exactly at
  TONE − FREQ, within 1 kHz, with no overflows and no MCU timeouts.
  - Software-only shifts within ±2.5 MHz of the hardware center deliver
    samples 0.6–6 ms after `setFrequency`.
  - Hardware retunes deliver new samples after 22–48 ms; older generations
    are dropped.
  - After close, mac80211's channel is back.
- **gr-osmosdr** (`osmosdr_test.py`, gqrx's path): `soapy=0,driver=mt7921`
  streams 19.76 MS/s over 1.5 s including one retune, and the tone is exact
  before and after.
- **Gain** (`soapy_gain_test.py`, 2026-10-14, DENSE80 at 5775 MHz on a busy
  primary, CW at +7.3 MHz): AGC 236–292 gain dips/s; manual 0 at every
  setting. Tone level: 32 dB = 366–369, 34 dB = 450, 12 dB = 42; AGC
  1,641–1,670. Element/overall mapping and the driver state read back as
  set; 0 overflows in 9 s at 80 MS/s; close restores the AGC.
- **Python bindings quirk:** under Python 3.14, `SoapySDR.Device({...})` with
  a dict fails with "no match"; use the string form
  `Device('driver=mt7921')` after `Device.enumerate()`.
