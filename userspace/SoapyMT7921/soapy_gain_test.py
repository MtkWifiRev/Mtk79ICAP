#!/usr/bin/env python3
"""Gain control through the SoapySDR module (AGC vs manual hold).

  SOAPY_SDR_PLUGIN_PATH=build soapy_gain_test.py [FREQ_MHZ] [TONE_MHZ] [--seg S]

Streams CS16 at 80.0032 MS/s (driver iq_dense) with a CW tone at TONE_MHZ and
steps through AGC, setGainMode(False) at the default gain, element and overall
gains, and back to AGC. Per step it reports the gain readbacks, the driver's
gain line (debugfs iq_stream_stats, if readable), and for a settled 0.25 s
slice: median tone amplitude, gain dips per second (tone >= 2 dB down for
>= 9.6 us) and zero-run (>= 8 samples) rate. Overflows are counted.
"""
import argparse, glob, time
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CS16, SOAPY_SDR_OVERFLOW, SOAPY_SDR_TIMEOUT

ap = argparse.ArgumentParser()
ap.add_argument('freq', nargs='?', type=float, default=5775)
ap.add_argument('tone', nargs='?', type=float, default=5782.3)
ap.add_argument('--seg', type=float, default=1.5)
a = ap.parse_args()
RATE = 80.0032e6


def driver_gain():
    for p in glob.glob('/sys/kernel/debug/ieee80211/phy*/mt76/spectrum/iq_stream_stats'):
        try:
            for line in open(p):
                if line.startswith('gain_mode'):
                    return line.split(' gain_errors')[0].strip()
        except OSError:
            pass
    return '?'


def stats(x, nu):
    z = x * np.exp(-2j * np.pi * nu * np.arange(len(x)))
    good = x != 0
    B = 256
    nb = len(x) // B
    g = good[:nb * B].reshape(nb, B)
    amp = np.abs((z[:nb * B].reshape(nb, B) * g).sum(1) / np.maximum(g.sum(1), 1))
    db = 20 * np.log10(np.maximum(amp, 1e-3) / np.median(amp))
    d = np.diff(np.concatenate([[0], (db < -2).astype(np.int8), [0]]))
    dips = np.count_nonzero((np.nonzero(d == -1)[0] - np.nonzero(d == 1)[0]) >= 3)
    dz = np.diff(np.concatenate([[0], (~good).astype(np.int8), [0]]))
    zr = np.count_nonzero((np.nonzero(dz == -1)[0] - np.nonzero(dz == 1)[0]) >= 8)
    T = len(x) / RATE
    return np.median(amp), dips / T, zr / T


dev = SoapySDR.Device('driver=mt7921')
dev.setSampleRate(SOAPY_SDR_RX, 0, RATE)
dev.setFrequency(SOAPY_SDR_RX, 0, a.freq * 1e6)
print('gains', dev.listGains(SOAPY_SDR_RX, 0), 'range', dev.getGainRange(SOAPY_SDR_RX, 0),
      'LNA', dev.getGainRange(SOAPY_SDR_RX, 0, 'LNA'), 'LPF', dev.getGainRange(SOAPY_SDR_RX, 0, 'LPF'),
      'hasGainMode', dev.hasGainMode(SOAPY_SDR_RX, 0), 'mode', dev.getGainMode(SOAPY_SDR_RX, 0),
      'gain', dev.getGain(SOAPY_SDR_RX, 0))
steps = [('AGC', lambda: dev.setGainMode(SOAPY_SDR_RX, 0, True)),
         ('manual (default)', lambda: dev.setGainMode(SOAPY_SDR_RX, 0, False)),
         ('LPF 16 dB', lambda: dev.setGain(SOAPY_SDR_RX, 0, 'LPF', 16)),
         ('overall 12 dB', lambda: dev.setGain(SOAPY_SDR_RX, 0, 12)),
         ('overall 32 dB', lambda: dev.setGain(SOAPY_SDR_RX, 0, 32)),
         ('AGC', lambda: dev.setGainMode(SOAPY_SDR_RX, 0, True))]
st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0])
dev.activateStream(st)
buf = np.zeros(2 * (1 << 20), np.int16)
ovf = tmo = got = 0
nu = (a.tone - a.freq) * 1e6 / RATE
results = []
# let the first tune settle
t_end = time.monotonic() + 1.0
while time.monotonic() < t_end:
    r = dev.readStream(st, [buf], 1 << 20, timeoutUs=500000)
for name, fn in steps:
    fn()
    t_apply = time.monotonic()
    keep, kept = [], 0
    while time.monotonic() - t_apply < a.seg:
        r = dev.readStream(st, [buf], 1 << 20, timeoutUs=500000)
        if r.ret == SOAPY_SDR_OVERFLOW:
            ovf += 1; continue
        if r.ret == SOAPY_SDR_TIMEOUT:
            tmo += 1; continue
        if r.ret < 0:
            raise SystemExit('readStream error %d' % r.ret)
        got += r.ret
        if time.monotonic() - t_apply > 0.4 and kept < int(0.25 * RATE):
            keep.append(buf[:2 * r.ret].copy()); kept += r.ret
    results.append([name, dev.getGainMode(SOAPY_SDR_RX, 0), dev.getGain(SOAPY_SDR_RX, 0),
                    dev.getGain(SOAPY_SDR_RX, 0, 'LNA'), dev.getGain(SOAPY_SDR_RX, 0, 'LPF'),
                    driver_gain(), np.concatenate(keep)])
dev.deactivateStream(st)
dev.closeStream(st)
# analysed after the stream: reading must not pause (the driver ring holds ~0.2 s at 320 MB/s)
for r in results:
    r[-1] = stats(r[-1].astype(np.float32).view(np.complex64), nu)
print('%-17s %-5s %5s %4s %4s  %-62s %6s %7s %8s' % ('step', 'agc', 'gain', 'LNA', 'LPF', 'driver', 'tone', 'dips/s', 'zruns/s'))
for name, agc, g, gl, gp, drv, (amp, dips, zr) in results:
    print('%-17s %-5s %5.0f %4.0f %4.0f  %-62s %6.0f %7.1f %8.1f' % (name, agc, g, gl, gp, drv, amp, dips, zr))
print('samples %d, overflows %d, timeouts %d; after close: %s' % (got, ovf, tmo, driver_gain()))
