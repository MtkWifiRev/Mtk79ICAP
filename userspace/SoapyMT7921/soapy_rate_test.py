#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Sustained-rate check through the SoapySDR module.

  SOAPY_SDR_PLUGIN_PATH=build soapy_rate_test.py RATE FREQ_MHZ SECONDS [TONE_MHZ]

Streams CS16 at RATE for SECONDS, counting samples, overflows and timeouts.
With TONE_MHZ, the FFT peak of the last 1 Mi samples is compared with
TONE - FREQ.
"""
import sys, time
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CS16, SOAPY_SDR_OVERFLOW, SOAPY_SDR_TIMEOUT

rate, freq, secs = float(sys.argv[1]), float(sys.argv[2]) * 1e6, float(sys.argv[3])
tone = float(sys.argv[4]) * 1e6 if len(sys.argv) > 4 else None
SoapySDR.Device.enumerate()
dev = SoapySDR.Device('driver=mt7921')
dev.setSampleRate(SOAPY_SDR_RX, 0, rate)
dev.setFrequency(SOAPY_SDR_RX, 0, freq)
st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, [0])
dev.activateStream(st)
buf = np.zeros(2 * (1 << 20), np.int16)
got = ovf = tmo = 0
t0 = None
last = None
tail = []
while True:
    r = dev.readStream(st, [buf], 1 << 20, timeoutUs=500000)
    if r.ret == SOAPY_SDR_OVERFLOW:
        ovf += 1
        continue
    if r.ret == SOAPY_SDR_TIMEOUT:
        tmo += 1
        continue
    if r.ret < 0:
        print('error', r.ret)
        break
    if t0 is None:
        t0 = time.monotonic()
    got += r.ret
    tail.append(buf[:2 * r.ret].copy())
    while sum(len(t) for t in tail) > 4 * (1 << 18):
        tail.pop(0)
    if time.monotonic() - t0 >= secs:
        last = np.concatenate(tail)
        break
dt = time.monotonic() - t0
dev.deactivateStream(st)
dev.closeStream(st)
print('rate %.4f MS/s requested, got %d samples in %.2f s = %.3f MS/s; overflows %d, timeouts %d'
      % (dev.getSampleRate(SOAPY_SDR_RX, 0) / 1e6, got, dt, got / dt / 1e6, ovf, tmo))
if tone is not None and last is not None and len(last) >= 2 * (1 << 18):
    x = last[-2 * (1 << 18):].astype(np.float32).view(np.complex64)
    sp = np.abs(np.fft.fft(x[:1 << 18] * np.hanning(1 << 18)))
    f = np.fft.fftfreq(1 << 18, 1 / dev.getSampleRate(SOAPY_SDR_RX, 0))[np.argmax(sp)]
    print('tone at %+.4f MHz (expected %+.4f)' % (f / 1e6, (tone - freq) / 1e6))
