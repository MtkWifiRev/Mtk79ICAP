#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Tune through the SoapySDR module and check where a CW tone lands.

  SOAPY_SDR_PLUGIN_PATH=build soapy_tune_test.py TONE_MHZ STEP [STEP ...]

STEP is FREQ_MHZ, or r20 / r40 to switch the sample rate (channel width) at
the current frequency. For each step: apply it, then 1 Mi samples. Prints
the tone offset found by FFT (expected: TONE - FREQ), the time from
setFrequency to the first sample at the new setting (as seen by the reader),
and overflows.
"""
import sys, time
import numpy as np
import SoapySDR
from SoapySDR import SOAPY_SDR_RX, SOAPY_SDR_CF32, SOAPY_SDR_OVERFLOW, SOAPY_SDR_TIMEOUT

tone = float(sys.argv[1]) * 1e6
steps = sys.argv[2:]
freqs = [float(f) * 1e6 for f in steps if not f.startswith('r')]
SoapySDR.Device.enumerate()   # loads the plugin modules (make() alone does not)
dev = SoapySDR.Device('driver=mt7921')   # dict args do not convert under Python 3.14
dev.setFrequency(SOAPY_SDR_RX, 0, freqs[0])
st = dev.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
dev.activateStream(st)
buf = np.zeros(65536, np.complex64)

def collect(n, t_start):
    out, first, ovf = [], None, 0
    got = 0
    while got < n:
        r = dev.readStream(st, [buf], len(buf), timeoutUs=500000)
        if r.ret == SOAPY_SDR_OVERFLOW:
            ovf += 1
            continue
        if r.ret == SOAPY_SDR_TIMEOUT or r.ret < 0:
            raise SystemExit('readStream: %d' % r.ret)
        if first is None:
            first = time.monotonic() - t_start
        out.append(buf[:r.ret].copy()); got += r.ret
    return np.concatenate(out)[:n], first, ovf

collect(1 << 21, time.monotonic())     # settle after activate
f = freqs[0]
for step in steps:
    t0 = time.monotonic()
    if step.startswith('r'):
        dev.setSampleRate(SOAPY_SDR_RX, 0, 40001600 if step == 'r40' else 20000800)
    else:
        f = float(step) * 1e6
        dev.setFrequency(SOAPY_SDR_RX, 0, f)
    rate = dev.getSampleRate(SOAPY_SDR_RX, 0)
    x, first, ovf = collect(1 << 20, t0)
    X = np.abs(np.fft.fftshift(np.fft.fft(x * np.hanning(len(x)))))
    off = (np.argmax(X) - len(x) / 2) / len(x) * rate
    print('%-8s %.3f MHz @ %.4f MS/s: tone at %+.3f MHz (expected %+.3f), first sample %.1f ms, overflows %d'
          % (step, f / 1e6, rate / 1e6, off / 1e6, (tone - f) / 1e6, first * 1e3, ovf))
dev.deactivateStream(st)
dev.closeStream(st)
