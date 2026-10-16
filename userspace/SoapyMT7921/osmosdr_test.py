#!/usr/bin/env python3
"""gqrx's path in miniature: gr-osmosdr source with a gqrx device string.

  SOAPY_SDR_PLUGIN_PATH=build osmosdr_test.py [TONE_MHZ] [F1_MHZ F2_MHZ]

Streams to a file sink, retunes through osmosdr after 1 s (as gqrx does),
and reports the sample rate seen over wall time plus the tone offset
before and after the retune.
"""
import os, sys, time
import numpy as np
import osmosdr
from gnuradio import gr, blocks

tone = float(sys.argv[1]) * 1e6 if len(sys.argv) > 1 else 2441.639e6
f1 = float(sys.argv[2]) * 1e6 if len(sys.argv) > 2 else 2441.0e6
f2 = float(sys.argv[3]) * 1e6 if len(sys.argv) > 3 else 2446.5e6
path = '/tmp/osmo.cf32'

tb = gr.top_block()
src = osmosdr.source(args='soapy=0,driver=mt7921')
src.set_sample_rate(20000800)
src.set_center_freq(f1)
sink = blocks.file_sink(gr.sizeof_gr_complex, path, False)
sink.set_unbuffered(False)
tb.connect(src, sink)
tb.start()
time.sleep(0.5)
s0, t0 = os.path.getsize(path), time.monotonic()
time.sleep(0.5)
n1 = os.path.getsize(path) // 8
src.set_center_freq(f2)
time.sleep(1.0)
s1, t1 = os.path.getsize(path), time.monotonic()
tb.stop(); tb.wait()
rate = src.get_sample_rate()
print('streamed %.2f MS/s over %.2f s wall time (incl. one retune)'
      % ((s1 - s0) / 8 / (t1 - t0) / 1e6, t1 - t0))
x = np.fromfile(path, np.complex64)
os.remove(path)
for name, part, f in (('before', x[:n1], f1), ('after', x[n1 + int(0.2 * rate):], f2)):
    seg = part[-(1 << 20):]
    X = np.abs(np.fft.fftshift(np.fft.fft(seg * np.hanning(len(seg)))))
    off = (np.argmax(X) - len(seg) / 2) / len(seg) * rate
    print('%-6s tone at %+.3f MHz (expected %+.3f)' % (name, off / 1e6, (tone - f) / 1e6))
