# Firmware patcher

`mt7961-iq-fw-v65.py` builds the I/Q firmware `iq_dma_v65` for the
MT7921AU (MT7961) from the user's own linux-firmware file. It contains only
the bytes the patch changes (2,834 bytes in 11 runs) plus two checksums. No
MediaTek firmware is redistributed: the user patches the copy their
distribution installed.

```sh
python3 mt7961-iq-fw-v65.py                  # finds /lib/firmware/mediatek/WIFI_RAM_CODE_MT7961_1.bin{,.zst,.xz}
python3 mt7961-iq-fw-v65.py STOCK.bin.zst -o WIFI_RAM_CODE_MT7961_1.bin #optional: pass the stock fw to the script manually
sudo install -m 644 WIFI_RAM_CODE_MT7961_1.bin /lib/firmware/mediatek/
# then reload the driver or replug the stick
```

**Input check:** the input must be the exact stock build 20260224110949,
sha256 `b94217a951518a9c14095765f367bc5dd7698f2dc033941d6f18fc2ebd6a2ab9`.
- Any other file is refused, and an already patched one is reported as such.
- The result is checked against
  `1c740c6fe17d9e3ccc175aaef74e10e5187b2bc404ec49096f8a21f1e9714e4f` before
  it is written.
- Python 3 standard library only; `.zst` input needs the `zstd` tool.

**What the result changes:**
- **The I/Q stream code:** hooks plus a patch body in code RAM.
- **The version label `iq_dma_v65`:** the trailer's `fw_ver`, shown as "WM
  Firmware Version" in the kernel log and by `ethtool -i`. It is never loaded
  into the chip.
- **The driver's check:** mt7921-drv streams only when the loaded firmware
  carries an `iq_dma_vNN` label (NN >= 65). On stock firmware `open()` fails
  with "Operation not supported", because the stream writes fixed MCU
  addresses that only the patch defines.

**Installing and updates:**
- The kernel tries the uncompressed name before `.zst`/`.xz`, so the stock
  file can stay.
- If the distribution ships this firmware uncompressed, a linux-firmware
  update overwrites the patched file. The driver then refuses to stream
  until it is patched again.
- If the firmware is in the initramfs, rebuild that.
- To go back to stock, delete the patched file.

