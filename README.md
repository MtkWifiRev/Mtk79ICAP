# Mtk79ICAP: an MT7921AU Wi-Fi stick as an SDR receiver

**What it is:** a fork of the Linux [mt76](https://github.com/openwrt/mt76)
driver plus a small firmware patch. Together they stream raw 14-bit I/Q
samples from the chip's internal capture engine (ICAP) over USB.
- **Hardware:** MediaTek MT7921AU / MT7961 USB sticks (`0e8d:7961`).
- **Use:** in your SDR applications through SoapySDR, alongside normal
  monitor-mode reception.

| | |
|---|---|
| Bands | 2.4 GHz and 5 GHz |
| Widths / rates | 20 MHz (20.0008 MS/s), 40 MHz (40.0016 MS/s), 80 MHz (80.0032 MS/s, continuous) |
| Tuning | any frequency: 5 MHz hardware raster plus a software NCO, 15–20 ms per hardware retune |
| Gain | chip AGC, or a fixed gain: LNA 0–18 dB and LPF 0–16 dB elements |
| Interfaces | `/dev/mt7921_iq_<phy>`, `libmt7921iq` + `mt7921iq_rx`, SoapySDR module `driver=mt7921` |

> Experimental. Levels are not calibrated, and some register settings were
> found by reverse engineering. A wedged stick needs a replug.

## 1. Firmware

**Why a patched firmware:** the stream needs a patched WM firmware. We don't
redistribute MediaTek's file. The patcher builds it from the copy your
distribution installed (linux-firmware build `20260224110949`, sha256
`b94217a9…`); anything else is refused.

```sh
python3 firmware/mt7961-iq-fw-v65.py          # finds /lib/firmware/mediatek/WIFI_RAM_CODE_MT7961_1.bin{,.zst,.xz}
sudo install -m 644 WIFI_RAM_CODE_MT7961_1.bin /lib/firmware/mediatek/
```

- **Which file loads:** the uncompressed file takes priority over the
  distribution's `.zst`/`.xz`.
- **Check:** after loading, the kernel log shows `WM Firmware Version:
  iq_dma_v65`.
- **Back to stock:** delete the file.
- **Stock firmware:** the driver streams only on firmware with this label.
  On stock firmware, opening the I/Q device fails with "Operation not
  supported".

## 2. Build and install the driver

You need the kernel headers of the running kernel. It is tested on Linux 7.0
(Ubuntu 24.04 HWE) and 7.2 (Arch).

```sh
make -j"$(nproc)" -C /lib/modules/$(uname -r)/build M=$PWD modules \
  KCFLAGS="-include linux/version.h -include $PWD/compat-mt7921.h" \
  CONFIG_MT76_USB=m CONFIG_MT76_CONNAC_LIB=m CONFIG_MT792x_LIB=m CONFIG_MT792x_USB=m \
  CONFIG_MT7921_COMMON=m CONFIG_MT7921U=m \
  CONFIG_MT76_SDIO= CONFIG_MT76x02_LIB= CONFIG_MT76x02_USB= CONFIG_MT76x0_COMMON= \
  CONFIG_MT76x2_COMMON= CONFIG_MT7603E= CONFIG_MT7615_COMMON= CONFIG_MT7915E= \
  CONFIG_MT7996E= CONFIG_MT7925_COMMON= CONFIG_MT7921E= CONFIG_MT7921S= CONFIG_MT76_NPU=
sudo make -C /lib/modules/$(uname -r)/build M=$PWD INSTALL_MOD_DIR=updates modules_install
sudo depmod -a
```

- **Why the long command line:** the `CONFIG_*` overrides build only the
  modules the stick needs. Distribution kernels enable the whole mt76
  family, which would otherwise be pulled in.
- **Output:** seven modules (`mt76`, `mt76-usb`, `mt76-connac-lib`,
  `mt792x-lib`, `mt792x-usb`, `mt7921-common`, `mt7921u`). In `updates/`
  they take precedence over the in-tree ones.
- **Signing:** on kernels that sign modules at install, signing fails
  without the distribution's key, and the modules install unsigned. That is
  fine unless your kernel enforces signatures (Secure Boot lockdown); then
  sign them with your own enrolled key.
- **Other mt76 drivers:** while these modules are installed, the in-tree
  mt76 drivers of other chips (`mt7925u`, `mt7915e`, `mt7663u`, …) resolve
  against this `mt76` and no longer load.
- **Kernel updates:** rebuild and reinstall after every kernel update;
  `updates/` is per kernel version.
- **Uninstall:**
  `sudo rm -r /lib/modules/$(uname -r)/updates/{mt76*,mt792x*,mt7921} && sudo depmod -a`.
- **Docker build:** `build.sh` does the same build inside a container for
  Ubuntu kernels.

**Module options** (`/etc/modprobe.d/mt7921-iq.conf`):

```
options mt7921_common iq_dense=1 iq_ring_kb=256
options mt76_usb rx_urbs=512
```

- `iq_dense=1`: continuous dense formats (40 MHz at 160 MB/s; 80 MHz at
  320 MB/s, on 5 GHz and 2.4 GHz). Without it, 80 MHz on 5 GHz comes as
  snapshots only.
- `iq_ring_kb=256`: a 256 KiB capture ring.
- `rx_urbs=512`: more USB buffering against host stalls.

**Load and set up**, once per boot or replug, as root:

```sh
# unload the whole stack: "modprobe -r mt7921u" alone leaves the old mt76 core loaded
sudo modprobe -r mt7921u mt7921_common mt792x_usb mt792x_lib mt76_connac_lib mt76_usb mt76
sudo modprobe mt7921u                                # (or reboot)
sudo userspace/mt7921iq-setup.sh                     # puts the stick's interface into monitor mode
```

**Access without root:** install `userspace/udev/99-mt7921-iq.rules` and
add yourself to the group `plugdev`, or, for a quick test,
`sudo chmod 666 /dev/mt7921_iq_*` (lost on a driver reload or replug).
Streaming and retuning then work as a
normal user.

## 3. Userspace and SDR applications

```sh
cmake -S userspace/SoapyMT7921 -B build-soapy && cmake --build build-soapy
sudo cmake --install build-soapy                     # or: export SOAPY_SDR_PLUGIN_PATH=$PWD/build-soapy
SoapySDRUtil --probe=driver=mt7921
```

The build also produces `mt7921iq_rx`, a recorder in the style of `rtl_sdr`:

```sh
mt7921iq_rx -F 2442 -w 80 -t 10 out.cs16   # 80 MHz centred at 2442 MHz, 10 s
```

**gqrx:**
- **Device:** choose "Other…" and enter `soapy=0,driver=mt7921`.
- **Input rate:** `20000800`, `40001600` or `80003200`. The rate selects the
  channel width.
- **Gain:** "Hardware AGC" switches the chip's AGC. With it off, the LNA and
  LPF sliders set a fixed gain.
- **CPU:** gqrx handles 20 and 40 MS/s comfortably. 80 MS/s needs a fast
  machine.

**SDR++:**
- **Source:** "SoapySDR", then the device "MT7921AU I/Q (phyN)", and one of
  the three sample rates.
- **Gain:** the AGC toggle and the LNA/LPF sliders work as in gqrx.
- **Module path:** SDR++ finds the module like any other SoapySDR module
  (installed, or via `SOAPY_SDR_PLUGIN_PATH`).

**Things to know:**
- Only one program can open the stream at a time.
- A frequency change within ±2.5 MHz of the hardware centre causes no gap.
- Monitor-mode frame capture keeps working while you stream with the AGC.
  With a fixed gain the chip stops decoding frames.
- **Fixed gain** removes the AGC's gain dips on other stations' frames
  (hundreds per second on a busy channel) at the same SNR. The output is
  then about 13 dB lower, because the AGC's digital gain is missing. The
  gain steps are measured on 5 GHz; on 2.4 GHz they differ.
- **Known issue:** 80 MHz on 2.4 GHz has no vendor channel; the driver
  widens a 40 MHz channel itself. The usable band is about −35…+39 MHz
  around the centre. Signal quality varies from one stream start to the next
  (cause still open), so reopen if it looks poor.

## Opens
- Enable TX
- Explore DBDC (Dual-Band Dual-Concurrent)

## Acknowledgements

- **[BirriaSDR](https://github.com/luigifcruz/BirriaSDR)** (Luigi Cruz)
  streams from the same dongle without any vendor firmware. We managed to get 80MHz bandwith working by adoping its USB transport method.
- **[w1700k-sdr](https://github.com/h0m3us3r/w1700k-sdr)** (h0m3us3r) does
  the same on an MT7996. Its documented register map of the shared capture
- **[Shuffle](https://github.com/shuffle2):** the firmware region CRC algorithm and NDS32
  support for Ghidra.
