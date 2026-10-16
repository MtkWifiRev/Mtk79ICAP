#!/bin/sh
# SPDX-License-Identifier: MIT
# One-time (root) setup for SDR use of an mt7921u stick with the fork driver:
# put its interface into monitor mode so /dev/mt7921_iq_<phy> can stream.
# Retuning afterwards goes through the device (no root needed with the
# udev rule in udev/99-mt7921-iq.rules).
#
#   mt7921iq-setup.sh [FREQ_MHZ]      (default 5785)
set -eu
FREQ=${1:-5785}
dev=$(ls /dev/mt7921_iq_* 2>/dev/null | head -1)
[ -n "$dev" ] || { echo "no /dev/mt7921_iq_* (fork driver loaded?)" >&2; exit 1; }
phy=${dev#/dev/mt7921_iq_}
ifc=$(ls /sys/class/ieee80211/$phy/device/net | head -1)
[ -n "$ifc" ] || { echo "no interface on $phy" >&2; exit 1; }
if command -v nmcli >/dev/null 2>&1; then
	nmcli device set "$ifc" managed no 2>/dev/null || true
fi
ip link set "$ifc" down
iw dev "$ifc" set type monitor
ip link set "$ifc" up
iw dev "$ifc" set freq "$FREQ" HT20
echo "$ifc ($phy) in monitor mode at $FREQ MHz; stream device $dev"
