#!/bin/sh
# SPDX-License-Identifier: BSD-3-Clause-Clear
# Build the mt76 modules for the mt7921 USB stick (0e8d:7961) inside an
# Ubuntu 24.04 container, for the Ubuntu kernel in KVER below (set it to your
# kernel; the native build in the top-level README needs no container).
#
#   ./build.sh            build modules
#   ./build.sh clean      clean build artifacts
#
# First run builds the toolchain image (downloads the kernel headers once).
set -eu

KVER=7.0.0-31-generic
IMAGE=mt76-kbuild31:24.04
SRC=$(cd "$(dirname "$0")" && pwd)
TARGET=${1:-modules}

# The toolchain image caches build-essential + the kernel headers so repeat
# builds don't re-download ~150 MB. Context lives outside $SRC so it is not
# swept into the kernel build.
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "==> building $IMAGE"
    CTX=$(mktemp -d /home/"$(id -un)"/mt76-kbuild-ctx.XXXXXX)
    cat > "$CTX/Dockerfile" <<EOF
FROM ubuntu:24.04
RUN apt-get update && \\
    apt-get install -y --no-install-recommends \\
        build-essential bc bison flex libelf-dev kmod \\
        linux-headers-$KVER && \\
    rm -rf /var/lib/apt/lists/*
WORKDIR /src
EOF
    docker build -t "$IMAGE" "$CTX"
    rm -rf "$CTX"
fi

# Two non-obvious bits:
#
# 1. KCFLAGS injects <linux/version.h> (upstream mt76.h uses LINUX_VERSION_CODE
#    without including it) plus compat-mt7921.h, which stubs one mac80211
#    helper this kernel lacks. See that header for the rationale.
#
# 2. Ubuntu's kernel .config enables the whole mt76 family, so every unwanted
#    chip is force-disabled on the command line -- otherwise the build pulls in
#    mt76x0/mt7915/mt7996/... and fails on unrelated code. Command-line make
#    variables override the values from the kernel config.
#
# NOTE: compat-mt7921.h's stub is only valid because mt7921 does not set
# MT_DRV_HW_PS_BUFFERING. Do not enable mt7915/mt7996 here without revisiting it.
exec docker run --rm -v "$SRC:/src" -w /src "$IMAGE" bash -c '
set -eu
make -j"$(nproc)" -C /lib/modules/'"$KVER"'/build M=/src '"$TARGET"' \
  KCFLAGS="-include linux/version.h -include /src/compat-mt7921.h" \
  CONFIG_MT76_USB=m CONFIG_MT76_CONNAC_LIB=m \
  CONFIG_MT792x_LIB=m CONFIG_MT792x_USB=m \
  CONFIG_MT7921_COMMON=m CONFIG_MT7921U=m \
  CONFIG_MT76_SDIO= CONFIG_MT76x02_LIB= CONFIG_MT76x02_USB= \
  CONFIG_MT76x0_COMMON= CONFIG_MT76x2_COMMON= CONFIG_MT7603E= \
  CONFIG_MT7615_COMMON= CONFIG_MT7915E= CONFIG_MT7996E= \
  CONFIG_MT7925_COMMON= CONFIG_MT7921E= CONFIG_MT7921S= CONFIG_MT76_NPU=
chown -R '"$(id -u):$(id -g)"' /src
'
