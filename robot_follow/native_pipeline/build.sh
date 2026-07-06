#!/bin/bash
# Cross-compile the native detection pipeline against the Hailo Yocto SDK.
#
# Usage:
#   ./build.sh                # build for the default SDK at /opt/poky/4.0.23
#   TOOLCHAIN_DIR=/path/to/sdk ./build.sh
#   ./build.sh --clean        # wipe build dir first

set -eEuo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
TOOLCHAIN_DIR="${TOOLCHAIN_DIR:-/opt/poky/4.0.23}"
BUILD_DIR="${SCRIPT_DIR}/build"

if [ "${1:-}" = "--clean" ]; then
    echo "Wiping ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
fi

# The Yocto SDK env-setup script is named after the TUNE_PKGARCH. The
# Hailo Kirkstone SDK uses `armv8a-poky-linux`; some other Yocto SDKs
# use the generic `aarch64-poky-linux`. Probe both.
ENV_SCRIPT=""
for cand in environment-setup-armv8a-poky-linux environment-setup-aarch64-poky-linux; do
    if [ -f "${TOOLCHAIN_DIR}/${cand}" ]; then
        ENV_SCRIPT="${TOOLCHAIN_DIR}/${cand}"
        break
    fi
done
if [ -z "${ENV_SCRIPT}" ]; then
    echo "ERROR: SDK env-setup not found under ${TOOLCHAIN_DIR}" >&2
    echo "       Looked for environment-setup-armv8a-poky-linux and -aarch64-poky-linux" >&2
    echo "       Install the Yocto SDK first (see README.md)" >&2
    exit 1
fi

# shellcheck disable=SC1090
source "${ENV_SCRIPT}"

# The SDK env exports CC/CXX/PKG_CONFIG_PATH/etc. and points meson at
# the cross-file it ships at
# /opt/poky/<ver>/sysroots/x86_64-pokysdk-linux/usr/share/meson/aarch64-poky-linux-meson.cross
# Probe whether meson has actually configured the build dir (presence of
# build.ninja). Re-run setup on first build OR after a failed setup left
# the dir empty.
if [ ! -f "${BUILD_DIR}/build.ninja" ]; then
    rm -rf "${BUILD_DIR}"
    meson setup "${BUILD_DIR}" "${SCRIPT_DIR}"
fi
ninja -C "${BUILD_DIR}"

echo
echo "Built: ${BUILD_DIR}/df_native_pipeline"
echo "Deploy: scp ${BUILD_DIR}/df_native_pipeline root@10.0.0.1:/home/root/native_pipeline/"
