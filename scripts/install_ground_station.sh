#!/bin/bash
################################################################################
# Ground Station — Full Install Script
#
# Installs OpenHD + OpenHD-SysUtils + QOpenHD on an x86_64 or RPi machine.
# Repos must already be cloned to ~/ (OpenHD, OpenHD-SysUtils, qopenHD).
#
# Usage:  sudo ./install_ground_station.sh [--platform <rpi|rpi5|ubuntu-x86>]
#
# If --platform is not given, auto-detects from /proc/cpuinfo and uname.
################################################################################

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root (sudo)."
    exit 1
fi

HOME_DIR="$(eval echo ~${SUDO_USER:-$USER})"
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

# Parse args
PLATFORM=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --platform) PLATFORM="$2"; shift 2 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

# Auto-detect platform
if [ -z "$PLATFORM" ]; then
    ARCH="$(uname -m)"
    if [ "$ARCH" = "x86_64" ]; then
        PLATFORM="ubuntu-x86"
    elif [ "$ARCH" = "aarch64" ]; then
        MODEL="$(cat /proc/device-tree/model 2>/dev/null || echo "")"
        case "$MODEL" in
            *"Raspberry Pi 5"*) PLATFORM="rpi5" ;;
            *"Raspberry Pi"*)   PLATFORM="rpi"  ;;
            *) echo "ERROR: Unknown aarch64 device: $MODEL"; exit 1 ;;
        esac
    else
        echo "ERROR: Unsupported architecture: $ARCH"
        exit 1
    fi
fi

echo "Platform: $PLATFORM"

echo ""
echo "=========================================="
echo " Step 1/5: Install system prerequisites"
echo "=========================================="
apt-get install -y dkms iw

echo ""
echo "=========================================="
echo " Step 2/5: Install OpenHD dependencies"
echo "=========================================="
cd "$HOME_DIR/OpenHD"
./install_build_dep.sh "$PLATFORM"

echo ""
echo "=========================================="
echo " Step 3/5: Build OpenHD + SysUtils + WiFi driver"
echo "=========================================="
./build_native.sh all

echo ""
echo "=========================================="
echo " Step 4/5: Install & build QOpenHD"
echo "=========================================="
cd "$HOME_DIR/qopenHD"
./install_build_dep.sh "$PLATFORM"

# Init submodules if not already done
git submodule update --init --recursive

# Compile Qt translation files (required before build)
sudo -u "${SUDO_USER:-$USER}" lrelease translations/*.ts
sudo -u "${SUDO_USER:-$USER}" cp translations/*.qm qml/

mkdir -p build/release
cd build/release
qmake ../..
make -j$(nproc)

echo ""
echo "=========================================="
echo " Step 5/5: Deploy config files"
echo "=========================================="
mkdir -p /usr/local/share/openhd
if [ -f "$REPO_DIR/df_params.json" ]; then
    cp "$REPO_DIR/df_params.json" /usr/local/share/openhd/df_params.json
    echo "df_params.json deployed."
else
    echo "WARNING: $REPO_DIR/df_params.json not found, skipping."
fi

if [ ! -f /usr/local/share/openhd/txrx.key ]; then
    echo "No txrx.key found — generating new one."
    echo "IMPORTANT: Copy this key to the air unit, or copy the air unit's key here."
    dd if=/dev/urandom of=/usr/local/share/openhd/txrx.key bs=32 count=1 2>/dev/null
fi

echo ""
echo "=========================================="
echo " Ground station install complete!"
echo "=========================================="
echo ""
echo "Platform:  $PLATFORM"
echo "Binaries:"
echo "  OpenHD:        /usr/local/bin/openhd"
echo "  SysUtils:      /usr/local/bin/openhd_sys_utils"
echo "  QOpenHD:       $HOME_DIR/qopenHD/build/release/release/QOpenHD"
echo ""
echo "Run:"
echo "  scripts/start_ground.sh"
