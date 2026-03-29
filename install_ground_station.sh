#!/bin/bash
################################################################################
# x86_64 Ground Station — Full Install Script
#
# Installs OpenHD + OpenHD-SysUtils + QOpenHD on an x86_64 Ubuntu machine.
# Repos must already be cloned to ~/ (OpenHD, OpenHD-SysUtils, qopenHD).
#
# Usage:  sudo ./install_ground_station.sh
################################################################################

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: Must run as root (sudo)."
    exit 1
fi

HOME_DIR="$(eval echo ~${SUDO_USER:-$USER})"

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
./install_build_dep.sh ubuntu-x86

echo ""
echo "=========================================="
echo " Step 3/5: Build OpenHD + SysUtils + WiFi driver"
echo "=========================================="
./build_native.sh all

echo ""
echo "=========================================="
echo " Step 3/4: Install QOpenHD dependencies"
echo "=========================================="
cd "$HOME_DIR/qopenHD"
./install_build_dep.sh ubuntu-x86

echo ""
echo "=========================================="
echo " Step 4/5: Build QOpenHD"
echo "=========================================="
cd "$HOME_DIR/qopenHD"

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
DRONE_FOLLOW_DIR="$HOME_DIR/tappas_apps/repos/hailo-drone-follow"
if [ -f "$DRONE_FOLLOW_DIR/df_params.json" ]; then
    cp "$DRONE_FOLLOW_DIR/df_params.json" /usr/local/share/openhd/df_params.json
    echo "df_params.json deployed."
else
    echo "WARNING: $DRONE_FOLLOW_DIR/df_params.json not found, skipping."
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
echo "Binaries:"
echo "  OpenHD:        /usr/local/bin/openhd"
echo "  SysUtils:      /usr/local/bin/openhd_sys_utils"
echo "  QOpenHD:       $HOME_DIR/qopenHD/build/release/release/QOpenHD"
echo ""
echo "Run:"
echo "  sudo /usr/local/bin/openhd --ground"
echo "  $HOME_DIR/qopenHD/build/release/release/QOpenHD"
