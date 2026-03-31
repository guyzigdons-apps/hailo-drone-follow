#!/bin/bash
# Runs OpenHD air and drone-follow side by side on the drone (RPi)

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
OPENHD_BIN="/usr/local/bin/openhd"

if [ ! -f "$OPENHD_BIN" ]; then
    echo "Error: openhd not found at $OPENHD_BIN"
    exit 1
fi

# Start OpenHD air in the background
sudo "$OPENHD_BIN" --air &
OPENHD_PID=$!
sleep 3

# Set up drone-follow environment and run
cd "$REPO_DIR"
source setup_env.sh
export DISPLAY=:0
drone-follow --input rpi --openhd-stream --connection tcpout://127.0.0.1:5760 --tiles-x 1 --tiles-y 1 &
FOLLOW_PID=$!

trap "kill $FOLLOW_PID 2>/dev/null; sudo kill $OPENHD_PID 2>/dev/null; wait" EXIT

echo "OpenHD PID: $OPENHD_PID"
echo "drone-follow PID: $FOLLOW_PID"
echo "Press Ctrl+C to stop both"

wait
