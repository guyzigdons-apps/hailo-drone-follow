#!/bin/bash
set -euo pipefail

LOG_TAG="drone-network-mode"
AP_CONNECTION="HailoDrone-AP"
DRONE_USER="hailo"
DRONE_SERVICE="drone-follow.service"
SETTLE_TIMEOUT=30
CHECK_INTERVAL=5
FORCE_FIELD_FLAG="/boot/firmware/field-mode"

log() { logger -t "$LOG_TAG" "$*"; echo "$(date '+%Y-%m-%d %H:%M:%S') $*"; }

is_wifi_connected() {
    local con
    con=$(nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
          | grep ':wlan0$' | cut -d: -f1)
    [ -n "$con" ] && [ "$con" != "$AP_CONNECTION" ]
}

# Force field mode: skip WiFi check, always start AP + drone-follow
if [ -f "$FORCE_FIELD_FLAG" ]; then
    log "FORCE FIELD MODE — $FORCE_FIELD_FLAG exists. Skipping WiFi check."
else
    log "Waiting up to ${SETTLE_TIMEOUT}s for known WiFi..."
    elapsed=0
    while [ $elapsed -lt $SETTLE_TIMEOUT ]; do
        if is_wifi_connected; then
            log "HOME MODE — connected to known WiFi. Drone app will NOT start."
            exit 0
        fi
        sleep $CHECK_INTERVAL
        elapsed=$((elapsed + CHECK_INTERVAL))
        log "No WiFi yet (${elapsed}s/${SETTLE_TIMEOUT}s)"
    done
fi

log "FIELD MODE — no known WiFi. Starting AP on wlan1 + drone-follow."

# Wait for wlan1 (USB WiFi adapter) to appear AND become ready for activation.
# NM transitions: unmanaged -> unavailable -> disconnected.
# The device is only usable once it reaches "disconnected" (supplicant ready).
DEVICE_TIMEOUT=30
DEVICE_INTERVAL=2
elapsed=0
while true; do
    wlan1_state=$(nmcli -t -f DEVICE,STATE device status 2>/dev/null \
                  | grep '^wlan1:' | cut -d: -f2 || true)
    if [ "$wlan1_state" = "disconnected" ] || [ "$wlan1_state" = "connected" ]; then
        break
    fi
    if [ $elapsed -ge $DEVICE_TIMEOUT ]; then
        log "ERROR — wlan1 not ready after ${DEVICE_TIMEOUT}s (state: ${wlan1_state:-not found}). Cannot start AP."
        exit 1
    fi
    sleep $DEVICE_INTERVAL
    elapsed=$((elapsed + DEVICE_INTERVAL))
    log "Waiting for wlan1 to be ready (${elapsed}s/${DEVICE_TIMEOUT}s, state: ${wlan1_state:-not found})..."
done
log "wlan1 ready (state: $wlan1_state)."

nmcli connection up "$AP_CONNECTION"
log "AP active on wlan1: SSID=HailoDrone IP=10.0.0.1 (5GHz ch36)"

sudo -u "$DRONE_USER" XDG_RUNTIME_DIR="/run/user/$(id -u $DRONE_USER)" \
    systemctl --user start "$DRONE_SERVICE"
log "drone-follow started."
