# system/ — Networking & Boot Services

Dual-interface networking setup for the drone RPi:

- **wlan0 (built-in RPi WiFi)** — connects to home/dev WiFi
- **wlan1 (TP-Link USB adapter)** — dedicated AP for field ops (5GHz, channel 36)

## Files

| File | Purpose |
|------|---------|
| `install.sh` | Creates NM AP profile on wlan1, installs udev rule, enables boot service |
| `uninstall.sh` | Reverses everything `install.sh` does |
| `drone-network-mode.sh` | Boot script: waits for WiFi on wlan0, falls back to AP mode on wlan1 |
| `drone-network-mode.service` | systemd unit that runs `drone-network-mode.sh` at boot |
| `enable_access_point.sh` | Toggle AP on/off |
| `71-usb-wifi.rules` | udev rule pinning TP-Link USB adapter to `wlan1` by MAC address |

## Install

```bash
cd system
sudo ./install.sh
```

Then reboot. The boot service will automatically choose home or field mode.

## Uninstall

```bash
cd system
sudo ./uninstall.sh
```

## How It Works (Boot Behavior)

1. On boot, `drone-network-mode.service` runs `drone-network-mode.sh`
2. If `/boot/firmware/field-mode` exists → skip to step 4 (force field mode)
3. Waits up to 30s for wlan0 to connect to a known WiFi network
4. **Home mode** (WiFi found): exits — drone app does not start
5. **Field mode** (no WiFi): activates AP on wlan1, then starts `drone-follow.service`

## Force Field Mode

When flying near known WiFi networks, the auto-detection may pick up your home WiFi and skip starting the AP. To force field mode regardless:

```bash
# Enable — always start AP + drone-follow on boot:
sudo touch /boot/firmware/field-mode

# Disable — return to auto-detection:
sudo rm /boot/firmware/field-mode
```

The flag file is on the boot partition, so it can also be created/removed by plugging the SD card into another computer.

## Stopping the App

To stop the drone-follow app (this session only — it will restart on next boot):

```bash
systemctl --user stop drone-follow.service
```

To also bring down the AP:

```bash
sudo systemctl stop drone-network-mode.service
sudo nmcli connection down HailoDrone-AP
```

To **disable permanently** so it doesn't start on next boot:

```bash
# Option 1 — Remove the field-mode flag (returns to auto-detection):
sudo rm /boot/firmware/field-mode

# Option 2 — Disable the boot service entirely:
sudo systemctl disable drone-network-mode.service
```

Re-enable with `sudo systemctl enable drone-network-mode.service` or re-create the field-mode flag.

## Manual AP Control

Toggle the AP on or off:

```bash
./enable_access_point.sh
```

Runs it again to stop. Check status with `nmcli device status`.

**Connect a device:** Join the `HailoDrone` SSID (password: `hailodrone`). The drone is at `10.0.0.1`.

wlan0 remains unaffected — you can have home WiFi and the AP running simultaneously.

## Adding Known WiFi Networks

```bash
sudo nmcli device wifi connect "MyNetwork" password "mypassword"
```

Any network saved in NetworkManager counts as "known" for the home mode check.
