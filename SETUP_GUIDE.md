# Hailo Drone-Follow + OpenHD — Complete Setup Guide

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Repository Map](#repository-map)
4. [Air Unit Setup (RPi5 + Hailo8)](#air-unit-setup)
5. [Ground Unit Setup (RPi4)](#ground-unit-setup)
6. [Running the System](#running-the-system)
7. [The df_params.json Schema](#the-df_paramsjson-schema)
8. [Adding New Parameters](#adding-new-parameters)
9. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

The drone-follow parameter control path spans three software layers across
two Raspberry Pi units connected via wifibroadcast:

```
┌──────────────────────── AIR UNIT (RPi5 + Hailo8) ─────────────────────────┐
│                                                                            │
│  ┌──────────────────────┐    UDP JSON      ┌───────────────────────────┐  │
│  │  drone-follow        │◄────────────────►│  OpenHD air               │  │
│  │  (Python)            │  port 5510/5511  │  hailo_follow_bridge.cpp  │  │
│  │                      │                  │                           │  │
│  │  • Applies params    │  5510: OpenHD→Py │  • Loads df_params.json   │  │
│  │    to control loop   │  5511: Py→OpenHD │  • MAVLink param server   │  │
│  │  • Reports current   │                  │  • Persists values to disk│  │
│  │    values back       │                  │  • Translates MAVLink ↔   │  │
│  └──────────────────────┘                  │    UDP JSON               │  │
│                                            └─────────────┬─────────────┘  │
│                                              wifibroadcast (MAVLink relay) │
└──────────────────────────────────────────────────────────┼─────────────────┘
                                                           │ RF link
┌──────────────────────────────────────────────────────────┼─────────────────┐
│                                          GROUND (RPi4)   │                 │
│                                            ┌─────────────▼─────────────┐  │
│                                            │  OpenHD ground            │  │
│                                            │  • MAVLink relay          │  │
│                                            └─────────────┬─────────────┘  │
│                                                          │                 │
│                                            ┌─────────────▼─────────────┐  │
│                                            │  QOpenHD                  │  │
│                                            │  • Loads df_params.json   │  │
│                                            │    (UI metadata only)     │  │
│                                            │  • DroneFollow settings   │  │
│                                            │    tab: sliders/switches  │  │
│                                            │  • PARAM_EXT_SET/GET      │  │
│                                            └───────────────────────────┘  │
└───────────────────────────────────────────────────────────────────────────┘

  ┌──────────────┐
  │ df_params.json│  ← Single source of truth for all DF_ parameters
  │              │     Deployed to both units (no rebuild needed)
  └──────────────┘
```

### Parameter Set Flow (user moves a slider in QOpenHD)

```
QOpenHD slider
  → MAVLink PARAM_EXT_SET
    → OpenHD ground (relay)
      → wifibroadcast RF
        → OpenHD air / hailo_follow_bridge
          → persist value to disk
          → UDP JSON {"param": "kp_yaw", "value": 5.1} to port 5510
            → drone-follow Python (applies immediately)
          → MAVLink PARAM_EXT_ACK back to QOpenHD
```

### Parameter Readback Flow (QOpenHD reads current values)

```
drone-follow Python
  → UDP JSON {"params": {"kp_yaw": 5.1, ...}} to port 5511
    → OpenHD air / hailo_follow_bridge (updates cache)
      → MAVLink PARAM_EXT_VALUE
        → wifibroadcast RF
          → OpenHD ground (relay)
            → QOpenHD (updates slider position)
```

### The Bridge (hailo_follow_bridge.cpp)

The bridge sits inside OpenHD on the air unit and translates between two
protocols:

- **MAVLink side**: Registers all DF_ parameters from `df_params.json` as
  MAVLink extended parameters. QOpenHD (via OpenHD ground relay) can get/set
  them using standard PARAM_EXT_SET/PARAM_EXT_REQUEST_READ messages.

- **UDP JSON side**: Communicates with the drone-follow Python app on localhost:
  - Port **5510** (OpenHD → Python): `{"param": "kp_yaw", "value": 5.1}`
  - Port **5511** (Python → OpenHD): `{"params": {"kp_yaw": 5.1, "kp_fwd": 3.0, ...}}`

### The df_params.json Schema

A single JSON file defines every DF_ parameter — its MAVLink ID, type, range,
default, group, label, and description. All three layers read this same file:

- **OpenHD air** (C++): Loads at startup to register MAVLink params dynamically
- **QOpenHD** (QML): Loads at tab open to auto-generate sliders/switches/spinboxes
- **drone-follow** (Python): Can read defaults from this file

When you add a parameter to `df_params.json` and deploy it to both units, the
new param appears in QOpenHD's UI and works end-to-end — no C++ or QML rebuild
needed.

---

## Hardware Requirements

| Unit   | Board     | Extras                                          |
|--------|-----------|-------------------------------------------------|
| Air    | RPi5      | Hailo8 M.2 AI accelerator, RPi camera (IMX708), Wi-Fi adapter (monitor mode capable, e.g. RTL88x2BU) |
| Ground | RPi4      | Wi-Fi adapter (same model as air), HDMI display  |

Both units need:
- Raspberry Pi OS Bookworm (64-bit / aarch64)
- Internet access during setup (for cloning repos and installing deps)

---

## Repository Map

| Repo | GitHub URL | Branch | Deployed on |
|------|-----------|--------|-------------|
| **OpenHD** | `https://github.com/barakbk-hailo/OpenHD.git` | `feature/hailo-apps-integration` | Air + Ground |
| **OpenHD-SysUtils** | `https://github.com/barakbk-hailo/OpenHD-SysUtils.git` | `main` | Air + Ground |
| **QOpenHD** | `https://github.com/barakbk-hailo/qopenHD.git` | `fix/rpi4-hw-decode` | Ground only |
| **hailo-drone-follow** | `git@github.com:guyzigdons-apps/hailo-drone-follow.git` | `feature/openhd-integration-new` | Air only |

> **Note**: hailo-drone-follow uses SSH git URL. You need SSH key access to the
> guyzigdons-apps org, or use HTTPS if available.

---

## Air Unit Setup

### 1. Clone repositories

```bash
cd ~

# OpenHD (with submodules — contains nlohmann/json, spdlog, mavlink-headers, wifibroadcast)
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git

# OpenHD-SysUtils (must be a sibling directory to OpenHD)
git clone -b main \
    https://github.com/barakbk-hailo/OpenHD-SysUtils.git

# hailo-drone-follow
git clone -b feature/openhd-integration-new \
    git@github.com:guyzigdons-apps/hailo-drone-follow.git
```

### 2. Build and install OpenHD

```bash
cd ~/OpenHD

# Install all dependencies + build + install binary to /usr/local/bin/openhd
sudo ./build_native.sh all
```

This script:
- Detects the platform (RPi5) automatically
- Installs build and runtime dependencies (cmake, libpcap, libsodium, GStreamer, etc.)
- Builds OpenHD-SysUtils and installs it
- Builds OpenHD and installs the `openhd` binary to `/usr/local/bin/openhd`
- Builds and installs the RTL88x2BU Wi-Fi driver

If you only need to rebuild after code changes (deps already installed):

```bash
cd ~/OpenHD/OpenHD
cmake -S . -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release -j$(nproc)
sudo cp build_release/openhd /usr/local/bin/openhd
```

### 3. Install hailo-drone-follow

```bash
cd ~/hailo-drone-follow

# Full install: hailo-apps platform, Python venv, Hailo bindings, UI
./install.sh
```

This creates a Python virtual environment at `~/hailo-drone-follow/venv/` and
installs all dependencies including MAVSDK, hailo-apps, and Hailo Python
bindings.

### 4. Deploy df_params.json

```bash
# Copy to the production path that OpenHD reads at startup
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/hailo-drone-follow/df_params.json /usr/local/share/openhd/df_params.json
```

### 5. Create OpenHD encryption key (first-time only)

OpenHD expects an encryption key file. If it doesn't exist:

```bash
sudo mkdir -p /usr/local/share/openhd
# Generate a key or create a dummy one for bench testing:
sudo dd if=/dev/urandom of=/usr/local/share/openhd/txrx.key bs=32 count=1 2>/dev/null
```

> Both air and ground must use the **same key file** for wifibroadcast
> encryption. Copy it to the ground unit as well.

---

## Ground Unit Setup

### 1. Clone repositories

```bash
cd ~

# OpenHD
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git

# OpenHD-SysUtils
git clone -b main \
    https://github.com/barakbk-hailo/OpenHD-SysUtils.git

# QOpenHD
git clone -b fix/rpi4-hw-decode \
    https://github.com/barakbk-hailo/qopenHD.git

# df_params.json (only needed for QOpenHD UI rendering)
# Copy from hailo-drone-follow repo or download separately
```

### 2. Build and install OpenHD

```bash
cd ~/OpenHD
sudo ./build_native.sh all
```

Same as the air unit — the script auto-detects RPi4.

### 3. Build QOpenHD

Install Qt5 build dependencies first:

```bash
cd ~/qopenHD
sudo ./install_build_dep.sh rpi
```

Then build with qmake:

```bash
cd ~/qopenHD
mkdir -p build/release && cd build/release
qmake ../..
make -j$(nproc)
```

The binary is at `~/qopenHD/build/release/QOpenHD`.

### 4. Deploy df_params.json

QOpenHD reads `df_params.json` to dynamically render the DroneFollow settings
tab. Copy it to the production path:

```bash
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/path/to/df_params.json /usr/local/share/openhd/df_params.json
```

> You can also place it at `~/hailo-drone-follow/df_params.json` as a fallback
> path. QOpenHD checks `/usr/local/share/openhd/df_params.json` first.

### 5. Copy encryption key

Copy the same `txrx.key` from the air unit:

```bash
sudo mkdir -p /usr/local/share/openhd
scp pi@<air-unit-ip>:/usr/local/share/openhd/txrx.key /tmp/txrx.key
sudo cp /tmp/txrx.key /usr/local/share/openhd/txrx.key
```

### 6. Switch to CLI-only mode (recommended for production)

QOpenHD runs best with direct GPU access (EGLFS), which requires no desktop:

```bash
sudo systemctl set-default multi-user.target
sudo reboot
```

To switch back to desktop later:
```bash
sudo systemctl set-default graphical.target
sudo reboot
```

---

## Running the System

### Step 1: Start OpenHD on the Air Unit

```bash
sudo /usr/local/bin/openhd --air --clean-start
```

- `--air` — configures as air unit (camera + transmit)
- `--clean-start` — wipes old settings to avoid conflicts

Leave this running. You should see:
```
Using external camera type (HAILO_AI).
OpenHD was successfully started.
Loaded df_params.json from /usr/local/share/openhd/df_params.json
```

### Step 2: Start drone-follow on the Air Unit (second terminal)

```bash
cd ~/hailo-drone-follow
source venv/bin/activate
drone-follow --input rpi --openhd-stream --horizontal-mirror --connection tcpout://127.0.0.1:5760 --tiles-x 1 --tiles-y 1
```

- `--input rpi` — use RPi camera (IMX708) via libcamera
- `--openhd-stream` — feed processed video into OpenHD's GStreamer pipeline
- `--horizontal-mirror` — mirror the image horizontally
- `--connection tcpout://127.0.0.1:5760` — connect to the flight controller via MAVLink TCP

> **Note**: drone-follow must be started AFTER OpenHD air, because it connects
> to OpenHD's shared memory video pipeline and MAVLink TCP server.

### Step 3: Start OpenHD on the Ground Unit

```bash
sudo /usr/local/bin/openhd --ground --clean-start
```

Leave this running.

### Step 4: Start QOpenHD on the Ground Unit (second terminal)

**CLI-only mode** (no desktop — recommended):

```bash
cd ~/qopenHD
sudo env -u DISPLAY -u WAYLAND_DISPLAY \
    QT_QPA_PLATFORM=eglfs \
    QT_QPA_EGLFS_KMS_ATOMIC=1 \
    QT_QPA_EGLFS_KMS_CONFIG=$HOME/qopenHD/rpi_qt_eglfs_kms_config.json \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    ./build/release/QOpenHD_hailo_dynamic -platform eglfs
```

**With desktop** (Wayland/labwc running):

```bash
WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 \
    ./build/release/QOpenHD_hailo_dynamic -platform wayland
```

### Using the DroneFollow Settings Tab

1. Open QOpenHD → tap the settings icon (gear)
2. Click **DroneFollow** in the left sidebar
3. All DF_ parameters appear as sliders/switches grouped by category
4. Move a slider → the value is sent via MAVLink to the air unit → applied in real-time
5. A busy indicator shows when changes are in-flight

---

## The df_params.json Schema

Located at: `~/hailo-drone-follow/df_params.json`
Deployed to: `/usr/local/share/openhd/df_params.json` (on both units)

```json
{
  "version": 1,
  "groups": [
    {"id": "yaw",    "label": "YAW CONTROL",       "order": 1},
    {"id": "fwd",    "label": "FORWARD / BACKWARD", "order": 2},
    ...
  ],
  "params": [
    {
      "id": "kp_yaw",
      "mavlink_id": "DF_KP_YAW",
      "type": "float",
      "default": 5.0,
      "min": 0.0,
      "max": 20.0,
      "step": 0.1,
      "group": "yaw",
      "order": 1,
      "label": "Kp Yaw",
      "description": "Proportional gain for yaw tracking.",
      "read_only": false
    },
    ...
  ]
}
```

### Field Reference

| Field        | Description |
|-------------|-------------|
| `id`         | Internal Python field name (used in UDP JSON IPC) |
| `mavlink_id` | MAVLink parameter name (max 16 chars, must start with `DF_`) |
| `type`       | `"float"` → slider, `"int"` → spin box, `"bool"` → toggle switch |
| `default`    | Default value (used when no persisted value exists) |
| `min`/`max`  | Range limits. For `float`: actual values. For `int` sent via MAVLink: values are ×100 internally |
| `step`       | Increment step for the UI control |
| `group`      | Must match a group `id` from the `groups` array |
| `order`      | Sort order within the group |
| `label`      | Display name in QOpenHD |
| `description`| Tooltip / help text |
| `read_only`  | If `true`, displayed as read-only text (no control) |

---

## Adding New Parameters

To add a new parameter (e.g. `DF_MY_PARAM`):

### 1. Edit df_params.json

Add a new entry to the `params` array:

```json
{
  "id": "my_param",
  "mavlink_id": "DF_MY_PARAM",
  "type": "float",
  "default": 1.0,
  "min": 0.0,
  "max": 10.0,
  "step": 0.1,
  "group": "flight",
  "order": 10,
  "label": "My Parameter",
  "description": "Description of what this parameter does.",
  "read_only": false
}
```

### 2. Deploy to both units

```bash
# On the machine where you edited the JSON:
sudo cp df_params.json /usr/local/share/openhd/df_params.json

# Copy to the other unit:
scp df_params.json pi@<other-unit-ip>:/home/pi/hailo-drone-follow/df_params.json
ssh pi@<other-unit-ip> "sudo cp /home/pi/hailo-drone-follow/df_params.json /usr/local/share/openhd/df_params.json"
```

### 3. Restart

- Restart OpenHD on the air unit (it loads the JSON at startup)
- Restart QOpenHD on the ground (it loads the JSON when opening the DroneFollow tab)

No recompilation needed. The new parameter will:
- Appear in QOpenHD's DroneFollow tab automatically
- Be accepted by the bridge on the air unit
- Be persisted across restarts
- Be forwarded to the Python app via UDP JSON (if the Python app handles it)

### 4. (Optional) Handle in Python

To make the drone-follow Python app react to the new parameter, read it from
the `controller_config` dict that the OpenHD bridge populates:

```python
# In your control loop:
my_param = controller_config.get("my_param", 1.0)
```

---

## Troubleshooting

### QOpenHD

| Symptom | Solution |
|---------|----------|
| "Could not open display" / "X11 connection rejected" | Use `sudo env -u DISPLAY -u WAYLAND_DISPLAY ...` |
| "XDG_RUNTIME_DIR not set" | Add `XDG_RUNTIME_DIR=/tmp/runtime-root` to the command |
| "Permission denied" on page flip | Switch to CLI-only mode or use `-platform wayland` |
| No video (black screen, GUI works) | Check air unit is running and transmitting |
| DroneFollow tab is empty | Check `/usr/local/share/openhd/df_params.json` exists and is valid JSON |
| Slider changes don't reach air | Check OpenHD ground is running and wifibroadcast link is up |

### OpenHD

| Symptom | Solution |
|---------|----------|
| "Unable to open txrx.key" | Generate key file (see setup step 5) |
| "Restarting camera due to no frame" | drone-follow not started yet, or GStreamer SHM pipeline not connected |
| "df_params.json not found" | Copy JSON to `/usr/local/share/openhd/df_params.json` |
| Wi-Fi adapter not detected | Plug in adapter BEFORE starting OpenHD; check driver is installed |

### drone-follow

| Symptom | Solution |
|---------|----------|
| "Connection failed: Invalid connection URL" | Use `tcpout://` format (not `tcp://`). Ensure `mavsdk_drone.py` fix is applied |
| "hailo_display not found in pipeline" | Normal warning when running headless with `--openhd-stream` |
| Camera not detected | Check `libcamera-hello` works; ensure no other process holds the camera |
| MAVSDK server won't start | Kill any leftover `mavsdk_server` processes: `pkill -f mavsdk_server` |

### Rebuilding After Code Changes

**OpenHD** (air or ground — after editing C++ files):
```bash
cd ~/OpenHD/OpenHD
cmake --build build_release -j$(nproc)
sudo cp build_release/openhd /usr/local/bin/openhd
```

**QOpenHD** (ground — after editing QML or C++ files):
```bash
cd ~/qopenHD/build/release
make -j$(nproc)
# Binary is at ./QOpenHD
```

> **QML-only changes** (like editing `HailoDroneFollowPanel.qml`) still require
> a rebuild because QML files are compiled into the binary via `qml.qrc`.

**drone-follow** (air — Python, no build needed):
```bash
# Just restart the process. Python changes take effect immediately.
```
