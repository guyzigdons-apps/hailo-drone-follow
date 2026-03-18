# Hailo Drone-Follow + OpenHD — Setup Guide

> **See also:**
> [PARAMETERS.md](PARAMETERS.md) — architecture, parameter flow, df_params.json schema
> | [RESOLUTION_CONTROL.md](RESOLUTION_CONTROL.md) — resolution change mechanism
> | [TROUBLESHOOTING.md](TROUBLESHOOTING.md) — common issues & debug commands

---

## Hardware

| Unit   | Board | Extras |
|--------|-------|--------|
| Air    | RPi5  | Hailo8 M.2, RPi Camera Module 3 (IMX708), monitor-mode Wi-Fi adapter |
| Ground | RPi4/5 | Same Wi-Fi adapter model, HDMI display |

Both: Raspberry Pi OS Bookworm 64-bit.

---

## Repositories

| Repo | Branch | Where |
|------|--------|-------|
| [OpenHD](https://github.com/barakbk-hailo/OpenHD.git) | `feature/hailo-apps-integration` | Air + Ground |
| [OpenHD-SysUtils](https://github.com/barakbk-hailo/OpenHD-SysUtils.git) | `main` | Air + Ground |
| [QOpenHD](https://github.com/barakbk-hailo/qopenHD.git) | `fix/rpi4-hw-decode` | Ground |
| [hailo-drone-follow](git@github.com:guyzigdons-apps/hailo-drone-follow.git) | `feature/openhd-integration-new` | Air |

---

## Air Unit Setup

### 1. Install Hailo Prerequisites

```bash
sudo apt update
sudo apt install -y dkms
sudo apt install -y hailo-all
```

After installation, verify the Hailo device is detected:
```bash
hailortcli fw-control identify
```

Fix permissions on Hailo resource files (required for the pipeline to read JSON configs):
```bash
sudo chmod 644 /usr/local/hailo/resources/json/*.json
```

### 2. Clone

```bash
cd ~
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git
git clone -b main https://github.com/barakbk-hailo/OpenHD-SysUtils.git
git clone -b feature/openhd-integration-new \
    git@github.com:guyzigdons-apps/hailo-drone-follow.git
```

### 3. Build OpenHD

```bash
cd ~/OpenHD && sudo ./build_native.sh all
```

Rebuilding after code changes only:
```bash
cd ~/OpenHD/OpenHD
sudo cmake --build build_release -j$(nproc)
sudo cp build_release/openhd /usr/local/bin/openhd
```

### 4. Install drone-follow

```bash
cd ~/hailo-drone-follow && ./install.sh
```

### 5. Deploy df_params.json & encryption key

```bash
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/hailo-drone-follow/df_params.json /usr/local/share/openhd/df_params.json

# First-time only — generate encryption key (must match ground unit):
sudo dd if=/dev/urandom of=/usr/local/share/openhd/txrx.key bs=32 count=1 2>/dev/null
```

### 6. Configure Camera Mode

See the [Camera Modes](#camera-modes) section below for choosing and
configuring Mode A or Mode B.

---

## Ground Unit Setup

### 1. Clone

```bash
cd ~
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git
git clone -b main https://github.com/barakbk-hailo/OpenHD-SysUtils.git
git clone -b fix/rpi4-hw-decode https://github.com/barakbk-hailo/qopenHD.git
```

### 2. Build OpenHD

```bash
cd ~/OpenHD && sudo ./build_native.sh all
```

### 3. Build QOpenHD

```bash
cd ~/qopenHD && sudo ./install_build_dep.sh rpi
mkdir -p build/release && cd build/release
qmake ../.. && make -j$(nproc)
```

Binary: `~/qopenHD/build/release/QOpenHD`

### 4. Deploy df_params.json & encryption key

```bash
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/path/to/df_params.json /usr/local/share/openhd/df_params.json
scp pi@<air-ip>:/usr/local/share/openhd/txrx.key /tmp/txrx.key
sudo cp /tmp/txrx.key /usr/local/share/openhd/txrx.key
```

### 5. CLI-only mode (recommended)

```bash
sudo systemctl set-default multi-user.target && sudo reboot
```

---

## Camera Modes

There are two integration modes. Both use the Hailo8 for AI detection.

### Mode A — Camera Type 5 (`X_CAM_TYPE_HAILO_AI`)

drone-follow **owns the camera** — it captures directly from the RPi camera,
runs Hailo inference, draws overlay, encodes to H.264, and streams RTP to
OpenHD which treats it as an external video source.

**Command:**
```bash
drone-follow --input rpi --openhd-stream --horizontal-mirror \
    --connection tcpout://127.0.0.1:5760
```
> **Note:** `--horizontal-mirror` is only for selfie mode (front-facing camera).
> Omit for rear-facing.

**Configuration:**
1. Set camera type to **5** (HAILO_AI) in QOpenHD camera settings, or edit directly:
   ```bash
   sudo vim /boot/openhd/camera1.txt
   ```
2. Make sure no `hailo.txt` flag file exists:
   ```bash
   sudo rm -f /boot/openhd/hailo.txt
   ```
3. Resolution is controlled by drone-follow CLI arguments (`--width`, `--height`).

### Mode B — Shared Memory (`hailo.txt` flag) *(recommended)*

OpenHD **owns the camera** — it captures from libcamera as normal, encodes
for WFB transmission, and also tees raw NV12 frames to a shared-memory socket.
drone-follow reads from SHM and performs AI inference only (no encoding).

**Command:**
```bash
drone-follow --input shm:///tmp/openhd_raw_video --no-display \
    --connection tcpout://127.0.0.1:5760
```

**Configuration:**
1. Set camera type to a **normal libcamera type** in QOpenHD, or edit directly:
   ```bash
   sudo vim /boot/openhd/camera1.txt
   ```
   Common values: `31` = IMX219, `32` = IMX708.
2. Create the SHM flag file so OpenHD exposes raw video via shared memory:
   ```bash
   sudo mkdir -p /boot/openhd
   sudo touch /boot/openhd/hailo.txt
   ```
3. Resolution changes via QOpenHD work seamlessly (auto-detected via SHM metadata).

---

## Running the System

### Step 1 — Air: Start OpenHD

```bash
sudo /usr/local/bin/openhd --air --clean-start
```

### Step 2 — Air: Start drone-follow

**Camera Type 5 mode** (Mode A):
```bash
cd ~/hailo-drone-follow && source venv/bin/activate
drone-follow --input rpi --openhd-stream --horizontal-mirror \
    --connection tcpout://127.0.0.1:5760 \
    --tiles-x 1 --tiles-y 1
```

**SHM mode** (Mode B):
```bash
cd ~/hailo-drone-follow && source venv/bin/activate
drone-follow --input shm:///tmp/openhd_raw_video --no-display \
    --connection tcpout://127.0.0.1:5760 \
    --tiles-x 1 --tiles-y 1
```

> Start drone-follow **after** OpenHD.

### Step 3 — Ground: Start OpenHD

```bash
sudo /usr/local/bin/openhd --ground --clean-start
```

### Step 4 — Ground: Start QOpenHD

**CLI-only (EGLFS)**:
```bash
cd ~/qopenHD
sudo env -u DISPLAY -u WAYLAND_DISPLAY \
    QT_QPA_PLATFORM=eglfs QT_QPA_EGLFS_KMS_ATOMIC=1 \
    QT_QPA_EGLFS_KMS_CONFIG=$HOME/qopenHD/rpi_qt_eglfs_kms_config.json \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    ./build/release/QOpenHD_hailo_dynamic -platform eglfs
```

**With desktop (Wayland)**:
```bash
WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 \
    ./build/release/QOpenHD_hailo_dynamic -platform wayland
```

---

## Rebuilding After Code Changes

| Component | Command |
|-----------|---------|
| OpenHD (C++) | `cd ~/OpenHD/OpenHD && sudo cmake --build build_release -j$(nproc) && sudo cp build_release/openhd /usr/local/bin/openhd` |
| QOpenHD (C++/QML) | `cd ~/qopenHD/build/release && make -j$(nproc)` |
| drone-follow (Python) | No build — just restart the process |
