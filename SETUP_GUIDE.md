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

### 1. Install system prerequisites

```bash
# DKMS is required for the WiFi driver build (not pulled automatically)
sudo apt install -y dkms

# Hailo runtime — the installer needs root for directory creation,
# so run with sudo:
sudo apt install -y hailo-all

# Node.js & npm — needed by the drone-follow UI build step.
# Not available in the default repos on Bookworm; install manually:
curl -fsSL https://deb.nodesource.com/setup_20.x | sudo bash -
sudo apt install -y nodejs
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

> **Important — WiFi driver & reboot:** The `all` target builds the WiFi
> driver (rtl88x2bu via DKMS). A kernel update or `apt upgrade` on the next
> reboot can overwrite the driver module. If Wi-Fi stops working after a
> reboot, rebuild **only** the driver:
> ```bash
> cd ~/OpenHD && sudo ./build_native.sh driver
> sudo reboot
> ```

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

### 6. Enable SHM passthrough (for SHM mode)

Create the marker file so OpenHD exposes raw video via shared memory:

```bash
sudo mkdir -p /boot/openhd
sudo touch /boot/openhd/hailo.txt
```

---

## Ground Unit Setup

### 1. Install system prerequisites

```bash
sudo apt install -y dkms
```

### 2. Clone

```bash
cd ~
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git
git clone -b main https://github.com/barakbk-hailo/OpenHD-SysUtils.git
git clone -b fix/rpi4-hw-decode https://github.com/barakbk-hailo/qopenHD.git
```

### 3. Build OpenHD

```bash
cd ~/OpenHD && sudo ./build_native.sh all
```

> **Important — WiFi driver & reboot:** Same caveat as the air unit.
> If Wi-Fi breaks after a reboot, rebuild the driver alone:
> ```bash
> cd ~/OpenHD && sudo ./build_native.sh driver
> sudo reboot
> ```

### 4. Build QOpenHD

```bash
cd ~/qopenHD && sudo ./install_build_dep.sh rpi
mkdir -p build/release && cd build/release
qmake ../.. && make -j$(nproc)
```

Binary: `~/qopenHD/build/release/QOpenHD`

### 5. Deploy df_params.json & encryption key

```bash
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/path/to/df_params.json /usr/local/share/openhd/df_params.json
scp pi@<air-ip>:/usr/local/share/openhd/txrx.key /tmp/txrx.key
sudo cp /tmp/txrx.key /usr/local/share/openhd/txrx.key
```

### 6. CLI-only mode (recommended)

```bash
sudo systemctl set-default multi-user.target && sudo reboot
```

---

## Camera Modes

There are two integration modes. Both use the Hailo8 for AI detection.

### Mode A — SHM Passthrough (recommended)

OpenHD owns the camera (libcamerasrc). It creates a shared-memory branch
(`/tmp/openhd_raw_video`) so drone-follow reads raw NV12 frames without
opening the camera itself.

**Requirements:**
- `/boot/openhd/hailo.txt` must exist (enables the SHM branch)
- Camera type in OpenHD: any RPi libcamera type (e.g. IMX708 = type 32)
- drone-follow uses `--input shm:///tmp/openhd_raw_video`
- Resolution changes via QOpenHD work seamlessly (auto-detected via metadata)

### Mode B — Hailo Camera Type 5

drone-follow owns the camera directly and sends processed H264 video to
OpenHD via UDP RTP.

**Requirements:**
- Set camera type to **Hailo AI (5)** in QOpenHD camera settings
- drone-follow uses `--input rpi --openhd-stream`
- OpenHD receives the encoded stream instead of capturing from libcamerasrc
- Resolution is controlled by drone-follow CLI arguments (`--width`, `--height`)

---

## Running the System

### Step 1 — Air: Start OpenHD

```bash
sudo /usr/local/bin/openhd --air
```

### Step 2 — Air: Start drone-follow

**SHM mode** (Mode A):
```bash
cd ~/hailo-drone-follow && source venv/bin/activate
drone-follow --input shm:///tmp/openhd_raw_video \
    --openhd-stream --horizontal-mirror \
    --connection tcpout://127.0.0.1:5760 \
    --tiles-x 1 --tiles-y 1
```

**Hailo camera type mode** (Mode B):
```bash
cd ~/hailo-drone-follow && source venv/bin/activate
drone-follow --input rpi --openhd-stream --horizontal-mirror \
    --connection tcpout://127.0.0.1:5760 \
    --tiles-x 1 --tiles-y 1
```

> Start drone-follow **after** OpenHD.

### Step 3 — Ground: Start OpenHD

```bash
sudo /usr/local/bin/openhd --ground
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

## Ground Recording & Offline Embedding

QOpenHD can record the live video stream on the ground unit along with
detection metadata and HUD overlay data. Embedding (compositing BBs and HUD
onto the video) is done offline with a Python script.

### Recording (in QOpenHD)

Open the **Ground Recording** sidebar panel (Panel 9). Controls:

- **Start/Stop Recording** — tees the raw H.264 stream to file (zero CPU overhead)
- **Save HUD overlay** toggle — when ON, captures HUD graphics as sparse RGBA
  tiles alongside the video (`.osd` file)

On stop, the raw `.h264` is automatically muxed to `.mp4` and the raw file is
deleted. Recordings are saved to `~/Videos/`:

| File | Contents |
|------|----------|
| `ground_YYYYMMDD_HHMMSS.mp4` | Raw video (H.264 in MP4 container) |
| `ground_YYYYMMDD_HHMMSS.jsonl` | Detection bounding boxes (one JSON line per frame) |
| `ground_YYYYMMDD_HHMMSS.osd` | HUD overlay (OSD3 binary — sparse RGBA tiles) |

### Embedding (offline Python script)

The embed tool composites detections and/or HUD overlay onto the recorded
video. It runs on the Pi (when OpenHD is off) or on any machine with
`ffmpeg`, `numpy`, and `Pillow`.

**Location:** `~/qopenHD/tools/embed_recording.py`

**Install dependencies** (if not already available):
```bash
pip install numpy Pillow
```

**Basic usage:**
```bash
# Embed latest recording — detections + HUD at 1080p (defaults):
python3 ~/qopenHD/tools/embed_recording.py ~/Videos/ground_20260324_165522.mp4

# Detections only, keep original resolution:
python3 ~/qopenHD/tools/embed_recording.py ~/Videos/ground_20260324_165522.mp4 \
    --no-hud -r original

# HUD only, no bounding boxes:
python3 ~/qopenHD/tools/embed_recording.py ~/Videos/ground_20260324_165522.mp4 \
    --no-detections

# Process all recordings in a directory:
python3 ~/qopenHD/tools/embed_recording.py ~/Videos/ --all
```

**CLI flags:**

| Flag | Default | Description |
|------|---------|-------------|
| `--detections` / `--no-detections` | on | Include detection bounding boxes |
| `--hud` / `--no-hud` | on | Include HUD overlay |
| `-r WxH` | `1920x1080` | Output resolution (`original` to keep source res) |
| `--crf N` | 20 | H.264 quality (lower = better, 0–51) |
| `--preset` | fast | x264 speed/quality tradeoff |
| `--suffix` | `_embed` | Output filename suffix |
| `--all` | off | Process all recordings in directory |

**To use on another machine**, copy the recording files (`.mp4`, `.jsonl`,
`.osd`) and the script to the host — no QOpenHD build required.

---

## Rebuilding After Code Changes

| Component | Command |
|-----------|---------|
| OpenHD (C++) | `cd ~/OpenHD/OpenHD && sudo cmake --build build_release -j$(nproc) && sudo cp build_release/openhd /usr/local/bin/openhd` |
| QOpenHD (C++/QML) | `cd ~/qopenHD/build/release && make -j$(nproc)` |
| drone-follow (Python) | No build — just restart the process |
