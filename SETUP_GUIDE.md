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

See the [Camera Modes](#camera-modes) section below for choosing and
configuring Mode A or Mode B.

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

## x86_64 Ground Station (Laptop / Desktop)

An x86_64 Ubuntu machine can run the full ground station stack. No Hailo
hardware is needed on the ground side — video is decoded in software via
FFmpeg/libavcodec. The build system auto-detects x86_64 and configures
everything accordingly (SSSE3 FEC, `LinuxBuild` Qt config, `__desktoplinux__`
define).

### Prerequisites

- Ubuntu 22.04+ (64-bit)
- Monitor-mode USB WiFi adapter (same model as the air unit — e.g. rtl88x2bu)
- System packages:
  ```bash
  sudo apt install -y dkms iw
  ```

### 1. Clone

Same as the RPi ground unit, but clone QOpenHD **with submodules**:
```bash
cd ~
git clone --recurse-submodules -b feature/hailo-apps-integration \
    https://github.com/barakbk-hailo/OpenHD.git
git clone -b main https://github.com/barakbk-hailo/OpenHD-SysUtils.git
git clone --recurse-submodules -b fix/rpi4-hw-decode \
    https://github.com/barakbk-hailo/qopenHD.git
```

> If you forgot `--recurse-submodules` on qopenHD:
> ```bash
> cd ~/qopenHD && git submodule update --init --recursive
> ```

### 2. Automated install (recommended)

A bundled script in this repo handles deps, builds, and config deployment in one step.
It auto-detects the platform (x86_64 / RPi5 / RPi4), or you can override with `--platform`:
```bash
cd ~/hailo-drone-follow
sudo ./scripts/install_ground_station.sh
# Or explicitly: sudo ./scripts/install_ground_station.sh --platform ubuntu-x86
```

### 3. Manual install (step-by-step)

If you prefer to run each step yourself:

**Install OpenHD dependencies + build:**
```bash
cd ~/OpenHD
sudo ./install_build_dep.sh ubuntu-x86
sudo ./build_native.sh build        # builds SysUtils + OpenHD, installs to /usr/local/bin/
```

> **Note:** Use `build` to compile OpenHD only. You also need the WiFi driver
> for the monitor-mode USB adapter:
> ```bash
> sudo ./build_native.sh driver
> sudo reboot
> ```
> Or run `sudo ./build_native.sh all` to do deps + build + driver in one step.

**Install QOpenHD dependencies + build:**
```bash
cd ~/qopenHD
sudo ./install_build_dep.sh ubuntu-x86

# Compile Qt translation files (required before build):
lrelease translations/*.ts
cp translations/*.qm qml/

mkdir -p build/release && cd build/release
qmake ../.. && make -j$(nproc)
```

> **Note:** On Ubuntu 22.04 the `install_build_dep.sh` script may fail on
> `t64`-suffixed Qt packages. The patched version in this repo auto-detects
> the correct package names for your Ubuntu version.

Binary location: `~/qopenHD/build/release/release/QOpenHD` (note the double
`release` — qmake puts the output one level deeper on Linux).

**Deploy config files:**
```bash
sudo mkdir -p /usr/local/share/openhd
sudo cp ~/hailo-drone-follow/df_params.json /usr/local/share/openhd/df_params.json

# Copy encryption key from air unit (must match):
scp pi@<air-ip>:/usr/local/share/openhd/txrx.key /tmp/txrx.key
sudo cp /tmp/txrx.key /usr/local/share/openhd/txrx.key
```

### 4. Running

Use the bundled start script (launches both OpenHD ground + QOpenHD):
```bash
./scripts/start_ground.sh
```

Or manually:
```bash
# Terminal 1 — OpenHD ground:
sudo /usr/local/bin/openhd --ground

# Terminal 2 — QOpenHD (Wayland):
WAYLAND_DISPLAY=wayland-0 XDG_RUNTIME_DIR=/run/user/1000 \
    ~/qopenHD/build/release/release/QOpenHD -platform wayland

# Or under X11:
~/qopenHD/build/release/release/QOpenHD
```

> **Differences from RPi ground:**
> - Video decoding uses software libavcodec (FFmpeg) instead of RPi MMAL hardware decoder
> - No EGLFS — use Wayland or X11 platform
> - Reboot required after WiFi driver install (`build_native.sh driver`)

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
sudo /usr/local/bin/openhd --air
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
