# Drone Follow

AI-powered person-following drone application using Hailo NPU for real-time detection, ByteTracker for multi-object tracking, and MAVSDK for PX4 flight control. Includes ReID (re-identification) to recover a lost target by appearance.

Runs on Raspberry Pi 5 + Hailo-8L or on an x86_64 dev machine with a Hailo-8 PCIe card.

For complete setup and deployment instructions with OpenHD, see [SETUP_GUIDE.md](SETUP_GUIDE.md).

## Installation

### Prerequisites

- **Ubuntu 22.04+** with Python 3.10+
- **Hailo device** (Hailo-8, Hailo-8L, or Hailo-10H) — PCIe card or M.2 module
- **HailoRT driver** — must be installed before running the installer
- **Node.js / npm** (optional, for the web UI)
- For **simulation**: Gazebo Garden, `python3-gz-transport13`, `python3-gz-msgs10`
- For **real drone**: PX4-compatible flight controller (e.g. Cube Orange+)

### Step 1: Install HailoRT Driver

Download the `.deb` package for your platform from the [Hailo Developer Zone](https://hailo.ai/developer-zone/software-downloads/):

```bash
sudo dpkg -i hailort_<version>_<arch>.deb
sudo reboot
hailortcli fw-control identify   # verify device detected
```

### Step 2: Run the Installer

```bash
./install.sh
```

Auto-detects your Hailo device type (hailo8 / hailo8l). Options:

| Flag | Description |
|---|---|
| `--hailo-apps-dir DIR` | Use an existing hailo-apps checkout (default: `./hailo-apps`) |
| `--skip-hailo-apps` | Skip hailo-apps clone and system deps |
| `--skip-ui` | Skip UI npm install and build |
| `--skip-python` | Skip Python dependency installation |

### Step 3: Verify

```bash
source setup_env.sh
drone-follow --help
```

## Quick Start

```bash
source setup_env.sh

# Dev machine with USB camera + flight controller over serial:
drone-follow --input usb --serial --ui

# RPi with camera + Cube Orange+ over USB serial:
drone-follow --input rpi --serial --ui

# Simulation (Gazebo camera + PX4 SITL):
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui

# Real drone with OpenHD (starts OpenHD air + drone-follow):
scripts/start_air.sh
```

## How It Works

1. **Detection** — The Hailo NPU runs YOLOv8n on every frame to detect people.
2. **Tracking** — ByteTracker assigns persistent IDs across frames.
3. **Target selection** — By default the drone auto-follows the largest person in frame (**AUTO** mode). The operator can click a person in the web UI or use the REST API to lock onto a specific person (**LOCKED** mode).
4. **ReID recovery** — If the tracker loses the target (occlusion, fast turn), ReID compares appearance embeddings to re-identify the person under a new track ID. Works in both auto and locked modes. If ReID cannot recover the target within the search timeout (`--reid-timeout`, default 20s), the app returns to auto mode.
5. **Control** — A PID-style controller computes yaw, forward/backward, and altitude commands at 10 Hz and sends them to the flight controller via MAVSDK offboard mode.

## Key CLI Options

| Flag | Default | Description |
|---|---|---|
| `--input SOURCE` | — | Camera source: `rpi`, `usb`, `udp://host:port`, `shm://path`, or file path |
| `--serial` | off | Connect via USB serial (`/dev/ttyACM0`); overrides `--connection` |
| `--connection URL` | `udpin://0.0.0.0:14540` | MAVSDK connection string |
| `--takeoff-landing` | off | Auto arm/takeoff/land. Without this, the pilot switches to OFFBOARD via GCS. |
| `--ui` | off | Enable web UI with live video and click-to-follow (port 5001) |
| `--record` | off | Record video + detection overlays for the entire session |
| `--target-bbox-height` | `0.3` | Desired person size in frame (0-1). Adjustable mid-flight via UI. |
| `--target-altitude` | `3.0` | Target altitude in metres. Also used as takeoff height. |
| `--yaw-only` / `--no-yaw-only` | on | Yaw only: no forward/backward movement. Use `--no-yaw-only` for full follow. |
| `--no-reid` | off | Disable ReID re-identification |
| `--reid-timeout` | `20.0` | Seconds to search via ReID before returning to auto mode |
| `--no-display` | off | Headless mode (no display window) |
| `--openhd-stream` | off | Send overlay video to OpenHD via UDP RTP instead of display |

Run `drone-follow --help` for the full list.

## Web UI

Enable with `--ui` (served on port 5001). Provides live MJPEG video, detection overlays, and click-to-follow target selection.

### Status Bar

Shows real-time telemetry:
- **Following indicator** — Current mode: "Auto (largest person)", "Following: ID X" (locked), or "Idle (paused)".
- **Velocity readout** — Mode (TRACK / SEARCH / ORBIT) and commanded velocities.
- **Performance** — FPS, latency, CPU%, memory, Hailo NN core utilization, and chip temperature.
- **Record / Clear Target** buttons.

### Controller Parameters

**Operational:**

| Control | Range | Default | Description |
|---|---|---|---|
| **Target Size** | 5%-100% | 30% | Desired person bbox height. Drone approaches if smaller, retreats if larger. |
| **Target Alt** | 1-20 m | 3.0 | Target altitude. Changed mid-flight via slider. |
| **Yaw Only** | ON/OFF | ON | When ON, drone only rotates — no translation. |
| **Mode** | FOLLOW/ORBIT | FOLLOW | FOLLOW: approach/retreat. ORBIT: circle the person. |
| **Orbit Speed** | 0.2-3.0 m/s | 1.0 | Lateral speed in orbit mode. |
| **Direction** | CW/CCW | CW | Orbit direction. |

**Tuning:**

| Control | Range | Default | Description |
|---|---|---|---|
| **KP Yaw** | 0-10 | 5.0 | Yaw proportional gain. Higher = faster rotation. |
| **KP Forward** | 0-10 | 3.0 | Forward/approach gain. 0 disables forward movement entirely. |
| **KP Backward** | 0-10 | 5.0 | Backward/retreat gain. Higher than forward for safety. |
| **Yaw Smooth** | ON/OFF | ON | Low-pass filter on yaw commands. |
| **Yaw Alpha** | 0.05-1.0 | 0.3 | EMA smoothing factor for yaw. Lower = smoother. |
| **Fwd Smooth** | ON/OFF | ON | EMA smoothing on forward velocity. |
| **Fwd Alpha** | 0.05-1.0 | 0.1 | EMA factor for forward smoothing. |

## REST API

Always running on port 8080 (change with `--follow-server-port`).

| Endpoint | Description |
|---|---|
| `GET /status` | Current state: `following_id`, `last_seen`, `available_ids` |
| `POST /follow/<id>` | Follow person with tracking ID. Returns 404 if ID not in frame. |
| `POST /follow/clear` | Clear target, return to auto mode (follow largest). Clears ReID gallery. |

```bash
curl http://localhost:8080/status
curl -X POST http://localhost:8080/follow/3
curl -X POST http://localhost:8080/follow/clear
```

## Follow Modes

| Mode | Description |
|---|---|
| **AUTO** (default) | Follows the largest person in frame. No operator input needed. If the person leaves, the next largest is selected. |
| **LOCKED** | Operator clicks a person in the UI or uses `POST /follow/<id>`. ReID gallery is built for recovery. |
| **IDLE** | Drone holds position, ignores all detections. Set via OpenHD ground station (`follow_id = -1`). |

- Clicking "Clear Target" in the UI (or `POST /follow/clear`) returns to AUTO mode.
- ReID galleries are built for both auto-selected and locked targets, so the drone can recover its target after temporary occlusion in either mode.

## ReID Re-identification

When the tracker loses a **locked** target (occlusion, ID switch, fast movement), ReID uses appearance embeddings to find them again among visible detections.

- **Gallery** — While following any target (auto or locked), the app periodically extracts appearance embeddings and stores up to 10 in a gallery.
- **Recovery** — When the target is lost, all visible persons are compared against the gallery. The best match above the similarity threshold is re-identified as the target.
- **Timeout** — If ReID cannot recover the target within the search timeout (default 20s, configurable via `--reid-timeout`), the app clears the gallery and returns to auto mode (follow largest person).
- **Seamless** — The UI shows the original ID throughout; the track ID change is handled internally.

Enabled by default. Disable with `--no-reid`. Tune with:
- `--reid-model PATH` — HEF model path (default: `repvgg_a0_person_reid_512.hef`)
- `--update-interval N` — Frames between gallery updates (default: 30)
- `--reid-timeout SECONDS` — Seconds to search via ReID before returning to auto mode (default: 20)

## Simulation (Bundled PX4 SITL)

PX4 SITL + Gazebo Garden using a bundled PX4-Autopilot submodule (v1.14.0) at `sim/PX4-Autopilot`.

```bash
# One-time setup (inits submodule + builds PX4):
sim/setup_sim.sh

# Terminal 1 — Start PX4 SITL + Gazebo + video bridge:
sim/start_sim.sh --bridge --world 2_person_world

# Terminal 2 — Run drone-follow:
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
```

**Key ports:** `14540/udp` (MAVLink), `5600/udp` (video from Gazebo)

**Bundled worlds** in `sim/worlds/`: `2_person_world`, `2_persons_diagonal`, `random_walk`. Pass `--world NAME` to `start_sim.sh`.

**Remote simulation** (sim on one machine, drone-follow on another):
```bash
# Sim machine:
sim/start_sim.sh --remote <DRONE_APP_IP> --world 2_person_world

# Drone-follow machine:
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
```

**Simulation configs** in `sim/configs/`: `simulation.json` (yaw-only), `simulation_follow.json` (full follow with reduced speeds).

**USB camera with sim:** Always add `--yaw-only` — forward/altitude commands based on bbox size are unsafe because the webcam sees the real world, not the sim.

## OpenHD Integration

For FPV video with detection overlays streamed to an OpenHD ground station:

```bash
drone-follow --input shm:///tmp/openhd_raw_video --openhd-stream --ui --serial
```

The pipeline reads raw video from OpenHD's shared memory, runs detection, and streams H264+RTP back to OpenHD. Resolution is auto-detected from `/tmp/openhd_raw_video.meta`. The pipeline auto-recovers from OpenHD restarts (resolution changes, socket reconnection).

An OpenHD MAVLink parameter bridge (`OpenHDBridge`) allows controlling follow parameters from QOpenHD.

## JSON Config Files

Store controller settings in JSON instead of CLI flags:

```bash
# Save current defaults
drone-follow --save-config my_config.json

# Run with a config file (CLI flags still override)
drone-follow --config configs/outdoor_follow.json --input rpi --serial --ui
```

### Bundled Presets

| Preset | Mode | Description |
|---|---|---|
| `outdoor_follow.json` | Full follow | Real drone outdoor. 5m altitude, conservative speeds. |
| `outdoor_yaw_only.json` | Yaw-only | Real drone, rotation only. Safe for first outdoor tests. |
| `outdoor_orbit.json` | Orbit | Cinematic circling at 1.5 m/s, 5m altitude. |

## Performance Monitoring

The status bar displays real-time metrics: FPS, callback latency, host CPU%, RSS memory, Hailo chip temperature, and NN core utilization.

NN core utilization is read from HailoRT's monitor data (`/tmp/hmon_files/`), enabled automatically by the app. To also view per-model utilization in a separate terminal:

```bash
hailortcli monitor
```

## Boot Service

Auto-start drone-follow + OpenHD at boot via systemd:

```bash
sudo scripts/boot/install.sh          # one-time install
```

Edit `~/Desktop/drone-follow.conf` to enable/disable:
```
ENABLED=true    # set to false to disable auto-start
```

Uninstall: `sudo scripts/boot/uninstall.sh`

## Architecture

```
drone_follow/
  follow_api/          Pure domain logic (types, config, controller math, shared state)
  drone_api/           MAVSDK adapter (offboard velocity, takeoff/landing, telemetry)
  pipeline_adapter/    Hailo/GStreamer pipeline, ByteTracker, ReID manager
  servers/             HTTP servers (follow API, web UI + MJPEG, OpenHD bridge)
  ui/                  React web dashboard
  drone_follow_app.py  Composition root and CLI entrypoint
reid_analysis/         ReID embedding extraction and gallery matching strategies
configs/               Real-drone controller presets (outdoor_follow, outdoor_orbit, etc.)
sim/
  PX4-Autopilot/       PX4 git submodule (v1.14.0)
  bridge/              Gazebo camera -> UDP video bridge
  configs/             Simulation parameter presets
  worlds/              Gazebo world SDF files
  mavlink_relay.py     UDP relay for remote simulation
  setup_sim.sh         One-time sim setup
  start_sim.sh         Launch PX4 SITL + Gazebo + bridge
```

**Data flow:**
```
Camera -> GStreamer -> Hailo NPU (YOLOv8n) -> ByteTracker
  -> Target Selection (auto: largest / locked: specific ID)
  -> ReID (if locked target lost, --reid-timeout) -> SharedDetectionState
  -> Control Loop (10 Hz) -> MAVSDK Offboard Velocity -> PX4 Flight Controller
```

The `follow_api` package has zero external dependencies, making the controller logic testable without hardware.
