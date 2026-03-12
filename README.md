# Drone Follow Demo

## Installation

### Prerequisites

- **Hailo application infrastructure** (hailort, hailo-tappas-core, Python bindings).
- **Python 3.9+** with the project virtual environment activated (`source setup_env.sh`).
- **MAVSDK** for drone connection and offboard control (installed via pip in the project venv).
- For **simulation**: PX4 SITL and Gazebo (e.g. `make px4_sitl gz_x500_mono_cam` from PX4-Autopilot). See [PX4 Ubuntu Dev Environment Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu) for toolchain installation.
- For **real drone**: PX4 (or compatible) firmware and a connection (e.g. USB or UDP).

### Steps

1. **Install dependencies**:
   ```bash
   ./install.sh
   source setup_env.sh
   ```

2. **Optional – Web UI** (for `--ui` with live video and click-to-follow):

   Requires **Node.js / npm** to be installed.
   ```bash
   cd drone_follow/ui
   npm install
   npm run build
   cd -
   ```
   The built UI is served from `drone_follow/ui/build` when you run with `--ui`.
   Running with `--ui` without building first will exit with an error message.

3. **Optional – MOT evaluation dependencies**:
   ```bash
   pip install motmetrics opencv-python
   ```

4. **Verify** the app parses and shows help:
   ```bash
   drone-follow --help
   ```

## Instructions

1. Run PX4 SITL with Gazebo and x500 drone with mono camera:
   ```bash
   make px4_sitl gz_x500_mono_cam
   ```
   Run `make` from your PX4-Autopilot directory.

2. Run QGroundControl:
   ```bash
   ./QGroundControl-x86_64.AppImage
   ```

3. Run the drone follow application:
   ```bash
   drone-follow --input udp://0.0.0.0:5600 --target-bbox-height 0.5
   ```

## Easy World Loading

Use `--world` to automatically load a custom Gazebo world (SDF) before PX4 starts. This symlinks your world as PX4's `default.sdf`, so `make px4_sitl gz_x500_mono_cam` picks it up with no extra configuration. The original world is restored after the drone connects.

```bash
drone-follow --input udp://0.0.0.0:5600 --target-distance 8 \
    --px4-path ~/PX4-Autopilot --world 2_person_world
```

`--px4-path` is required when using `--world`.

### World resolution

| Argument | Resolves to |
|---|---|
| `2_person_world` (bare name) | `drone_follow/sdf_examples/2_person_world.sdf` |
| `my_world.sdf` (relative with .sdf) | `drone_follow/sdf_examples/my_world.sdf`, then CWD |
| `/absolute/path/to/world.sdf` | Used as-is |

### Bundled worlds

- `2_person_world` – Two people standing in the scene.
- `2_persons_diagonal` – Two people placed diagonally.
- `random_walk` – A person walking a random path.

### Key options (match the app)

- **Target size:** Use either `--target-bbox-height <0–1>` (target height in the image) or `--target-distance <metres>` (real-world distance). They are mutually exclusive. `--target-distance` requires `--fixed-altitude`.
- **`--fixed-altitude`** (default) – Hold altitude constant. Use `--no-fixed-altitude` for vertical following.
- **`--takeoff-landing`** – Enable automatic arm/takeoff/land. Without this flag (default), the app waits for the pilot to switch to OFFBOARD mode via GCS or RC.
- **`--yaw-only`** – Only yaw to center the person; no forward/backward or altitude movement (see Yaw-Only Mode below).
- **`--tracker {byte,fast}`** – Select the tracking algorithm (see Tracker Selection below).
- **`--ui`** – Enable web UI with live video and click-to-follow (requires UI built; see Optional – Web UI above).
- **Input/connection:** Pipeline input is set with `--input` (e.g. `udp://0.0.0.0:5600`, `rpi`, `usb`). MAVLink connection defaults to `udpin://0.0.0.0:14540`; override with `--connection` or use `--serial` for a serial link.

### Running drone_follow on a different PC than the simulation

You can run the PX4 SITL simulation (e.g. on a Raspberry Pi) on one machine and the drone_follow application on another. Video and MAVLink are sent over UDP to the host running drone_follow.

**1. On the simulation machine (e.g. RPI with PX4 SITL)**

Set the video/MAVLink host to the IP of the PC that will run drone_follow, then start PX4:

```bash
PX4_VIDEO_HOST_IP=<MAVLINK_HOST_IP> make px4_sitl gz_x500_mono_cam
```

Replace `<MAVLINK_HOST_IP>` with the IP address of the machine where you run `drone-follow`.

**2. (Optional) Persistent MAVLink config on PX4**

You can add this to `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` so MAVLink targets the drone_follow host:

```bash
mavlink start -t <MAVLINK_HOST_IP> -o 14560 -r 100000
```

**3. On the drone_follow machine**

Run the app so it listens for video on UDP 5600 and MAVLink on UDP 14560:

```bash
drone-follow --input udp://0.0.0.0:5600 --connection udp://0.0.0.0:14560 --target-bbox-height 0.5
```

Ensure the video bridge (or PX4 video stream) and MAVLink are sent to this machine's IP; use the same ports (5600 for video, 14560 for MAVLink) on the simulation side.

## Tracker Selection

The app supports two tracking algorithms, selectable with `--tracker`:

| Tracker | Flag | Description | Speed |
|---|---|---|---|
| **ByteTracker** | `--tracker byte` (default) | Lightweight two-stage tracker with high/low confidence matching | ~435 fps |
| **FastTracker** | `--tracker fast` | Occlusion-aware tracker with velocity damping, ROI constraints, and IoU suppression | ~184 fps |

FastTracker is vendored from [Hamidreza-Hashempoor/FastTracker](https://github.com/Hamidreza-Hashempoor/FastTracker) with heavy dependencies (torch, lap, cython_bbox) replaced by scipy/numpy equivalents. No extra installation needed.

```bash
# Run with ByteTracker (default):
drone-follow --input usb --serial --ui

# Run with FastTracker:
drone-follow --input usb --serial --ui --tracker fast
```

## MOT Evaluation Tool

A standalone tool for benchmarking trackers on MOT Challenge datasets (e.g. [MOT17](https://motchallenge.net/data/MOT17/)). No Hailo hardware needed — runs the tracker on pre-computed detections.

### Quick start

Point it at a MOT sequence directory:

```bash
python -m drone_follow.tools.mot_eval /path/to/MOT17/train/MOT17-04-SDP
```

This auto-detects `det/det.txt`, `gt/gt.txt`, `img1/`, and `seqinfo.ini` from the directory and produces:

- **MOT metrics table** (MOTA, IDF1, HOTA, FP, FN, IDs) + performance stats (FPS, init time)
- **Tracker video** (`MOT17-04-SDP.mp4`) with colored bounding boxes and track IDs
- **Tracking results** (`results_MOT17-04-SDP.txt`) in MOT Challenge format

### Comparing trackers

```bash
# ByteTracker:
python -m drone_follow.tools.mot_eval /path/to/MOT17-04-SDP --tracker byte --visualize byte.mp4

# FastTracker:
python -m drone_follow.tools.mot_eval /path/to/MOT17-04-SDP --tracker fast --visualize fast.mp4
```

### Pipeline visualization (requires Hailo)

Run the Hailo detection pipeline on the image sequence and visualize the tracked output:

```bash
python -m drone_follow.tools.mot_eval /path/to/MOT17-04-SDP \
    --visualize-pipeline hailo_output.mp4
```

### Manual mode

Override auto-detection with explicit paths:

```bash
python -m drone_follow.tools.mot_eval \
    --detections /path/to/det.txt \
    --gt /path/to/gt.txt \
    --images-dir /path/to/img1 \
    --tracker fast \
    --frame-rate 30 \
    -o results.txt \
    --visualize output.mp4
```

### Metrics

- **MOTA / IDF1** — Requires `pip install motmetrics`
- **HOTA** — Always available (native implementation, no extra deps)
- **Performance** — FPS, init time, average update time

## HTTP Control Server

The application includes an HTTP server (running on port 8080 by default) that provides status information and target selection capabilities.

### Default Behavior

By default (no target set), the drone follows the person with the **largest bounding box** in the frame.

### API Endpoints

- `GET /status` - Get current tracking status
  - Returns: `{"following_id": <id or null>, "last_seen": <timestamp or null>, "available_ids": [list of IDs]}`

- `POST /follow/<detection_id>` - Start following a specific tracked person
  - Returns 200: `{"status": "success", "following_id": <id>}` if the ID is found in the current frame
  - Returns 404: `{"status": "error", "message": "...", "available_ids": [...]}` if the ID is not in the current frame

- `POST /follow/clear` - Clear target and return to following the largest person
  - Returns: `{"status": "success", "following_id": null, "message": "Cleared target, now following largest person"}`

### Basic Usage

The server is always available and shows status:

```bash
# Check current status
curl http://localhost:8080/status
```

### Target Selection with Tracking IDs

Tracking IDs are available by default, so you can select a specific person to follow:

1. Run the app:
   ```bash
   drone-follow --input udp://0.0.0.0:5600 --target-bbox-height 0.5
   ```

2. Check which people are visible:
   ```bash
   curl http://localhost:8080/status
   # Returns: {"following_id": null, "last_seen": null, "available_ids": [1, 3, 5]}
   ```

3. Select a specific person to follow:
   ```bash
   # Follow the person with tracking ID 3
   curl -X POST http://localhost:8080/follow/3
   # Returns: {"status": "success", "following_id": 3}

   # If ID not found:
   # Returns: {"status": "error", "message": "Detection ID 42 not found in current frame", "available_ids": [1, 3, 5]}
   ```

4. Clear target and return to following the largest person:
   ```bash
   curl -X POST http://localhost:8080/follow/clear
   ```

### Configuration

- The follow server is always running. Change its port with `--follow-server-port <port>` (default: 8080).

## Yaw-Only Mode

Use `--yaw-only` to disable all forward/backward and altitude movement. The drone will only rotate to keep the person centered in the frame. This is also available as a toggle in the web UI.

Note: `--forward-gain 0` now also fully disables forward/backward motion (including the safety backward retreat).

## Web UI Controls

The web UI (`--ui`, served on port 5001) provides live video, detection overlays, and real-time tuning of the controller. All changes take effect immediately.

### Status Bar

- **Following indicator** — Shows which person is being tracked (by ID) or "Auto (largest person)" if no specific target is selected.
- **Velocity readout** — Current mode (TRACK/SEARCH/ORBIT) and commanded velocities: forward, lateral, down, and yaw.
- **Record** — Start/stop recording the video stream.
- **Clear Target** — Stop following a specific person and revert to auto (largest person).

### Controller Parameters

**Operational (top of panel):**

| Control | Range | Default | Description |
|---|---|---|---|
| **Target Size** | 5% – 100% | 30% | Desired person bounding box height as percentage of frame. The drone approaches if the person is smaller than this, retreats if larger. Increase to keep the person closer, decrease for more distance. |
| **Target Alt** | 1 – 20 m | 3.0 | Target altitude. Used as initial takeoff height (with `--takeoff-landing`) and as a go-to altitude when changed mid-flight. |
| **Yaw Only** | ON/OFF | OFF | When ON, disables all forward/backward and altitude movement. The drone only rotates to keep the person centered. Useful for testing or when only rotation is safe (e.g. simulation with USB camera). |
| **Mode: FOLLOW / ORBIT** | — | FOLLOW | FOLLOW: drone faces and approaches/retreats from the person. ORBIT: drone circles around the person while maintaining yaw lock, adding lateral velocity. |
| **Orbit Speed** | 0.2 – 3.0 m/s | 1.0 | Lateral speed during orbit mode. Only visible when Mode is ORBIT. |
| **Direction: CW / CCW** | — | CW | Orbit direction: clockwise or counter-clockwise. Only visible when Mode is ORBIT. |

**Tuning (below operational controls):**

| Control | Range | Default | Description |
|---|---|---|---|
| **KP Yaw** | 0 – 10 | 5.0 | Yaw proportional gain. Higher = faster rotation to center the person. Uses sqrt response to avoid oscillation. |
| **KP Forward** | 0 – 10 | 3.0 | Forward/approach proportional gain. Controls how aggressively the drone moves toward a distant person. Set to 0 to disable forward/backward movement entirely. |
| **KP Backward** | 0 – 10 | 5.0 | Backward/retreat proportional gain. Controls retreat speed when too close. Higher than KP Forward by default for safety. |
| **Yaw Smooth** | ON/OFF | ON | Low-pass filter on yaw commands. Reduces jitter but adds slight lag. |
| **Yaw Alpha** | 0.05 – 1.0 | 0.3 | EMA smoothing factor for yaw. Lower = smoother (more lag), higher = more responsive. Only active when Yaw Smooth is ON. |
| **Fwd Smooth** | ON/OFF | ON | EMA smoothing on forward velocity. Reduces sudden speed changes. |
| **Fwd Alpha** | 0.05 – 1.0 | 0.1 | EMA factor for forward smoothing. Lower = smoother, higher = more responsive. |

### How Target Size Works

The controller compares the detected person's bounding box height (0–1, fraction of frame) against the Target Size value:
- **Person smaller than target** → drone flies forward (approach)
- **Person larger than target** → drone flies backward (retreat)
- **Person matches target (within dead zone)** → no forward/backward movement

A 5% dead zone (relative to target size) prevents oscillation around the setpoint.

## Architecture

```
drone_follow/
  follow_api/          Pure domain logic (no HW deps) — types, config, controller math, shared state
  drone_api/           MAVSDK flight controller adapter — offboard velocity commands, takeoff/landing
  pipeline_adapter/    Hailo/GStreamer pipeline + tracker — detection, tracking, target selection
    byte_tracker.py    ByteTracker implementation + adapter
    fast_tracker.py    FastTracker adapter (wraps vendored _fasttracker/)
    _fasttracker/      Vendored FastTracker source (no torch/lap/cython_bbox deps)
    tracker_factory.py Tracker factory — create_tracker("byte"|"fast", ...)
    tracker.py         Tracker protocol, TrackedObject, MetricsTracker
  servers/             HTTP servers — follow target REST API (port 8080), web UI with MJPEG (port 5001)
  tools/               Standalone utilities
    mot_eval.py        MOT evaluation tool — benchmark trackers on MOT Challenge datasets
  sim/                 Simulation helpers (PX4 world loading)
  ui/                  React web dashboard (built separately with npm)
  drone_follow_app.py  Composition root and CLI entrypoint
```

**Data flow:** Camera → GStreamer → Hailo-8L inference → Tracker (in callback) → `SharedDetectionState` → Control loop (10 Hz) → MAVSDK offboard velocity command.

The `follow_api` package has zero external dependencies, making the controller logic easy to test without hardware.

## JSON Config Files

Instead of passing many CLI flags, you can store controller settings in a JSON file and load them with `--config`. CLI flags still override JSON values.

### Usage

```bash
# Save current defaults to a file, then edit it
drone-follow --save-config my_config.json

# Run with a config file
drone-follow --config configs/simulation.json --input usb --serial --ui

# CLI flags override JSON values
drone-follow --config configs/outdoor_follow.json --target-altitude 8 --input rpi --serial --ui
```

### Bundled Presets

The `configs/` directory includes ready-to-use presets:

| Preset | Mode | Description |
|---|---|---|
| `simulation.json` | Yaw-only | Safe for SITL testing with USB camera. Forward/backward disabled, longer search timeout, debug logging. |
| `simulation_follow.json` | Full follow (sim) | SITL with forward/backward enabled. Reduced speeds and gains for safe indoor/sim testing, debug logging. |
| `outdoor_follow.json` | Full follow | Real drone outdoor flight. Approaches/retreats to maintain target size, 5m altitude, conservative speeds. |
| `outdoor_yaw_only.json` | Yaw-only (outdoor) | Real drone, rotation only. Safe for first outdoor tests — no translation, 5m altitude. |
| `outdoor_orbit.json` | Orbit | Cinematic circling at 1.5 m/s, 5m altitude. Keeps target at 20% frame height for wider shots. |

### Examples

**Simulation** (USB camera + PX4 SITL via Docker):
```bash
drone-follow --config configs/simulation.json \
    --input usb --connection tcp://127.0.0.1:5760 --ui
```

**Real drone — follow mode** (RPi camera + Cube Orange+ over USB serial):
```bash
drone-follow --config configs/outdoor_follow.json \
    --input rpi --serial --ui
```

**Real drone — orbit mode**:
```bash
drone-follow --config configs/outdoor_orbit.json \
    --input rpi --serial --ui
```

### Creating Your Own Config

Only include the fields you want to change — anything omitted uses the built-in defaults. For a complete list of fields, run:

```bash
drone-follow --save-config full_defaults.json
```

## Key Configuration Parameters

| Parameter | Default | Description |
|---|---|---|
| `--tracker` | `byte` | Tracking algorithm: `byte` (ByteTracker) or `fast` (FastTracker) |
| `--target-bbox-height` | `0.3` | Desired person size in frame (0–1) |
| `--target-distance` | — | Desired horizontal distance in metres (requires `--fixed-altitude`) |
| `--forward-gain` | `3.0` | Proportional gain for forward/backward |
| `--backward-gain` | `5.0` | Proportional gain for backward (retreat) |
| `--yaw-gain` | `5` | Proportional gain for yaw |
| `--max-forward` | `2.0` | Max forward speed (m/s) |
| `--max-backward` | `3.0` | Max backward speed (m/s) |
| `--search-enter-delay` | `2.0` | Seconds without detection before search starts |
| `--search-timeout` | `60` | Seconds of search before auto-landing |
| `--smooth-forward` / `--no-smooth-forward` | on | Enable/disable forward velocity smoothing |

Run `drone-follow --help` for the full list.
