# Drone Follow Demo

## Installation

### Prerequisites

- **Hailo application infrastructure** (hailort, hailo-tappas-core, Python bindings).
- **Python 3.9+** with the project virtual environment activated (`source setup_env.sh`).
- **MAVSDK** for drone connection and offboard control (installed via pip in the project venv).
- For **simulation**: Gazebo Garden (`gz-garden`), `python3-gz-transport13`, `python3-gz-msgs10`. The bundled PX4 SITL is set up automatically via `sim/setup_sim.sh`.
- For **real drone**: PX4 (or compatible) firmware and a connection (e.g. USB serial or UDP).

### Steps

1. **Run the installer** (creates venv, installs hailo-apps + Python deps, builds UI):
   ```bash
   ./install.sh
   ```

2. **Verify** the app parses and shows help:
   ```bash
   source setup_env.sh
   drone-follow --help
   ```

## Simulation (Bundled PX4 SITL)

PX4 SITL + Gazebo Garden runs natively using a bundled PX4-Autopilot git submodule (v1.14.0) at `sim/PX4-Autopilot`. A video bridge pipes the Gazebo camera feed to the Hailo pipeline via UDP.

```bash
# One-time setup (inits submodule + builds PX4 — takes 10-20 min first time):
sim/setup_sim.sh

# Terminal 1 — Start PX4 SITL + Gazebo + video bridge:
sim/start_sim.sh --bridge --world 2_person_world

# Terminal 2 — Run drone-follow:
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
```

**Key ports:** `14540/udp` (MAVLink), `5600/udp` (video from Gazebo via bridge)

### Bundled worlds

Located in `drone_follow/sdf_examples/`:

- `2_person_world` – Two people walking in the scene.
- `2_persons_diagonal` – Two people walking diagonally.
- `random_walk` – A person walking a random path.

## Running on Real Hardware

```bash
source setup_env.sh

# RPi with camera + Cube Orange+ over USB serial:
drone-follow --input rpi --serial --ui

# Dev machine with USB camera + flight controller:
drone-follow --input usb --serial --ui
```

### Key options

- **`--target-bbox-height <0–1>`** – Desired person size in frame (default: 0.3). Adjustable mid-flight via UI.
- **`--target-altitude <metres>`** – Target altitude (default: 3.0). Also used as takeoff height with `--takeoff-landing`.
- **`--takeoff-landing`** – Enable automatic arm/takeoff/land. Without this flag (default), the app waits for the pilot to switch to OFFBOARD mode via GCS or RC.
- **`--yaw-only`** (default: on) – Only yaw to center the person; no forward/backward movement. Use `--no-yaw-only` for full follow (see Yaw-Only Mode below).
- **`--ui`** – Enable web UI with live video and click-to-follow.
- **Input/connection:** Pipeline input is set with `--input` (e.g. `udp://0.0.0.0:5600`, `rpi`, `usb`). MAVLink connection defaults to `udpin://0.0.0.0:14540`; override with `--connection` or use `--serial` for a serial link.

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
   drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
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

Yaw-only mode is **on by default** (`--yaw-only`). The drone only rotates to keep the person centered in the frame — no forward/backward or altitude movement. Use `--no-yaw-only` for full follow. This is also available as a toggle in the web UI.

Note: `--forward-gain 0` also fully disables forward/backward motion (including the safety backward retreat).

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
| **Yaw Only** | ON/OFF | ON | When ON, disables all forward/backward and altitude movement. The drone only rotates to keep the person centered. Use `--no-yaw-only` for full follow. |
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
  pipeline_adapter/    Hailo/GStreamer pipeline + ByteTracker — detection, tracking, target selection
  servers/             HTTP servers — follow target REST API (port 8080), web UI with MJPEG (port 5001)
  sdf_examples/        Gazebo world SDF files for simulation
  ui/                  React web dashboard (built separately with npm)
  drone_follow_app.py  Composition root and CLI entrypoint
sim/
  PX4-Autopilot/       PX4 git submodule (v1.14.0)
  bridge/              Gazebo camera → UDP video bridge
  patches/             PX4 model patches (camera sensor)
  setup_sim.sh         One-time sim setup (build PX4)
  start_sim.sh         Launch PX4 SITL + Gazebo + bridge
```

**Data flow:** Camera → GStreamer → Hailo-8L inference → ByteTracker (in callback) → `SharedDetectionState` → Control loop (10 Hz) → MAVSDK offboard velocity command.

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

**Simulation** (Gazebo camera + bundled PX4 SITL):
```bash
drone-follow --config configs/simulation.json \
    --input udp://0.0.0.0:5600 --takeoff-landing --ui
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
| `--target-bbox-height` | `0.3` | Desired person size in frame (0–1). Adjustable mid-flight via UI. |
| `--target-altitude` | `3.0` | Target altitude in metres. Also used as takeoff height with `--takeoff-landing`. |
| `--forward-gain` | `3.0` | Proportional gain for forward/backward |
| `--backward-gain` | `5.0` | Proportional gain for backward (retreat) |
| `--yaw-gain` | `5` | Proportional gain for yaw |
| `--max-forward` | `2.0` | Max forward speed (m/s) |
| `--max-backward` | `3.0` | Max backward speed (m/s) |
| `--search-enter-delay` | `2.0` | Seconds without detection before search starts |
| `--search-timeout` | `60` | Seconds of search before auto-landing |
| `--smooth-forward` / `--no-smooth-forward` | on | Enable/disable forward velocity smoothing |

Run `drone-follow --help` for the full list.
