# Drone Follow Demo

## Installation

### Prerequisites

- **Hailo application infrastructure** (hailort, hailo-tappas-core, Python bindings).
- **Python 3.9+** with the project virtual environment activated (`source setup_env.sh`).
- **MAVSDK** for drone connection and offboard control (installed via pip in the project venv).
- For **simulation**: PX4 SITL and Gazebo (e.g. `make px4_sitl gz_x500_mono_cam` from PX4-Autopilot). See [PX4 Ubuntu Dev Environment Setup](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu) for toolchain installation.
- For **real drone**: PX4 (or compatible) firmware and a connection (e.g. USB or UDP).

### Steps

1. **Create a virtual environment and install dependencies**:
   ```bash
   python -m venv venv --system-site-packages
   ./install.sh
   source setup_env.sh
   pip install -e .
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

3. **Verify** the app parses and shows help:
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

3. Run the video bridge:
   ```bash
   python drone_follow/tools/video_bridge.py
   ```

4. Run the drone follow application:
   ```bash
   drone-follow --input udp://0.0.0.0:5600 --target-bbox-height 0.5
   ```

## Easy World Loading

Use `--world` to automatically load a custom Gazebo world (SDF) before PX4 starts. This symlinks your world as PX4's `default.sdf`, so `make px4_sitl gz_x500_mono_cam` picks it up with no extra configuration. The original world is restored after the drone connects.

```bash
drone-follow --input udp://0.0.0.0:5600 --target-distance 8 --fixed-altitude \
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
- **`--fixed-altitude`** – Hold altitude constant; use with `--target-distance` for distance-based following.
- **`--no-takeoff-landing`** – Do not take off or land; assume the drone is already in offboard mode (e.g. when you arm and switch to offboard yourself).
- **`--yaw-only`** – Only yaw to center the person; no forward/backward or altitude movement (see Yaw-Only Mode below).
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

## Architecture

```
drone_follow/
  follow_api/          Pure domain logic (no HW deps) — types, config, controller math, shared state
  drone_api/           MAVSDK flight controller adapter — offboard velocity commands, takeoff/landing
  pipeline_adapter/    Hailo/GStreamer pipeline + ByteTracker — detection, tracking, target selection
  servers/             HTTP servers — follow target REST API (port 8080), web UI with MJPEG (port 5001)
  tools/               Standalone utilities (Gazebo video bridge)
  sim/                 Simulation helpers (PX4 world loading)
  ui/                  React web dashboard (built separately with npm)
  drone_follow_app.py  Composition root and CLI entrypoint
```

**Data flow:** Camera → GStreamer → Hailo-8L inference → ByteTracker (in callback) → `SharedDetectionState` → Control loop (10 Hz) → MAVSDK offboard velocity command.

The `follow_api` package has zero external dependencies, making the controller logic easy to test without hardware.

## Key Configuration Parameters

| Parameter | Default | Description |
|---|---|---|
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
