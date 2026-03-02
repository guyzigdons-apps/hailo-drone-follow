# Drone Follow Demo

## Installation

### Prerequisites

- **Hailo application infrastructure** (hailort, hailo-tappas-core, Python bindings). See the [main installation guide](../../../../doc/user_guide/installation.md) for your platform.
- **Python 3.8+** with the project virtual environment activated (e.g. `source setup_env.sh`).
- **MAVSDK** for drone connection and offboard control (installed via pip in the project venv).
- For **simulation**: PX4 SITL and Gazebo (e.g. `make px4_sitl gz_x500_mono_cam` from PX4-Autopilot).
- For **real drone**: PX4 (or compatible) firmware and a connection (e.g. USB or UDP).

### Steps

1. **Install hailo-apps** (from the repository root):
   ```bash
   cd /path/to/hailo-apps
   sudo ./install.sh
   source setup_env.sh
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r hailo_apps/python/pipeline_apps/drone_follow/requirements.txt
   ```

3. **Optional – Web UI** (for `--ui` with live video and click-to-follow):

   Requires **Node.js / npm** to be installed.
   ```bash
   cd hailo_apps/python/pipeline_apps/drone_follow/ui
   npm install
   npm run build
   cd -
   ```
   The built UI is served from `ui/build` when you run with `--ui`.
   Running with `--ui` without building first will exit with an error message.

4. **Verify** the app parses and shows help:
   ```bash
   python drone_follow_app.py --help
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
   python tools/video_bridge.py
   ```

4. Run the drone follow application:
   ```bash
   python drone_follow_app.py --input udp://0.0.0.0:5600 --target-bbox-height 0.5
   ```

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

Replace `<MAVLINK_HOST_IP>` with the IP address of the machine where you run `drone_follow_app.py`.

**2. (Optional) Persistent MAVLink config on PX4**

You can add this to `ROMFS/px4fmu_common/init.d-posix/px4-rc.mavlink` so MAVLink targets the drone_follow host:

```bash
mavlink start -t <MAVLINK_HOST_IP> -o 14560 -r 100000
```

**3. On the drone_follow machine**

Run the app so it listens for video on UDP 5600 and MAVLink on UDP 14560:

```bash
python drone_follow_app.py --input udp://0.0.0.0:5600 --connection udp://0.0.0.0:14560 --target-bbox-height 0.5
```

Ensure the video bridge (or PX4 video stream) and MAVLink are sent to this machine’s IP; use the same ports (5600 for video, 14560 for MAVLink) on the simulation side.

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
   python drone_follow_app.py --input udp://0.0.0.0:5600 --target-bbox-height 0.5
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
