# CLAUDE.md — Project Context for drone-follow

## Project Overview

A Hailo-based drone-follow application that uses an AI pipeline (GStreamer + Hailo NPU) for person detection and MAVSDK for PX4 drone control. Runs on a Raspberry Pi 5 with Hailo-8L accelerator mounted on a drone with a Cube Orange+ flight controller.

## Architecture

- **`drone_follow/follow_api/`** — Pure domain logic (follow controller, geometry)
- **`drone_follow/drone_api/mavsdk_drone.py`** — MAVSDK adapter; CLI args, connection, control loop
- **`drone_follow/pipeline_adapter/`** — Hailo/GStreamer detection pipeline
- **`drone_follow/drone_follow_app.py`** — Main entry point (`main()`), wires everything together

## Key CLI Flags

- `--serial [DEVICE]` — Connect via USB serial (default: `/dev/ttyACM0`); overrides `--connection`
- `--serial-baud RATE` — Baud rate (default: 57600)
- `--connection URL` — MAVSDK connection string (default: `udpin://0.0.0.0:14540` for simulation)
- `--takeoff-landing` — Enable auto arm/takeoff/land (default: off — drone must already be airborne)
- `--fixed-altitude` / `--no-fixed-altitude` — Keep altitude fixed (default: on)

## Drone Connection

### USB Serial (real hardware)
The Cube Orange+ connects via USB as `/dev/ttyACM0`. Using `--serial` builds the connection string `serial:///dev/ttyACM0:57600` and passes it to MAVSDK.

### UDP (simulation)
Without `--serial`, defaults to `udpin://0.0.0.0:14540` for SITL/Gazebo.

## PX4 Offboard Mode

### How it works in this app
By default (no `--takeoff-landing`), the app streams zero setpoints and waits for the pilot to switch to OFFBOARD mode via GCS or RC. The app never commands the mode switch itself. Use `--takeoff-landing` to enable auto arm/takeoff/land.

### Required PX4 Parameters (set via QGroundControl)
- `COM_RC_IN_MODE = 4` — Allow flight without RC transmitter
- `COM_RCL_EXCEPT` bit 2 set — Ignore RC loss in offboard mode
- `COM_OF_LOSS_T` — Offboard signal loss timeout (default ~1 s)
- `COM_OBL_RC_ACT` — Failsafe action on offboard loss

### PX4 Documentation
- Main docs: https://docs.px4.io/main/en/
- Offboard mode: https://docs.px4.io/main/en/flight_modes/offboard.html

## Running

```bash
# Real drone over USB (RPi):
./run_drone.sh

# Dev machine with USB camera + flight controller:
source setup_env.sh
drone-follow --input usb --serial --ui

# Simulation (UDP, no --serial):
source setup_env.sh
drone-follow --input rpi --connection udpin://0.0.0.0:14540 --takeoff-landing
```

## Virtual Environment

All dependencies are in `venv/`. Always `source setup_env.sh` before running.

## Development Machine Setup (x86_64)

This repo can also run on an x86_64 development machine instead of the RPi target. Differences from RPi:

- **Camera:** Use `--input usb` instead of `--input rpi` (auto-detects USB webcam).
- **Hailo:** Requires a Hailo-8 PCIe card with `hailort` and `hailo-tappas-core` system deb packages installed.
- **Flight controller:** The Cube Orange+ connects via USB serial at `/dev/ttyACM0`, same as on the RPi.
- **Simulation:** PX4 SITL + Gazebo Garden runs in Docker via the sibling `hailo_drone_control` project (see below).

### Installation

Prerequisites:
- Ubuntu 22.04 with Python 3.10+
- `hailort` and `hailo-tappas-core` deb packages installed (match your Hailo device)
- `hailo-apps-internal` repo cloned (must be on the `feature/combined-fixes-and-features` branch)
- Hailo Python wheel files for your platform (`.whl` for hailort and tappas-core)
- Node.js / npm (optional, for the web UI)

```bash
# 1. Create venv (--system-site-packages needed for GStreamer/gi bindings)
python3 -m venv --system-site-packages venv
source venv/bin/activate
pip install --upgrade pip

# 2. Install Hailo Python wheels (adjust paths and versions to match your setup)
pip install /path/to/hailort-<version>-<cpXX>-<cpXX>-linux_x86_64.whl
pip install /path/to/hailo_tappas_core_python_binding-<version>-py3-none-any.whl

# 3. Install hailo-apps (provides the `hailo` Python module with compiled .so bindings)
cd /path/to/hailo-apps-internal
git checkout feature/combined-fixes-and-features
pip install -e .
cd -

# 4. Install drone-follow
pip install -e .

# 5. Build the web UI (optional, for --ui flag)
cd drone_follow/ui && npm install && npm run build && cd -
```

Verify: `drone-follow --help`

### Running on a dev machine

```bash
source setup_env.sh

# With USB camera + real flight controller over serial:
drone-follow --input usb --serial --ui

# With USB camera + PX4 SITL via Docker (see Simulation section):
drone-follow --input usb --connection tcp://127.0.0.1:5760 \
    --yaw-only --ui
```

### Simulation (Docker-based PX4 SITL)

PX4 SITL + Gazebo Garden runs in Docker via the `hailo_drone_control` sibling project. The Docker stack uses docker-compose profiles and includes these containers:

| Container | Role | Network IP |
|-----------|------|------------|
| `px4-sitl` | PX4 SITL + Gazebo + mavlink-routerd | 172.28.0.10 |
| `qgroundcontrol` | Ground Control Station GUI | 172.28.0.20 |
| `px4-control` | Python control scripts runner | 172.28.0.30 |
| `px4-test-runner` | Integration test runner | 172.28.0.100 |

**Exposed host ports** (from `px4-sitl`):
- `5760/tcp` — MAVLink TCP server (recommended for drone-follow)
- `14540/udp` — MAVSDK API for external clients
- `14551/udp` — QGC for external GCS

The `px4-sitl` container runs `mavlink-routerd` internally, which routes MAVLink telemetry to all other containers and external clients. The `full` profile starts all containers.

```bash
# First time setup:
cd /path/to/hailo_drone_control
cp env.example .env

# Start the simulator (full profile — SITL + QGC + control + test-runner):
xhost +local:docker
./scripts/px4ctl.sh start

# Check status:
./scripts/px4ctl.sh status

# Stop the simulator:
./scripts/px4ctl.sh stop
```

Once the simulator is running, connect drone-follow using:
```bash
drone-follow --input usb --connection tcp://127.0.0.1:5760 \
    --yaw-only --ui
```

Then arm, take off, and switch to OFFBOARD mode from QGroundControl.

Note: `--input usb` uses the local USB webcam since Gazebo's camera feed is not bridged to the host. Because the USB camera sees the real world (not the sim), forward/altitude commands based on bbox size don't make sense — the drone will fly into the ground or backwards at max speed. Always use `--yaw-only` (rotation only) or `--fixed-altitude` in simulation. `--target-distance` should NOT be used with a USB camera in sim as the altitude/distance mismatch causes unsafe commands.
