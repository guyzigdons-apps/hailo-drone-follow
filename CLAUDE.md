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
- `--target-altitude M` — Target altitude in metres (default: 3.0). Also used as takeoff height with `--takeoff-landing`. Adjustable mid-flight via UI.
- `--target-bbox-height` — Desired person size in frame 0–1 (default: 0.3). Adjustable mid-flight via UI "Target Size" slider.
- `--yaw-only` / `--no-yaw-only` — Yaw only mode (default: on). Use `--no-yaw-only` for full follow with forward/backward movement.

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

# Simulation (see Simulation section for full setup):
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
```

## Virtual Environment

All dependencies are in `venv/`. Always `source setup_env.sh` before running.

## Development Machine Setup (x86_64)

This repo can also run on an x86_64 development machine instead of the RPi target. Differences from RPi:

- **Camera:** Use `--input usb` instead of `--input rpi` (auto-detects USB webcam).
- **Hailo:** Requires a Hailo-8 PCIe card with `hailort` and `hailo-tappas-core` system deb packages installed.
- **Flight controller:** The Cube Orange+ connects via USB serial at `/dev/ttyACM0`, same as on the RPi.
- **Simulation:** Bundled PX4 SITL + Gazebo Garden (see Simulation section below).

### Installation

Prerequisites:
- Ubuntu 22.04 with Python 3.10+
- `hailort` and `hailo-tappas-core` deb packages installed (match your Hailo device)
- Node.js / npm (optional, for the web UI)

```bash
# Run the installer (clones hailo-apps, creates venv, installs everything, builds UI)
./install.sh

# Options:
#   --hailo-apps-dir DIR   Use existing hailo-apps checkout (default: ./hailo-apps)
#   --skip-hailo-apps      Skip hailo-apps clone and system deps (if already set up)
#   --skip-ui              Skip UI npm install and build
#   --skip-python          Skip Python dependency installation
```

Verify: `source setup_env.sh && drone-follow --help`

### Running on a dev machine

```bash
source setup_env.sh

# With USB camera + real flight controller over serial:
drone-follow --input usb --serial --ui

# With Gazebo camera + PX4 SITL (see Simulation section):
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui

# With USB camera + PX4 SITL (yaw only — forward commands unsafe with real webcam):
drone-follow --input usb --yaw-only --ui
```

### Simulation (Bundled PX4 SITL)

PX4 SITL + Gazebo Garden runs natively using a bundled PX4-Autopilot git submodule (v1.14.0) at `sim/PX4-Autopilot`. A video bridge pipes the Gazebo camera feed to the Hailo pipeline via UDP.

**Prerequisites:** Gazebo Garden (`gz-garden`), `python3-gz-transport13`, `python3-gz-msgs10`

```bash
# One-time setup (inits submodule + builds PX4 — takes 10-20 min first time):
sim/setup_sim.sh

# Terminal 1 — Start PX4 SITL + Gazebo + video bridge:
sim/start_sim.sh --bridge --world 2_person_world

# Terminal 2 — Run drone-follow:
source setup_env.sh
drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui
```

**Key ports:**
- `14540/udp` — MAVLink (PX4 MAVSDK API, default `--connection`)
- `5600/udp` — Video feed from Gazebo (via video bridge)

**Bundled worlds** in `drone_follow/sdf_examples/`: `2_person_world`, `2_persons_diagonal`, `random_walk`
Pass `--world NAME` to `start_sim.sh` to load a custom world (uses PX4's native `PX4_GZ_WORLD` env var).

**USB camera with sim:** If using `--input usb` instead of the Gazebo camera, always add `--yaw-only` — forward/altitude commands based on bbox size are unsafe because the webcam sees the real world, not the sim.
