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
- `--no-takeoff-landing` — Skip arming/takeoff; wait for OFFBOARD mode from GCS instead

## Drone Connection

### USB Serial (real hardware)
The Cube Orange+ connects via USB as `/dev/ttyACM0`. Using `--serial` builds the connection string `serial:///dev/ttyACM0:57600` and passes it to MAVSDK.

### UDP (simulation)
Without `--serial`, defaults to `udpin://0.0.0.0:14540` for SITL/Gazebo.

## PX4 Offboard Mode

### How it works in this app
With `--no-takeoff-landing`, the app waits for the pilot/GCS to switch to OFFBOARD mode, then starts sending velocity setpoints. The drone must already be armed and airborne.

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
# Real drone over USB:
./run_drone.sh

# Simulation (UDP, no --serial):
source venv/bin/activate
drone-follow --input rpi --connection udpin://0.0.0.0:14540
```

## Virtual Environment

All dependencies are in `venv/`. Always `source venv/bin/activate` before running.
