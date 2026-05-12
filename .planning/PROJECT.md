# drone-follow

## What This Is

A Hailo-NPU vision pipeline that tracks a person in real time and drives a robot to follow them. Today the robot is a quadrotor (Raspberry Pi 5 + Hailo-8L + Cube Orange+ via MAVSDK/PX4), with operator overrides surfaced through a web UI and the OpenHD ground link. Designed for outdoor person-following missions where the operator may be on-foot with a phone.

## Core Value

The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.

## Requirements

### Validated

<!-- Shipped and confirmed valuable. -->

- ✓ **VIS-01**: Hailo-NPU person detection at video rate (RPi/USB/UDP/SHM inputs) — v1.0
- ✓ **VIS-02**: ByteTracker multi-person tracking with stable IDs — v1.0
- ✓ **VIS-03**: ReID-based re-acquisition after occlusion (gallery + drift protection + raw-detection fallback) — v1.0
- ✓ **CTRL-01**: Pure follow-controller (`compute_velocity_command`) returns `VelocityCommand(forward, down, yawspeed)` independent of robot stack — v1.0
- ✓ **CTRL-02**: Yaw-centring P loop with deadband and signed sqrt response — v1.0
- ✓ **CTRL-03**: Bbox-distance forward controller (asymmetric gains, scale-invariant relative-error) — v1.0
- ✓ **CTRL-04**: Frame-edge safety (top/bottom margin fade + push) — v1.0
- ✓ **CTRL-05**: Altitude-hold P loop driven by `target_altitude` (drone-specific) — v1.0
- ✓ **CTRL-06**: Velocity clamping + per-axis EMA + forward-axis slew-rate cap — v1.0
- ✓ **DRONE-01**: MAVSDK adapter with offboard handshake, arm/takeoff/land lifecycle, detached `mavsdk_server` — v1.0
- ✓ **DRONE-02**: USB serial (`/dev/ttyACM0`) and UDP (sim) connection paths — v1.0
- ✓ **DRONE-03**: Offboard-loss watchdog (pilot-handover safe; never auto-switches mode) — v1.0
- ✓ **UI-01**: Web UI with MJPEG live video, click-to-follow target locking, live PID/altitude tuning — v1.0
- ✓ **UI-02**: OpenHD bridge (RTP video over UDP 5500, x264 SW encode, dynamic bitrate from QOpenHD WFB) — v1.0
- ✓ **UI-03**: Follow-mode contract over OpenHD: `follow_id = -1` IDLE / `0` AUTO / `N` LOCKED — v1.0
- ✓ **REC-01**: Pure-GStreamer recording branch (x264enc → matroskamux → filesink) toggleable mid-flight — v1.0
- ✓ **SIM-01**: PX4 SITL + Gazebo Garden bundled via submodule (`sim/PX4-Autopilot`) with video bridge + MAVLink relay — v1.0
- ✓ **SIM-02**: Configurable worlds (`person_in_front`, `2_person_world`, `random_walk`, `walk_across_then_approach`, `circle_around`, `2_persons_diagonal`) — v1.0
- ✓ **OPS-01**: `scripts/start_air.sh` (modes A/B for OpenHD camera ownership) + boot service driven by `~/Desktop/drone-follow.conf` — v1.0
- ✓ **OPS-02**: Ground-station install path (no Hailo dependency) via `scripts/install_ground_station.sh` + headless eglfs fallback — v1.0
- ✓ **OPS-03**: Dual-WiFi networking (wlan0 home + wlan1 field-AP, MAC-pinned by udev) — v1.0

### Active

<!-- Current scope. Building toward these. Filled by the current milestone. -->

(Set by current milestone — see Current Milestone section.)

### Out of Scope

<!-- Explicit boundaries. Includes reasoning to prevent re-adding. -->

- Indoor / GPS-denied operation — outside PX4 offboard + altitude-hold assumptions and Hailo-pipeline FOV calibration
- Multi-target simultaneous follow — single-target controller is the validated abstraction; multi-target would require new fusion logic and UI contracts
- Onboard mission planning / waypoint scheduling — out-of-scope for follow-mode; GCS owns mission planning
- Non-person detection classes (vehicles, animals) — ReID gallery and bbox-height-driven distance loop are tuned for human proportions

## Context

- **Hardware target**: Raspberry Pi 5 (8 GB) with Hailo-8L M.2 hat, Cube Orange+ flight controller over USB serial, IMX219/IMX708 CSI camera or USB webcam, TP-Link USB WiFi for field AP.
- **Submodule**: `hailo-apps/` (branch `community_plugins`) provides the GStreamer/Hailo pipeline plumbing and shared venv at `./hailo-apps/venv_hailo_apps` (built with `--system-site-packages`).
- **Drone-control stack**: MAVSDK Python over PX4. Offboard mode is pilot-initiated by default; `--takeoff-landing` opts into auto-arm/takeoff/land.
- **ReID**: dedicated HEF model loaded by `hailo-apps`; gallery sizing + drift/duplicate thresholds tuned empirically (`docs/tracking-reid-algorithm.md`).
- **OpenHD integration**: drone-follow can own the CSI camera and encode (Mode A) or read SHM frames OpenHD provides (Mode B). Mode is configured at install time and persisted via `primary_camera_type` in `air_camera_generic.json`.
- **Operator UX**: web UI on the air unit for dev/debug; OpenHD link for field ops. Mutually exclusive (`--openhd` vs `--webui`).

## Constraints

- **Tech stack**: Python 3.10+, GStreamer 1.20+, MAVSDK Python, hailort (apt deb), hailo-tappas-core, Vite 8 / Node 20 for UI.
- **Hardware**: Hailo-8L M.2 for air unit; ground station has no Hailo dependency. RPi5 has no HW H.264 encoder → x264 SW encode is the only OpenHD path.
- **Compatibility**: Cube Orange+ via USB serial only (`/dev/ttyACM0`). PX4 parameters `COM_RC_IN_MODE=4` + `COM_RCL_EXCEPT` bit 2 required for RC-less offboard.
- **Pairing**: OpenHD air/ground share a `txrx.key`; never use `openhd --clean-start` on a paired pair (regenerates the key, breaks WFB link).
- **Boot**: `drone-follow-boot.service` reads `~/Desktop/drone-follow.conf` (ENABLED, MODE) and dispatches to `scripts/start_air.sh`.

## Key Decisions

<!-- Decisions that constrain future work. Add throughout project lifecycle. -->

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Pure `follow_api/` separated from `drone_api/` and `pipeline_adapter/` | Allows the controller / detection / actuator layers to evolve independently and be tested without GStreamer or MAVSDK | ✓ Good |
| `VelocityCommand(forward, down, yawspeed)` as the actuator boundary type | Robot-agnostic 3-tuple; `down=0` is fine for ground robots, so the same controller targets future non-drone platforms | ✓ Good |
| MAVSDK over direct MAVLink (pymavlink) | Async API, batteries-included offboard/telemetry, supports detached `mavsdk_server` for graceful Ctrl+C → land | ✓ Good |
| Bundle PX4 SITL as submodule (`sim/PX4-Autopilot` v1.14) instead of system install | Reproducible sim, no host PX4 install required, version pinned | ✓ Good |
| OpenHD owns video link; drone-follow owns AI | Avoids re-implementing WFB; mode A/B lets the project flex which side owns the camera | ✓ Good |
| Implicit-display rule (display ON when no UI flag, OFF when `--webui`/`--openhd` set) | DX-friendly defaults: `drone-follow --input usb` just works on a desk | ✓ Good |

---
*Last updated: 2026-05-12 after bootstrap to v1.0-shipped baseline*
