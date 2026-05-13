---
marp: true
theme: default
paginate: true
size: 16:9
---

# drone-follow → robot-follow

## v1.1 design review

From a drone-only app to a robot-generic follow platform.

<sub>Branch `feature/rover-support` • 2026-05-13</sub>

---

## The talk in one diagram

```
                    TODAY                                    v1.1

      Camera ─► Hailo ─► Track ─► ReID ─┐         Camera ─► Hailo ─► Track ─► ReID ─┐
                                        ▼                                            ▼
                              follow_api/controller                       follow_api/controller
                                        │                                            │
                                        ▼                                            ▼
                              VelocityCommand                                RobotCommand
                                        │                                            │
                                        ▼                                            ▼
                              MAVSDK ─► PX4 ─► drone                    ┌──── Robot protocol ────┐
                                                                        │                        │
                                                                        ▼                        ▼
                                                              MavsdkDroneAdapter      Ros2RoverAdapter
                                                                        │                        │
                                                                        ▼                        ▼
                                                                  PX4 ─► drone           rclpy ─► /cmd_vel
                                                                                          ─► Gazebo rover
```

Pipeline / controller / ReID / web UI: **unchanged**. Only the actuator boundary moves.

---

## Hardware (what runs in the field)

- **Compute:** Raspberry Pi 5 (8 GB) + Hailo-8L M.2
- **Camera:** IMX219 / IMX708 CSI, or USB webcam (dev)
- **Flight controller:** Cube Orange+ via USB serial @ 57 600 baud
- **Radio:** OpenHD WFB over TP-Link USB WiFi (`wlan1`)
- **Ground:** 2nd RPi5 + QOpenHD, or phone
- **Dev:** built-in WiFi (`wlan0`) for SSH, simultaneous with WFB

---

## The job, in five layers

```
┌──────────────────────────────────────────────────────────┐
│ 1. Camera frames @ 30 fps  (CSI / USB / UDP / SHM)        │
└─────────────────────────┬─────────────────────────────────┘
                          │ GStreamer
┌─────────────────────────▼─────────────────────────────────┐
│ 2. Hailo person detection (tiled → unified bboxes)        │
└─────────────────────────┬─────────────────────────────────┘
                          │ HailoDetection
┌─────────────────────────▼─────────────────────────────────┐
│ 3. ByteTracker  +  ReID gallery / drift / raw fallback    │
└─────────────────────────┬─────────────────────────────────┘
                          │ Detection(cx, cy, bbox_h)
┌─────────────────────────▼─────────────────────────────────┐
│ 4. follow_api/controller  yaw + forward + alt + safety    │
└─────────────────────────┬─────────────────────────────────┘
                          │ VelocityCommand
┌─────────────────────────▼─────────────────────────────────┐
│ 5. Actuator: MAVSDK ► PX4   (v1.1: also rclpy ► /cmd_vel) │
└───────────────────────────────────────────────────────────┘
```

`VelocityCommand` is **3 floats**. The whole milestone hinges on that boundary.

---

## Drone vs rover — capability matrix

| Capability                 | Drone (today)            | Rover (v1.1)             |
|----------------------------|--------------------------|--------------------------|
| Offboard handshake         | **required** (PX4)       | none                     |
| Arm / takeoff / land       | optional                 | n/a                      |
| Altitude control           | **active** P loop        | n/a                      |
| Yaw units                  | **deg/s** (PX4)          | **rad/s** (`Twist`)      |
| Wire protocol              | MAVLink via MAVSDK       | ROS 2 `Twist`            |
| Actuator stack             | mavsdk_server subprocess | rclpy + executor thread  |
| Bottom-edge safety policy  | retreat (tilt headroom)  | slow / stop              |
| Yaw-spin search on loss    | allowed                  | typically denied         |

→ `Capabilities` dataclass + `Robot` protocol formalise this.

---

## Codebase today

```
drone_follow/
├── drone_follow_app.py        ◄── composition root + CLI
├── follow_api/                ◄── pure domain (stdlib only)
│   ├── types.py               VelocityCommand, Detection
│   ├── state.py               SharedDetectionState, FollowTargetState
│   ├── config.py              ControllerConfig
│   └── controller.py          compute_velocity_command
├── drone_api/                 ◄── MAVSDK
│   └── mavsdk_drone.py
├── pipeline_adapter/          ◄── Hailo + GStreamer + tracker + ReID
│   ├── hailo_drone_detection_manager.py   (callback hot path)
│   ├── vision_branches.py     (display / record / openhd / webui)
│   ├── reid_manager.py
│   ├── byte_tracker.py / fast_tracker.py / tracker_factory.py
├── servers/                   ◄── follow + web (MJPEG) + openhd_bridge
├── ui/                        ◄── Vite 8 / Node 20
└── tests/
```

---

## Threading model

```
   GStreamer C lib (Hailo callback)            Main thread (asyncio)
   ─────────────────────────────────           ─────────────────────────────────
   detection_callback(persons)                 live_control_loop @ ~30 Hz
        │                                           │
        │  ByteTracker.update(persons)              │  read latest detection
        │  ReIDManager.update_gallery               │  compute_velocity_command
        │  shared_state.update(detection)           │  VelocityCommandAPI.send
        ▼                                           ▼
        SharedDetectionState  ◄── lock ────►        MAVSDK offboard stream
                                                    (telemetry tasks cache alt)

   FollowServer (HTTP)        WebServer (HTTP + SSE + MJPEG)        OpenHDBridge (UDP)
   ─────────────────────      ────────────────────────────────       ────────────────────
   click-to-follow            live tuning, video, status             follow_id, bbox msgs,
   FollowTargetState          SharedDetectionState                   bitrate feedback
```

3 worlds: detection writes, control reads, servers expose. All cross-thread state is lock-guarded.

---

## Boundary types

```python
# follow_api/types.py — no third-party imports

@dataclass
class VelocityCommand:
    forward_m_s: float       # +ve = forward, body frame
    down_m_s:    float       # +ve = down, NED  (drone only)
    yawspeed_deg_s: float    # +ve = CW looking down

@dataclass
class Detection:
    label: str
    confidence: float
    center_x: float          # 0.0 – 1.0
    center_y: float          # 0.0 – 1.0
    bbox_height: float       # 0.0 – 1.0
    timestamp: float
```

This file is the contract. Renaming `VelocityCommand` → `RobotCommand` is Phase 3.

---

## Follow modes (state machine)

```
                ┌─────────────────────────────────────────┐
                │            target_state                 │
                │                                         │
                │   ┌────────┐   click bbox    ┌────────┐ │
                │   │        │ ───────────►    │        │ │
                │   │  AUTO  │                 │ LOCKED │ │
                │   │        │ ◄───────────    │        │ │
                │   └────────┘  reid timeout   └────────┘ │
                │     ▲   │                      ▲   │    │
                │     │   │  ground:             │   │    │
                │     │   │  follow_id=-1        │   │    │
                │     │   ▼                      │   ▼    │
                │     │ ┌────────┐               │  pause │
                │     └─┤  IDLE  │ ◄─────────────┘        │
                │       └────────┘                        │
                └─────────────────────────────────────────┘
```

| Mode    | target_id | paused | explicit_lock |
|---------|-----------|--------|---------------|
| AUTO    | None      | False  | False         |
| LOCKED  | N         | False  | True          |
| IDLE    | —         | True   | —             |

---

## Controller — yaw + forward

```
   ┌────────────────────────────────┐       ┌────────────────────────────────┐
   │  YAW  (always on)              │       │  FORWARD  (--no-yaw-only)      │
   │                                │       │                                │
   │   error = cx - 0.5             │       │   factor = target_bbox/bbox −1 │
   │                                │       │                                │
   │   yawspeed                     │       │   forward                      │
   │     ▲    ┌─signed sqrt         │       │     ▲    ┌── kp_distance       │
   │     │   /                      │       │     │   /  (approach)          │
   │  ───┼──/───► error             │       │  ───┼──/────► factor           │
   │     │ /                        │       │     │/                         │
   │  dead│                         │       │  dead│  ┌── kp_distance_back   │
   │  zone│                         │       │  zone│ /   (retreat — steeper) │
   │                                │       │                                │
   │   deg/s, clamped to            │       │   m/s, clamped, then           │
   │   ±max_yawspeed                │       │   EMA + slew-rate cap          │
   └────────────────────────────────┘       └────────────────────────────────┘
```

Asymmetric retreat gain — no rangefinder, the only safety margin is bbox-size vs panic threshold.

---

## Controller — altitude + frame-edge safety

```
   ┌────────────────────────────────┐       ┌─────────────────────────────────┐
   │  ALTITUDE  (drone only)        │       │  FRAME-EDGE SAFETY              │
   │                                │       │                                 │
   │   err = current - target_alt   │       │   ┌──────────────────────────┐  │
   │   down = kp_alt * err          │       │   │ push (force away)        │  │
   │   clamp(±max_climb/down)       │       │   │═══════════════════════ ═ │margin
   │                                │       │   │ fade (natural→0)         │  │
   │  Reads:  drone.telemetry.      │       │   │═══════════════════════ ═ │  │
   │          position() cache      │       │   │                          │  │
   │                                │       │   │     (controller zone)    │  │
   │  v1.1: capability-gated;       │       │   │                          │  │
   │        rover ignores entirely. │       │   │═══════════════════════ ═ │  │
   │                                │       │   │ fade                     │  │
   │                                │       │   │═══════════════════════ ═ │margin
   │                                │       │   │ push                     │  │
   │                                │       │   └──────────────────────────┘  │
   │                                │       │                                 │
   │                                │       │ Top → forward, Bottom → back    │
   │                                │       │ (drone). Rover: bottom = stop.  │
   └────────────────────────────────┘       └─────────────────────────────────┘
```

---

## Detection callback (hot path)

```
   per frame  (≈ 30 fps, on GStreamer streaming thread)
   ────────────────────────────────────────────────────
        persons : list[HailoDetection]   ← Hailo + tile cropper
              │
              ▼
        tracker.update(persons)  ──►  list[TrackedDetection]
              │
              ▼
        ┌─────────────────────────────────────────────┐
        │ choose target:                              │
        │   LOCKED  → find by track_id                │
        │   ↳ miss  → ReIDManager.try_reidentify      │
        │   ↳ miss  → score_visible_persons (RAW)     │
        │   AUTO    → largest_by_bbox                 │
        └─────────────────────────────────────────────┘
              │
              ▼
        ReIDManager.update_gallery(target)   ◄── drift / dup gates
              │
              ▼
        shared_state.update(detection, available_ids)
              │
              ▼
        vision_branches pad probe:
          • strip tile rectangles
          • retag target's HailoDetection → class_id 99
            (overlay YAML draws it bigger/greener)
```

---

## ReID — three similarity bands

```
            sim vs gallery
    0.0 ─────────────────────────────────────────────────── 1.0
        │    DRIFT     │    NORMAL    │    DUPLICATE       │
        │  sim < 0.6   │ 0.6 ≤ s ≤ 0.9│   sim > 0.9        │
        │              │              │                    │
        │  → don't     │  → store     │  → skip            │
        │    store     │    FIFO-out  │  → every Nth dup:  │
        │  → run       │    oldest    │    refresh oldest  │
        │    re-acquire│              │                    │
        │    pass      │              │                    │
```

**Plus raw-detection fallback:** if tracker emits zero tracks but raw `persons` exist, score them directly. Locked-follow survives tracker dropouts.

**Knobs:** `--reid-threshold 0.75`, `--reid-drift-threshold 0.6`, `--reid-duplicate-threshold 0.9`, `--reid-refresh-every 5`, `--reid-timeout 20.0`, `--update-interval 10`.

---

## Vision branches (output tree)

```
   source ─► tile_cropper ─► user_callback ─►┐
                                              │ output_tee
        ┌─────────────────────────────────────┼─────────────────────────┐
        ▼                                     ▼                         ▼
   [--webui]                            [--openhd]            [--display / --record]
   MJPEG appsink                        RTP/UDP H.264         retag target class_id=99
   (client renders                      to OpenHD ground      ↓
    bboxes from JSON)                                         hailooverlay_community
                                                              (show-tiles=false, YAML)
                                                              ↓ local_tee
                                                       ┌──────┴──────┐
                                                       ▼             ▼
                                                  [--display]    [--record]
                                                  fpsdisplaysink valve+x264+mkv
```

- `--webui` and `--openhd` are **mutually exclusive** (single network encoder).
- Implicit-display rule: if neither UI flag is set, `--display` defaults **on**.
- No HW H.264 on RPi5 → x264 software encode for both.

---

## MAVSDK adapter + servers

**`drone_api/mavsdk_drone.py`** — only place that imports `mavsdk`.

```
   VelocityCommand  ──►  VelocityCommandAPI.send  ──►  drone.offboard.set_velocity_body
                          (clamp ► EMA ► slew cap)        (VelocityBodyYawspeed)

   detached  mavsdk_server  ◄── subprocess ──►  PX4 over UDP / serial
   (survives Ctrl+C → graceful land)
```

Lifecycle (when `--takeoff-landing` set): arm → takeoff → setpoint loop → land. Otherwise: stream zeros, wait for pilot OFFBOARD switch. Offboard-loss watchdog **never auto-switches modes**.

**Three servers** (all run in their own threads):

| Server          | Port  | Surface                                              |
|-----------------|-------|------------------------------------------------------|
| FollowServer    | 8080  | HTTP click-to-follow / target API                    |
| WebServer       | 5001  | Vite UI + MJPEG + SSE + live-tune endpoints          |
| OpenHDBridge    | 5500  | RTP video + control msgs to QOpenHD                  |

---

## CLI & configuration

```
   Pipeline:    --input {rpi|usb|udp://…|shm://…}, --input-codec, --tiles-x/y
   Drone:       --connection, --serial, --takeoff-landing, --target-altitude,
                --target-bbox-height, --yaw-only / --no-yaw-only
   ReID:        --reid-* (8 flags)         Tracker:  --tracker {byte|fast}
   UI:          --webui / --openhd / --display / --record (mutex rules apply)
   Controller:  every numeric ControllerConfig field (also live-tunable)
```

Config layering (lowest → highest priority):

```
   ControllerConfig dataclass defaults
              │
              ▼
   JSON file via --config
              │  (configs/simulation.json, simulation_follow.json, df_params.json)
              ▼
   CLI flags (--kp-yaw=…, --max-forward=…)
              │
              ▼
   Live UI sliders (web UI + OpenHD bridge)
```

---

## Run-time scenarios

```
   Dev: USB cam + real Cube                Sim: PX4 SITL + Gazebo
   ┌──────┐   USB   ┌─────────────┐        gz sim ──gz-transport──► video_bridge.py
   │laptop│◄────────│ drone-follow│                                  │
   │ +Cube│  serial │ --input usb │                       UDP 5600 ◄─┘   ► drone-follow
   │ Or+  │◄────────│ --serial    │        ┌─UDP 14540────► PX4 SITL ◄─MAVSDK──┘
   └──────┘         │ --webui     │
                    └─────────────┘

   Real flight, Mode A (drone-follow owns CSI cam)
   CSI ─► drone-follow ─► overlay+x264 ─► OpenHD air ─► WFB ─► OpenHD gnd ─► QOpenHD
         (--input rpi --openhd)

   Real flight, Mode B (OpenHD owns CSI cam, tees SHM)
   CSI ─► OpenHD ─tee─► /tmp/openhd_raw_video ─► drone-follow (AI only)
                  └──► WFB ─► QOpenHD
```

`scripts/start_air.sh --mode {stream|shm}` selects mode; `install_air.sh` must match.

---

## Boot service (field deployment contract)

```
   systemd
      │
      ▼
   /etc/systemd/system/drone-follow-boot.service     ◄── name preserved (RENAME-04)
      │
      ▼
   scripts/boot/drone-follow-boot.sh
      │
      ▼
   ~/Desktop/drone-follow.conf       ENABLED=true               ◄── operator-editable
      │                              MODE=stream|shm
      ▼
   scripts/start_air.sh [--mode …]
      │
      ▼
   drone-follow --input rpi --openhd --connection tcpout://127.0.0.1:5760 …
```

Three Pis in the field share this contract. Breaking it = breaking the fleet.

---

# Part 2 — v1.1 changes

What `feature/rover-support` ships.

---

## Where the drone-only assumption leaks (11 sites)

| Where                                    | Leak                              | Fix                                       |
|------------------------------------------|-----------------------------------|-------------------------------------------|
| `drone_follow_app.py`                    | `run_drone()`, drone args always  | `run_robot()`, dispatch on `--robot`      |
| `follow_api/types.py`                    | `VelocityCommand`, `down_m_s`     | `RobotCommand`; altitude optional         |
| `follow_api/controller.py`               | altitude P unconditional          | gate on `has_altitude`                    |
| `follow_api/controller.py`               | bottom-edge = retreat-from-tilt   | capability-gate, rover = slow/stop        |
| `follow_api/controller.py`               | yaw-spin on loss assumed allowed  | capability flag                           |
| `follow_api/config.py`                   | altitude fields hard-coded floats | `Optional[float]`, gated `validate()`     |
| `drone_api/mavsdk_drone.py`              | yaw deg/s baked in                | adapter boundary normalises               |
| `--takeoff-landing/--target-altitude/--serial` | always in `--help`         | two-pass argparse                         |
| `hailo_…manager.py:1271`                 | ByteTracker knobs hard-coded      | config-driven (RINT-03)                   |
| `web_server` + `openhd_bridge`           | parallel tunable-field lists      | `ControllerConfig.tunable_fields()`       |
| 3× implicit-display rule duplications    | scattered                         | single source in `vision_branches`        |

55 numbered requirements in `.planning/REQUIREMENTS.md`.

---

## The 6 phases

```
   Phase 1 ── Rename                Phase 2 ── Cleanup
   drone_follow → robot_follow      Dead code, dups, hot-path races
   (mechanical)                     (CLEAN-01..18)
        │                                  │
        └─────────────►◄───────────────────┘
                       │
                       ▼
   ╔═══════════════════════════════════════════╗
   ║  Phase 3 ── Robot protocol + Capabilities ║
   ║  MavsdkDroneAdapter; --robot flag         ║
   ║  CRITICAL GATE                            ║
   ║  drone SITL must pass end-to-end here     ║
   ╚════════════════════╤══════════════════════╝
                        │
            ┌───────────┴───────────┐
            ▼                       ▼
   Phase 4 ── Rover adapter  Phase 5 ── Rover sim
   rclpy + Twist + cmd_vel   Gazebo Garden SDF +
   SIGINT-safe thread        cmd_vel bridge +
                             video_bridge.py reuse
            │                       │
            └───────────┬───────────┘
                        ▼
   Phase 6 ── Sim integration
   rover-safe defaults, bottom-edge repurpose,
   ByteTracker config, end-to-end test
```

Phases 4 & 5 are parallelisable after Phase 3 passes.

---

## Phase 1 — Rename

**Goal:** `pip show robot_follow` works, `pip show drone_follow` returns nothing. Both `drone-follow --help` and `robot-follow --help` produce identical output. Boot service unit file unchanged on disk.

```
   git mv drone_follow robot_follow      ◄── single atomic commit
            │
            ├─ 48 internal imports rewritten
            ├─ drone_follow_app.py  →  robot_follow_app.py
            ├─ pyproject.toml:   name = "robot-follow"
            │                    scripts: robot-follow + drone-follow (alias)
            ├─ install.sh:       pip uninstall drone-follow -y  (defensive)
            ├─ shell scripts:    setup_env.sh / install_air.sh / start_air.sh
            ├─ docs:             every example rewritten; one alias note
            │                    in README + CLAUDE.md
            └─ git grep gate:    drone_follow|from drone_follow|import drone_follow
                                 → only docs/history whitelist matches
```

**Stays put:** `drone_api/` subdir (renames in Phase 3), `ui/`, `recordings/`, boot unit file, `~/Desktop/drone-follow.conf`. **Branch scope:** lives only on `feature/rover-support`.

---

## Phase 2 — Cleanup (18 items, three buckets)

| Dead code (10)     | Duplication (5)                  | Hot-path (3)                          |
|--------------------|----------------------------------|---------------------------------------|
| world_loader.py    | mavsdk_server pkill helper       | **CLEAN-16** SSE race (next slide)    |
| bench_reid_callback| 3× argparse pre-parsers → 1      | CLEAN-17 socket-per-call → reuse      |
| `--vfov`           | telemetry position+alt tasks     | CLEAN-18 O(n) dedup → O(1) dict       |
| `--mission-duration`| tunable_fields() unification     |                                       |
| serial_baud=115200 fallback | branch-decision tree unified |                                  |
| strip_tiles… alias |                                  |                                       |
| available_ids=None |                                  |                                       |
| shutdown_read_fd   |                                  |                                       |
| _NULLABLE_FIELDS   |                                  |                                       |
| create_app(controller_config=) |                      |                                       |
```
All audited pre-milestone — no archaeology required during execution.
```

---

## Phase 2 — CLEAN-16 (SSE race)

```
   BEFORE  (web_server.py:84)                    AFTER
   ─────────────────────────────                 ──────────────────────────────
   def _on_frame(...):                           def _on_frame(...):
       self._frame_event.set()                       with self._cond:
       self._frame_event.clear()                         self._frame_seq += 1
                                                         self._cond.notify_all()

   def _sse_handler():                           def _sse_handler():
       self._frame_event.wait(timeout=2.0)           last = -1
       # race: clear() may fire                      while True:
       #       between set() and second                 with self._cond:
       #       consumer's wait()                            self._cond.wait_for(
       # → consumer falls through                              lambda: self._frame_seq != last,
       #   2 s timeout                                          timeout=2.0)
                                                              last = self._frame_seq
```

Two browser tabs watching the MJPEG — today, one stalls. Tomorrow, both update on every frame.

---

## Phase 3 — Robot protocol + Capabilities

```python
@dataclass(frozen=True)
class Capabilities:
    has_altitude:               bool
    needs_offboard_handshake:   bool
    needs_takeoff_landing:      bool
    yaw_units:                  Literal["deg/s", "rad/s"]

@dataclass
class RobotCommand:           # replaces VelocityCommand
    forward:  float           # m/s
    yaw:      float           # in `yaw_units`
    altitude: float = 0.0     # m, ignored when has_altitude=False

class Robot(Protocol):
    capabilities: Capabilities
    async def connect(self) -> None: ...
    async def start_session(self) -> None: ...
    async def send_command(self, cmd: RobotCommand) -> None: ...
    async def send_zero(self) -> None: ...
    async def shutdown(self) -> None: ...
```

```
   DroneCapabilities = Capabilities(True,  True,  T/F, "deg/s")
   RoverCapabilities = Capabilities(False, False, False, "rad/s")
```

---

## Phase 3 — live_control_loop capability gates

```
   ┌──────────────────────────────────────────────────────────────┐
   │  compute_velocity_command(detection, config)                 │
   │       │                                                       │
   │       ▼                                                       │
   │  base RobotCommand(forward, yaw)                              │
   │       │                                                       │
   │       ├─ if caps.has_altitude:                                │
   │       │       cmd.altitude = altitude_p_loop(curr, target)   │
   │       │                                                       │
   │       ├─ if caps.has_altitude:                                │
   │       │       drone_bottom_edge_retreat(cmd, det)             │
   │       │  else:                                                 │
   │       │       rover_bottom_edge_slowstop(cmd, det)            │
   │       │                                                       │
   │       └─ if caps.can_yaw_spin_to_search:                      │
   │               yaw_spin_search(cmd)                            │
   │                                                               │
   └──────────────────────────────────────────────────────────────┘
                                  │
                                  ▼
                          robot.send_command(cmd)
```

ControllerConfig altitude fields become `Optional[float]`; `validate()` skips altitude relationship checks when `has_altitude=False`.

---

## Phase 3 — `--robot` two-pass argparse

```
   PASS 1                            PASS 2 (conditional)
   ──────────                        ─────────────────────────
   pre.add_argument("--robot",       parser = build_common()
       choices=["drone","rover"],    if pre_args.robot == "drone":
       default="drone")                  add_drone_args(parser)
   pre_args, _ = pre.parse_known()   else:
                                         add_rover_args(parser)
                                     args = parser.parse_args()
```

- `--robot drone --help` shows `--takeoff-landing`, `--target-altitude`, `--serial`
- `--robot rover --help` shows `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id`
- Folds into the existing pre-parse stack (output flags, ReID, tracker) — CLEAN-12 collapses all of those into one stage.

---

## Phase 4 — rover adapter (rclpy on a thread)

```
   ┌───────────────────────────────────────────────────────────────┐
   │ Main thread (asyncio)                                         │
   │   live_control_loop ──► robot.send_command(cmd)               │
   │                              │                                │
   │                              ▼                                │
   │                       publisher.publish(Twist)                │
   │                       (thread-safe — rcl lock-protected)      │
   │                              │                                │
   ├──────────────────────────────┼────────────────────────────────┤
   │ rclpy-spin thread            │  (background, daemon)          │
   │   while not shutdown:        │                                │
   │     executor.spin_once(0.05) │                                │
   │                              │                                │
   │                              ▼                                │
   │                       Twist on /cmd_vel  ───► ROS / gz bridge │
   └───────────────────────────────────────────────────────────────┘
```

```python
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # ← non-negotiable
node = Node("robot_follow_rover")
pub  = node.create_publisher(Twist, "/cmd_vel", 10)
exec = SingleThreadedExecutor(); exec.add_node(node)
threading.Thread(target=spin_loop, daemon=True).start()
```

---

## Phase 4 — the SIGINT trap

```
   ┌─────────────────────────────────────────────────────────────────┐
   │ default rclpy.init()                                            │
   │                                                                 │
   │   installs rclpy's SIGINT handler ──►  silently overrides       │
   │                                        drone-follow's on_signal │
   │                                                                 │
   │   Ctrl+C → rclpy says "shutting down"                           │
   │         → publisher stops                                       │
   │         → BUT: no zero-velocity command sent                    │
   │         → rover keeps moving on last Twist                      │
   └─────────────────────────────────────────────────────────────────┘

   ┌─────────────────────────────────────────────────────────────────┐
   │ rclpy.init(signal_handler_options=SignalHandlerOptions.NO)      │
   │                                                                 │
   │   drone-follow's on_signal stays installed                      │
   │   Ctrl+C → on_signal → send_zero() → shutdown() → rover stops   │
   └─────────────────────────────────────────────────────────────────┘
```

Post-init assertion in `start_session`:

```python
assert signal.getsignal(signal.SIGINT) is on_signal, "rclpy stole SIGINT"
```

Covered by RINT-06 end-to-end test (zero `/cmd_vel` within 100 ms of Ctrl+C).

---

## Phase 5 — rover sim (one command)

```
   start_rover_sim.sh
       │
       ├─► gz sim sim/rover/worlds/walk_across_then_approach.sdf
       │     │
       │     ▼
       │   rover.sdf:  <plugin filename="gz-sim-diff-drive-system"
       │                       name="gz::sim::systems::DiffDrive">
       │                   <topic>cmd_vel</topic>          ◄── override default
       │               </plugin>
       │               + camera sensor
       │
       ├─► ros2 run ros_gz_bridge parameter_bridge \
       │       /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
       │
       └─► python sim/bridge/video_bridge.py \
               --topic /model/rover/sensor/camera/image \
               --host 127.0.0.1 --port 5600   ◄── EXISTING tool, reused
```

drone-follow runs unchanged: `robot-follow --robot rover --input udp://0.0.0.0:5600`.

**No new camera shim.** `ros_gz_bridge` is for cmd_vel only.

---

## Phase 5 — Garden gotchas (all silent failures)

| Field                 | WRONG (Ignition era)                          | RIGHT (Garden era)                   |
|-----------------------|-----------------------------------------------|--------------------------------------|
| SDF plugin filename   | `libignition-gazebo-diff-drive-system.so`     | `gz-sim-diff-drive-system`           |
| SDF plugin name       | `ignition::gazebo::systems::DiffDrive`        | `gz::sim::systems::DiffDrive`        |
| Bridge msg type       | `…@ignition.msgs.Twist`                       | `…@gz.msgs.Twist`                    |
| Apt bridge package    | `ros-humble-ros-gz-bridge` (Fortress)         | `ros-humble-ros-gzgarden-bridge`     |
| DiffDrive topic       | `/model/<n>/cmd_vel` (default)                | override `<topic>cmd_vel</topic>`    |
| Camera bridge         | `ros_gz_image_bridge` (~15 Hz ceiling)        | reuse `video_bridge.py` (gz-transport) |

Validate after launch:

```
   $ gz topic -l                              # /cmd_vel + /model/rover/sensor/...
   $ gz topic -e -t /cmd_vel                  # 1 subscriber, 0 publishers (yet)
```

---

## Phase 6 — rover-safe defaults + bottom-edge repurpose

```
   configs/rover_simulation.json
   ─────────────────────────────
   {
     "max_forward":   1.0,         (was 4.0 for drone)
     "max_backward":  0.5,
     "kp_yaw":        60,          (was 120 — narrower yaw dynamics)
     "kp_distance":   1.0,
     "_omitted":      "all altitude knobs — capability-gated off",
     "tracker": {
       "track_thresh":  0.4,
       "track_buffer":  30,        (was 90 → 1 s vs 3 s @ 30 fps)
       "match_thresh":  0.8
     }
   }
```

```
   bottom-edge safety
   ───────────────────────────────────────────────────────────────
   if caps.has_altitude:                if not caps.has_altitude:
       # DRONE: retreat for tilt           # ROVER: slow / stop
       forward -= retreat_force            forward = 0   if person too low
                                           yaw still active
```

Plus RINT-03: ByteTracker knobs become config-driven (today hard-coded at `hailo_…manager.py:1271`).

---

## Phase 6 — end-to-end test (RINT-04 + RINT-06)

```
   test_rover_follow_walk_across_then_approach
   ────────────────────────────────────────────
        start sim (walk_across_then_approach world)
              │
              ▼
        launch robot-follow --robot rover --config rover_simulation.json
              │
              ▼
        wait for first detection            ◄── timeout 10 s
              │
              ▼
        capture target_id
              │
              ▼
        run 45 s
              │
              ▼
   ASSERT  target_id_lost_events == 0
   ASSERT  final_target_id == initial_target_id
              │
              ▼
        Ctrl+C
              │
              ▼
   ASSERT  /cmd_vel messages within 100 ms == 0     ◄── RINT-06
   ASSERT  rover stopped within 1 s
```

The only test that exercises the SIGINT-preservation pattern in anger.

---

## Before / after architecture

```
        TODAY                                       v1.1

   drone_follow_app.py                         robot_follow_app.py
        │                                            │
        │                                            │  pre-parse --robot
        ▼                                            ▼
   run_drone()                                  run_robot(args)
        │                                            │
        ▼                                            ├─►  MavsdkDroneAdapter
   follow_api / pipeline_adapter                     │      (moved from drone_api/
        │  (unchanged)                               │       to robot_api/adapters/)
        ▼                                            │
   VelocityCommand                                   └─►  Ros2RoverAdapter   ◄── NEW
        │                                                   │ rclpy + Twist
        ▼                                                   │ on /cmd_vel
   drone_api.mavsdk_drone                              [capability-gated controller]
        │                                                   │
        ▼                                                   ▼
   PX4 ─► drone                                       PX4 │ Gazebo rover
                                                          │
                                              NEW:        │
                                              robot_api/robot.py
                                              robot_api/adapters/ros2_rover.py
                                              sim/rover/
                                              configs/rover_simulation.json
```

The 4 boxes labelled "unchanged" are 80% of the codebase.

---

## Risks (ranked)

| # | Risk                                                    | Severity | Mitigation                                                  |
|---|---------------------------------------------------------|----------|-------------------------------------------------------------|
| 1 | rclpy steals SIGINT → rover keeps moving on Ctrl+C      | **HIGH** | `SignalHandlerOptions.NO`; post-init assert; RINT-06 test   |
| 2 | Phase 3 regression breaks drone SITL                    | **HIGH** | Critical gate — full SITL test must pass before Phase 4/5   |
| 3 | `_rclpy_pybind11` ImportError in hailo venv             | MEDIUM   | Defensive import; `setup_env.sh` auto-sources ROS           |
| 4 | Garden silent SDF load failure (`ignition::` prefix)    | MEDIUM   | PITFALLS.md + README; `gz topic -l` validation step         |
| 5 | `ros_gz_bridge` 15 Hz camera ceiling                    | MEDIUM   | **Avoid**: reuse `video_bridge.py` (gz-transport)           |
| 6 | Stale `pip show drone-follow` on deployed units         | LOW      | `install.sh` runs `pip uninstall drone-follow -y`           |
| 7 | Two-pass argparse `--help` confusion                    | LOW      | Smoke-tested in Phase 3                                     |
| 8 | Gazebo Garden EOL (Nov 2024)                            | LOW      | SDF already uses `gz::` (Harmonic-compatible)               |

Risks 1 + 2 dominate; both are gated by tests.

---

## Out of scope (v1.1)

| Excluded                         | Reason                                                   |
|----------------------------------|----------------------------------------------------------|
| Real rover hardware              | Sim can't validate HW safety → v1.2                      |
| Nav2 / ros2_control              | Goal-based pathing, wrong message types                  |
| SLAM / mapping                   | Requires LiDAR / depth — not on BoM                      |
| Non-Humble ROS 2 distros         | Iron EOL Dec 2024; Jazzy has no Jammy binaries           |
| Gazebo Harmonic migration        | Garden EOL but SDF is already `gz::`-compatible          |
| Multi-robot fleet control        | Single-target controller is the validated abstraction    |
| Mid-run robot-type switching     | Requires pipeline restart                                |
| OpenHD for rovers                | OpenHD = WFB/MAVLink; rover = ROS 2 → its own milestone  |
| PX4 Rover via MAVSDK             | Chose ROS 2 cmd_vel; tracked as v2 HW-09                 |

Tracked in `.planning/REQUIREMENTS.md` § Out of Scope / v2.

---

## Where we are now / next

```
   feature/rover-support  (5 commits ahead of main, all docs)
   ─────────────────────────────────────────────────────────────
   .planning/PROJECT.md
   .planning/REQUIREMENTS.md           55 numbered reqs
   .planning/ROADMAP.md                6 phases
   .planning/research/                 stack / features / arch / pitfalls
   .planning/phases/01-rename/01-CONTEXT.md      ◄── decisions locked
   .planning/presentations/v1_1_overview.md      ◄── this deck

                                ▼
                          /gsd:plan-phase 1
                                ▼
                       .planning/phases/01-rename/01-PLAN.md
                                ▼
                          /gsd:execute-phase 1
                                ▼
                          atomic rename commit
```

Questions / pushback / "we should also…" → now is the moment.
