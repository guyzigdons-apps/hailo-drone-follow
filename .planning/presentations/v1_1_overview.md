# drone-follow → robot-follow

## v1.1 design review

### From a drone-only app to a robot-generic follow platform

**Author:** v1.1 planning notes
**Date:** 2026-05-13
**Branch:** `feature/rover-support`
**Audience:** engineers new to this project

---

## The talk in one slide

We have a working **drone-follow** app: a Raspberry Pi 5 on a quadrotor uses a Hailo-8L NPU to detect a person, and MAVSDK / PX4 to fly toward them and keep them in frame. Outdoor, autonomous, no operator input required.

**v1.1 generalises the actuator side.** The pipeline / controller / ReID / web UI stay where they are. The MAVSDK drone code moves behind a `Robot` protocol. A second adapter — ROS 2 Humble publishing `geometry_msgs/Twist` on `/cmd_vel` — appears alongside it. A Gazebo Garden rover sim closes the loop.

**Why now:** the controller is already robot-agnostic in shape (it emits `VelocityCommand(forward, down, yawspeed)`). The drone-only assumptions are leaking in 11 specific places in the composition root, config, and live-control loop. v1.1 names the abstraction and pays off the leakage in one milestone, sim-only.

**Out of scope:** real rover hardware (deferred to v1.2), Nav2 / ros2_control, SLAM, multi-robot.

---

## What we run today: the hardware

- **Compute:** Raspberry Pi 5 (8 GB), Hailo-8L M.2 hat on the PCIe slot
- **Camera:** IMX219 / IMX708 CSI camera, or a USB webcam for dev
- **Flight controller:** Cube Orange+ (PX4 firmware) over USB serial `/dev/ttyACM0` @ 57 600 baud
- **Radio link:** OpenHD-based WFB (WiFiBroadcast) using a TP-Link USB WiFi adapter (`wlan1`)
- **Ground station:** any RPi5 / PC running `openhd --ground` + QOpenHD
- **Dev networking:** RPi's built-in WiFi (`wlan0`) for SSH from a laptop, while `wlan1` runs WFB to the drone simultaneously

Two RPi5s, one drone, one phone (or kiosk display) on the ground. That's the field deployment.

---

## What it does: the job in five layers

```
┌─────────────────────────────────────────────────────────┐
│ 1. Camera frames at 30 fps (CSI, USB, UDP, or SHM)      │
└────────────────────────┬────────────────────────────────┘
                         │ GStreamer
┌────────────────────────▼────────────────────────────────┐
│ 2. Hailo NPU person detection at video rate             │
│    (tiling, then unified bboxes back at full res)       │
└────────────────────────┬────────────────────────────────┘
                         │ HailoDetections per frame
┌────────────────────────▼────────────────────────────────┐
│ 3. ByteTracker → stable per-person IDs                  │
│    + ReID gallery for re-acquisition after occlusion    │
└────────────────────────┬────────────────────────────────┘
                         │ Detection(center_x, center_y, bbox_height)
┌────────────────────────▼────────────────────────────────┐
│ 4. Follow controller: yaw + forward + altitude + safety │
│    Emits VelocityCommand(forward, down, yawspeed)       │
└────────────────────────┬────────────────────────────────┘
                         │ VelocityCommand
┌────────────────────────▼────────────────────────────────┐
│ 5. MAVSDK adapter → PX4 offboard velocity setpoints     │
│    Plus servers: web UI, OpenHD bridge, follow server   │
└─────────────────────────────────────────────────────────┘
```

The boundary between layers 4 and 5 is the **`VelocityCommand` type** — three floats. That's what makes v1.1 cheap.

---

## Why "robot-follow" — the wedge

The follow controller already produces a **robot-agnostic** command. Layer 5 is the only thing that has to know we're a drone:

- Offboard handshake (PX4 idiom)
- Arm / takeoff / land lifecycle
- Altitude-hold P loop
- Yaw units in **deg/s** (PX4 convention)
- MAVSDK connection lifecycle, detached `mavsdk_server`

For a differential-drive **rover** with ROS 2:

- No offboard handshake — `/cmd_vel` publish-only
- No arm / takeoff / land
- No altitude
- Yaw units in **rad/s** (`geometry_msgs/Twist.angular.z`)
- rclpy Node lifecycle, executor on a background thread

The shape of the actuator interface is the same. The implementation differs. **That's the textbook setup for a Protocol.**

---

## The codebase today: five packages, one entry point

```
drone_follow/
├── drone_follow_app.py        ← composition root + CLI
├── follow_api/                ← pure domain logic, stdlib only
│   ├── types.py               (VelocityCommand, Detection)
│   ├── state.py               (SharedDetectionState, FollowTargetState)
│   ├── config.py              (ControllerConfig — all tunable knobs)
│   └── controller.py          (compute_velocity_command)
├── drone_api/                 ← MAVSDK adapter
│   └── mavsdk_drone.py        (VelocityCommandAPI, run_live_drone, add_drone_args)
├── pipeline_adapter/          ← Hailo + GStreamer + ByteTracker + ReID
│   ├── hailo_drone_detection_manager.py   (callback, branch tree)
│   ├── vision_branches.py     (display / record / openhd / webui)
│   ├── reid_manager.py        (gallery, drift, raw fallback)
│   ├── byte_tracker.py        ('byte' tracker impl)
│   ├── fast_tracker.py        (default tracker)
│   ├── tracker_factory.py     (--tracker selector)
│   └── tracker.py             (tracker interface)
├── servers/                   ← HTTP / video servers
│   ├── follow_server.py       (target selection HTTP API)
│   ├── web_server.py          (web UI + MJPEG)
│   └── openhd_bridge.py       (RTP over UDP + control msgs)
├── ui/                        ← Vite 8 / Node 20 frontend
└── tests/
```

`reid_analysis/` is a sibling top-level package (HEF benchmarking / embedding extraction tools), not in the main app's hot path.

---

## Big-picture data flow

```
Camera ──► tiling+detection ──► ByteTracker ──► tracks ──┐
   (Hailo HEF)                                            │
                                                          │
              ReIDManager  ◄───── target id ──────────────┤
                  │                                       │
                  │ (raw-detection fallback              ▼
                  │  on tracker miss)             SharedDetectionState
                  ▼                                       │
            compute_velocity_command  ◄─────  control loop @ ~30 Hz
                  │                            (in asyncio task)
                  ▼
            VelocityCommand
                  │
                  ▼
            VelocityCommandAPI  (clamps, EMA, slew)
                  │
                  ▼
            MAVSDK offboard setpoint stream  ──► PX4  ──► motors
```

Three threads in play:

1. **GStreamer streaming thread** (owned by the C library; runs the pad probes + Python user callback)
2. **Main asyncio loop** (drone control loop, MAVSDK calls, telemetry tasks)
3. **HTTP server threads** (web UI + OpenHD bridge + follow server)

`SharedDetectionState` is a lock-protected handoff between thread 1 (writes detections) and thread 2 (reads them).

---

## Boundary types: tiny, deliberate

```python
@dataclass
class VelocityCommand:
    forward_m_s: float        # body frame, +ve = forward
    down_m_s: float           # +ve = down (PX4 NED convention)
    yawspeed_deg_s: float     # +ve = clockwise looking down

@dataclass
class Detection:
    label: str
    confidence: float
    center_x: float           # 0.0–1.0 normalised
    center_y: float           # 0.0–1.0 normalised
    bbox_height: float        # 0.0–1.0 normalised
    timestamp: float
```

`follow_api/types.py` has no imports beyond `dataclasses`. **This file is the contract** — the rest of `follow_api` only uses these types and `ControllerConfig`. No MAVSDK, no Hailo, no GStreamer.

That clean boundary is what makes a `Robot` protocol introduction in Phase 3 a refactor, not a rewrite.

---

## Threading model in detail

```
┌─────────────────────────────────────────────────────────────────┐
│  Process                                                         │
│                                                                  │
│  ┌──────────────────┐    ┌────────────────────────────────────┐ │
│  │ GStreamer C lib  │    │ Main thread (asyncio loop)         │ │
│  │ (Hailo callback) │    │  ┌──────────────────────────────┐  │ │
│  │   ↓ writes       │    │  │ live_control_loop @ ~30 Hz   │  │ │
│  │ SharedDetection  │    │  │   read SharedDetectionState  │  │ │
│  │ State (lock)     │◄───┼──┤   call controller            │  │ │
│  │   ↓ writes       │    │  │   send VelocityCommand → mav │  │ │
│  │ available_ids    │    │  └──────────────────────────────┘  │ │
│  └──────────────────┘    │  ┌──────────────────────────────┐  │ │
│                          │  │ MAVSDK Python (own subprocess │  │ │
│                          │  │   mavsdk_server bridges to    │  │ │
│                          │  │   PX4 over UDP / serial)      │  │ │
│                          │  └──────────────────────────────┘  │ │
│                          └────────────────────────────────────┘ │
│                                                                  │
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────────┐│
│  │ FollowServer    │  │ WebServer        │  │ OpenHDBridge    ││
│  │ (HTTP thread)   │  │ (HTTP thread +   │  │ (UDP thread +   ││
│  │  click-to-follow│  │  SSE + MJPEG)    │  │  MAVLink-ish)   ││
│  └─────────────────┘  └──────────────────┘  └─────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

The asyncio loop owns the **drone**. Detections cross the lock boundary into the loop. Servers are independent. Cross-thread mutations all funnel through lock-protected `SharedDetectionState` / `FollowTargetState` / `SharedUIState`.

---

## Controller, part 1: yaw centring

`compute_velocity_command(detection, config) → VelocityCommand`

The yaw component centres the target horizontally:

```
   error = center_x - 0.5          # normalised, [-0.5, +0.5]
   if |error| < dead_zone_x:       # deadband
       yawspeed = 0
   else:
       # signed sqrt response: gentle near 0, firm far from 0
       yawspeed = kp_yaw * sign(error) * sqrt(|error| - dead_zone_x)
       yawspeed = clamp(yawspeed, -max_yawspeed, +max_yawspeed)
```

- **deadband** stops jitter on a centred target
- **signed sqrt** beats linear P: the response near the deadband edge is small (avoids jerky corrections) but ramps up faster than linear at large errors

Yaw is emitted in **deg/s**. PX4's `VelocityBodyYawspeed.yawspeed_deg_s` consumes it directly.

---

## Controller, part 2: bbox-distance forward loop

The drone has no rangefinder to the target. It infers distance from **bbox height**.

```
   factor = (target_bbox_height / detection.bbox_height) - 1.0
                          │
                          ├── bbox << target  → factor > 0  → fly forward
                          ├── bbox >> target  → factor < 0  → fly backward
                          └── bbox == target  → factor = 0  → hold

   gain  = kp_distance      if factor > 0   (approach — gentle)
           kp_distance_back if factor < 0   (retreat — aggressive)

   forward = clamp(gain * factor, -max_backward, +max_forward)
```

**Why `target/bbox - 1` and not `bbox - target`:**

Bbox size ∝ 1/distance. The ratio form gives a **scale-invariant relative error**: a person at 2× target distance produces `factor = 1` regardless of absolute bbox size. A linear `bbox - target` controller would emit weak commands on small far-away bboxes and over-correct on huge close ones.

**Asymmetric gains:** retreat is stronger than approach because there's no rangefinder — if a person is too close, the only safety margin is bbox-size relative to the panic threshold `max_bbox_height_safety`.

---

## Controller, part 3: altitude hold (drone-specific)

```
   altitude_error = current_altitude - target_altitude
   down_speed     = kp_alt_hold * altitude_error          # NED frame
   down_speed     = clamp(down_speed, -max_climb_speed, +max_down_speed)
```

Simple P loop. `target_altitude` defaults to 3 m, adjustable mid-flight via the web UI's altitude slider or via OpenHD.

The current altitude is read from MAVSDK telemetry (`drone.telemetry.position()`) into a thread-safe cache; the control loop reads the cache rather than awaiting telemetry per tick.

**This is the most obvious drone-only block in the controller.** Capability-gating it for rovers (Phase 3) is one of the cleanest changes in v1.1.

---

## Controller, part 4: frame-edge safety

Two stacked zones at the top and bottom of the frame:

```
   ┌─────────────────────────────────────┐ ← frame top
   │ safety push zone (force away)       │
   │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │ top margin
   │ pre-margin fade zone                │ (natural cmd fades to 0)
   │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
   │                                     │
   │       (main controller zone)        │
   │                                     │
   │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │
   │ pre-margin fade zone                │
   │ ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ │ bottom margin
   │ safety push zone (force away)       │
   └─────────────────────────────────────┘ ← frame bottom
```

When the target bbox edge crosses into a margin:

- **Top edge in top margin** = person too far / climbing out of frame → emit **forward** push
- **Bottom edge in bottom margin** = person too close / tilting out of frame → emit **backward** push

The **fade zone** smooths the transition: in the zone just outside the margin, the natural command in the offending direction is linearly scaled to 0 as the bbox edge approaches the margin. This kills the limit-cycle "approach → at-bottom → back-off → approach" oscillation.

**For rovers (Phase 6),** the bottom-edge rule changes meaning: "person too low in frame" → slow/stop. No tilt, no retreat. Same call site, different policy, capability-gated.

---

## Shared state and follow modes

`SharedDetectionState` (lock-protected):

- `_detection: Optional[Detection]` — latest detection of the active target
- `_frame_count: int` — heartbeat
- `_available_ids: set[int]` — currently visible track IDs (drives UI)

`FollowTargetState` (lock-protected):

- `_target_id: Optional[int]`
- `_paused: bool` (IDLE mode)
- `_explicit_lock: bool` (operator-selected vs auto-picked)

Three follow modes derived from those fields:

| Mode    | `target_id` | `paused` | `explicit_lock` | Meaning                                      |
|---------|-------------|----------|-----------------|----------------------------------------------|
| IDLE    | —           | True     | —               | Hold position, ignore detections             |
| AUTO    | None        | False    | False           | Follow whoever is biggest in frame           |
| LOCKED  | N           | False    | True            | Follow specific person; ReID gallery active  |

Mode transitions are driven by the web UI (click bbox), OpenHD (`follow_id` MAVLink param: -1 / 0 / N), and ReID-timeout (LOCKED → AUTO after `--reid-timeout`).

---

## Pipeline adapter: the detection callback

The Hailo pipeline calls a Python callback on the GStreamer streaming thread, once per frame:

```python
def detection_callback(persons: list[HailoDetection]):
    # 1. Pass to tracker → returns tracked instances with stable IDs
    tracked = tracker.update(persons)

    # 2. Compute "available_ids" set (visible track IDs)

    # 3. Decide which detection to follow:
    target = follow_state.get_target()
    chosen = None
    if mode == LOCKED:
        chosen = find_by_track_id(tracked, target)
        if chosen is None and reid_manager:
            chosen = reid_manager.try_reidentify(tracked, persons)  # ReID
    elif mode == AUTO:
        chosen = largest_by_bbox(tracked)

    # 4. Optionally sample an embedding into the ReID gallery
    if reid_manager and chosen:
        reid_manager.update_gallery(chosen)   # ← drift/duplicate check

    # 5. Write to SharedDetectionState
    shared_state.update(chosen, available_ids)

    # 6. Side-effects: vision_branches tile-stripping + bbox highlight pad probe
```

**This is the hot path.** It runs at video rate (30+ fps), inside the streaming thread. Any blocking call here drops frames. The ReID work is the heaviest part — see next slide.

---

## ByteTracker — multi-person tracking

A classic two-stage Kalman-filter + Hungarian-matcher tracker:

```
Detection bbox at frame N    Predicted track from frame N-1
        │                              │
        └──────────► Hungarian ◄───────┘
                     matching (IoU)
                          │
              ┌───────────┼────────────┐
              ▼           ▼            ▼
           matched     new track    lost track
              │           │            │
              ▼           ▼            ▼
           update      activate    age (kept up
           Kalman      after K     to `track_buffer`
                       hits        frames before
                                   deletion)
```

Key knobs (currently **hard-coded** in `hailo_drone_detection_manager.py` — Phase 6 makes them config-driven):

- `track_thresh = 0.4` — minimum detection confidence to enter the matcher
- `track_buffer = 90` (3 s @ 30 fps) — frames to keep a "lost" track alive
- `match_thresh = 0.8` — IoU threshold for matching

For rovers, `track_buffer = 30` (1 s) is more appropriate: ground-perspective motion magnitudes are smaller, so a person tracked for too long after they leave the frame can mis-match a different person who walks in.

---

## ReID — gallery, drift gate, raw-detection fallback

When the operator **locks** onto a person (or AUTO mode runs long enough), the ReID manager builds a **gallery** of appearance embeddings for that person, sampled every `--update-interval` frames.

**Three similarity bands** govern what to do with each new sample:

```
    sim vs gallery
       0.0  ─────────────────────────────────────────────► 1.0
            ▲                ▲                 ▲
            │  DRIFT BAND    │ NORMAL BAND     │ DUPLICATE BAND
            │  sim < 0.6     │ 0.6 ≤ sim ≤ 0.9 │ sim > 0.9
            │                │                 │
            │  → suspect     │ → store         │ → skip
            │    tracker     │   (FIFO oldest  │   (refresh oldest
            │    drift; do   │    if full)     │    every Nth dup)
            │    NOT store;  │                 │
            │    run         │                 │
            │    re-acquire  │                 │
            │    pass        │                 │
```

**Drift defense:** if a new in-track embedding is too dissimilar to the gallery, the manager assumes the tracker has silently swapped onto a different person. It does NOT store the bad embedding (which would poison the gallery), and it runs an immediate **re-acquisition** pass over the visible tracks — if a different track matches the gallery above `--reid-threshold`, the lock switches to it.

**Raw-detection fallback:** if ByteTracker temporarily emits zero confirmed tracks for the locked person but the Hailo detector still sees raw persons, the manager scores those raw bboxes directly. Locked-follow survives a tracker dropout.

Threshold knobs: `--reid-threshold 0.75`, `--reid-drift-threshold 0.6`, `--reid-duplicate-threshold 0.9`, `--reid-refresh-every 5`, `--reid-timeout 20.0`, `--update-interval 10`.

---

## Vision branches: who consumes the encoded video

After detection, the pipeline tees into up to four branches. Only what the operator asks for is built.

```
   source → tile_cropper → user_callback ┐
                                          │ output_tee
              ┌───────────────────────────┼────────────────────────┐
              │                           │                        │
              ▼                           ▼                        ▼
     [--webui]                     [--openhd]            [--display / --record]
      MJPEG appsink                RTP H.264 / UDP        identity pad probe:
       (clean frame;               to OpenHD ground       retag target's
        client renders                                    HailoDetection
        bboxes from JSON)                                 with class_id 99
                                                              │
                                                              ▼
                                                  hailooverlay_community
                                                  (show-tiles=false,
                                                   per-class YAML style)
                                                              │
                                                              ▼ local_tee
                                          ┌───────────────────┴────────────────┐
                                          ▼                                    ▼
                                  [--display]                           [--record]
                                  videoconvert +                         valve +
                                  fpsdisplaysink                         x264enc +
                                                                         matroskamux +
                                                                         filesink
```

**Important details:**

- **Implicit-display rule:** if neither `--webui` nor `--openhd` is set, `--display` defaults ON. (You sit at a desk with a USB cam; `drone-follow --input usb` opens a window with overlay.)
- **`--webui` and `--openhd` are mutually exclusive** — both want to be the single network video encoder.
- **No HW H.264 on RPi5** — x264 software encode for both OpenHD and `--record`.

---

## Drone adapter (`drone_api/mavsdk_drone.py`)

All MAVSDK imports are confined here. Highlights:

- **`VelocityCommandAPI.send(cmd)`** wraps `drone.offboard.set_velocity_body(VelocityBodyYawspeed)`:
  - Clamps to `max_forward / max_backward / max_yawspeed / max_climb_speed / max_down_speed`
  - Per-axis exponential low-pass (EMA) filter
  - Forward-axis slew-rate cap (prevents tilt transients)
- **`_to_mavsdk(VelocityCommand) → VelocityBodyYawspeed`** — the type bridge
- **Connection lifecycle:**
  - Spawns `mavsdk_server` as a detached subprocess (so Ctrl+C → graceful land doesn't kill the bridge)
  - Connects via `udpin://0.0.0.0:14540` (sim) or `serial:///dev/ttyACM0:57600` (real drone)
- **Offboard handshake:**
  - With `--takeoff-landing`: arm → takeoff → set_velocity_body loop
  - Without `--takeoff-landing` (default): stream zero setpoints, wait for pilot-initiated OFFBOARD switch
- **Watchdog:** offboard-loss → return-to-pilot, never auto-switch modes
- **Telemetry tasks:** position + altitude subscribers writing to thread-safe caches consumed by the control loop

`add_drone_args(parser)` registers `--connection`, `--serial`, `--serial-baud`, `--takeoff-landing`, `--target-altitude`, `--mission-duration` on the shared parser.

---

## Servers: where the operator interacts

**`follow_server.py`** — HTTP API for target selection. Used by the web UI's click-to-follow and by tests. Port `8080`. Endpoints set/get `FollowTargetState`.

**`web_server.py`** — Web UI on `--webui-port` (default `5001`). Serves the Vite build, MJPEG stream at `--webui-fps` fps, SSE event stream for detections/bboxes/config, and HTTP endpoints for live controller tuning (every numeric `ControllerConfig` field is sliderable mid-flight). Click-to-follow happens here.

**`openhd_bridge.py`** — On the air unit, parallel surface to the web UI when flying with QOpenHD. Listens on UDP for QOpenHD's status messages (WFB link quality recommendation, follow_id changes), and emits bbox + status messages to the ground station for QOpenHD's overlay rendering. Bitrate of the OpenHD video stream is updated dynamically from QOpenHD's bandwidth recommendation.

Web UI and OpenHD bridge are **mutually exclusive** — both want to be the single H.264 encoder downstream of `output_tee`.

---

## CLI surface — grouped

```
Pipeline:    --input {rpi|usb|udp://...|shm://...}
             --input-codec, --tiles-x, --tiles-y, …

Drone:       --connection, --serial[=DEV], --serial-baud,
             --takeoff-landing, --target-altitude,
             --target-bbox-height,
             --mission-duration

Tracker:     --tracker {byte|fast}, --update-interval

ReID:        --reid-model, --no-reid,
             --reid-threshold, --reid-timeout,
             --reid-drift-threshold, --reid-duplicate-threshold,
             --reid-refresh-every, --reid-min-gallery-for-drift-check,
             --reid-bootstrap-consistency, --reid-overlap-skip-iou,
             --reid-dump-embeddings

Controller:  --kp-yaw, --kp-distance, --kp-distance-back, --kp-alt-hold,
             --max-forward, --max-backward, --max-yawspeed, …
             (also live-tunable from the web UI / OpenHD)

App / UI:    --webui [--webui-port --webui-fps],
             --openhd [--openhd-port --openhd-bitrate],
             --display, --record [--record-output --record-bitrate],
             --follow-server-port, --log-perf, --test-log,
             --horizontal-mirror, --vertical-mirror, --yaw-only,
             --config <path-to-json>
```

Composition root (`drone_follow_app.py`) does a **three-pass argparse pre-parse** today (output flags, ReID flags, tracker flags) to wire dependencies before the full parser runs. Phase 2 collapses these into one. Phase 3 adds a fourth `--robot` pre-parse.

---

## Config — ControllerConfig + JSON

`follow_api/config.py` defines `ControllerConfig`, a dataclass holding every numeric knob the controller reads: gains, max speeds, deadbands, fade margins, ReID thresholds (the controller-relevant ones), altitude limits.

Three layers, increasing priority:

1. **Code defaults** — the dataclass field defaults
2. **JSON config** — `--config configs/simulation.json` overrides
3. **CLI flags** — final word

Bundled configs:

- `configs/simulation.json` — yaw-only, safe gains, used by SITL by default
- `configs/simulation_follow.json` — full follow with reduced max speeds
- `df_params.json` (repo root) — real-drone parameters

Live tunability: the web UI exposes every numeric `ControllerConfig` field as a slider; values round-trip via the HTTP API. (Phase 2 task CLEAN-14 unifies this list with the OpenHD bridge's parallel list.)

---

## Run-time scenarios

```
Dev: USB webcam + real Cube Orange+
   ┌────────────────┐  USB cam   ┌──────────────┐
   │ Laptop / dev   │◄───────────┤ drone-follow │
   │ machine        │            │  --input usb │
   │ + Cube Orange+ │  serial    │  --serial    │
   │   /dev/ttyACM0 ├────────────┤  --webui     │
   └────────────────┘            └──────────────┘

Sim: PX4 SITL + Gazebo
   gz sim ──gz-transport──► video_bridge.py ─UDP─► drone-follow
       │                       (port 5600)
       └─MAVLink (UDP 14540)──► PX4 SITL ◄─MAVSDK─ drone-follow

Real flight (Mode A — drone-follow owns the camera):
   CSI cam ──► drone-follow ──► overlay+x264 ──► OpenHD air ──► WFB ──► OpenHD ground ──► QOpenHD
              (--input rpi    (--openhd port            (binary
               --openhd        5500)                     detection
               --tiles-x 2)                              payload)

Real flight (Mode B — OpenHD owns the camera, drone-follow reads SHM):
   CSI cam ──► OpenHD ──tee──► /tmp/openhd_raw_video ──► drone-follow ──► AI only
                          └──► WFB → QOpenHD       (--input shm://...)
```

`scripts/start_air.sh --mode {stream|shm}` invokes drone-follow with the right flags; `install_air.sh --mode {stream|shm}` configures OpenHD's `primary_camera_type` to match.

---

## Boot service + field deployment

```
systemd
   │
   ▼
/etc/systemd/system/drone-follow-boot.service
   │
   ▼
/usr/local/bin/drone-follow-boot.sh
   │
   ▼
~/Desktop/drone-follow.conf     ENABLED=true
   │                            MODE=stream|shm   (optional)
   ▼
scripts/start_air.sh [--mode stream|shm]
   │
   ▼
drone-follow --input rpi --openhd --connection tcpout://127.0.0.1:5760 ...
```

**Key invariants:**

- `~/Desktop/drone-follow.conf` is the operator-editable switch. Disabling boot = open file, `ENABLED=false`, reboot.
- `drone-follow-boot.service` runs as the `hailo` user, not root.
- The systemd **unit file name** and the **config file name** are operator-visible artifacts. They must not change between releases. (This pins Phase 1's RENAME-04: rename the package, keep the unit/conf names.)

Three Pis in the field — two air units and one ground station — all use this contract. Breaking it = breaking a deployed fleet.

---

## Simulation today (drone)

`sim/` contains the bundled PX4 SITL setup:

- `sim/PX4-Autopilot/` — git submodule pinned to v1.14.0
- `sim/setup_sim.sh` — first-time build (10–20 min)
- `sim/start_sim.sh [--bridge] [--world NAME] [--remote IP]` — launches `gz sim`, PX4 SITL, the video bridge, and optionally a MAVLink relay for remote-sim setups
- `sim/bridge/video_bridge.py` — **the crucial piece for v1.1**: subscribes to a Gazebo camera topic over **gz-transport** and pushes H.264 over UDP on port 5600. No ROS dependency, no `ros_gz_image_bridge`.
- `sim/worlds/` — `person_in_front`, `2_person_world`, `random_walk`, `walk_across_then_approach`, `circle_around`, `2_persons_diagonal`

**`video_bridge.py` is what makes Phase 5 cheap.** The rover sim reuses it verbatim, just pointed at the rover's camera topic. No new camera shim needed.

---

# Part 2: What changes in `feature/rover-support`

The remaining slides cover the v1.1 milestone: six phases, 55 numbered requirements, one critical gate.

---

## Why now — the drone-only leakage

The controller is robot-agnostic by accident, not by design. Eleven concrete places leak the drone assumption:

| Where | What leaks | v1.1 fix |
|-------|-----------|----------|
| `drone_follow_app.py` | `run_drone()` entry name, `add_drone_args()` always registered | `run_robot()`, dispatch on `--robot` |
| `follow_api/types.py` | Type is named `VelocityCommand`; `down_m_s` makes no sense for a rover | Rename to `RobotCommand`, capability-gate `altitude` field |
| `follow_api/controller.py` | Altitude P loop unconditional | Gate on `capabilities.has_altitude` |
| `follow_api/controller.py` | Bottom-edge "retreat from tilt" assumes a quadrotor | Capability-gated; rover repurpose = slow/stop |
| `follow_api/controller.py` | Search-after-loss yaw spin assumes the platform can spin in place | Capability-gated (rovers may not spin) |
| `follow_api/config.py` | Altitude fields are floats with hard-coded defaults | `Optional[float]`, gated `validate()` |
| `drone_api/mavsdk_drone.py` | Yaw in deg/s baked in | Adapter boundary normalises; rover converts to rad/s |
| `--takeoff-landing`, `--target-altitude`, `--serial` | Always visible in `--help` | Two-pass argparse hides drone flags from `--robot rover` |
| `pipeline_adapter/hailo_drone_detection_manager.py:1271` | ByteTracker knobs hard-coded | Config-driven |
| `web_server.py`, `openhd_bridge.py` | Parallel lists of tunable fields | Single `ControllerConfig.tunable_fields()` |
| Output-branch tree | Implicit-display rule defined in three places | Single source in `vision_branches` |

Each row maps to one or more numbered requirements in `.planning/REQUIREMENTS.md`.

---

## What "robot-generic" buys us

- **Rover sim today.** End-to-end follow-the-person in Gazebo Garden, no real hardware needed. Cheap validation of the controller on a totally different platform.
- **Real rover tomorrow (v1.2).** Same adapter, different `/cmd_vel` target. No code change expected — just topic config and a hardware checkout.
- **Future actuators are 1 file + 1 protocol implementation.** PX4 Rover via MAVSDK, ArduPilot Rover, a tethered camera dolly, anything that takes velocity commands.
- **Cleanup forced by abstraction.** The 18 cleanup items (CLEAN-01..18) — confirmed dead code, duplications, hot-path races — are easier to land cleanly with the package rename and capability gates already in place.

The whole milestone is **sim-only**, so safety risk is bounded. Real hardware is a separate milestone with its own checkout list.

---

## The 6-phase plan

```
                  Phase 1 ── Rename
                  drone_follow → robot_follow (mechanical only)
                            │
                            ▼
                  Phase 2 ── Cleanup
                  Dead code, duplications, hot-path races (CLEAN-01..18)
                            │
                            ▼
                  ╔═════════ Phase 3 ═════════╗
                  ║   Robot protocol +         ║
                  ║   Capabilities;            ║
                  ║   MavsdkDroneAdapter       ║
                  ║   wraps existing drone     ║
                  ║   CRITICAL GATE            ║
                  ║   (drone path must still   ║
                  ║   pass SITL end-to-end)    ║
                  ╚═════════╤══════════════════╝
                            │
              ┌─────────────┴─────────────┐
              │                           │
              ▼                           ▼
       Phase 4 ── Rover adapter   Phase 5 ── Rover sim
       (rclpy / Twist /            (Gazebo Garden SDF,
        cmd_vel; SIGINT-safe       cmd_vel ros_gz_bridge,
        executor thread)           video_bridge.py reuse)
              │                           │
              └─────────────┬─────────────┘
                            │
                            ▼
                  Phase 6 ── Sim integration
                  Rover defaults, bottom-edge repurpose,
                  ByteTracker config, end-to-end test
```

Phases 4 and 5 are **independent** once Phase 3 lands — they can be done in parallel by two developers, or interleaved.

---

## Phase 1 — Rename (in detail)

**Goal:** `pip show robot_follow` works, `pip show drone_follow` returns nothing. `drone-follow --help` and `robot-follow --help` produce identical output. Every internal `from drone_follow.*` import becomes `from robot_follow.*`. Boot service unit file name unchanged on disk.

**Decisions locked (from `.planning/phases/01-rename/01-CONTEXT.md`):**

- **One atomic commit:** `git mv drone_follow robot_follow` + 48 import rewrites + pyproject + scripts + docs all land together. Every commit on the branch stays buildable; bisect-friendly.
- **`drone_follow_app.py` is also renamed** → `robot_follow_app.py`. Module and package share the family name.
- **pyproject:** `name = "robot-follow"`. Two console scripts pointing at the same target:
  ```toml
  robot-follow = "robot_follow.robot_follow_app:main"
  drone-follow = "robot_follow.robot_follow_app:main"
  ```
- **Install migration:** `install.sh` runs `pip uninstall drone-follow -y` before `pip install -e .` so deployed units don't keep stale metadata.
- **UI and recordings stay nested** — `robot_follow/ui/`, `robot_follow/recordings/`.
- **Docs:** every `drone-follow --...` example rewritten to `robot-follow --...`; one alias note near the top of README + CLAUDE.md.
- **Branch scope:** lives only on `feature/rover-support`. `main` stays `drone_follow` until v1.1 merges.

**Verification gate:**

- Pre-commit: lint + `python -c 'import robot_follow'` + both `--help` outputs match
- Pre-push: full pytest on a Hailo-capable host
- `git grep -nE 'drone_follow|from drone_follow|import drone_follow'` returns only whitelist matches (alias notes + `.planning/MILESTONES.md` history)

**Out of scope here:** the `drone_api` / `follow_api` / `pipeline_adapter` sub-package names stay — they rename / reorganise in Phase 3 (`drone_api` → `robot_api/adapters/`). Cleanup is Phase 2.

---

## Phase 2 — Cleanup (in detail)

**Goal:** Dead code deleted, duplications merged, hot-path races fixed — **before** structural changes in Phase 3. Easier to refactor a clean codebase than a noisy one.

**Dead-code deletions (10 items):**

```
CLEAN-01  sim/world_loader.py                 orphan; start_sim.sh uses PX4_GZ_WORLD
CLEAN-02  scripts/bench_reid_callback.py      imports non-existent reid_worker
CLEAN-03  follow_api/config.py --vfov         defined end-to-end, never read
CLEAN-04  --mission-duration                  undocumented 5-min auto-land hazard
CLEAN-05  serial_baud=115200 fallback         unreachable; would lie if it fired
CLEAN-06  strip_tiles_and_highlight_target    no callers
CLEAN-07  available_ids=None default          every caller passes a set
CLEAN-08  shutdown_read_fd                    unreachable from caller
CLEAN-09  _NULLABLE_FIELDS = set() branches   dead value==0→None branch
CLEAN-10  create_app(controller_config=)      no caller passes; attached later
```

**Duplication merges (5 items):**

```
CLEAN-11  mavsdk_server pkill reaper          one helper in the adapter, two callers
CLEAN-12  3 throwaway argparse pre-parsers    collapse to one
CLEAN-13  _telemetry_position_task +          merge — both subscribe drone.telemetry.position()
          _telemetry_altitude_task
CLEAN-14  web _CONFIG_FIELDS +                replace with ControllerConfig.tunable_fields()
          openhd _CONFIG_PARAMS
CLEAN-15  Branch-decision tree                centralised in vision_branches;
                                              implicit-display rule defined once
```

**Hot-path wins (3 items):**

```
CLEAN-16  web_server._frame_event set+clear   race — SSE readers fall through to 2s timeout
                                              under multi-client load. Replace with
                                              monotonic counter + Condition.notify_all()
CLEAN-17  _send_immediate_report opens        socket per call — reuse listener's socket
          new socket every call
CLEAN-18  hailo_..._manager:88 linear-scan   replace with one-shot {id(p): p} lookup dict
          dedup over persons list
```

CLEAN-16 is the one with operator-visible impact — when two browser tabs both watch the MJPEG, today one of them stalls on the 2 s SSE timeout. Fixing it is small and worth a slide.

---

## Phase 2 — hot-path detail (CLEAN-16)

**Today** in `web_server.py:84`:

```python
def _on_frame(self, frame_bytes):
    with self._lock:
        self._latest_frame = frame_bytes
    self._frame_event.set()
    self._frame_event.clear()      # ← race
```

```python
def _sse_handler(self):
    while True:
        self._frame_event.wait(timeout=2.0)   # falls through after 2 s
        if not self._frame_event.is_set():    # ← almost always False
            yield "keepalive"
            continue
        ...
```

When two SSE clients are reading, one consumer sees `set()` and races into `wait()` between the `set()` and `clear()` — but the other consumer that was woken first already consumed the event. Second consumer waits the full 2 s, drops a frame, displays stale state.

**Fix:** monotonic counter + `Condition.notify_all()`:

```python
def _on_frame(self, frame_bytes):
    with self._lock:
        self._latest_frame = frame_bytes
        self._frame_seq += 1
    with self._cond:
        self._cond.notify_all()

def _sse_handler(self):
    last_seen = -1
    while True:
        with self._cond:
            self._cond.wait_for(lambda: self._frame_seq != last_seen, timeout=2.0)
            last_seen = self._frame_seq
        ...
```

Every consumer wakes on every frame, regardless of how many consumers there are. The 2 s timeout still serves as keep-alive only.

---

## Phase 3 — Robot protocol + Capabilities (the abstraction)

**Goal:** `Robot` protocol and `Capabilities` are the only actuator boundary. Drone path runs end-to-end behind `MavsdkDroneAdapter`. `--robot` CLI flag exists. **This is the critical gate.**

New module `robot_api/robot.py`:

```python
from typing import Protocol, Literal, runtime_checkable
from dataclasses import dataclass

@dataclass(frozen=True)
class Capabilities:
    has_altitude: bool                            # drone=True, rover=False
    needs_offboard_handshake: bool                # drone=True, rover=False
    needs_takeoff_landing: bool                   # drone=True (when flag set), rover=False
    yaw_units: Literal["deg/s", "rad/s"]          # drone=deg/s, rover=rad/s

@dataclass
class RobotCommand:
    forward: float                                # m/s, body frame
    yaw: float                                    # in `yaw_units`
    altitude: float = 0.0                         # m, ignored when has_altitude=False

@runtime_checkable
class Robot(Protocol):
    capabilities: Capabilities

    async def connect(self) -> None: ...
    async def start_session(self) -> None: ...
    async def send_command(self, cmd: RobotCommand) -> None: ...
    async def send_zero(self) -> None: ...
    async def shutdown(self) -> None: ...
```

`VelocityCommand` is **renamed** to `RobotCommand` in `types.py`. The drone adapter's wire format is unchanged (`forward → linear x`, `yaw deg/s → yawspeed_deg_s`, `altitude → down via P loop`). The rover adapter ignores `altitude` and converts `yaw` deg/s → rad/s at the boundary.

---

## Phase 3 — live_control_loop capability gates

Today, the live control loop unconditionally does altitude hold, bottom-edge tilt retreat, and yaw-spin-on-loss. Each becomes capability-gated:

```python
# ABS-04: altitude P loop
if robot.capabilities.has_altitude:
    cmd = apply_altitude_p_loop(cmd, current_altitude, target_altitude, config)

# ABS-05: bottom-edge "retreat from tilt" safety
if robot.capabilities.has_altitude:
    cmd = apply_drone_bottom_edge_retreat(cmd, detection, config)
else:
    cmd = apply_rover_bottom_edge_slowstop(cmd, detection, config)   # Phase 6

# ABS-06: yaw-spin recovery on tracker loss
if robot.capabilities.can_yaw_spin_to_search:   # deferred to RoverCapabilities subclass
    cmd = apply_yaw_spin_search(cmd, search_state)
```

`ControllerConfig` altitude fields become `Optional[float]`:

```python
@dataclass
class ControllerConfig:
    # ...
    min_altitude:        Optional[float] = None
    max_altitude:        Optional[float] = None
    target_altitude:     Optional[float] = None
    kp_alt_hold:         Optional[float] = None
    max_climb_speed:     Optional[float] = None
    max_down_speed:      Optional[float] = None
    max_bbox_height_safety: Optional[float] = None
    top_margin_safety:   Optional[float] = None
    bottom_margin_safety: Optional[float] = None

    def validate(self):
        if self.has_altitude:
            assert self.min_altitude is not None < self.target_altitude < self.max_altitude
        # else: skip altitude-relationship checks
```

---

## Phase 3 — two-pass argparse

The CLI surface depends on `--robot`. Drone users see `--takeoff-landing`, `--target-altitude`, `--serial`. Rover users see `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id` (Phase 4 adds those). Neither set should see the other in `--help`.

**Two-pass argparse:**

```python
# Pass 1: pre-parse just --robot
pre = argparse.ArgumentParser(add_help=False)
pre.add_argument("--robot", choices=["drone", "rover"], default="drone")
pre_args, _ = pre.parse_known_args()

# Pass 2: build full parser conditionally
parser = build_full_parser()  # everything common
if pre_args.robot == "drone":
    add_drone_args(parser)
else:
    add_rover_args(parser)
args = parser.parse_args()
```

Implementation lives in `drone_follow_app.py` → `robot_follow_app.py`. The composition root's existing three-pass pre-parse (output flags, ReID, tracker) absorbs this fourth pass naturally.

The `run_drone()` function becomes `run_robot(args)`:

```python
def run_robot(args):
    if args.robot == "drone":
        robot = MavsdkDroneAdapter(args)
    elif args.robot == "rover":
        robot = Ros2RoverAdapter(args)
    asyncio.run(run_session(robot, args))
```

---

## Phase 4 — Rover adapter: rclpy on a background thread

**Goal:** `Ros2RoverAdapter` publishes `geometry_msgs/Twist` on `/cmd_vel`, integrates cleanly with the asyncio control loop, **does not break drone-follow's SIGINT handling.**

Pattern (validated against `rclpy` Humble source):

```python
import threading, signal, rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist

class Ros2RoverAdapter:
    capabilities = Capabilities(
        has_altitude=False, needs_offboard_handshake=False,
        needs_takeoff_landing=False, yaw_units="rad/s",
    )

    def __init__(self, args):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # ← critical
        self._node = Node("robot_follow_rover")
        self._pub = self._node.create_publisher(Twist, args.cmd_vel_topic, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._shutdown = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True, name="rclpy-spin")

    def _spin(self):
        while not self._shutdown.is_set():
            self._executor.spin_once(timeout_sec=0.05)

    async def start_session(self):
        self._thread.start()
        # post-init assertion:
        from robot_follow.robot_follow_app import on_signal
        assert signal.getsignal(signal.SIGINT) is on_signal, "rclpy stole SIGINT"

    async def send_command(self, cmd):
        msg = Twist()
        msg.linear.x = cmd.forward
        msg.angular.z = math.radians(cmd.yaw) if yaw_unit_was_deg else cmd.yaw
        self._pub.publish(msg)               # rcl_publish is lock-protected — thread-safe

    async def send_zero(self):
        await self.send_command(RobotCommand(0.0, 0.0))

    async def shutdown(self):
        self._shutdown.set()
        self._thread.join(timeout=1.0)
        self._executor.shutdown()
        try:
            self._node.destroy_node()
        finally:
            rclpy.try_shutdown()
```

`add_rover_args(parser)` registers `--cmd-vel-topic` (default `/cmd_vel`), `--ros-namespace`, `--ros-domain-id`.

---

## Phase 4 — the SIGINT trap

**This is the single most important slide in the deck.**

`rclpy.init()` **installs its own SIGINT handler by default.** Drone-follow has its own SIGINT handler (`on_signal` in the composition root) that does the graceful-land sequence on Ctrl+C. If rclpy's handler wins, **Ctrl+C will silently stop publishing without commanding zero velocity** — the rover keeps moving on its last commanded twist until safety timeouts (or never).

Per the rclpy Humble source: `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` opts out. Use **exactly this incantation**. Verify post-init:

```python
assert signal.getsignal(signal.SIGINT) is on_signal, \
    "rclpy.init clobbered SIGINT — pass SignalHandlerOptions.NO"
```

Other gotchas from `.planning/research/PITFALLS.md`:

- **`rclpy.spin()`** is blocking-forever. **`spin_once(timeout_sec=0.05)`** in a loop is the only stoppable variant.
- **`_rclpy_pybind11` ImportError** if the hailo-apps venv is activated without sourcing `/opt/ros/humble/setup.bash`. The Python package is in `--system-site-packages` but the C extension lives at `/opt/ros/humble/lib/...`. Fix: `setup_env.sh` auto-sources ROS when `--robot rover` is detected (venv first, ROS second).
- **asyncio.Event is not thread-safe for cross-thread `set()`.** Use `threading.Event` for the rclpy shutdown, or `loop.call_soon_threadsafe`.

Phase 4 includes a smoke-test in the adapter's start_session that asserts the handler — fail fast if Humble changes behaviour.

---

## Phase 5 — Rover sim: SDF + bridge + camera

**Goal:** `sim/rover/start_rover_sim.sh` launches Gazebo Garden with a differential-drive rover. `/cmd_vel` from ROS reaches the sim. Camera frames reach drone-follow on UDP port 5600. One command.

```
sim/rover/
├── rover.sdf                     ← DiffDrive plugin + camera sensor
├── worlds/
│   ├── walk_across_then_approach.sdf  (rover-adapted)
│   ├── random_walk.sdf
│   └── circle_around.sdf
├── start_rover_sim.sh            ← one-command launch
├── setup_rover_sim.sh            ← apt installs
└── README.md                     ← Garden EOL note + Harmonic migration
```

`start_rover_sim.sh` does three things:

```bash
# 1. Gazebo Garden + rover world
gz sim sim/rover/worlds/walk_across_then_approach.sdf &

# 2. ros_gz_bridge for cmd_vel ONLY (not camera)
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &

# 3. video_bridge.py for camera (existing tool)
python sim/bridge/video_bridge.py \
    --topic /model/rover/sensor/camera/image \
    --host 127.0.0.1 --port 5600 &
```

drone-follow runs unchanged, `--input udp://0.0.0.0:5600`. **No new camera shim.** This is the key architectural call.

---

## Phase 5 — Garden gotchas

Gazebo Garden has multiple Ignition→Gazebo rename traps. Every one of these is a **silent load failure** if you get it wrong: the SDF parses cleanly, the bridge starts cleanly, you just never see a topic.

```
SDF plugin name:
  WRONG:  <plugin filename="libignition-gazebo-diff-drive-system.so"
                  name="ignition::gazebo::systems::DiffDrive">
  RIGHT:  <plugin filename="gz-sim-diff-drive-system"
                  name="gz::sim::systems::DiffDrive">

Bridge message type:
  WRONG:  /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
  RIGHT:  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

Apt package for the bridge:
  WRONG:  ros-humble-ros-gz-bridge          (Fortress era)
  RIGHT:  ros-humble-ros-gzgarden-bridge    (Garden era)

DiffDrive default topic:
  WRONG (default):   /model/<name>/cmd_vel
  RIGHT (override):  add <topic>cmd_vel</topic> inside the plugin block
                     so ros_gz_bridge sees /cmd_vel on both sides
```

`.planning/research/PITFALLS.md` has the full list. The rover SDF README documents these in the file alongside the code so the next person doesn't re-discover them.

**Validation step:** `gz topic -l` after `start_rover_sim.sh` should show `/cmd_vel` and the rover camera topic. `gz topic -e -t /cmd_vel` should show a single subscriber (ros_gz_bridge) and zero publishers until you run drone-follow.

---

## Phase 6 — Rover-safe defaults + bottom-edge repurpose

**Goal:** Rover follows a walking actor end-to-end in Gazebo Garden. Rover-safe defaults. SIGINT shuts down cleanly. Port isolation from PX4 SITL documented.

New config `configs/rover_simulation.json`:

```json
{
  "max_forward": 1.0,
  "max_backward": 0.5,
  "max_yawspeed": 60.0,         "_yaw_units_note": "still in deg/s here; adapter converts",
  "kp_yaw":  60.0,              "_note": "lower than drone's 120 — narrower yaw dynamics",
  "kp_distance": 1.0,
  "kp_distance_back": 1.5,

  "_altitude_knobs_omitted": "rover has no altitude — capability-gated off",

  "tracker": {
    "track_thresh": 0.4,
    "track_buffer": 30,         "_note": "1s @ 30fps (was 90 / 3s for drone)",
    "match_thresh": 0.8,
    "frame_rate": 30
  }
}
```

**Bottom-edge safety repurpose** (RINT-02):

- Drone: bottom edge in margin → person too close → **retreat for tilt headroom**
- Rover: bottom edge in margin → person too low in frame → **slow / stop**

Same call site in `controller.py`, capability-gated:

```python
if robot.capabilities.has_altitude:
    cmd = apply_bottom_edge_retreat(cmd, detection, config)        # existing
else:
    cmd = apply_bottom_edge_slowstop(cmd, detection, config)       # new
```

`track_buffer = 30` for rover matters because ground-perspective motion is smaller in pixel-space than aerial; a track held for 3 s is more likely to mis-match a different person walking through.

---

## Phase 6 — ByteTracker config + end-to-end test

**RINT-03:** ByteTracker knobs (`track_thresh`, `track_buffer`, `match_thresh`, `frame_rate`) become config-driven. Today they're hard-coded at `hailo_drone_detection_manager.py:1271`. After RINT-03:

```python
# Driven by config, not literals
tracker = ByteTracker(
    track_thresh = config.tracker.track_thresh,
    track_buffer = config.tracker.track_buffer,
    match_thresh = config.tracker.match_thresh,
    frame_rate   = config.tracker.frame_rate,
)
```

**RINT-04 — the deterministic end-to-end test:**

```python
def test_rover_follow_walk_across_then_approach():
    sim = start_rover_sim(world="walk_across_then_approach")
    try:
        app = launch_robot_follow(
            robot="rover",
            config="configs/rover_simulation.json",
            input="udp://0.0.0.0:5600",
            test_log="/tmp/rover_run.jsonl",
        )
        wait_for_first_detection(app, timeout=10.0)
        target_id = app.current_target_id()
        # let it run for the full walk pattern
        run_for(seconds=45)
        log = parse_test_log("/tmp/rover_run.jsonl")
        assert log.target_id_lost_events == 0
        assert log.final_target_id == target_id            # never swapped
    finally:
        app.shutdown(timeout=5.0)
        assert app.cmd_vel_messages_after_shutdown(window_ms=100) == 0   # RINT-06
        sim.shutdown()
```

The shutdown clause is **RINT-06** — zero further `/cmd_vel` messages within 100 ms of Ctrl+C; rover stops within 1 s. This is the only test that exercises the SIGINT-preservation pattern from Phase 4 in anger.

---

## Before / after architecture

```
                              BEFORE (today)                               AFTER (v1.1 ship)
              ┌─────────────────────────────────────┐  ┌─────────────────────────────────────┐
              │                                     │  │                                     │
              │  drone_follow_app.py                │  │  robot_follow_app.py                │
              │       │                             │  │       │                             │
              │       ▼                             │  │       │  pre-parse --robot          │
              │  run_drone()                        │  │       ▼                             │
              │       │                             │  │  run_robot(args)                    │
              │       ▼                             │  │       │                             │
              │  follow_api / pipeline_adapter      │  │       ├─► MavsdkDroneAdapter        │
              │       │  (unchanged)                │  │       │   (was drone_api/, now      │
              │       ▼                             │  │       │    robot_api/adapters/)     │
              │  VelocityCommand                    │  │       │                             │
              │       │                             │  │       └─► Ros2RoverAdapter (NEW)    │
              │       ▼                             │  │           │ rclpy + Twist           │
              │  drone_api.mavsdk_drone             │  │           │ /cmd_vel publish        │
              │       │                             │  │                                     │
              │       ▼                             │  │  follow_api / pipeline_adapter      │
              │  PX4 offboard ──► drone             │  │       │                             │
              │                                     │  │       ▼                             │
              │                                     │  │  RobotCommand                       │
              │                                     │  │       │                             │
              │                                     │  │       ▼ (capabilities-aware)        │
              │                                     │  │  PX4 offboard │ /cmd_vel + Gazebo   │
              └─────────────────────────────────────┘  └─────────────────────────────────────┘
                                                       │ NEW:                                │
                                                       │   robot_api/robot.py (protocol)     │
                                                       │   robot_api/adapters/ros2_rover.py  │
                                                       │   sim/rover/                        │
                                                       │   configs/rover_simulation.json     │
                                                       └─────────────────────────────────────┘
```

The pipeline / controller / ReID / servers / web UI **do not move**. Phase 1 renames their parent. Phase 2 cleans them up. Phase 3 swaps the actuator below them.

---

## Risks ranked

| # | Risk | Severity | Mitigation |
|---|------|----------|------------|
| 1 | **rclpy steals SIGINT** — drone-follow's graceful-shutdown handler is silently replaced; Ctrl+C stops publishing without commanding zero | HIGH (rover can keep moving) | `SignalHandlerOptions.NO` mandatory at `rclpy.init`; post-init assertion in `start_session`; covered by RINT-06 end-to-end test |
| 2 | `_rclpy_pybind11` import fails inside hailo-apps venv | MEDIUM (dev friction) | Defensive `import rclpy` raises friendly error; `setup_env.sh` auto-sources `/opt/ros/humble` after venv |
| 3 | Gazebo Garden silent SDF load failure on `ignition::` prefix | MEDIUM (Phase 5 debug time) | Documented in PITFALLS.md and `sim/rover/README.md`; `gz topic -l` validation step in `start_rover_sim.sh` |
| 4 | Camera path 15 Hz ceiling if ros_gz_bridge is used | MEDIUM (perf cliff) | Phase 5 reuses `video_bridge.py` (gz-transport direct), bypassing the bridge entirely |
| 5 | Phase 3's MavsdkDroneAdapter regression — drone path breaks | HIGH (loses field-deployed feature) | Critical gate: full SITL drone follow-the-person test must pass before Phase 4/5 start |
| 6 | `pip show drone-follow` keeps stale metadata on deployed units after Phase 1 rename | LOW (cosmetic) | `install.sh` runs `pip uninstall drone-follow -y` defensively |
| 7 | Two-pass argparse complexity — `--help` confusion | LOW | Smoke-tested in Phase 3; the four pre-parses sequence into one in CLEAN-12 |
| 8 | Garden EOL (Nov 2024); Harmonic migration eventually needed | LOW (sim-only) | SDF already uses `gz::` prefixes (Harmonic-compatible); documented in `sim/rover/README.md` |

Risks 1 and 5 dominate. Both are gated by tests that fail loudly if they're not addressed.

---

## Out of scope for v1.1

Explicit boundaries, with reasoning so nobody adds them mid-stream:

| Feature | Reason |
|---------|--------|
| Real rover hardware | Sim cannot validate hardware safety. Deferred to v1.2 with its own milestone. |
| Nav2 / Nav stack integration | Goal-based pathing, incompatible with reactive bbox-driven follow. Massive dep footprint. |
| `ros2_control` + hardware interface | Overkill for sim-only; expects `TwistStamped` not plain `Twist`, conflicting with our DiffDrive SDF. |
| SLAM / mapping | Orthogonal concern. Requires LiDAR or depth camera not on the BoM. |
| ROS 2 distros other than Humble | Iron is EOL Dec 2024; Jazzy has no Jammy binaries. Humble (EOL May 2027) is the only valid choice for Ubuntu 22.04. |
| `TwistStamped` variant | Not needed for plain DiffDrive SDF; would only matter for `ros2_control`. |
| Gazebo Harmonic migration | Garden is EOL but acceptable for sim-only; migration cost is low because SDF `gz::` prefixes are already Harmonic-compatible. Defer until forced. |
| Multi-robot simultaneous control | Single-target controller is the validated abstraction. Multi-robot fleet would require new state mediation. |
| Web UI "robot type" selector | Robot type is set at launch via `--robot`; mid-run switching would require a pipeline restart. |
| OpenHD integration for rovers | OpenHD is WFB/MAVLink; rover stack is ROS 2. Re-purposing OpenHD for rovers is its own milestone. |
| PX4 Rover via MAVSDK | Chose ROS 2 cmd_vel over MAVSDK Rover for v1.1; tracked as v2 HW-09 for evaluation later. |

These items are tracked in `.planning/REQUIREMENTS.md` under "Out of Scope" and "v2 Requirements."

---

## Where we are right now

```
Branch:       feature/rover-support  (5 commits ahead of main, all docs)
Codebase:     unchanged — drone_follow is still drone_follow today
Main:         still ships as drone_follow

Planning artefacts in this branch:
  .planning/PROJECT.md           ← v1.1 milestone declared
  .planning/REQUIREMENTS.md      ← 55 numbered requirements
  .planning/ROADMAP.md           ← 6 phases with success criteria
  .planning/research/            ← stack / features / arch / pitfalls / summary
  .planning/phases/01-rename/01-CONTEXT.md   ← Phase 1 decisions captured today

Next:         /gsd:plan-phase 1   →   produces 01-PLAN.md
              /gsd:execute-phase 1   →   actually performs the rename
```

The deck you're reading is itself an artefact of the planning step. It exists at `.planning/presentations/v1_1_overview.md` and is committed alongside the planning docs.

---

## Glossary (one-pager for the unfamiliar)

| Term | What it means here |
|------|---------------------|
| **Hailo-8L** | M.2 / PCIe AI accelerator chip. We use it for person detection via a HEF model file. |
| **HEF** | Hailo Executable Format — compiled neural net for the NPU. |
| **GStreamer** | C-based media pipeline framework. We build a pipeline of elements (source → tile → detect → tee → sinks) and a pad probe lets Python intercept buffers + metadata. |
| **MAVSDK** | Python (and C++) client lib for MAVLink, the standard drone-protocol stack. Talks to PX4. |
| **PX4** | Flight controller firmware running on the Cube Orange+. Accepts offboard velocity setpoints from MAVSDK. |
| **Offboard mode** | PX4 mode where an external companion computer streams setpoints. We use velocity-body setpoints. |
| **ByteTracker** | Multi-object tracker. Takes per-frame detections, returns tracks with stable IDs across frames. |
| **ReID** | Re-identification — embedding-based re-acquisition of a target after it disappears and reappears. |
| **OpenHD** | Open-source WFB (WiFiBroadcast) stack — high-resolution video + telemetry over long range using cheap WiFi adapters. We run it on the air and ground Pis. |
| **QOpenHD** | The ground-side GUI for OpenHD. Renders the video and the overlay HUD. |
| **WFB** | WiFiBroadcast — broadcast-mode WiFi packets, no association required, used by OpenHD. |
| **rclpy** | Python client for ROS 2. We use it in Phase 4 for the rover adapter. |
| **Gazebo Garden** | Sim engine (the rebranded Ignition Gazebo). Uses `gz::` prefixes; the `ignition::` prefix is a footgun. |
| **gz-transport** | Gazebo's pub/sub message transport. Camera streams flow over this. |
| **DiffDrive** | Differential-drive plugin in Gazebo — two-wheel rover physics + `/cmd_vel` sink. |
| **`/cmd_vel`** | ROS 2 standard topic name for a velocity command (`geometry_msgs/Twist`). |

---

## Open questions / things we'll learn during execution

These are deliberately not in CONTEXT.md — they're things only execution can answer, captured here so we don't forget them.

1. **rclpy SIGINT behaviour on the specific Humble version installed.** Documented to work with `SignalHandlerOptions.NO`, but worth smoke-testing in Phase 4 before the full adapter is written. (Captured as "Carried concern" in `.planning/STATE.md`.)
2. **Rover camera gz topic name.** SDF defines a camera sensor; Gazebo's actual topic name is conventionally `/model/<name>/sensor/<sensor>/image` but worth confirming with `gz topic -l` before hardcoding in `start_rover_sim.sh`. (Carried concern.)
3. **Whether `pyproject.toml` needs an explicit `[tool.pytest.ini_options]` testpaths update** after the rename, or whether discovery is package-relative enough that the `git mv` carries it. Spot-check during Phase 1.
4. **Whether the `drone-follow-dev` skill in `.claude/skills/` needs renaming or just internal text updates** — out of Phase 1 scope, but eventually decides.
5. **Field deployment first-merge story** — when `feature/rover-support` lands on `main`, what's the operator-visible behaviour for an air unit that pulls + reboots? Defensive `pip uninstall drone-follow -y` in install.sh covers it; smoke-test on a real unit before the merge.

---

## End

Questions, pushback, or "we should also..." → flag now. Phase 1 plan is the next step; everything in this deck is up for revision until then.

**Source files for further reading:**

- `.planning/PROJECT.md` — milestone declaration
- `.planning/REQUIREMENTS.md` — all 55 numbered requirements
- `.planning/ROADMAP.md` — phases, success criteria, dependency graph
- `.planning/research/SUMMARY.md` — research synthesis (start here for the rover side)
- `.planning/research/PITFALLS.md` — the gotchas list, ranked
- `.planning/phases/01-rename/01-CONTEXT.md` — Phase 1 decisions in detail
- `CLAUDE.md` — current-codebase reference for engineers
- `docs/tracking-reid-algorithm.md` — full ReID per-frame flow
