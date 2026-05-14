---
marp: true
theme: default
paginate: true
size: 16:9
style: |
  section { font-size: 22px; }
  h1 { font-size: 38px; }
  h2 { font-size: 32px; }
  pre, code { font-size: 16px; }
  table { font-size: 18px; }
---

<!-- _class: lead -->

# drone-follow

## design review + the rover branch

A follow-the-person app today; a robot-generic follow platform tomorrow.

<sub>Branch `feature/rover-support` • Internal eng review</sub>

---

## What this talk is

Two halves, roughly equal time.

**Part 1 — How the app works today.** Enough that everyone can read the codebase after this. We'll go layer by layer: camera, detection, tracking + ReID, controller, actuator.

**Part 2 — What's changing on `feature/rover-support`.** The thesis (only the actuator boundary moves), the six phases, the risky bits.

Goal: leave the room with a shared mental model + agreement on the phase order.

---

# Part 1 — How it works today

---

## The app in 60 seconds

A Hailo-based GStreamer pipeline detects people in a camera feed. The largest person becomes the follow target (AUTO), or the operator clicks one in the web UI (LOCKED). A control loop converts the target's bbox position + size into yaw / forward / altitude setpoints and streams them to PX4 via MAVSDK.

Three modes, one wire format, one hot path.

| Mode    | How you enter it                | Behavior                            |
|---------|---------------------------------|-------------------------------------|
| AUTO    | Default at boot                 | Follow the biggest person in frame  |
| LOCKED  | Click a person in the UI        | Follow that person, ReID-protected  |
| IDLE    | Ground station `follow_id = -1` | Hold position, ignore detections    |

---

## Hardware in the field

- **Compute** — Raspberry Pi 5 (8 GB) + Hailo-8L M.2
- **Camera** — IMX219 / IMX708 CSI, or USB webcam for dev
- **Flight controller** — Cube Orange+ over USB serial @ 57600 baud
- **Radio** — OpenHD WFB over TP-Link USB WiFi (`wlan1`)
- **Ground** — second RPi5 + QOpenHD, or a phone
- **Dev path** — built-in WiFi (`wlan0`) for SSH, simultaneous with WFB

Two WiFi interfaces, both up at once. We pin the TP-Link adapter to `wlan1` by MAC so the air unit always uses the right radio for WFB.

---

## Data flow — five layers, one direction

```
   ┌───────────────────────────────────────────────────────────┐
   │ 1. Camera frames @ 30 fps   (CSI / USB / UDP / SHM)        │
   └─────────────────────────┬──────────────────────────────────┘
                             │  GStreamer
   ┌─────────────────────────▼──────────────────────────────────┐
   │ 2. Hailo person detection  (tiled → unified bboxes)        │
   └─────────────────────────┬──────────────────────────────────┘
                             │  HailoDetection
   ┌─────────────────────────▼──────────────────────────────────┐
   │ 3. ByteTracker  +  ReID gallery / drift / raw fallback     │
   └─────────────────────────┬──────────────────────────────────┘
                             │  Detection (cx, cy, bbox_h)
   ┌─────────────────────────▼──────────────────────────────────┐
   │ 4. follow_api/controller   yaw + forward + altitude + safety │
   └─────────────────────────┬──────────────────────────────────┘
                             │  VelocityCommand  (3 floats)
   ┌─────────────────────────▼──────────────────────────────────┐
   │ 5. MAVSDK ► PX4 ► drone                                    │
   └────────────────────────────────────────────────────────────┘
```

Everything flows one way. No backpressure, no event loop crossing inside the hot path.

---

## Components & coupling — who depends on whom

```
       ┌─────────────────── THIRD-PARTY (we don't own) ────────────────────┐
       │  GStreamer · Hailo · ByteTracker · MAVSDK · Flask/SSE · picamera2 │
       └────────┬─────────────────┬──────────────────┬──────────────────┬─┘
                │ couples to      │ couples to       │ couples to       │
        ┌───────▼────────┐ ┌──────▼────────┐ ┌───────▼────────┐  ┌──────▼──┐
        │ pipeline_      │ │  drone_api/   │ │   servers/     │  │   ui/   │
        │   adapter/     │ │  mavsdk_drone │ │  web+openhd+   │  │ (Vite + │
        │ vision/reid/   │ │   .py         │ │   follow       │  │  React) │
        │ tracker/cb     │ │               │ │                │  │         │
        └───────┬────────┘ └──────┬────────┘ └───────┬────────┘  └────┬────┘
                │ writes          │ reads/sends      │ reads / writes │ HTTP
                │ Detection       │ Velocity         │ state          │
                ▼                 ▼                  ▼                │
        ┌──────────────────────────────────────────────────────────┐ │
        │              follow_api/   ◄── PURE CORE                 │◄┘
        │              (no third-party imports)                    │
        │                                                          │
        │    types.py ── VelocityCommand · Detection · RobotMode   │
        │       ▲                                                  │
        │       │                                                  │
        │   controller.py ──► state.py ◄── config.py               │
        │   (pure funcs)      (mutex)      (dataclass)             │
        └──────────────────────────────────────────────────────────┘
```

**All arrows point inward.** `follow_api/` has zero outgoing dependencies — it doesn't know GStreamer, MAVSDK, or Flask exist. Everything else is an adapter that translates a third-party world into `Detection` / `VelocityCommand`.

**The contact surface is two files** — `types.py` (the format) and `state.py` (the mutex). That's the entire seam.

This is why the rover work is tractable: only the `drone_api/` box swaps; nothing else in the diagram knows the difference.

---

## Threading model — three threads, narrow contact

```
  GStreamer thread ──► detection callback ──► writes shared state
        │                                            │
        │                                  (locks, mutated atomics)
        │                                            │
  asyncio loop  ──────► control loop ──► reads state ──► MAVSDK
                            │
                            └──► servers (Flask/SSE on their own threads)
```

- **GStreamer** owns the camera, Hailo, ByteTracker, ReID. The detection callback runs per frame.
- **Asyncio loop** owns the controller + MAVSDK adapter. Reads detection results via shared state.
- **HTTP servers** (web UI / OpenHD bridge / follow API) on their own threads. Talk to state, never to the callback.

The contact surface is `follow_api/state.py`. Mutex discipline lives there.

---

## Follow modes as a state machine

```
                   click in UI / follow_id=N
                ┌────────────────────────────┐
                │                            ▼
       ┌─────► AUTO ─────► (largest person each frame) ─┐
       │        │                                       │
  reset│        │ follow_id=-1                          │
       │        ▼                                       │
       │      IDLE ◄──────── follow_id=-1 ───────────── │
       │        │                                       │
       │        │ follow_id=0                           │
       │        └──────────────────────────────────────►│
       │                                               │
       │                              ┌──────────────────────┐
       │                              ▼                      │
       └─────────── LOCKED ─────► ReID gallery, drift guard, │
                     ▲             raw-detection fallback    │
                     │                      │                │
                     └── ReID timeout (20s) ┘  (gallery clears, back to AUTO)
```

`follow_id` is the wire: `-1`=IDLE, `0`=AUTO, `N`=LOCKED to track N.

---

## The controller — what each loop actually computes

| Channel    | Input                              | Output            | Type                 |
|------------|------------------------------------|-------------------|----------------------|
| Yaw        | bbox horizontal offset from centre | rad/s body yaw    | P-loop               |
| Forward    | bbox height vs target size         | m/s forward       | P-loop, gated        |
| Altitude   | current `down` vs `--target-altitude` | m/s down       | P-loop, **drone-only** |
| Safety     | bbox bottom near frame edge        | retreat / clip    | **guard** (not P)    |

All of these are pure functions in `follow_api/controller.py` over the latest `Detection`. The MAVSDK adapter is what turns the 3-float command into PX4 setpoints.

Two of these four channels are drone-specific. That's a leak — we come back to it in Part 2.

---

## The detection callback — the hot path

Runs once per frame on the GStreamer thread. Order matters:

```
   1. parse Hailo metadata → list of person detections
   2. unify tile detections (NMS across tiles, dedup)
   3. ByteTracker.update(detections) → tracks with ids
   4. follow-mode decision:
        AUTO   → pick biggest track
        LOCKED → look for the locked id
                 if missing: try_reidentify() on visible tracks
                             else score_visible_persons() on raw
                             else hold
   5. update shared state with chosen Detection
   6. (every N frames) sample ReID embedding for drift / enrichment
```

Step 6 is the only "slow" step (HailoCropper + embedding HEF). It runs every `--update-interval` frames (default 30). Steps 1–5 are deterministic and bounded.

---

## ReID — three similarity bands

```
              cosine similarity to gallery
   0 ───────────── 0.6 ────────── 0.9 ───────── 1.0
        DRIFT       │  ENRICHMENT  │  DUPLICATE
                    │              │
   • don't store    │ • add to     │ • skip
   • re-acquire     │   gallery    │ • every 5th: refresh
     immediately    │   (FIFO)     │   oldest entry
```

- Below `--reid-drift-threshold` (0.6) → suspected tracker drift; re-acquire on visible tracks; if a different track wins, switch the lock.
- Between 0.6 and 0.9 → normal enrichment; FIFO-replace when gallery is full.
- Above 0.9 → near-duplicate; skip but periodically refresh so the gallery doesn't go stale.

The first 6 samples bypass the drift gate — single-seed drift detection is too brittle.

**Raw-detection fallback:** if ByteTracker activates zero tracks for the locked person but raw `persons` are visible, ReID scores them directly. Locked-follow survives tracker dropouts.

---

## Vision branches — what comes out

The pipeline fans out into up to four output branches built in `vision_branches.py`:

```
                              ┌─► display    (X11 + tile-strip + highlight)
                              │
   Hailo + overlay ─► tee ────┼─► record     (x264 → matroska on disk)
                              │
                              ├─► webui      (MJPEG to /stream)
                              │
                              └─► openhd     (x264 → RTP UDP to OpenHD)
```

- **Implicit-display rule** — `--display` defaults ON only when no UI flag is set.
- **`--webui` and `--openhd` are mutually exclusive** — both want the encoded stream.
- Tile rectangles + locked-target highlighting are done in a pure-metadata pad probe (`highlight_target` in `vision_branches.py`). No pixel copies for overlay decisions.

---

## Operational shape

**Boot service on the air unit.** systemd → `drone-follow-boot.sh` → reads `~/Desktop/drone-follow.conf` → if `ENABLED=true`, runs `scripts/start_air.sh`. `MODE=stream|shm` selects who owns the camera.

**OpenHD camera modes:**
- **Mode A (stream)** — drone-follow owns the CSI camera, encodes, pushes RTP to OpenHD.
- **Mode B (shm)** — OpenHD owns the camera, tees raw NV12 to SHM, drone-follow reads it.

**Dev path** — `drone-follow --input usb --webui` opens a USB webcam, serves the web UI on :5001, no OpenHD. Or `--input udp://0.0.0.0:5600` for Gazebo SITL.

**Ground unit** — pure `openhd --ground` + QOpenHD GUI. No Python, no Hailo, no drone-follow code.

---

# Part 2 — What changes on `feature/rover-support`

---

## The thesis

> Only the **actuator boundary** moves.

Camera, Hailo, ByteTracker, ReID, controller, web UI, OpenHD, recording — **unchanged**.

We're replacing one box at the bottom of the diagram.

```
   today                                  v1.1
   ─────                                  ────
        ... pipeline ...                       ... pipeline ...
              │                                       │
              ▼                                       ▼
      VelocityCommand                          RobotCommand
              │                                       │
              ▼                          ┌──── Robot protocol ────┐
       MAVSDK ► PX4                      ▼                        ▼
                                  MavsdkDroneAdapter       Ros2RoverAdapter
                                         │                        │
                                         ▼                        ▼
                                    PX4 ► drone           rclpy ► /cmd_vel
                                                                  ► Gazebo rover
```

If we keep that promise, drone behavior cannot regress.

---

## Where "drone" leaks today

11 sites where the drone-only assumption is baked in:

| Where                                  | Leak                              |
|----------------------------------------|-----------------------------------|
| `drone_follow_app.py`                  | `run_drone()`; drone args always parsed |
| `follow_api/types.py`                  | `VelocityCommand.down_m_s` is mandatory |
| `follow_api/controller.py` (×3)        | altitude P unconditional; bottom-edge=retreat-from-tilt baked in; yaw-spin on loss baked in |
| `follow_api/config.py`                 | altitude fields hard-coded floats |
| `drone_api/mavsdk_drone.py`            | yaw deg/s baked in                |
| CLI                                    | `--takeoff-landing` / `--target-altitude` / `--serial` always visible in `--help` |
| `hailo_…manager.py`                    | ByteTracker knobs hard-coded      |
| `web_server` + `openhd_bridge`         | parallel tunable-field lists      |
| Branch-decision tree                   | 3× implicit-display rule duplications |

55 numbered requirements (RENAME / CLEAN / ABS / ROVER / RSIM / RINT) — see `.planning/REQUIREMENTS.md`.

---

## The shape of the change — `Robot` + `Capabilities`

The controller does not know what it's moving. It knows **which axes exist** and **what units they take**. That's it.

```
   class Axis(Enum):                       @dataclass
       FORWARD  = "forward"   # body x      class Capabilities:
       YAW      = "yaw"       # body z          axes: frozenset[Axis]
       ALTITUDE = "altitude"  # body z (down)   yaw_unit: Literal["rad/s","deg/s"]
                                                # … purely mechanical. No policy.

   class Robot(Protocol):                  class MavsdkDroneAdapter(Robot):
       async def connect(self): ...            caps = Capabilities(
       async def start_session(self): ...          axes={FORWARD, YAW, ALTITUDE},
       async def send(self, cmd): ...              yaw_unit="deg/s")
       async def shutdown(self): ...           # adapter-internal: retreat-from-tilt,
       caps: Capabilities                      #                    arm/takeoff/land

   @dataclass                              class Ros2RoverAdapter(Robot):
   class RobotCommand:                         caps = Capabilities(
       forward_m_s: float = 0.0                    axes={FORWARD, YAW},
       yaw_rate:    float = 0.0                    yaw_unit="rad/s")
       down_m_s:    float = 0.0                # adapter-internal: slow-near-edge
       # adapter reads only caps.axes
```

```
   controller.compute(detection, caps, config) -> RobotCommand
      ├── if YAW      in caps.axes:  cmd.yaw_rate    = yaw_p(...)
      ├── if FORWARD  in caps.axes:  cmd.forward_m_s = forward_p(...)
      └── if ALTITUDE in caps.axes:  cmd.down_m_s    = alt_p(...)
```

**What's NOT in `Capabilities`:** `bottom_edge_policy`, `yaw_spin_on_loss`, `supports_takeoff`. Those are robot-specific behaviors — they live inside the adapter, not in the controller's vocabulary.

---

## Coupling — before / after

The whole milestone, in one diagram:

```
   TODAY                                       v1.1
   ─────                                       ────

   controller.py                               controller.py
        │                                            │
        │ constructs VelocityCommand                 │ constructs RobotCommand
        │ (down_m_s mandatory)                       │ (down_m_s optional, gated on caps)
        │                                            │
        ▼                                            ▼
   ┌─────────────────────┐                  ┌───────────────────────────┐
   │  drone_api/         │                  │      Robot  (protocol)     │
   │  mavsdk_drone.py    │                  │  + Capabilities (dataclass)│
   │                     │                  └────────────┬───────────────┘
   │  • assumes altitude │                   ┌───────────┴────────────┐
   │  • yaw in deg/s     │                   │                        │
   │  • bottom = retreat │                   ▼                        ▼
   │  • takeoff/land     │           MavsdkDroneAdapter        Ros2RoverAdapter
   └──────────┬──────────┘             (drone_api → robot_api)  (rclpy + Twist)
              │                                  │                        │
              ▼                                  ▼                        ▼
             PX4                                PX4                    /cmd_vel
                                                                       Gazebo / rover

   COUPLING                                    DECOUPLING
   ────────                                    ───────────
   • follow_api knows it's a drone:            • follow_api knows axes only:
       - altitude always present                   - "which of {FWD, YAW, ALT}?"
       - yaw is deg/s                              - "what units does YAW take?"
       - bottom-edge = retreat-from-tilt       • behaviors (retreat-from-tilt,
       - takeoff/land are flags                  slow-near-edge, takeoff/land,
                                                 yaw-spin-on-loss) live IN the
                                                 adapter, not in the controller
   • CLI always parses --takeoff-landing       • two-pass argparse hides
     even on rover                               drone-only flags on --robot rover
   • drone_api lives under drone-named pkg     • drone_api → robot_api/adapters/
```

We're **not** rewriting the controller — we're shrinking what it knows.

Before: `controller → MAVSDK (via VelocityCommand with drone-shaped fields, and drone-shaped policy baked into the controller itself)`

After: `controller(detection, axes) → RobotCommand → {MavsdkDrone | Ros2Rover}` — the controller is robot-agnostic; behaviors live where they belong, in the adapter.

The follow_api stays "no third-party imports" — and now also "no robot-shaped assumptions".

---

## Phase order — six phases, one critical gate

```
   Phase 1 ── Rename                Phase 2 ── Cleanup
   drone_follow → robot_follow      18 items: dead code, dups,
   alias preserved                  hot-path races (CLEAN-16 SSE)
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
   SIGINT-safe thread        cmd_vel + video_bridge.py
            │                       │
            └───────────┬───────────┘
                        ▼
                Phase 6 ── Sim integration
                rover-safe defaults, bottom-edge repurpose,
                ByteTracker config, end-to-end test
```

Phases 4 & 5 are independent once Phase 3 lands → parallelisable.

---

## Phase 1 — Rename

**Goal:** the package is `robot_follow`; `drone-follow` and `robot-follow` both work; no field deployment breaks.

Single mechanical commit:

```
git mv drone_follow robot_follow
   ├─ 48 internal imports rewritten
   ├─ pyproject.toml: name="robot-follow"; scripts: robot-follow + drone-follow alias
   ├─ install.sh: pip uninstall drone-follow -y  (defensive)
   ├─ shell scripts: setup_env.sh / install_air.sh / start_air.sh
   └─ git grep gate: drone_follow imports → only docs/history whitelist
```

**Stays put:** `drone_api/` (renames in Phase 3), boot unit file on disk, `~/Desktop/drone-follow.conf`. Branch-scoped — only on `feature/rover-support`; main stays shippable.

---

## Phase 2 — Cleanup

18 items, three buckets. Audited pre-milestone — no archaeology during execution.

| Dead code (10)               | Duplication (5)                | Hot-path (3)                          |
|------------------------------|--------------------------------|---------------------------------------|
| `sim/world_loader.py`        | mavsdk_server pkill helper     | **CLEAN-16** SSE race                 |
| `bench_reid_callback.py`     | 3× argparse pre-parsers → 1    | socket-per-call → reuse               |
| `--vfov` / `--mission-duration` | telemetry position+alt tasks | O(n) dedup → O(1) dict                |
| stale fallbacks + aliases    | `tunable_fields()` unification |                                       |
|                              | branch-decision tree unified   |                                       |

**The race that matters — CLEAN-16:** on `_on_frame` we currently `set()` then immediately `clear()` an Event. With two SSE consumers, one can race through the gap and fall through to the 2 s timeout, showing a black tab. Fix: replace with a `Condition` + monotonic `frame_seq`; consumers wait on `seq != last_seen`.

---

## Phase 3 — Abstraction (the critical gate)

This is where the protocol lands. **Drone SITL must pass end-to-end on this phase**; if it doesn't, no rover work starts.

Concrete artefacts:

- `robot_api/robot.py` — `Robot` protocol, `Capabilities`, `RobotCommand`
- `robot_api/adapters/mavsdk_drone.py` — moved from `drone_api/`
- `--robot {drone,rover}` CLI flag, default `drone`
- **Two-pass argparse**: pre-parse `--robot`, then build the parser with only the relevant subset of flags. `--robot drone --help` shows `--takeoff-landing`; `--robot rover --help` does not.
- `run_robot()` composition root replaces `run_drone()`

Side benefit: the controller becomes a pure function of `(Detection, Capabilities, Config) → RobotCommand`. No robot type leaks in. Easier to unit-test than today, and rover-ready by construction.

---

## Phase 4 — Rover adapter

**Goal:** `Ros2RoverAdapter` publishes `geometry_msgs/Twist` on `/cmd_vel`; the asyncio loop is happy; SIGINT is preserved.

```
   asyncio loop ──► adapter.send(RobotCommand)
                            │
                            ▼  (queue or direct publish)
                  rclpy executor thread ──► /cmd_vel  Twist{linear.x, angular.z}
```

**The SIGINT trap** — `rclpy.init()` installs its own `SIGINT` handler. If we let it, Ctrl+C kills rclpy first, the asyncio loop never sees it, and we leak `/cmd_vel` setpoints after the user thinks the app is dead.

Fix: snapshot drone-follow's handler, init rclpy, restore the handler, assert `signal.getsignal(SIGINT) is on_signal`. ROVER-02 + ROVER-08 are explicit tests for exactly this.

**Friendly failure** — on a machine without `/opt/ros/humble`, `--robot rover` must raise a readable `RuntimeError("ROS 2 not sourced")`, not an `ImportError` traceback.

---

## Phase 5 — Rover sim

**Goal:** `sim/rover/start_rover_sim.sh` launches a Gazebo Garden diff-drive rover; cmd_vel arrives over ROS; camera reaches drone-follow on UDP 5600 — same path as PX4 SITL today.

```
   ROS  ─Twist─► ros_gz_bridge ─►  Gazebo (diff-drive plugin)
                                       │
                                       ▼
                              rover camera (gz topic)
                                       │
                              video_bridge.py  (gz-transport → UDP H.264)
                                       │
                                       ▼
                              drone-follow --input udp://0.0.0.0:5600
```

**Garden silent failures (each one bit us in research):**

- `ros-humble-ros-gzgarden-bridge` (Garden) ≠ `ros-humble-ros-gz-bridge` (Fortress). Wrong package = no errors, no bridge.
- SDF plugin filename `gz-sim-diff-drive-system`; `ignition::` prefix = silent load failure.
- Use the existing `video_bridge.py`, not `ros_gz_image_bridge` — same path the drone sim uses today.

---

## Phase 6 — Sim integration

End-to-end test, rover-safe defaults, and the last leaky drone assumption.

- `configs/rover_simulation.json` — no altitude knobs, `track_buffer ≈ 30` (rover sees the target longer through occlusions), lower `kp_yaw`.
- `configs/drone_simulation.json` — unchanged.
- **Bottom-edge safety moves out of the controller.** Controller emits `forward=0` when person is below safe zone — that's all it knows. The drone adapter additionally applies retreat-from-tilt; the rover adapter applies slow / stop. No `bottom_edge_policy` flag in `follow_api`.
- ByteTracker config-driven (RINT-03) — was hard-coded.

**The integration test (RINT-04):**

```
   start_rover_sim --world walk_across_then_approach
   drone-follow --robot rover --config configs/rover_simulation.json
   → rover follows the walking actor through the full walk pattern
   → Ctrl+C produces zero /cmd_vel within 100 ms  (RINT-06)
```

---

## Risks, ranked

| Rank | Risk                                            | Mitigation                                |
|------|-------------------------------------------------|-------------------------------------------|
| 1    | rclpy SIGINT handler leaks `/cmd_vel` on Ctrl+C | snapshot/restore + ROVER-02 assertion     |
| 2    | Two-pass argparse breaks an obscure flag combo  | golden `--help` snapshot tests per `--robot` |
| 3    | Gazebo Garden silent failures eat a day each    | research notes in phase plan; checked-in `gz topic -l` baseline |
| 4    | CLEAN-16 fix changes SSE timing → web UI flake  | second-tab MJPEG test before merge        |
| 5    | The rename misses an internal `from drone_follow` import in a script | `git grep` gate in CI for the milestone branch |
| 6    | Drone SITL regression hidden until Phase 6      | gate at Phase 3 — full drone SITL must pass before any rover work starts |

---

## Out of scope (v1.1)

- **Real rover hardware** — sim-only. Hardware is v1.2.
- **Multi-robot / fleet** — one app, one robot, always.
- **Path planning** — still a reactive follow controller. No waypoints.
- **Indoor / GPS-denied** — drone path still assumes PX4 + GPS.
- **Persistent ReID across runs** — gallery is in-memory only.
- **New detection model** — same Hailo person detector.

These are open in `MILESTONES.md` for v1.2+; flag them if a Phase 3 design choice closes the door on any.

---

## Where we are now / questions for the room

- ✅ Roadmap + 55 requirements in `.planning/REQUIREMENTS.md`
- ✅ Phase 1 plan being drafted (`.planning/phases/01-rename/`)
- ⬜ Phases 2–6 plans — TBD, one per phase
- ⬜ ROS 2 Humble on at least one dev machine — needed before Phase 4 starts

**Open questions:**

1. Do we want a `--robot=auto` mode that probes `/opt/ros/humble` and the MAVLink wire to guess? (Default: no — explicit is better here.)
2. Should `Capabilities` be a dataclass or a TypedDict? (Leaning dataclass for `match` patterns in the controller.)
3. Phase 4 vs Phase 5 — who owns the SDF / world files? Sim or adapter dir?
