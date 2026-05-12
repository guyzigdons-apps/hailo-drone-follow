# Project Research Summary

**Project:** hailo-drone-follow v1.1 — Robot abstraction + rover support (sim-only)
**Domain:** ROS 2 cmd_vel rover adapter, Gazebo Garden differential-drive sim, rclpy-asyncio integration
**Researched:** 2026-05-12
**Confidence:** HIGH (stack packages verified on-machine), MEDIUM (rclpy threading patterns from community sources)

## Executive Summary

v1.1 generalizes drone-follow from a PX4-only app into a robot-generic follow platform. The existing
`follow_api/controller.py` already produces a robot-agnostic `VelocityCommand(forward, down, yawspeed)`;
v1.1 formalizes this with a `Robot` protocol and a new `Capabilities` struct, wraps the MAVSDK drone
behind it, and adds a ROS 2 Humble adapter that publishes `geometry_msgs/Twist` on `/cmd_vel` for
differential-drive rovers. A Gazebo Garden rover sim (`sim/rover/`) completes the end-to-end loop.
All four research areas agree on ROS 2 Humble as the only viable distribution for Ubuntu 22.04 and on
a background-thread rclpy executor to bridge rclpy's blocking `spin()` into the existing asyncio event
loop.

The recommended approach is a 6-phase implementation ordered by dependency: stabilize the Robot
protocol and rename `VelocityCommand` first (no behaviour change), then clean config, then wrap the
drone adapter, then build the rover adapter, then build the rover sim, then do end-to-end validation.
The critical gate is Phase 3 — the Robot protocol must be validated with the drone path before either
adapter diverges. The rover sim camera plumbing should reuse the existing `sim/bridge/video_bridge.py`
gz-transport to UDP H.264 path, not a ROS Image bridge (see Camera Plumbing Decision below).

The primary risk area is rclpy-asyncio integration: `rclpy.init()` installs a SIGINT handler by default
that will silently override drone-follow's custom Ctrl+C landing sequence. This must be treated as a
hard design requirement (`signal_handler_options=SignalHandlerOptions.NO`), not an optional tweak. A
second risk is venv/PYTHONPATH ordering: the hailo-apps venv must be activated before
`/opt/ros/humble/setup.bash` is sourced, and `setup_env.sh` must be extended to auto-source ROS 2
when present.

## Key Findings

### Recommended Stack

All new dependencies are available as Ubuntu 22.04 apt binaries and have been verified on the dev
machine. ROS 2 Humble is the only LTS that targets Jammy as its primary platform (Jazzy has no Jammy
binaries; Iron is EOL since Dec 2024). The ros_gz Garden bridge uses a distinct package name
(`ros-humble-ros-gzgarden-bridge`) that conflicts with the Fortress bridge — do not install both.

**Core new technologies:**
- `ros-humble-ros-base` + `ros-humble-geometry-msgs`: rclpy + Twist type — minimal non-GUI install; no ros-desktop
- `ros-humble-ros-gzgarden-bridge` (v0.244.11, apt-verified): ROS to Gazebo Garden cmd_vel bridge — Garden-specific, NOT `ros-humble-ros-gz` (Fortress)
- `gz-garden` (v1.0.0, apt-verified): sim runtime with `libgz-sim7-plugins` including DiffDrive; already a project prerequisite
- rclpy `SingleThreadedExecutor` on background thread: bridges blocking `spin()` into asyncio — `publisher.publish()` is thread-safe in Humble

**Critical version and naming constraints:**
- `gz.msgs.Twist` prefix in bridge config (NOT `ignition.msgs.Twist`) — wrong prefix causes silent bridge failure
- `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system` in SDF — `ignition::` prefix causes silent load failure on Garden
- `setup_env.sh` must auto-source `/opt/ros/humble/setup.bash` after venv activation (venv first, then ROS)

### Expected Features

All v1.1 features are P1 low-complexity items. The full feature list is in
`.planning/research/FEATURES.md`.

**Must have (v1.1 table stakes):**
- `ros2_rover.py` adapter: background-thread rclpy node publishes `geometry_msgs/Twist` to `/cmd_vel`
- `--robot rover` CLI flag with two-pass argparse pre-parse so rover users never see drone flags in `--help`
- Gazebo Garden rover SDF with `DiffDrive` plugin and camera sensor (`sim/rover/`)
- `ros_gz_bridge` for cmd_vel (ROS to Gazebo direction only)
- `video_bridge.py` reuse pointing `--topic` at rover camera gz topic
- `sim/rover/start_rover_sim.sh` one-command launch (mirrors `start_sim.sh` pattern)
- `configs/rover_simulation.json` with rover-safe defaults (lower speeds, tuned frame-edge margins)

**Defer to v1.2:**
- Real rover hardware path (same adapter, different `/cmd_vel` target — no code change expected)
- E-stop topic subscription for hardware safety
- twist_mux teleop override integration
- `/odom` subscription for closed-loop distance estimation

**Explicitly excluded (anti-features):**
- Nav2 integration: massive dependency footprint; incompatible with real-time bbox-driven follow
- ros2_control + hardware interface: overkill for sim-only, also expects TwistStamped not plain Twist
- SLAM / mapping: orthogonal concern, requires LiDAR or depth camera

### Camera Plumbing Decision: video_bridge.py reuse (NOT ros_gz_image_bridge)

**Recommendation: use the existing `sim/bridge/video_bridge.py` (gz-transport direct subscription to
UDP H.264) for the rover sim camera, not a ROS Image bridge.**

FEATURES.md and ARCHITECTURE.md disagreed on this point. ARCHITECTURE.md proposed
`ros_gz_image_bridge` to `sensor_msgs/Image` ROS topic to a new Python shim with rclpy subscription.
FEATURES.md proposed reusing `video_bridge.py` directly via gz-transport. The FEATURES.md
recommendation is stronger for these reasons:

1. Already validated: `video_bridge.py` does gz-transport to H.264 UDP today for the drone sim. Zero new infrastructure.
2. No throughput ceiling: The `ros_gz_bridge` image path has a documented ~15 Hz publish ceiling in Gazebo Garden (GitHub `gazebosim/ros_gz#368`, also cited in PITFALLS.md). `video_bridge.py` via gz-transport has no such limit.
3. Lower latency: The ROS bridge path adds an extra hop (gz to ROS Image topic to consumer). Direct gz-transport subscription is the shortest path.
4. No ROS dependency on camera path: the ROS bridge approach requires rclpy running before the camera feeds — tightly coupling two separate processes.

The ARCHITECTURE.md itself notes "Alternative if ros_gz_bridge has issues: the gz-transport direct
subscription path... also works for the rover sim" — effectively conceding the FEATURES.md
recommendation. PITFALLS.md confirms the 15 Hz issue is real (Integration Gotchas table row for
"DiffDrive + bridge").

**Conclusion:** `sim/rover/` does NOT need a new `camera_udp_shim.py`. `start_rover_sim.sh` passes
the rover camera's gz topic to the existing `sim/bridge/video_bridge.py`.

### Architecture Approach

The existing architecture's clean layer boundaries make v1.1 surgical. The composition root
(`drone_follow_app.py`) dispatches to a `Robot` protocol object; the pipeline adapter, follow
controller, and servers are untouched. The MAVSDK drone code moves behind
`MavsdkDroneAdapter.start_session()`; the rover adapter wraps the rclpy publish loop in the same
async pattern. A single `run_robot()` replaces `run_drone()`.

**Major components and change scope:**
1. `robot_api/robot.py` (NEW): `Robot` protocol, `Capabilities`, `RobotCommand` (replaces `VelocityCommand`)
2. `robot_api/adapters/mavsdk_drone.py` (MOVED + wrapped): existing MAVSDK code behind `start_session()`
3. `robot_api/adapters/ros2_rover.py` (NEW): rclpy Node, `SingleThreadedExecutor` background thread, sync `publisher.publish()` from asyncio
4. `follow_api/config.py` (LIGHT CHANGE): altitude fields to `Optional[float]`, gated by `capabilities.has_altitude`
5. `sim/rover/` (NEW): `rover.sdf`, `start_rover_sim.sh`, `worlds/`, `configs/rover_simulation.json`
6. `drone_follow_app.py` (COMPOSITION ROOT): `run_robot()` replaces `run_drone()`, two-pass CLI pre-parse

**Thread model (unchanged):** Main thread owns GStreamer; `robot_thread` owns asyncio loop and `Robot.start_session`; `rclpy-spin` thread runs `SingleThreadedExecutor.spin_once` loop; HTTP threads own servers.

### Critical Pitfalls

1. **rclpy SIGINT handler override (DESIGN REQUIREMENT, not optional):** `rclpy.init()` installs its own SIGINT handler by default, silently overriding drone-follow's landing sequence. Hard requirement: always call `rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)`. Verify post-init with `signal.getsignal(signal.SIGINT)`.

2. **`_rclpy_pybind11` import fails inside venv:** If `setup_env.sh` is sourced without ROS, `import rclpy` either fails or finds the rclpy Python package but not the compiled C extension. Add a defensive import guard with a helpful error message; extend `setup_env.sh` to auto-source ROS 2 if present. Order: venv first, then ROS.

3. **Blocking `rclpy.spin()` in background thread:** `rclpy.spin(node)` blocks forever — the `shutdown` event is never checked. Use `executor.spin_once(timeout_sec=0.05)` in a loop checking `shutdown.is_set()`. Never use `executor.spin()` (no-timeout variant) in a stoppable thread.

4. **asyncio.Event cross-thread signalling:** `asyncio.Event` is not thread-safe for cross-thread `set()`. The rover adapter must use `threading.Event` for shutdown signalling or call `loop.call_soon_threadsafe` with a captured loop reference. Decide this at the Robot protocol boundary (Phase 1) before writing adapter code.

5. **DiffDrive SDF plugin name is era-specific:** `ignition::gazebo::systems::DiffDrive` causes a silent load failure on Garden. Use `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system`. Validate with `gz topic -l` before writing any Python.

6. **ros_gz_bridge topic name mismatch:** DiffDrive plugin defaults to `/model/<name>/cmd_vel` in Gazebo. Override with `<topic>cmd_vel</topic>` in SDF to keep bridge config trivial (`/cmd_vel` on both sides).

## Implications for Roadmap

### Phase 1: Robot Protocol + Rename
**Rationale:** Everything else depends on the `Robot` protocol being stable. Rename `VelocityCommand` to `RobotCommand` here to unblock both adapters. No behaviour change — safe to land and test with the drone path alone.
**Delivers:** `robot_api/robot.py` with `Robot`, `Capabilities`, `RobotCommand`; rename in `types.py` and `controller.py`; `setup_env.sh` ROS auto-source; asyncio.Event vs threading.Event decision documented at the boundary.
**Avoids:** asyncio cross-thread Event deadlock (Pitfall 4) — decide signalling type before any adapter code.

### Phase 2: Config Cleanup
**Rationale:** Altitude fields must be Optional before either adapter reads them; rover tests will fail at config validate otherwise. Light, self-contained change with no user-visible effect.
**Delivers:** Altitude fields `Optional[float]` in `ControllerConfig`; `capabilities.has_altitude` gate in `live_control_loop`; drone path unchanged.

### Phase 3: Drone Adapter Behind Protocol
**Rationale:** Critical gate. The Robot protocol is only proven when the existing drone path runs end-to-end behind `MavsdkDroneAdapter.start_session()`. Must pass before the rover adapter is written.
**Delivers:** `drone_api/mavsdk_drone.py` moved to `robot_api/adapters/mavsdk_drone.py`; `run_robot()` in `drone_follow_app.py`; two-pass CLI pre-parse (`--robot` sentinel).

### Phase 4: Rover Adapter
**Rationale:** Protocol is stable (Phase 3 passed). This is the core new work — publish-only rclpy node with asyncio bridge.
**Delivers:** `robot_api/adapters/ros2_rover.py` with background `SingleThreadedExecutor` spin loop; `--robot rover` CLI flag; `add_rover_args()`; deg/s to rad/s conversion at adapter boundary.
**Uses:** `ros-humble-ros-base`, `ros-humble-geometry-msgs`, `SignalHandlerOptions.NO` (mandatory).
**Avoids:** SIGINT override (Pitfall 1), blocking spin (Pitfall 3), `_rclpy_pybind11` import failure (Pitfall 2).

### Phase 5: Rover Sim
**Rationale:** Adapter exists; now build the sim it connects to. Keep `sim/rover/` parallel to `sim/` with no shared entrypoint.
**Delivers:** `sim/rover/rover.sdf` with `gz::sim::systems::DiffDrive` and camera sensor; `start_rover_sim.sh` launching gz sim + ros_gz_bridge (cmd_vel only) + existing `video_bridge.py` for camera; `configs/rover_simulation.json`.
**Uses:** `ros-humble-ros-gzgarden-bridge` for cmd_vel; `video_bridge.py` for camera (NOT ros_gz_image_bridge).
**Avoids:** DiffDrive name confusion (Pitfall 5), bridge topic mismatch (Pitfall 6), 15 Hz camera ceiling.

### Phase 6: End-to-End Integration Test
**Rationale:** Both halves exist; validate the full loop, tune rover behavior, check all items from the PITFALLS.md "Looks Done But Isn't" checklist.
**Delivers:** Rover following walking actor in Gazebo Garden; rover config values validated; port isolation from PX4 SITL confirmed; zero cmd_vel messages after shutdown confirmed.

### Phase Ordering Rationale
- Phases 1 through 3 are no-new-behaviour changes that can be reviewed and merged quickly with high confidence.
- Phase 3 is the only hard gate: drone path must pass before Phase 4 starts.
- Phases 4 and 5 are independent once Phase 3 lands and could be parallelized.
- Phase 6 is deliberately deferred until both adapter and sim exist.

### Research Flags

Phases with standard patterns (research not needed during planning):
- **Phase 1:** Standard Python Protocol + dataclass — no external dependencies.
- **Phase 2:** Dataclass Optional field gating — well-understood.
- **Phase 3:** Refactor of existing code, no new dependencies.
- **Phase 5 (camera path):** `video_bridge.py` is validated; DiffDrive SDF is documented.

Phases needing targeted validation during implementation:
- **Phase 4 (rover adapter):** Smoke-test `SignalHandlerOptions.NO` SIGINT behavior on this specific Humble version early, before full adapter implementation.
- **Phase 5 (rover SDF bring-up):** Run `gz topic -l` to confirm DiffDrive plugin loaded and verify rover camera gz topic name before hardcoding in `start_rover_sim.sh`.

## Confidence Assessment

| Area | Confidence | Notes |
|------|------------|-------|
| Stack | HIGH | All package names and versions verified via apt-cache on dev machine |
| Features | HIGH (Twist conventions) / MEDIUM (camera path) | Camera recommendation grounded in GitHub issue evidence; not yet smoke-tested |
| Architecture | HIGH | Based on direct codebase read; rclpy thread-safety confirmed in rclpy source |
| Pitfalls | MEDIUM | SIGINT and threading findings from GitHub issues and community reports; need early smoke-test |

**Overall confidence:** HIGH for implementation decisions; MEDIUM for rclpy signal and threading edge cases.

### Gaps to Address

- **SIGINT behavior under Humble specifically:** `SignalHandlerOptions.NO` is documented but should be smoke-tested in Phase 4 before full adapter build. Do not discover this in integration.
- **Rover camera gz topic name:** The actual gz-transport topic published by the rover SDF camera (`/model/rover/camera` vs `/camera`) needs confirmation when `rover.sdf` first loads — check with `gz topic -l` before hardcoding in `start_rover_sim.sh`.
- **Gazebo Garden EOL documentation:** Garden is EOL since Nov 2024. Rover SDF must use `gz::` prefixes throughout (already required for Garden) so migration to Harmonic is low-cost. Document migration notes in `sim/rover/`.

## Sources

### Primary (HIGH confidence)
- `apt-cache show ros-humble-ros-gzgarden-bridge` on dev machine — version 0.244.11-1002jammy confirmed
- `apt-cache show gz-garden` on dev machine — version 1.0.0-1~jammy, DiffDrive in libgz-sim7-plugins confirmed
- `hailo-apps/venv_hailo_apps/pyvenv.cfg` — `include-system-site-packages = true`, Python 3.10.13
- https://endoflife.date/ros-2 — Humble EOL May 2027; Iron EOL Dec 2024; Jazzy targets Noble 24.04
- https://gazebosim.org/docs/garden/ros2_integration/ — bridge syntax, Garden-era plugin names
- https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/publisher.py — `rcl_publish` thread-safety confirmed in source

### Secondary (MEDIUM confidence)
- https://github.com/gazebosim/ros_gz/issues/368 — ~15 Hz camera publish ceiling with ros_gz_bridge in Garden
- https://github.com/gazebosim/ros_gz/issues/318 — `gz.msgs.Twist` prefix required (not `ignition.msgs.Twist`) for Garden
- https://github.com/ros2/rclpy/issues/1461 — asyncio executor feature experimental in Humble
- https://github.com/m2-farzan/ros2-asyncio — background thread + asyncio pattern; most-cited FOSS reference
- https://github.com/ros2/rclpy/issues/400 — rclpy installs SIGINT handler by default; `SignalHandlerOptions.NO` documented

### Tertiary (LOW confidence, needs validation)
- `SignalHandlerOptions.NO` exact behavior on Humble — documented but not smoke-tested on this machine

---
*Research completed: 2026-05-12*
*Ready for roadmap: yes*
