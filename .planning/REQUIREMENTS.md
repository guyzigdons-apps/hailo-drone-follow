# Requirements: drone-follow → robot-follow v1.1

**Defined:** 2026-05-12
**Core Value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Milestone goal:** Generalize the app from drone-only to robot-generic — drone path unchanged, ROS 2 cmd_vel rover adapter shipped, Gazebo rover sim runs end-to-end follow-the-person.

## v1 Requirements (sim-only)

### Rename

Package rename `drone_follow` → `robot_follow`. Mechanical, atomic. `drone-follow` console script preserved as alias for boot service + muscle memory.

- [x] **RENAME-01**: Package directory `drone_follow/` renamed to `robot_follow/`; all internal imports updated to `from robot_follow.*`.
- [x] **RENAME-02**: `pyproject.toml` package name is `robot_follow`; console scripts: `robot-follow` (primary) and `drone-follow` (alias pointing at same `main()`).
- [x] **RENAME-03**: `setup_env.sh`, `install.sh`, and `scripts/start_air.sh` updated to reference the new package path; existing functionality preserved.
- [x] **RENAME-04**: `drone-follow-boot.service` systemd unit + `~/Desktop/drone-follow.conf` config name preserved (so existing field deployments keep working); only the underlying binary path changes.
- [x] **RENAME-05**: README, CLAUDE.md, TROUBLESHOOTING.md, `docs/*.md`, and `.claude/memory/*.md` updated to reflect the rename (drone-follow examples still valid via the alias).

### Cleanup

Audit-confirmed dead code, duplication merges, and hot-path wins identified pre-milestone (see `.planning/STATE.md` Accumulated Context).

#### Confirmed dead code (delete)

- [ ] **CLEAN-01**: `sim/world_loader.py` removed (orphan; `start_sim.sh` uses `PX4_GZ_WORLD`).
- [ ] **CLEAN-02**: `scripts/bench_reid_callback.py` removed (imports non-existent `reid_worker`).
- [ ] **CLEAN-03**: `follow_api/config.py` `vfov` field + `--vfov` flag removed (defined end-to-end but never read).
- [x] **CLEAN-04**: `mavsdk_drone.py:47` `--mission-duration` flag removed or documented (silently lands after 5 min, undocumented surprise hazard).
- [x] **CLEAN-05**: `drone_follow_app.py:54` `getattr(args, "serial_baud", 115200)` fallback replaced with direct `args.serial_baud` access (fallback unreachable; would lie if it fired).
- [ ] **CLEAN-06**: `vision_branches.py:397` `strip_tiles_and_highlight_target` alias removed (no callers).
- [ ] **CLEAN-07**: `follow_api/state.py:23` `available_ids=None` default + branch removed (every caller passes a set).
- [x] **CLEAN-08**: `mavsdk_drone.py:642` `shutdown_read_fd` param + pipe-reader block removed (unreachable from caller).
- [ ] **CLEAN-09**: `_NULLABLE_FIELDS = set()` removed from `web_server.py` and `openhd_bridge.py` along with the dead `value == 0 → None` branches it gates.
- [ ] **CLEAN-10**: `create_app(controller_config=...)` kwarg removed from `hailo_drone_detection_manager.py` (no caller passes it; attached later as attribute).

#### Duplication (merge)

- [ ] **CLEAN-11**: MAVSDK-server `pkill` reaper unified into a single helper in the adapter module; called from both the adapter's context-manager and the composition root's shutdown path.
- [ ] **CLEAN-12**: CLI pre-parsing in `drone_follow_app.py` collapsed from 3 throwaway parsers (`ui_pre`, `reid_pre`, `tracker_pre`) into a single pre-parse pass.
- [ ] **CLEAN-13**: `_telemetry_position_task` and `_telemetry_altitude_task` in `mavsdk_drone.py` merged (both subscribe to `drone.telemetry.position()` and write a cache).
- [ ] **CLEAN-14**: `web_server._CONFIG_FIELDS` and `openhd_bridge._CONFIG_PARAMS` replaced with a single `ControllerConfig.tunable_fields()` source-of-truth iterated by both consumers.
- [ ] **CLEAN-15**: Branch-decision tree (which output branches to build given `--display`/`--record`/`--webui`/`--openhd`) centralised in `vision_branches`; implicit-display rule defined in one place instead of three.

#### Hot-path wins

- [ ] **CLEAN-16**: `web_server.py:84` `_frame_event.set()` then immediate `clear()` race fixed — replace with monotonic counter + `Condition.notify_all()` (or per-consumer queue) so SSE readers don't fall through to the 2 s timeout under multi-client load.
- [ ] **CLEAN-17**: `openhd_bridge.py:332` `_send_immediate_report` reuses the listener thread's existing socket instead of opening a fresh `socket.socket(...)` per call.
- [ ] **CLEAN-18**: `hailo_drone_detection_manager.py:88` linear-scan dedup over `persons` (`next(q for q in persons if id(q) == prev)`) replaced with a one-shot `{id(p): p}` lookup dict built once per callback.

### Abstraction

Introduce `robot_api/` with `Robot` protocol + `Capabilities` and move the MAVSDK drone code behind it. Generic-vs-platform-specific seams cleaned in the same pass. Drone behaviour unchanged.

**Design constraint (axes-only):** `follow_api/controller` must not know what kind of robot it is moving. `Capabilities` describes **which axes exist** and **what units they take** — nothing else. Robot-specific behaviors (retreat-from-tilt, slow-near-edge, takeoff/land, yaw-spin-on-loss, offboard handshake) live inside the adapter, invisible to follow_api. This is the whole point of the abstraction; a `bottom_edge_policy` flag in `Capabilities` would smuggle robot knowledge back into the controller behind a flag. Don't.

- [ ] **ABS-01**: `robot_api/robot.py` defines:
  - `Axis` enum: `FORWARD` (body x, m/s), `YAW` (body z), `ALTITUDE` (body z down, m/s).
  - `Capabilities` dataclass: `axes: frozenset[Axis]`, `yaw_unit: Literal["deg/s", "rad/s"]`. **Mechanical only — no behavioral flags.**
  - `Robot` protocol: `connect`, `start_session`, `send_command`, `send_zero`, `shutdown`, plus `caps: Capabilities` attribute.
  - Lifecycle concerns (offboard handshake, arm/takeoff/land) are implemented inside the adapter's `connect` / `start_session` / `shutdown`, not exposed as capability flags.
- [ ] **ABS-02**: `RobotCommand(forward_m_s=0.0, yaw_rate=0.0, down_m_s=0.0)` replaces `VelocityCommand`. The controller writes only the channels whose axis is in `caps.axes`; the adapter reads only the channels in `caps.axes`. Drone adapter unchanged at the wire, rover adapter converts `yaw_rate` from the unit declared in `caps.yaw_unit` to rad/s.
- [ ] **ABS-03**: `drone_api/mavsdk_drone.py` moved to `robot_api/adapters/mavsdk_drone.py`; existing MAVSDK behaviour wrapped behind `MavsdkDroneAdapter.start_session()`. No-behaviour-change regression test: drone path still flies SITL.
- [ ] **ABS-04**: `live_control_loop` gates the altitude-hold P-loop on `Axis.ALTITUDE in capabilities.axes` (rover sees no `down_m_s` term emitted from controller).
- [ ] **ABS-05**: Bottom-edge frame safety moves out of the controller. Controller emits `forward_m_s=0` when the target's bbox bottom is below the safe-zone threshold — that's all it knows. Per-robot reactions (drone: retreat-from-tilt; rover: slow / stop) are implemented **inside each adapter's `send_command`**, not gated by a flag in follow_api. No `bottom_edge_policy` field in `Capabilities`.
- [ ] **ABS-06**: Yaw-spin search-on-loss moves out of the controller. When the target is lost the controller emits `send_zero()` (or holds, per existing logic) — that's all it knows. Per-robot search behaviors (drone: yaw-spin to scan; rover: no spin) are implemented **inside each adapter**, not gated by a flag in follow_api. No `yaw_spin_on_loss` field in `Capabilities`.
- [ ] **ABS-07**: `ControllerConfig` altitude fields (`min_altitude`, `max_altitude`, `target_altitude`, `kp_alt_hold`, `max_climb_speed`, `max_down_speed`, `max_bbox_height_safety`, `top_margin_safety`, `bottom_margin_safety`) become `Optional[float]`; `validate()` skips altitude relationship checks when `Axis.ALTITUDE not in capabilities.axes`.
- [ ] **ABS-08**: Composition root: `run_drone()` renamed to `run_robot()`; dispatches to the right adapter based on `--robot`. Existing `run_drone()` callers updated; behaviour unchanged for `--robot drone` (default).
- [ ] **ABS-09**: `--robot drone|rover` CLI flag (default `drone`) with two-pass argparse pre-parse so rover users don't see drone-only flags (`--takeoff-landing`, `--target-altitude`, `--serial`) in `--help` and vice versa.
- [ ] **ABS-10**: `setup_env.sh` auto-sources `/opt/ros/humble/setup.bash` if `--robot rover` is detected and ROS is installed; sourcing order is venv first, then ROS (per PITFALLS.md).
- [ ] **ABS-11**: Drone path regression: full SITL drone follow-the-person test passes with `--robot drone` (or default). Existing `--takeoff-landing`, `--target-altitude`, `--connection` paths unchanged.

### Rover adapter

ROS 2 Humble rclpy adapter publishing `geometry_msgs/Twist` on `/cmd_vel`. Background-thread executor bridges to the asyncio control loop. SIGINT handler preservation is mandatory per PITFALLS.md.

- [ ] **ROVER-01**: `robot_api/adapters/ros2_rover.py` defines `Ros2RoverAdapter` implementing `Robot`; wraps an `rclpy.Node` that creates a `Publisher[Twist]` on the configured topic.
- [ ] **ROVER-02**: `rclpy.init(signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)` is the only `init` call path — drone-follow's custom SIGINT handler survives. Post-init `signal.getsignal(signal.SIGINT)` assertion in the adapter's smoke path.
- [ ] **ROVER-03**: Background-thread spin uses `SingleThreadedExecutor.spin_once(timeout_sec=0.05)` in a loop checking the shutdown event. Never `rclpy.spin()` or `executor.spin()` (no-timeout variants).
- [ ] **ROVER-04**: Defensive `import rclpy` raises a friendly `RuntimeError` ("ROS 2 not sourced — run `source /opt/ros/humble/setup.bash`") if the C extension is missing; venv/PYTHONPATH ordering gotcha from PITFALLS.md.
- [ ] **ROVER-05**: `add_rover_args(parser)` registers `--cmd-vel-topic` (default `/cmd_vel`), `--ros-namespace` (default empty), `--ros-domain-id` (default 0). Loaded only when `--robot rover` is selected.
- [ ] **ROVER-06**: At adapter boundary, `RobotCommand.yaw` (deg/s) is converted to rad/s before assignment to `Twist.angular.z`; `forward` passes straight to `Twist.linear.x` (m/s on both sides).
- [ ] **ROVER-07**: `RoverCapabilities = Capabilities(axes=frozenset({Axis.FORWARD, Axis.YAW}), yaw_unit="rad/s")` — the rover has no `ALTITUDE` axis, full stop. No offboard / arm / takeoff lifecycle is exposed via `Capabilities`; `Ros2RoverAdapter.start_session()` is the no-op that captures "rover publishes immediately once the rclpy node is up".
- [ ] **ROVER-08**: Smoke test on Humble: after `Ros2RoverAdapter.start_session()`, drone-follow's `on_signal` (set in `drone_follow_app.py`) is the active SIGINT handler (`signal.getsignal(signal.SIGINT) is on_signal`). Test runs in CI / dev.

### Rover sim

Gazebo Garden with a differential-drive rover SDF, ros_gz_bridge for cmd_vel (NOT images), and the existing `video_bridge.py` reused for camera→UDP. New apt packages installable via `install.sh --rover` or `sim/rover/setup_rover_sim.sh`.

- [ ] **RSIM-01**: `sim/rover/rover.sdf` defines a differential-drive rover with `<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">` (Garden-era names — `ignition::` prefix is a silent-load-failure footgun per PITFALLS.md). Includes a forward-facing camera sensor.
- [ ] **RSIM-02**: SDF overrides the DiffDrive plugin's default topic with `<topic>cmd_vel</topic>` so ros_gz_bridge maps ROS `/cmd_vel` ↔ gz `/cmd_vel` cleanly (avoids the `/model/<name>/cmd_vel` default-namespace mismatch).
- [ ] **RSIM-03**: `sim/rover/worlds/` contains rover-adapted versions of existing actor walk patterns (`random_walk`, `walk_across_then_approach`, `circle_around`) on a ground plane.
- [ ] **RSIM-04**: `sim/rover/start_rover_sim.sh` one-command launch: starts `gz sim` with the rover world, starts `ros2 run ros_gz_bridge parameter_bridge` for cmd_vel using `geometry_msgs/Twist@gz.msgs.Twist]` syntax (Garden-correct prefix), starts the existing `sim/bridge/video_bridge.py` pointed at the rover camera gz topic for UDP output on port 5600.
- [ ] **RSIM-05**: `install.sh --rover` (or `sim/rover/setup_rover_sim.sh`) installs apt packages: `ros-humble-ros-base`, `ros-humble-geometry-msgs`, `ros-humble-ros-gzgarden-bridge` (NOT `ros-humble-ros-gz-bridge` which is Fortress). Hard-fails with a friendly error if `/opt/ros/humble` is missing or `gz-garden` is missing.
- [ ] **RSIM-06**: Camera path reuses `video_bridge.py` (gz-transport → UDP H.264). `--input udp://0.0.0.0:5600` keeps working with zero changes to `pipeline_adapter/`. No `ros_gz_image_bridge`, no new `camera_udp_shim.py` (camera plumbing decision per SUMMARY.md).
- [ ] **RSIM-07**: `sim/rover/README.md` documents Gazebo Garden EOL (Nov 2024), the Harmonic migration path (SDF prefixes already compatible), and the `gz topic -l` smoke-test step to confirm DiffDrive loaded + camera topic discovered.

### Sim integration

Rover-specific defaults, the bottom-edge safety repurposed for ground vehicles, ByteTracker re-tuned for rover dynamics, and end-to-end follow-the-person validation in the rover sim.

- [ ] **RINT-01**: `configs/rover_simulation.json` ships with rover-safe defaults: `max_forward=1.0`, no `max_forward_accel` slew cap (rovers don't have tilt-transients), no altitude knobs, lower `kp_yaw` to suit narrower yaw dynamics. Loaded via the existing config-load path.
- [ ] **RINT-02**: Bottom-edge frame safety implementation lands in each adapter (per ABS-05 design — not capability-gated in the controller). `MavsdkDroneAdapter` keeps "person too low → retreat-from-tilt" behavior. `Ros2RoverAdapter` adds "person too low → slow/stop" behavior. The controller's contribution is unchanged: emit `forward_m_s=0` when the bbox bottom is below the safe zone. Drone behaviour at the controller call site unchanged.
- [ ] **RINT-03**: ByteTracker knobs (`track_thresh`, `track_buffer`, `match_thresh`, `frame_rate`) become config-driven (currently hard-coded in `hailo_drone_detection_manager.py:1271`). Drone defaults preserve current values; rover defaults reduce `track_buffer` from 90 (3 s) to ~30 (1 s) to match ground-perspective motion magnitudes.
- [ ] **RINT-04**: End-to-end test in Gazebo rover sim: actor `walk_across_then_approach` world, rover starts in AUTO mode, picks up the largest person, follows for the full walk pattern without losing the target. Captured as a deterministic test in `drone_follow/tests/test_sim_worlds.py` (or rover equivalent) similar to existing drone sim tests.
- [ ] **RINT-05**: Port-isolation check: rover sim's MAVLink-side ports (none; just `/cmd_vel`) and video port (5600 UDP) coexist with PX4 SITL ports (14540 UDP MAVLink, 5600 UDP video). Documented that they cannot run simultaneously on the same machine without remapping the video port (`--openhd-port`-style flag for the rover sim camera if needed in a future milestone).
- [ ] **RINT-06**: SIGINT shutdown integration test: with rover sim running, Ctrl+C from drone-follow produces zero further `/cmd_vel` messages within 100 ms; rover stops within 1 s; rclpy node cleanly destroyed before `rclpy.try_shutdown()` (per PITFALLS.md "Looks Done But Isn't" checklist).

## v2 Requirements (deferred to v1.2 — real rover hardware)

Tracked but not in current roadmap. All require physical hardware to exercise.

### Real rover hardware path

- **HW-01**: Rover hardware definition (specific platform: TurtleBot4? Husky? ROSbot? Custom?). Drives all downstream items.
- **HW-02**: `--cmd-vel-topic` validated against real rover's topic name; namespace remapping if needed.
- **HW-03**: `/odom` subscription for closed-loop distance estimation (replaces bbox-only forward control on uncalibrated terrain).
- **HW-04**: `twist_mux` integration for teleop override + e-stop priority above the follow controller.
- **HW-05**: E-stop topic subscription (`/e_stop` or vendor-specific) wired to the adapter's `send_zero()` path.
- **HW-06**: Bumper / contact-sensor subscription wired to the bottom-edge safety repurpose (replaces "person too low" with "physical contact").
- **HW-07**: `/diagnostics` consumption for health-check display in the web UI.
- **HW-08**: Boot service variant: `scripts/start_rover.sh` analog of `start_air.sh`; `~/Desktop/robot-follow.conf` with `ROBOT=drone|rover` selector.

### Alternative actuator stacks (research)

- **HW-09**: ArduRover via MAVSDK (alternative to ROS 2 for some rover platforms); kept in v2 only because the existing MAVSDK adapter code may be reusable.

## Out of Scope

| Feature | Reason |
|---------|--------|
| Nav2 / Nav stack integration | Goal-based pathing; incompatible with reactive bbox-driven follow. Massive dep footprint. |
| `ros2_control` + hardware interface | Overkill for sim-only; expects `TwistStamped` not plain `Twist`, conflicting with the DiffDrive SDF plugin. |
| SLAM / mapping | Orthogonal concern; requires LiDAR or depth camera not on the air-unit BoM. |
| ROS 2 distros other than Humble | Iron is EOL since Dec 2024; Jazzy has no Jammy binaries. Humble (EOL May 2027) is the only valid choice for Ubuntu 22.04. |
| `TwistStamped` variant | Not needed for plain DiffDrive SDF plugin; would only matter if integrating with ros2_control later. |
| Gazebo Harmonic migration | Garden is EOL but acceptable for v1.1 sim-only. Migration cost is low because SDF `gz::` prefixes are already Harmonic-compatible. Defer until forced by a Harmonic-only feature. |
| Multi-robot simultaneous control | Single-target controller is the validated abstraction; multi-robot fleet would require new state mediation. |
| Web UI "robot type" selector | Robot type is set at launch via `--robot`; mid-run switching would require pipeline restart. |
| OpenHD integration for rovers | OpenHD is WFB/MAVLink; rover stack is ROS 2. Re-purposing OpenHD for rovers is its own milestone. |
| Real-hardware safety certification | Sim-only milestone; sim cannot validate hardware-specific safety. |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| RENAME-01 | Phase 1 | Complete |
| RENAME-02 | Phase 1 | Complete |
| RENAME-03 | Phase 1 | Complete |
| RENAME-04 | Phase 1 | Complete |
| RENAME-05 | Phase 1 | Complete |
| CLEAN-01 | Phase 2 | Pending |
| CLEAN-02 | Phase 2 | Pending |
| CLEAN-03 | Phase 2 | Pending |
| CLEAN-04 | Phase 2 | Complete |
| CLEAN-05 | Phase 2 | Complete |
| CLEAN-06 | Phase 2 | Pending |
| CLEAN-07 | Phase 2 | Pending |
| CLEAN-08 | Phase 2 | Complete |
| CLEAN-09 | Phase 2 | Pending |
| CLEAN-10 | Phase 2 | Pending |
| CLEAN-11 | Phase 2 | Pending |
| CLEAN-12 | Phase 2 | Pending |
| CLEAN-13 | Phase 2 | Pending |
| CLEAN-14 | Phase 2 | Pending |
| CLEAN-15 | Phase 2 | Pending |
| CLEAN-16 | Phase 2 | Pending |
| CLEAN-17 | Phase 2 | Pending |
| CLEAN-18 | Phase 2 | Pending |
| ABS-01 | Phase 3 | Pending |
| ABS-02 | Phase 3 | Pending |
| ABS-03 | Phase 3 | Pending |
| ABS-04 | Phase 3 | Pending |
| ABS-05 | Phase 3 | Pending |
| ABS-06 | Phase 3 | Pending |
| ABS-07 | Phase 3 | Pending |
| ABS-08 | Phase 3 | Pending |
| ABS-09 | Phase 3 | Pending |
| ABS-10 | Phase 3 | Pending |
| ABS-11 | Phase 3 | Pending |
| ROVER-01 | Phase 4 | Pending |
| ROVER-02 | Phase 4 | Pending |
| ROVER-03 | Phase 4 | Pending |
| ROVER-04 | Phase 4 | Pending |
| ROVER-05 | Phase 4 | Pending |
| ROVER-06 | Phase 4 | Pending |
| ROVER-07 | Phase 4 | Pending |
| ROVER-08 | Phase 4 | Pending |
| RSIM-01 | Phase 5 | Pending |
| RSIM-02 | Phase 5 | Pending |
| RSIM-03 | Phase 5 | Pending |
| RSIM-04 | Phase 5 | Pending |
| RSIM-05 | Phase 5 | Pending |
| RSIM-06 | Phase 5 | Pending |
| RSIM-07 | Phase 5 | Pending |
| RINT-01 | Phase 6 | Pending |
| RINT-02 | Phase 6 | Pending |
| RINT-03 | Phase 6 | Pending |
| RINT-04 | Phase 6 | Pending |
| RINT-05 | Phase 6 | Pending |
| RINT-06 | Phase 6 | Pending |

**Coverage:**
- v1 requirements: 55 total (5 RENAME + 18 CLEAN + 11 ABS + 8 ROVER + 7 RSIM + 6 RINT)
- Mapped to phases: 55 ✓
- Unmapped: 0 ✓

---
*Requirements defined: 2026-05-12*
*Last updated: 2026-05-12 — traceability filled by gsd-roadmapper*
