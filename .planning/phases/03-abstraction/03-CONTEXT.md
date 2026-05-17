# Phase 3: Abstraction - Context

**Gathered:** 2026-05-17
**Status:** Ready for planning

<domain>
## Phase Boundary

Introduce `robot_api/` with `Robot` protocol + `Capabilities` (axes-only) + `RobotCommand`. Move MAVSDK drone code behind `MavsdkDroneAdapter` at `robot_api/adapters/mavsdk_drone.py`. `--robot drone|rover` CLI flag with two-pass argparse (rover users see no drone flags in `--help`). `run_drone()` renamed to `run_robot()`; `VelocityCommand` no longer exists.

**Critical gate for v1.1.** No rover work (Phases 4–6) starts until this phase passes: full drone SITL must run end-to-end behind `MavsdkDroneAdapter` with no behavioral change.

Scope is the actuator boundary refactor. Rover adapter and rover sim are out of scope; this phase only delivers the seam they will plug into.

</domain>

<decisions>
## Implementation Decisions

### Robot protocol shape (carried forward from earlier phases)

- `Capabilities = {axes: frozenset[Axis], yaw_unit: Literal["deg/s", "rad/s"]}`. Axes-only. No behavioral policy fields. Decided 2026-05-14 during design review; see `feedback-robot-abstraction-axes-only` memory.
- `Axis` enum: `FORWARD`, `YAW`, `ALTITUDE`.
- `Robot` protocol methods: `connect`, `start_session`, `send_command`, `send_zero`, `shutdown`. Plus `caps: Capabilities` attribute.
- `RobotCommand(forward_m_s=0.0, yaw_rate=0.0, down_m_s=0.0)`. Controller writes only the channels in `caps.axes`; adapter reads only the channels in `caps.axes`.
- All robot-specific behaviors live **inside the adapter**, not in `follow_api/`: retreat-from-tilt, slow-near-edge, takeoff/land, yaw-spin-on-loss, offboard handshake.

### live_control_loop migration shape

- **Shared loop in `robot_api/orchestrator.py`.** `run_robot()` instantiates the adapter, calls `adapter.connect()` + `adapter.start_session()`, then runs the generic loop: `detection = shared_state.current()` → `cmd = controller.compute(detection, robot.caps, config)` → `robot.send_command(cmd, detection)`. Loop owns shutdown handling and timing; adapter owns everything drone-specific.
- **Altitude-hold P-loop lives inside `MavsdkDroneAdapter.send_command`.** Adapter reads its private `altitude_cache` (populated by its telemetry task), applies the altitude P correction to `cmd.down_m_s`, then forwards to MAVSDK. The controller only emits `down_m_s` when `Axis.ALTITUDE in caps.axes`. Rover's adapter never sees `down_m_s`.
- **Offboard handshake + telemetry tasks live inside `MavsdkDroneAdapter.start_session()`.** Lifecycle: `start_session()` waits for offboard mode, spawns the `_telemetry_position_task` (merged in Phase 2's CLEAN-13), spawns the offboard-watchdog task. `shutdown()` cancels them cleanly. Rover's `start_session()` is approximately a no-op (or `rclpy.init()`).
- **`shared_state` + `shutdown` event API stays as-is.** No changes to `follow_api/state.py` or pipeline → state plumbing. The orchestrator reads from `shared_state` directly.

### Controller / adapter boundary contract

- **`robot.send_command(cmd, detection)` signature** — detection passed alongside the command. Adapter inspects detection itself to drive robot-specific reactions (drone reads `bbox.center_y + bbox_height/2` to detect bottom-margin; rover ignores). Keeps the controller stateless and detection-agnostic.
- **Controller emits `forward_m_s = 0` when bbox is in the bottom safe-zone edge.** No retreat-from-tilt baked into the controller anymore. Drone adapter overlays retreat-from-tilt (porting `_apply_frame_edge_safety`'s fade-zone math from `controller.py`); rover adapter does nothing (`forward_m_s` stays 0). Drone behavior at the wire is unchanged; just relocated.
- **Target-lost: controller returns `None`.** When detection is `None`, controller doesn't compute. Orchestrator calls `robot.send_zero(last_detection)`. Drone adapter spins yaw based on `last_detection.center_x` (preserving today's search behavior); rover adapter emits `Twist(0, 0, 0)`. Each adapter decides what "zero" means per its capabilities.
- **Emergency safety (bbox > `max_bbox_height_safety`) stays in controller** as a generic emit-retreat (`forward_m_s = -max_backward` + centering yaw). Works for both drone and rover (both want to back off from a too-close person). NOT robot-shaped; NOT moved to adapter.

### CLI: two-pass argparse + `--robot` dispatch

- **Single pre-parser** (from Phase 2 CLEAN-12) — extend it to parse `--robot {drone,rover}` (default `drone`).
- **Robot-specific arg-loader functions** — `add_drone_args(parser)` registers `--takeoff-landing`, `--target-altitude`, `--serial`, `--serial-baud`, `--connection`, `--mission-duration`. `add_rover_args(parser)` registers `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id` (placeholder for Phase 4; Phase 3 lands the dispatch infrastructure, Phase 4 fills in the rover-specific flags).
- **`--help` shape:** `--robot drone --help` shows drone flags only; `--robot rover --help` shows rover flags only. Both share the common flags (`--input`, `--webui`, `--openhd`, `--display`, `--record`, ReID params, etc.).

### Regression test strategy

- **Primary gate: synthetic snapshot test at the controller boundary.** New file: `robot_follow/tests/test_robot_command_snapshot.py`.
- **~100 representative Detection cases** spanning happy path (target-centered, target-offset), edge zones (bbox at top margin, bbox at bottom margin), emergency safety (bbox > max_bbox_height_safety), and search/target-lost (detection=None with various last_detection states).
- **Snapshot fixture** (`robot_follow/tests/cases/drone_command_baseline.py` or similar) records the OLD `VelocityCommand` output sequence pre-rewrite. The test asserts: `controller.compute(det, drone_caps, config)` + `MavsdkDroneAdapter._translate_cmd(rc, det)` produces equivalent `VelocityBodyYawspeed` outputs for each case.
- **Operator-witnessed SITL run required before phase close.** Pattern from Phase 1's Verification A: operator runs `drone-follow --robot drone --input udp://0.0.0.0:5600 --takeoff-landing --webui` against `sim/start_sim.sh --world walk_across_then_approach`, confirms behavior unchanged. Verifier marks `human_needed` until operator approves.
- **Existing `test_controller.py` and `test_shared_state.py` must stay green.** No regression in the 176 existing passing tests.

### setup_env.sh ROS sourcing

- **Always source `/opt/ros/humble/setup.bash` if the file exists.** Idempotent. Drone users on a non-ROS machine see no change; rover users get ROS for free; drone users on a ROS-equipped machine source ROS but the drone path is unaffected (ROS env vars don't conflict with venv/Python).
- **Detection: `[ -f /opt/ros/humble/setup.bash ]`.** Single file check. No `command -v ros2` (chicken-and-egg before sourcing).
- **Source order: venv FIRST, then ROS.** Per PITFALLS.md — Python's `sys.path` resolution prefers venv site-packages over ROS's Python bindings, which is what we want.
- **If ROS is missing and user runs `--robot rover`:** drone-follow's rover adapter raises a friendly `RuntimeError("ROS 2 not sourced — run sudo apt install ros-humble-ros-base then re-source setup_env.sh")` at app startup (per ROVER-04). `setup_env.sh` itself does not fail.
- **No optimization-skip.** Drone-only users don't get a `DF_SKIP_ROS=1` escape hatch. Sourcing ROS costs ~100 ms; not worth the code path.

### Claude's Discretion

- `Robot.send_zero` exact docstring + whether `last_detection` is keyword or positional.
- Internal helper function names inside `MavsdkDroneAdapter` (e.g., `_translate_to_mavsdk`, `_apply_altitude_p`, `_apply_retreat_from_tilt`).
- Exact wording of the `RuntimeError` for missing ROS.
- File layout inside `robot_api/`: `robot.py` (protocol + types) + `adapters/__init__.py` + `adapters/mavsdk_drone.py` + `orchestrator.py`. Planner can refine.
- Whether `test_robot_command_snapshot.py` uses pytest parametrize or a single test iterating over cases. Either works.
- Whether the snapshot fixture is `.json` (data-only) or `.py` (Python module with named constants). Slight preference for `.py` for type hinting.

</decisions>

<specifics>
## Specific Ideas

- "Drone behavior unchanged" is the contract — every test that passes today must pass post-rewrite. The snapshot test is the CI guard against silent drift in the controller-output sequence; the SITL run is the guard against silent drift in MAVSDK-wire behavior.
- The `MavsdkDroneAdapter`'s internal layout should mirror today's `VelocityCommandAPI` class structure (lines 98-194 of `mavsdk_drone.py`). The `send`, `send_zero`, `send_raw` methods become `send_command`, `send_zero`, plus a private `_send_raw`. The smoothing + clamping logic that lives in `VelocityCommandAPI.send` (lines 130-180) stays inside `MavsdkDroneAdapter.send_command` — it's drone-specific (smoothing on top of asyncio scheduling and MAVSDK rate limits).
- `RobotCommand` field naming: `forward_m_s`, `yaw_rate`, `down_m_s`. Note `yaw_rate` is in `caps.yaw_unit` (drone: deg/s; rover: rad/s). Adapter normalizes at the wire.
- The composition root `run_robot()` should keep the existing async task-supervision shape (asyncio.gather + shutdown event); only the actuator-construction lines change.

</specifics>

<code_context>
## Existing Code Insights

### Reusable Assets

- **`VelocityCommandAPI`** (`robot_follow/drone_api/mavsdk_drone.py:98-194`) — current actuator boundary with `send/send_zero/send_raw`. Maps 1:1 to `Robot` protocol's `send_command/send_zero/shutdown`. Smoothing + clamping logic stays inside `MavsdkDroneAdapter.send_command` as-is.
- **`compute_velocity_command`** (`robot_follow/follow_api/controller.py:120`) — stays pure. Signature changes to `compute(detection, caps, config) → RobotCommand`. The internals (yaw, forward, frame-edge-safety, emergency-safety) stay functionally identical; only the return type renames.
- **Single pre-parser** (`robot_follow/robot_follow_app.py:213`, from CLEAN-12) — extended to add `--robot` dispatch.
- **`_apply_frame_edge_safety`** (`robot_follow/follow_api/controller.py:58-117`) — the bottom-margin retreat logic moves into `MavsdkDroneAdapter._apply_retreat_from_tilt` (or similar). Controller keeps the bottom-margin EDGE detection (and emits `forward_m_s=0`); adapter applies the FADE + retreat math.

### Established Patterns

- **Async control loop with `asyncio.gather` + `shutdown: asyncio.Event`** — `mavsdk_drone.py:438 live_control_loop`. Keep this pattern in `robot_api/orchestrator.py`.
- **Telemetry-cache dicts** (`telemetry_cache`, `altitude_cache`) — populated by background tasks, read by the control loop. After Phase 3, these live inside `MavsdkDroneAdapter` as private attributes. Phase 2's CLEAN-13 already merged the two telemetry tasks into one.
- **`source setup_env.sh` activator pattern** — env-exporting script that delegates to the submodule's setup. Easy to extend with the conditional ROS sourcing.
- **`ControllerConfig.tunable_fields()`** (`robot_follow/follow_api/config.py`, from CLEAN-14) — single source of truth for runtime-tunable params. Phase 3 doesn't change this; `Capabilities` is separate (axes + units only, not user-tunable).

### Integration Points

- `robot_follow/robot_follow_app.py:main()` — composition root. Becomes `run_robot()`; reads `--robot`, instantiates adapter, calls orchestrator.
- `robot_follow/follow_api/types.py` — `VelocityCommand` deletion + `Detection` unchanged. `RobotCommand` lives in `robot_api/robot.py` (NOT in `follow_api/types.py`, because `follow_api` should not import from `robot_api`).
- `pyproject.toml` — `[tool.setuptools] packages.include` adds `"robot_api*"`. No version bump needed (still v1.1.0.dev0).
- `setup_env.sh` — adds the conditional ROS source block after the venv activation.
- `robot_follow/follow_api/controller.py` — return type changes; internal logic mostly unchanged.

### Out-of-scope code (does not move/change in Phase 3)

- `robot_follow/pipeline_adapter/` — entire pipeline stays untouched.
- `robot_follow/servers/` — web_server / openhd_bridge / follow_server stay untouched (already iterate `tunable_fields()` from Phase 2; orthogonal to Phase 3).
- `robot_follow/follow_api/state.py`, `config.py` — unchanged.
- Rover-specific files (`robot_api/adapters/ros2_rover.py`, sim/rover/*) — Phase 4 / Phase 5.

</code_context>

<deferred>
## Deferred Ideas

- **Hot-reload `Capabilities`** (e.g., switching robots mid-run) — out of scope. Robot type is fixed at launch via `--robot`. Web UI should NOT expose a robot-type selector. Future v1.2+.
- **`ArduRover via MAVSDK` alternative actuator** — flagged in v2 (HW-09 in REQUIREMENTS.md). Out of v1.1 scope.
- **`Robot.reconnect()` semantics for MAVSDK disconnections** — current behavior is "log + continue"; Phase 3 keeps that. A more sophisticated reconnection state machine is v1.2+.
- **Property-based testing of controller invariants** — Hypothesis-style. Snapshot test covers regression; property-based tests are deferred enrichment.
- **`Capabilities` discovery from the wire** (e.g., probe MAVSDK to confirm altitude axis available) — for v1.1, capabilities are static per adapter class. Dynamic discovery is v2.

</deferred>

---

*Phase: 03-abstraction*
*Context gathered: 2026-05-17*
