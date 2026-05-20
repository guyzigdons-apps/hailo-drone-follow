# Roadmap: robot-follow v1.1

## Milestones

- ✅ **v1.0 Drone follow** - Phase 0 (shipped pre-GSD, 2026-04)
- 🚧 **v1.1 Robot abstraction + rover support (sim-only)** - Phases 1-6 (in progress)
- 📋 **v1.2 Real rover hardware** - Planned

## Phases

<details>
<summary>✅ v1.0 Drone follow (Phase 0) - SHIPPED 2026-04</summary>

Pre-GSD baseline. Full feature list in `.planning/MILESTONES.md` § v1.0.
Validated requirements: VIS-01..03, CTRL-01..06, DRONE-01..03, UI-01..03, REC-01, SIM-01..02, OPS-01..03.

</details>

### 🚧 v1.1 Robot abstraction + rover support (sim-only)

**Milestone Goal:** Generalize the app from drone-only to robot-generic — drone path unchanged, ROS 2 cmd_vel rover adapter shipped, Gazebo rover sim runs end-to-end follow-the-person.

- [x] **Phase 1: Rename** - Mechanical `drone_follow` → `robot_follow` package rename with `drone-follow` alias preserved (3/3 plans complete; ready for `/gsd:verify-work`)
- [x] **Phase 2: Cleanup** - Delete confirmed dead code, merge duplications, fix hot-path races
- [ ] **Phase 3: Abstraction** - `Robot` protocol + `Capabilities`, drone adapter behind protocol; CRITICAL GATE before rover work — all 12 plans landed but ABS-11 SITL gate failed (F3 surfaced in 03-12); awaiting gap closure 03-13/03-14
- [ ] **Phase 4: Rover adapter** - ROS 2 Humble rclpy node publishing `geometry_msgs/Twist`; parallel with Phase 5
- [ ] **Phase 5: Rover sim** - Gazebo Garden rover SDF + cmd_vel bridge + video_bridge.py camera path; parallel with Phase 4
- [x] **Phase 6: Sim integration** - Rover defaults, bottom-edge safety repurpose, ByteTracker config, end-to-end validation (completed 2026-05-20)

## Phase Details

### Phase 1: Rename
**Goal**: The package is `robot_follow`; `drone-follow` and `robot-follow` both work; no field deployment breaks.
**Depends on**: Phase 0 (pre-GSD baseline)
**Requirements**: RENAME-01, RENAME-02, RENAME-03, RENAME-04, RENAME-05
**Success Criteria** (what must be TRUE):
  1. `pip show robot_follow` shows the renamed package; `pip show drone_follow` returns nothing.
  2. `drone-follow --help` and `robot-follow --help` produce identical output (same `main()` entry point).
  3. `scripts/start_air.sh` on a fresh checkout runs without path errors; boot service unit file is unchanged on disk.
  4. All documentation examples using `drone-follow` still work via the alias; no internal import of `drone_follow` remains in the source tree.
**Plans**: 3 plans
  - [x] 01-01-PLAN.md — Wave 0: rewrite test_install_smoke.py as forward-compatible Phase 1 gate
  - [x] 01-02-PLAN.md — Wave 1: atomic rename commit (git mv + sed + pyproject + docs + reinstall + verify)
  - [x] 01-03-PLAN.md — Wave 2: manual-only verifications (install.sh re-run, boot service on disk)

### Phase 2: Cleanup
**Goal**: Dead code is deleted, duplications are merged, hot-path races are fixed; codebase is clean before structural changes.
**Depends on**: Phase 1
**Requirements**: CLEAN-01, CLEAN-02, CLEAN-03, CLEAN-04, CLEAN-05, CLEAN-06, CLEAN-07, CLEAN-08, CLEAN-09, CLEAN-10, CLEAN-11, CLEAN-12, CLEAN-13, CLEAN-14, CLEAN-15, CLEAN-16, CLEAN-17, CLEAN-18
**Success Criteria** (what must be TRUE):
  1. `sim/world_loader.py` and `scripts/bench_reid_callback.py` do not exist; `grep -r "world_loader\|bench_reid" .` returns nothing outside git history.
  2. `drone-follow --input usb --webui` starts and serves the web UI; no regression in any existing flag path.
  3. Web UI MJPEG stream delivers a frame to a second simultaneous browser tab without either tab falling through to the 2 s SSE timeout (CLEAN-16 race fixed).
  4. A single `ControllerConfig.tunable_fields()` call drives both `web_server` and `openhd_bridge` field lists; no parallel altitude knob lists remain (CLEAN-14).
  5. Branch-decision tree (display/record/webui/openhd selection) is defined in one place in `vision_branches`; implicit-display rule appears exactly once (CLEAN-15).
**Plans**: 8 plans
  - [x] 02-00-PLAN.md — Wave 1: xfail test scaffolds for CLEAN-15 and CLEAN-16 (gates for waves 4/5)
  - [x] 02-01-PLAN.md — Wave 2: dead-code deletes (CLEAN-01, CLEAN-02, CLEAN-06, CLEAN-10)
  - [x] 02-02-PLAN.md — Wave 2: config/state/NULLABLE cleanup (CLEAN-03, CLEAN-07 atomic, CLEAN-09)
  - [x] 02-03-PLAN.md — Wave 2: mavsdk_drone + app edits (CLEAN-04 doc, CLEAN-05 getattr, CLEAN-08 pipe-reader)
  - [x] 02-04-PLAN.md — Wave 3: mavsdk_drone duplication merges (CLEAN-11 reaper helper, CLEAN-13 telemetry merge Shape A)
  - [x] 02-05-PLAN.md — Wave 4: pre-parser collapse + decide_branches (CLEAN-12, CLEAN-15 — closes xfail)
  - [x] 02-06-PLAN.md — Wave 3: ControllerConfig.tunable_fields() source of truth (CLEAN-14)
  - [x] 02-07-PLAN.md — Wave 5: hot-path fixes (CLEAN-16 SSE race — closes xfail, CLEAN-17 socket reuse, CLEAN-18 lookup dict)

### Phase 3: Abstraction
**Goal**: `Robot` protocol and `Capabilities` are the only actuator boundary; drone path runs end-to-end behind `MavsdkDroneAdapter`; `--robot` CLI flag exists. This is the critical gate — rover work cannot start until this phase passes.
**Depends on**: Phase 2
**Requirements**: ABS-01, ABS-02, ABS-03, ABS-04, ABS-05, ABS-06, ABS-07, ABS-08, ABS-09, ABS-10, ABS-11
**Success Criteria** (what must be TRUE):
  1. Full SITL drone follow-the-person test passes with `--robot drone` (default); `--takeoff-landing`, `--target-altitude`, and `--connection` paths are unchanged.
  2. `--robot drone --help` shows drone-specific flags (`--takeoff-landing`, `--serial`); `--robot rover --help` does not show those flags (two-pass argparse pre-parse working).
  3. `robot_api/robot.py` exists with `Robot` protocol, `Capabilities`, and `RobotCommand`; `VelocityCommand` no longer exists in the codebase.
  4. `robot_api/adapters/mavsdk_drone.py` exists; `drone_api/mavsdk_drone.py` does not exist; `run_robot()` is the composition root entry point.
  5. On a machine with `/opt/ros/humble` installed, `setup_env.sh` with `--robot rover` detected auto-sources `/opt/ros/humble/setup.bash` after venv activation.
**Plans**: 14 plans
  - [x] 03-01-PLAN.md — Wave 1: test scaffolds for ABS-04/05/06 (snapshot + adapter pure-function tests + baseline fixture)
  - [x] 03-02-PLAN.md — Wave 1: test scaffolds for ABS-01/02/03/09/10 (protocol shape, layout smoke, CLI help dispatch, setup_env.sh, rename VelocityCommand shape test)
  - [x] 03-03-PLAN.md — Wave 2: add Axis/Capabilities/RobotCommand/SafetyContext to follow_api/types.py (ABS-01, ABS-02 types)
  - [x] 03-04-PLAN.md — Wave 3: scaffold robot_api/ package with Robot protocol + orchestrator skeleton (ABS-01 protocol)
  - [x] 03-05-PLAN.md — Wave 4: git mv drone_api/mavsdk_drone.py to robot_api/adapters/ + shim (ABS-03 move)
  - [x] 03-06-PLAN.md — Wave 4: introduce MavsdkDroneAdapter + R5 pure-function extracts + ~30 adapter tests (ABS-04, ABS-05, ABS-06)
  - [x] 03-07-PLAN.md — Wave 5 (ATOMIC): controller.compute(detection, caps, config) -> RobotCommand; delete VelocityCommand + VelocityCommandAPI + live_control_loop; migrate 103 test sites; populate snapshot baseline; ABS-07 Optional altitude fields
  - [x] 03-08-PLAN.md — Wave 6: two-pass argparse with --robot dispatch + run_robot() composition root (ABS-08, ABS-09)
  - [x] 03-09-PLAN.md — Wave 7: setup_env.sh conditional ROS source + delete drone_api/ directory (ABS-03 final, ABS-10)
  - [x] 03-10-PLAN.md — Wave 7 (checkpoint): operator-witnessed SITL drone follow gate (ABS-11) — FAILED (F1 + F2 surfaced; see 03-10-SUMMARY.md)
  - [x] 03-11-PLAN.md — Wave 8 (gap closure): thread ui_state through run_robot_loop + publish (fwd,down,yaw,mode) per branch + demote F2 INFO log to DEBUG (ABS-11)
  - [x] 03-12-PLAN.md — Wave 9 (gap closure checkpoint): operator SITL re-run gate after F1 + F2 fixes (ABS-11) — FAILED (F1+F2 closed, new F3 surfaced; see 03-12-SUMMARY.md)
  - [ ] 03-13-PLAN.md — Wave 10 (gap closure): widen do_POST /follow/<id> stale-id leniency in follow_server.py when len(available_ids)==1 + 3 new tests (ABS-11 / F3)
  - [ ] 03-14-PLAN.md — Wave 11 (gap closure checkpoint): operator SITL re-run gate (round 3) — F3 closed + F1/F2 stay closed + AUTO not regressed (ABS-11)

### Phase 4: Rover adapter
**Goal**: `Ros2RoverAdapter` publishes `geometry_msgs/Twist` on `/cmd_vel`; SIGINT handler is preserved; adapter integrates cleanly with the asyncio control loop.
**Depends on**: Phase 3 (critical gate must pass first)
**Note**: Phases 4 and 5 are independent once Phase 3 lands and can be developed in parallel.
**Requirements**: ROVER-01, ROVER-02, ROVER-03, ROVER-04, ROVER-05, ROVER-06, ROVER-07, ROVER-08
**Success Criteria** (what must be TRUE):
  1. After `Ros2RoverAdapter.start_session()`, `signal.getsignal(signal.SIGINT) is on_signal` — drone-follow's handler is active, not rclpy's (ROVER-02, ROVER-08 verified).
  2. `ros2 topic echo /cmd_vel` shows one `Twist` message per control tick while `drone-follow --robot rover` is running against a dummy detector input; `linear.x` is in m/s, `angular.z` is in rad/s (ROVER-06 conversion verified).
  3. `drone-follow --robot rover --help` shows `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id`; no drone-only flags visible.
  4. On a machine without ROS installed, `--robot rover` raises a friendly `RuntimeError` with "ROS 2 not sourced" message rather than an `ImportError` traceback.
**Plans**: 5 plans
  - [x] 04-01-PLAN.md — Wave 1: test scaffolds (~20 ros2_rover xfail tests + 6 CLI dispatch xfail tests)
  - [x] 04-02-PLAN.md — Wave 2: add_rover_args body in robot_follow_app.py (ROVER-05) + strip 6 CLI dispatch xfails
  - [x] 04-03-PLAN.md — Wave 2: Ros2RoverAdapter + ROVER_CAPS in robot_api/adapters/ros2_rover.py (ROVER-01..04, 06-07) + strip ~20 adapter xfails
  - [x] 04-04-PLAN.md — Wave 3: replace NotImplementedError stub in run_robot() with lazy Ros2RoverAdapter construction + integration smoke (ROVER-08 end-to-end)
  - [ ] 04-05-PLAN.md — Wave 4 (operator gate): real-rclpy SITL smoke — friendly error, Twist wire format, SIGINT preservation, drone-path-not-regressed

### Phase 5: Rover sim
**Goal**: `sim/rover/start_rover_sim.sh` launches a Gazebo Garden differential-drive rover world; cmd_vel arrives from ROS and camera video reaches drone-follow via UDP on port 5600.
**Depends on**: Phase 3 (critical gate must pass first)
**Note**: Phases 4 and 5 are independent once Phase 3 lands and can be developed in parallel.
**Setup note**: Install `ros-humble-ros-gzgarden-bridge` (Garden-specific), NOT `ros-humble-ros-gz-bridge` (Fortress). Use `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system` in SDF; `ignition::` prefix causes silent load failure on Garden.
**Requirements**: RSIM-01, RSIM-02, RSIM-03, RSIM-04, RSIM-05, RSIM-06, RSIM-07
**Success Criteria** (what must be TRUE):
  1. `gz topic -l` after `start_rover_sim.sh` shows a `/cmd_vel` gz topic and a rover camera gz topic; `gz topic -e -t /cmd_vel` confirms DiffDrive plugin loaded and receiving (RSIM-01, RSIM-02 verified).
  2. `drone-follow --input udp://0.0.0.0:5600` receives frames from the rover sim camera via `video_bridge.py` (gz-transport → UDP H.264); no `ros_gz_image_bridge`, no new camera shim (RSIM-06 camera plumbing decision).
  3. `sim/rover/start_rover_sim.sh` runs in one command from a clean terminal; required apt packages (`ros-humble-ros-gzgarden-bridge`) are listed in `install.sh --rover` with a friendly error if `/opt/ros/humble` is missing.
  4. At least one rover actor world (`walk_across_then_approach`) renders a walking person in Gazebo Garden; rover model appears on a ground plane.
**Plans**: 6 plans
  - [ ] 05-01-PLAN.md — Wave 1: sim/rover/rover.sdf (Garden DiffDrive + camera, RSIM-01/02)
  - [ ] 05-02-PLAN.md — Wave 1: sim/rover/worlds/{walk_across_then_approach,random_walk,circle_around}.sdf (RSIM-03)
  - [ ] 05-03-PLAN.md — Wave 1: sim/rover/README.md — Garden EOL + smoke-test step (RSIM-07)
  - [ ] 05-04-PLAN.md — Wave 2: install.sh --rover apt-install branch with friendly preflight errors (RSIM-05)
  - [ ] 05-05-PLAN.md — Wave 2: sim/rover/start_rover_sim.sh launcher with setsid cleanup + bridge wiring (RSIM-04, RSIM-06)
  - [ ] 05-06-PLAN.md — Wave 3: robot_follow/tests/test_rover_sim_smoke.py parse-only smoke tests (RSIM-01..07 regression guard)

### Phase 6: Sim integration
**Goal**: Rover follows a walking actor end-to-end in Gazebo Garden with rover-safe defaults; SIGINT shuts down cleanly with zero residual `/cmd_vel` messages; port isolation from PX4 SITL is documented.
**Depends on**: Phase 4 and Phase 5 (both must complete before Phase 6)
**Requirements**: RINT-01, RINT-02, RINT-03, RINT-04, RINT-05, RINT-06
**Success Criteria** (what must be TRUE):
  1. `drone-follow --robot rover --config configs/rover_simulation.json` follows a walking actor in the `walk_across_then_approach` Gazebo world for the full walk pattern without losing the target (RINT-04 deterministic test passes).
  2. Bottom-edge frame safety with `--robot rover` slows/stops the rover when the person is too low in frame; with `--robot drone` the same edge still triggers the retreat-from-tilt behavior (RINT-02 capability-gated, both branches verified).
  3. Ctrl+C from `drone-follow --robot rover` produces zero additional `/cmd_vel` messages within 100 ms; rclpy node is destroyed cleanly before `rclpy.try_shutdown()` (RINT-06 shutdown integration test).
  4. `configs/rover_simulation.json` ships with rover-safe defaults (no altitude knobs, `track_buffer` ≈ 30 frames, lower `kp_yaw`); `configs/drone_simulation.json` (or equivalent) still uses original values; both load without validation errors (RINT-01, RINT-03).
**Plans**: 7 plans
  - [x] 06-01-PLAN.md — Wave 1: rover_simulation.json + ControllerConfig ByteTracker fields + SafetyContext bbox_bottom_norm (RINT-01)
  - [x] 06-02-PLAN.md — Wave 1: sim/rover/README.md port-isolation table append (RINT-05)
  - [x] 06-03-PLAN.md — Wave 2: ByteTracker config-driven refactor in hailo_drone_detection_manager + create_app wiring (RINT-03)
  - [x] 06-04-PLAN.md — Wave 2: rover bottom-edge slow-down in ros2_rover.py.send_command (RINT-02)
  - [x] 06-05-PLAN.md — Wave 3: TestSigintShutdown SIGINT silence + timing tests (RINT-06)
  - [x] 06-06-PLAN.md — Wave 3: TestRoverWalkAcrossThenApproach E2E deterministic test (RINT-04, skip-on-no-gz)
  - [x] 06-07-PLAN.md — Wave 4: operator-witnessed rover sim full follow gate (autonomous: false)

## Progress

**Execution Order:**
Phases 1 → 2 → 3 → (4 and 5 in parallel) → 6

| Phase | Milestone | Plans Complete | Status | Completed |
|-------|-----------|----------------|--------|-----------|
| 1. Rename | v1.1 | 3/3 | Complete | 2026-05-14 |
| 2. Cleanup | v1.1 | 8/8 | Complete | 2026-05-17 |
| 3. Abstraction | v1.1 | 12/12 | In Progress (gate failed) |  |
| 4. Rover adapter | v1.1 | 4/5 | In Progress|  |
| 5. Rover sim | v1.1 | 0/6 | Planned | - |
| 6. Sim integration | v1.1 | 7/7 | Complete   | 2026-05-20 |
