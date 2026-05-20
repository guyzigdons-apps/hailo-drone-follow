---
phase: 06-sim-integration
verified: 2026-05-20T00:00:00Z
status: human_needed
score: 5/6 must-haves verified (1 deferred to operator)
overrides_applied: 0
human_verification:
  - test: "Run TestRoverWalkAcrossThenApproach end-to-end on the wired-up rover sim box"
    expected: "RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v reports PASSED (not SKIPPED, not FAILED); rover follows the walking actor through the full walk_across_then_approach pattern without losing the target"
    why_human: "Requires Gazebo Garden + ROS 2 Humble + Hailo accelerator + ros_gz_bridge all running on the same machine simultaneously. The test skips cleanly on this machine due to missing RUN_SIM_TESTS=1 and absent gz/ROS prerequisites."
  - test: "Confirm rover physically drives and tracks actor in Gazebo GUI (rows 2-3 of 06-07 scorecard)"
    expected: "Web UI status bar flips to 'Following: ID N'; rover moves toward/tracks the actor in the gz GUI; non-zero twist.linear.x and angular.z appear on gz topic -e -t /cmd_vel during follow"
    why_human: "Visual confirmation of physical motion in the simulator cannot be automated. The E2E test verifies telemetry but operator must observe the GUI."
  - test: "Confirm RINT-02 bottom-edge slow-down fires in running rover sim (row 4 of 06-07 scorecard)"
    expected: "When actor approaches to within ~15% of frame bottom, twist.linear.x drops to 0.0 on gz topic -e -t /cmd_vel while twist.angular.z may stay non-zero"
    why_human: "Although TestBottomEdgeNaturalStop pins the unit behavior, operator needs to confirm it fires at the right moment in the live gz scene. The 0.85 threshold is tied to real pixel geometry only observable at runtime."
  - test: "Confirm RINT-06 SIGINT shutdown in running rover sim (row 5 of 06-07 scorecard)"
    expected: "After Ctrl+C on robot-follow, gz topic -e -t /cmd_vel --duration 3 is silent; pgrep robot-follow returns empty within ~5 s"
    why_human: "TestSigintShutdown pins the mocked adapter contract; operator must verify the real rclpy-over-ROS path silences within 1 s in the live sim."
  - test: "Confirm no FATAL/ERROR in robot-follow console during rover sim run (row 6 of 06-07 scorecard)"
    expected: "grep -E 'FATAL|ERROR' in captured scrollback returns no entries beyond established pre-Phase-6 baseline"
    why_human: "Requires a live run."
  - test: "Drone path not regressed — separate run against PX4 SITL (row 7 of 06-07 scorecard)"
    expected: "robot-follow --robot drone --takeoff-landing against PX4 SITL still follows a person; drone path unchanged from Phase 3 baseline"
    why_human: "Requires PX4 SITL running. Architecture locks were held (mavsdk_drone.py, controller.py all byte-identical across Phase 6), but operator confirmation closes the human gate."
---

# Phase 6: Sim Integration — Verification Report

**Phase Goal:** Close RINT-01..06 — ship the rover-tuned ControllerConfig (`configs/rover_simulation.json`), wire ByteTracker to read from `controller_config`, land the rover bottom-edge slow-down, pin SIGINT shutdown contract, add the deterministic E2E rover sim test, and pass the operator gate.
**Verified:** 2026-05-20
**Status:** human_needed
**Re-verification:** No — initial verification

---

## Goal Achievement

### Observable Truths

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | `configs/rover_simulation.json` exists at repo root, loads via `ControllerConfig.from_json`, and produces rover-safe values (yaw_only=false, kp_yaw=3.0, max_forward=1.0, bytetracker_track_buffer=30) | VERIFIED | File present with exactly 18 keys. `test_rover_simulation_json_loads_with_rover_safe_defaults` PASSED. |
| 2 | ByteTracker knobs are config-driven: `create_app` reads from `controller_config.bytetracker_*`; hardcoded literals removed; drone defaults byte-identical to legacy call | VERIFIED | `hailo_drone_detection_manager.py` reads `_cfg.bytetracker_*`. 6 tests in `test_bytetracker_config.py` all PASS. No `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30` literal at call site. |
| 3 | Rover bottom-edge slow-down: `Ros2RoverAdapter.send_command` zeroes `twist.linear.x` when `safety_ctx.bbox_bottom_norm >= 0.85`; yaw preserved; drone `_apply_retreat_from_tilt` path unchanged | VERIFIED | `ROVER_BOTTOM_STOP_THRESHOLD = 0.85` named constant in `ros2_rover.py`. All 6 `TestBottomEdgeNaturalStop` tests PASS. Drone adapter `_apply_retreat_from_tilt` still uses `bbox_bottom_normalized`. |
| 4 | SIGINT shutdown contract pinned: after `shutdown()`, `send_command` / `send_zero` are no-ops (publisher None); shutdown completes in < 1 s | VERIFIED | `TestSigintShutdown` 3 tests all PASS. Adapter `_publisher` is cleared on shutdown. Unit-level correctness confirmed. |
| 5 | Port isolation documented in `sim/rover/README.md`: "Port usage comparison" table contrasts PX4 SITL (MAVLink UDP 14540) vs rover sim (ROS DDS, no MAVLink) | VERIFIED | Table present with all 7 required substrings. `test_readme_documents_port_isolation_table` PASSED. |
| 6 | RINT-04 deterministic E2E rover sim test (`TestRoverWalkAcrossThenApproach`) passes in the Gazebo rover sim with >= 90 close-approach frames, >= 80% detection, >= 80% ID retention | ? UNCERTAIN (needs operator) | Test exists in `test_sim_worlds.py` with correct skip-on-no-deps guards. On this machine: SKIPPED (RUN_SIM_TESTS not set, gz/ROS absent). Operator must run on wired-up rover box. 06-07 gate recorded `approved-with-deferral`. |

**Score:** 5/6 truths verified (1 needs operator on wired-up box)

---

### Deferred Items

Items addressed by the 06-07 `approved-with-deferral` operator gate — not regressions, not gaps.

| # | Item | Addressed In | Evidence |
|---|------|-------------|----------|
| 1 | Operator-witnessed end-to-end rover sim run (scorecard rows 1-8 in 06-07) | 06-07-SUMMARY.md (approved-with-deferral) | User explicitly chose to defer to a dedicated wired-up session; 06-07 mirrors the 03-14/04-05 deferral pattern. Automated tests provide structural coverage. |

---

### Required Artifacts

| Artifact | Expected | Status | Details |
|----------|----------|--------|---------|
| `configs/rover_simulation.json` | Rover-safe ControllerConfig defaults (18 keys, track_buffer=30, kp_yaw=3.0, yaw_only=false) | VERIFIED | File present. All 18 required keys confirmed. `ControllerConfig.from_json` round-trip passes. |
| `robot_follow/follow_api/config.py` | ControllerConfig + 4 bytetracker_* fields with drone-default values (0.4, 90, 0.5, 30) | VERIFIED | Lines 120-123 confirmed. `test_bytetracker_defaults_match_drone_hardcoded` PASSED. |
| `robot_follow/follow_api/types.py` | SafetyContext + `bbox_bottom_norm: Optional[float] = None`; populated in `from_detection`; None in `lost()` | VERIFIED | Line 115 confirmed. `from_detection` populates at line 125. `lost()` leaves None. Both SafetyContext tests PASS. |
| `robot_follow/tests/test_config_persistence.py` | 4 new tests covering bytetracker defaults + rover JSON override + SafetyContext field | VERIFIED | 4 new tests all PASS. |
| `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` | `create_app` reads from `controller_config.bytetracker_*` (not hardcoded literals) | VERIFIED | Lines 1288-1296 confirmed. Hardcoded literal pattern absent. |
| `robot_follow/robot_follow_app.py` | Path A wiring: `controller_config=ControllerConfig.from_args(pre_args)` passed to `create_app`; `--config` on pre-parser | VERIFIED | Line 451 confirmed. `test_robot_follow_app_passes_controller_config_to_create_app` PASSED. |
| `robot_follow/tests/test_bytetracker_config.py` | 6 tests locking RINT-03 contract | VERIFIED | All 6 tests PASS. |
| `robot_follow/robot_api/adapters/ros2_rover.py` | `ROVER_BOTTOM_STOP_THRESHOLD = 0.85` constant; `send_command` override block | VERIFIED | Constant at line 41. Override block at lines 144-145. |
| `robot_follow/tests/test_ros2_rover_adapter.py` | `TestBottomEdgeNaturalStop` (6 tests) + `TestSigintShutdown` (3 tests) | VERIFIED | Classes at lines 341 and 482. All 9 tests PASS. |
| `sim/rover/README.md` | "Port usage comparison" table with 7 required substrings inside "Port 5600 conflict" section | VERIFIED | Table at lines 142-147. All 7 substrings present. Existing section preserved byte-identical. |
| `robot_follow/tests/test_rover_sim_smoke.py` | `test_readme_documents_port_isolation_table` | VERIFIED | Test at line 274. PASSED. |
| `robot_follow/tests/test_sim_worlds.py` | `TestRoverWalkAcrossThenApproach` + `rover_sim_run` fixture with skip-on-no-deps | VERIFIED (structure) | Class at line 644. Fixture at line 543. `_rover_sim_prereqs_missing()` checks all deps. Skips cleanly. Execution on real sim: needs operator. |

---

### Key Link Verification

| From | To | Via | Status | Details |
|------|----|----|--------|---------|
| `configs/rover_simulation.json` | `ControllerConfig.from_json` | `{f.name for f in fields(cls)}` filter | VERIFIED | 18 keys all match valid ControllerConfig field names. Round-trip test PASSES. |
| `ControllerConfig.bytetracker_*` | `hailo_drone_detection_manager.py::create_app` | `_cfg = controller_config if not None else _CC()` + `create_tracker(track_thresh=_cfg.bytetracker_track_thresh, ...)` | VERIFIED | Lines 1288-1296 confirmed. |
| `robot_follow_app.py::pre-parser` | `create_app(controller_config=ControllerConfig.from_args(pre_args))` | Path A wiring | VERIFIED | Line 451 confirmed. `test_pre_parser_registers_config_flag` PASSES. |
| `SafetyContext.bbox_bottom_norm` | `Ros2RoverAdapter.send_command` | `if safety_ctx.bbox_bottom_norm is not None and >= 0.85: twist.linear.x = 0.0` | VERIFIED | Lines 144-145 of ros2_rover.py confirmed. |
| `Ros2RoverAdapter.shutdown()` | publisher silenced | `_publisher = None` on shutdown + `if self._publisher is None: return` guard in send_command/send_zero | VERIFIED | TestSigintShutdown pins this. |

---

### Data-Flow Trace (Level 4)

Applies to the ByteTracker config wiring (create_app receives real ControllerConfig, not a stub):

| Artifact | Data Variable | Source | Produces Real Data | Status |
|----------|---------------|--------|-------------------|--------|
| `hailo_drone_detection_manager.py::create_app` | `_cfg.bytetracker_track_buffer` | `ControllerConfig.from_args(pre_args)` via Path A | Yes — real config loaded from `--config` arg or defaults | FLOWING |
| `Ros2RoverAdapter.send_command` | `safety_ctx.bbox_bottom_norm` | `SafetyContext.from_detection(det)` in detection callback | Yes — det.center_y + det.bbox_height / 2 computed per frame | FLOWING |

---

### Behavioral Spot-Checks

| Behavior | Command | Result | Status |
|----------|---------|--------|--------|
| rover_simulation.json loads | `python -c "from robot_follow.follow_api.config import ControllerConfig; c = ControllerConfig.from_json('configs/rover_simulation.json'); assert c.bytetracker_track_buffer == 30 and c.kp_yaw == 3.0 and c.yaw_only == False"` | Assertion passes | PASS |
| ByteTracker drone defaults byte-identical | `python -c "from robot_follow.follow_api.config import ControllerConfig; c = ControllerConfig(); assert (c.bytetracker_track_thresh, c.bytetracker_track_buffer, c.bytetracker_match_thresh, c.bytetracker_frame_rate) == (0.4, 90, 0.5, 30)"` | Assertion passes | PASS |
| SafetyContext bbox_bottom_norm contract | `python -c "from robot_follow.follow_api.types import Detection, SafetyContext; det = Detection(label='person', confidence=0.9, center_x=0.5, center_y=0.75, bbox_height=0.30, timestamp=0.0); ctx = SafetyContext.from_detection(det); assert abs(ctx.bbox_bottom_norm - 0.90) < 1e-9; assert SafetyContext.lost().bbox_bottom_norm is None"` | Assertion passes | PASS |
| Hardcoded ByteTracker literals removed from call site | `grep -c 'track_thresh=0\.4, track_buffer=90, match_thresh=0\.5, frame_rate=30' robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` | Returns 0 (empty) | PASS |
| TestRoverWalkAcrossThenApproach skip-clean | `python -m pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v` | SKIPPED (RUN_SIM_TESTS not set) | SKIP (expected — needs wired box) |

---

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
|-------------|------------|-------------|--------|----------|
| RINT-01 | 06-01 | `configs/rover_simulation.json` with rover-safe defaults | SATISFIED | File present, loads cleanly, all 4 tests PASS |
| RINT-02 | 06-01 + 06-04 | Bottom-edge slow-down in rover adapter; drone retreat-from-tilt unchanged | SATISFIED | `ROVER_BOTTOM_STOP_THRESHOLD=0.85` in ros2_rover.py. `TestBottomEdgeNaturalStop` 6 tests PASS. Drone `_apply_retreat_from_tilt` unchanged. |
| RINT-03 | 06-01 + 06-03 | ByteTracker knobs config-driven (not hardcoded) | SATISFIED | Hardcoded literals removed from create_app. `test_bytetracker_config.py` 6 tests PASS. |
| RINT-04 | 06-06 | Deterministic E2E rover sim test in test_sim_worlds.py | SATISFIED (structure) / NEEDS OPERATOR (execution) | TestRoverWalkAcrossThenApproach exists with correct assertions and skip-on-no-deps. Execution on real sim box: deferred (06-07 approved-with-deferral). |
| RINT-05 | 06-02 | Port isolation documented in sim/rover/README.md | SATISFIED | "Port usage comparison" table present. All 7 required substrings. Regression-guard test PASSES. |
| RINT-06 | 06-05 | SIGINT shutdown: zero /cmd_vel after Ctrl+C; rclpy node cleanly destroyed | SATISFIED (unit) / NEEDS OPERATOR (integration) | TestSigintShutdown 3 tests PASS (mocked). Operator confirmation in live sim: deferred (06-07). |

---

### Anti-Patterns Found

| File | Line | Pattern | Severity | Impact |
|------|------|---------|----------|--------|
| No TBD / FIXME / XXX markers found in Phase 6 modified files | — | — | — | — |

No debt markers. No stub patterns. No hardcoded empty returns in the Phase 6 code paths.

---

### Architectural Locks Verified

All Phase 6 must-not-touch files confirmed byte-identical across all 7 plans (06-01..06-07):

- `robot_follow/robot_api/adapters/mavsdk_drone.py` — 0 lines changed vs Phase 6 start
- `robot_follow/follow_api/controller.py` — 0 lines changed
- `sim/bridge/video_bridge.py` — 0 lines changed
- `sim/rover/` SDF/scripts (model.sdf, model.config, worlds/*.sdf, start_rover_sim.sh) — 0 lines changed (only README.md was modified by 06-02)
- `Capabilities` dataclass — no behavioral flags added; axes-only contract intact (`axes: frozenset[Axis]`, `yaw_unit: Literal["deg/s", "rad/s"]` only)

---

### Human Verification Required

The following items cannot be automated — they require a wired-up rover sim session (Gazebo Garden + ROS 2 Humble + Hailo accelerator). These mirror the 06-07 deferred scorecard exactly.

#### 1. RINT-04 End-to-End Deterministic Test

**Test:** `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v`
**Expected:** PASSED (not SKIPPED, not FAILED) — >= 90 close-approach frames, >= 80% detection rate, >= 80% ID retention in the window
**Why human:** Requires Gazebo Garden + ROS 2 Humble + Hailo accelerator + rover sim running simultaneously. Test SKIPs cleanly on machines without these prerequisites.

#### 2. Rover Physically Drives and Tracks in gz GUI

**Test:** Launch `sim/rover/start_rover_sim.sh --world walk_across_then_approach` + `robot-follow --robot rover --input udp://0.0.0.0:5600 --config configs/rover_simulation.json --webui`. Observe gz GUI.
**Expected:** Rover spawns; walking actor visible; web UI status flips to "Following: ID N"; rover physically follows actor's path; `gz topic -e -t /cmd_vel` shows non-zero linear.x and angular.z during follow
**Why human:** Visual confirmation of physical motion in Gazebo is not automatable.

#### 3. RINT-02 Bottom-Edge Slow-Down in Live Sim

**Test:** During rover sim run, watch `gz topic -e -t /cmd_vel` as the actor approaches the rover (close enough to be in the bottom ~15% of frame).
**Expected:** `twist.linear.x` drops to 0.0 (while `twist.angular.z` may stay non-zero for recentering)
**Why human:** Unit tests (TestBottomEdgeNaturalStop) pin the mocked adapter behavior; operator confirms the threshold fires at the correct visual moment in the real sim.

#### 4. RINT-06 SIGINT Clean Shutdown in Live Sim

**Test:** After rover sim run, Ctrl+C on robot-follow. Then: `gz topic -e -t /cmd_vel --duration 3`.
**Expected:** No `/cmd_vel` messages in the 3-second window; `pgrep robot-follow` returns empty within ~5 s
**Why human:** TestSigintShutdown pins mocked adapter; operator confirms real rclpy node + spin thread stop cleanly in the full-process scenario.

#### 5. No FATAL/ERROR in Console

**Test:** `grep -E "FATAL|ERROR"` in the captured robot-follow scrollback from the rover sim run.
**Expected:** Zero entries beyond the established pre-Phase-6 baseline
**Why human:** Requires a live run.

#### 6. Drone Path Not Regressed

**Test:** Separately run `sim/start_sim.sh --bridge --world person_in_front` + `robot-follow --robot drone --takeoff-landing` and confirm drone still acquires + follows.
**Expected:** Drone follows a person; no regressions
**Why human:** Requires PX4 SITL. Code-side evidence (mavsdk_drone.py byte-identical across all Phase 6 plans) is compelling but operator confirmation closes the gate.

---

### Gaps Summary

No blocking gaps. The 1 UNCERTAIN truth (RINT-04 E2E execution) is gated by hardware/environment availability, not by absent or broken code. The test, fixture, and skip-on-no-deps infrastructure are all substantively correct and verified. The `approved-with-deferral` resume signal in 06-07-SUMMARY.md is the documented operator decision to defer the eyes-on session.

The pre-existing `test_friendly_error_when_rclpy_missing` failure (1 test in `test_ros2_rover_adapter.py`) is confirmed NOT a Phase 6 regression — it fails because `rclpy` IS installed on this dev machine. Excluded from the Phase 6 score per the stated pre-conditions.

**Overall:** Phase 6 code is complete and automated tests are green. The outstanding items are all in the operator-witnessed column of the 06-07 scorecard. The `human_needed` status routes these to the HUMAN-UAT surface for closure by the operator.

---

_Verified: 2026-05-20_
_Verifier: Claude (gsd-verifier)_
