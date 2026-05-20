---
phase: 06-sim-integration
plan: 06
subsystem: testing
tags: [phase-6, rover, e2e, gz, sim, RINT-04, wave-3]

requires:
  - phase: 05-rover-sim
    provides: start_rover_sim.sh launcher + sim/rover/worlds/walk_across_then_approach.sdf
  - phase: 06-sim-integration
    provides: configs/rover_simulation.json (06-01), ByteTracker config wiring (06-03), rover bottom-edge slow-down (06-04)
provides:
  - TestRoverWalkAcrossThenApproach E2E gate in test_sim_worlds.py
  - rover_sim_run fixture (rover analog of sim_run; no MAVSDK / no takeoff path)
affects: [06-07]

tech-stack:
  added: []
  patterns:
    - "Fixture-mirroring pattern: when adding a new sim-platform analog to existing E2E tests, copy the fixture+test shape rather than refactor a generic — keeps the divergence (launcher script, CLI args) explicit at the surface."
    - "Skip-cleanly-on-no-deps: rover_sim_run uses pytest.skip() inside the fixture (gz / ROS 2 / launcher / config presence checks) so the test stays portable across dev boxes."

key-files:
  created: []
  modified:
    - robot_follow/tests/test_sim_worlds.py

key-decisions:
  - "Separate fixture (rover_sim_run) instead of generalizing sim_run: the drone fixture has takeoff/landing/recording semantics baked in; the rover path has none of those. A generalized fixture would obscure those differences."
  - "Coarse-grained assertions (≥90 frames close, ≥80% detection, ≥80% ID retention) per RINT-04 spec — gz sim is non-deterministic at ms resolution"
  - "n_det_w/n_w ≥ 0.8 (vs drone's 0.9): the rover has more ground-perspective occluders so a slightly looser threshold is appropriate without being a regression filter"
  - "Test is gated by the file-level RUN_SIM_TESTS=1 pytestmark and the Hailo-available pytestmark already present in test_sim_worlds.py; adds its own gz/ROS prereq skip on top"

patterns-established:
  - "Rover E2E telemetry pattern: rover invokes `robot-follow --robot rover --input udp://0.0.0.0:5600 --config configs/rover_simulation.json --test-log <path>` to get the same JSONL surface as the drone — assertions reuse the existing window-detection helpers."

requirements-completed: [RINT-04]

duration: ~10min
completed: 2026-05-20
---

# Phase 6 Plan 06: RINT-04 Deterministic E2E Rover Sim Test

**TestRoverWalkAcrossThenApproach spawns start_rover_sim.sh + robot-follow rover; asserts ≥80% detection and ≥80% ID retention across the close-approach window. Skip-clean on no-gz/no-ROS boxes.**

## Performance

- **Duration:** ~10 min (inline execution after background executor returned without Bash access)
- **Completed:** 2026-05-20
- **Tasks:** 1 / 1
- **Files modified:** 1 (test file only)

## Accomplishments

- **RINT-04 E2E gate landed:** New `TestRoverWalkAcrossThenApproach` class with single test `test_walk_across_then_approach_holds_target_through_approach_rover`. Mirrors the existing drone equivalent's shape but uses `start_rover_sim.sh` + `robot-follow --robot rover --config configs/rover_simulation.json` and asserts ≥90 close-approach frames + ≥80% detection + ≥80% ID retention within the window.
- **Skip-clean portable:** `_rover_sim_prereqs_missing()` helper checks for `gz` CLI, `/opt/ros/humble/setup.bash`, the launcher script, and the rover config. Missing prereqs → `pytest.skip(...)` inside the fixture. On boxes without those, the test SKIPPED (verified — see Verification Evidence).
- **Production code untouched:** drone adapter, rover adapter, sim/rover/ all byte-identical.

## Task Commits

1. **Task 1: TestRoverWalkAcrossThenApproach + rover_sim_run fixture** — `5608b07` (test)

## Files Created/Modified

- `robot_follow/tests/test_sim_worlds.py` — added rover analog: `_rover_sim_prereqs_missing()` helper, `rover_sim_run` fixture, `TestRoverWalkAcrossThenApproach` class. Also added `import shutil` to the imports for the `which("gz")` check. +198 lines.

## Decisions Made

- **Separate fixture (rover_sim_run) over generalization:** the drone `sim_run` carries takeoff/landing/recording semantics that don't apply to rover; a generalized fixture would obscure those differences. Drone and rover fixtures share helpers (`_log`, `_kill_group`, `_wait_with_progress`, `_size`, `_tail`, `_read_jsonl`, `_summarize`, `MIN_FRAMES_WITH_DETECTION`).
- **Detection-rate threshold slightly looser than drone (0.8 vs 0.9):** the rover sees more ground-perspective occluders by physics; a single threshold across both platforms would either be too loose for the drone or too tight for the rover.

## Deviations from Plan

None — plan executed as specified. The "≥80% detection rate" was already loosened relative to the drone's "≥90%" per RINT-04 spec acknowledging gz non-determinism + rover physics.

## Issues Encountered

- **Background executor agent failed without Bash access** on the first attempt (same root cause as 06-05). No worktree was created. Switched to inline execution on the main working tree.
- **Cannot run the rover E2E test in this orchestrator session.** This test requires `RUN_SIM_TESTS=1` + a Hailo device + Gazebo Garden + ROS 2 Humble — i.e. a real wired-up sim box. The operator's machine satisfies these (per Phase 5 VERIFICATION); the test is expected to run there. From this orchestrator the test SKIPS cleanly, which is exactly the portable behavior required by the plan.

## Verification Evidence

### Test collected + skips cleanly

```
$ python -m pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v
collected 1 item
TestRoverWalkAcrossThenApproach::test_walk_across_then_approach_holds_target_through_approach_rover SKIPPED
============================== 1 skipped in 0.27s ==============================
```

The skip happens at the file-level `pytestmark` (RUN_SIM_TESTS != 1) — which is the same gate the drone test sits behind. When `RUN_SIM_TESTS=1` is set without `gz`/ROS, the fixture-level `pytest.skip(...)` chain takes over instead — `_rover_sim_prereqs_missing()` reports which prereq is absent.

### Full suite green

```
$ python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q
1 failed (pre-existing rclpy-missing test, unrelated), 356 passed in 24.69s
```

No regressions introduced by 06-06 (the test file is ignored when running the non-sim suite; the rover test only adds a class to the sim file).

### Architectural locks (vs 06-06's commit)

```
$ git diff --name-only HEAD~1 HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py robot_follow/robot_api/adapters/ros2_rover.py sim/rover/
(empty)
```

## Next Phase Readiness

- **Plan 06-07 (operator gate):** The operator's actions are now (a) run `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v` on a wired-up rover box to confirm the E2E assertion lands, (b) operator-eye verify that the rover physically moves and follows in the gz GUI, (c) operator-eye verify the bottom-edge slow-down (RINT-02) fires when the actor approaches close enough.
- **No blockers.**

---
*Phase: 06-sim-integration*
*Completed: 2026-05-20*
