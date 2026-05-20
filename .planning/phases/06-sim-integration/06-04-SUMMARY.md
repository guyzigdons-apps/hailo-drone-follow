---
phase: 06-sim-integration
plan: 04
subsystem: rover
tags: [phase-6, rover, bottom-edge, slow-stop, RINT-02, wave-2]

requires:
  - phase: 06-sim-integration
    provides: SafetyContext.bbox_bottom_norm (06-01) — the field this plan reads from the rover adapter
provides:
  - ROVER_BOTTOM_STOP_THRESHOLD = 0.85 named constant in ros2_rover.py
  - send_command override: twist.linear.x = 0.0 when bbox_bottom_norm >= 0.85; yaw preserved
  - TestBottomEdgeNaturalStop (6 tests) covering below/at/above threshold + yaw preserved + None + constant
affects: [06-05, 06-06]

tech-stack:
  added: []
  patterns:
    - "Adapter-local tuning: rover-specific safety threshold lives in the rover adapter as a NAMED CONSTANT (not in Capabilities, not in ControllerConfig). Per ABS-05 + feedback_robot_abstraction_axes_only.md — the controller stays robot-agnostic; the adapter owns robot-specific physical limits."
    - "twist-only override: send_command mutates the about-to-publish twist, never RobotCommand (cmd stays immutable for upstream)."
    - "Yaw preservation under forward-stop: when forward is zeroed for safety, yaw stays at cmd.yaw_rate so the robot can still rotate to recenter the target."

key-files:
  created: []
  modified:
    - robot_follow/robot_api/adapters/ros2_rover.py
    - robot_follow/tests/test_ros2_rover_adapter.py

key-decisions:
  - "Named constant (not magic number): ROVER_BOTTOM_STOP_THRESHOLD = 0.85 at module top — grep-friendly tuning site"
  - "twist-only override (not RobotCommand mutation): the controller's emitted cmd stays clean; only the publish-side twist is rewritten"
  - "Yaw preserved when forward stops: rover can rotate to recenter even when motion forward is suppressed; freezing both axes would be the wrong shape"
  - "Defensive None check: `bbox_bottom_norm is not None` first — belt-and-braces against any SafetyContext built without from_detection (e.g. SafetyContext.lost())"
  - "No Capabilities flag: per ABS-05 + feedback_robot_abstraction_axes_only.md, axes-only contract preserved. A future drone with different physics gets its own adapter-specific bottom-edge logic."

patterns-established:
  - "Bottom-edge slow-stop pattern: adapter reads safety_ctx.bbox_bottom_norm, compares to a named threshold, overrides the relevant twist axis to 0 while preserving orthogonal axes."

requirements-completed: [RINT-02]

duration: ~10min
completed: 2026-05-20
---

# Phase 6 Plan 04: Rover Bottom-Edge Slow-Down (RINT-02)

**Rover adapter publishes `twist.linear.x = 0.0` when `safety_ctx.bbox_bottom_norm >= 0.85` (named constant `ROVER_BOTTOM_STOP_THRESHOLD`); yaw preserved; drone path untouched.**

## Performance

- **Duration:** ~10 min
- **Completed:** 2026-05-20
- **Tasks:** 2 / 2
- **Files modified:** 2 (1 adapter, 1 test file)

## Accomplishments

- **RINT-02 wire path landed:** `Ros2RoverAdapter.send_command` reads `safety_ctx.bbox_bottom_norm` and, when ≥ `ROVER_BOTTOM_STOP_THRESHOLD` (0.85), overrides `twist.linear.x = 0.0` immediately before publish. Yaw axis is preserved.
- **Named-constant locked:** `ROVER_BOTTOM_STOP_THRESHOLD: float = 0.85` declared at module top of `ros2_rover.py` with a comment block citing RINT-02 + Q1 lock + Phase 6 user decision. Tuning happens at one greppable site.
- **6 new tests added:** `TestBottomEdgeNaturalStop` covers below threshold (0.80 — forward passes through), at threshold (0.85 — forward zeroed), above threshold (0.95), yaw preserved when forward zeroed, None bbox_bottom_norm short-circuits, and named-constant value lock.
- **Drone path provably unchanged:** `mavsdk_drone.py` BYTE-IDENTICAL across all 4 Phase 6 plans so far (06-01..06-04). The drone adapter never reads `bbox_bottom_norm` — it continues to read `bbox_bottom_normalized` via `_apply_retreat_from_tilt`.

## Task Commits

1. **Task 1: ros2_rover.py override + named constant** — `6009bce` (feat)
2. **Task 2: TestBottomEdgeNaturalStop with 6 tests** — `bb736ab` (test)

## Files Created/Modified

- `robot_follow/robot_api/adapters/ros2_rover.py` — added `ROVER_BOTTOM_STOP_THRESHOLD: float = 0.85` module constant; added conditional override in `send_command` before publish
- `robot_follow/tests/test_ros2_rover_adapter.py` — appended `TestBottomEdgeNaturalStop` class with 6 PASSING tests

## Decisions Made

All locked at plan time. No new decisions during execution. The 6th test (`test_none_bbox_bottom_norm_forward_passes_through`) was an optional addition in the plan; included for the belt-and-braces guard against minimal SafetyContext callers.

## Deviations from Plan

None — plan executed exactly as written. The agent stalled on the stream watchdog after writing both code commits and verifying tests passed, but before it could write SUMMARY.md. The orchestrator wrote and committed this SUMMARY.md after the worktree had completed its substantive work.

## Issues Encountered

- **Stream watchdog timeout** on the executor agent after verification completed; both implementation commits (`6009bce`, `bb736ab`) landed correctly before the stall. Per the workflow's completion-signal fallback, the orchestrator spot-checked: 2 commits present, 6 tests passing, locks clean — work was substantively complete, so the orchestrator wrote SUMMARY.md inline and merged the worktree.

## Verification Evidence

### Named constant declaration (ros2_rover.py:30-41)

```python
# RINT-02 / Q1 lock (Phase 6 CONTEXT, 2026-05-20): rover-specific slow-down
# when the person's bbox bottom is in the lowest 15% of the frame (the actor
# is too close to the rover; stop driving forward to avoid a collision).
# YAW IS PRESERVED — the rover can still rotate to recenter, just not drive
# forward. The drone adapter does NOT read this threshold; its existing
# retreat-from-tilt behavior operates on SafetyContext.bbox_bottom_normalized
# (the legacy field) via mavsdk_drone.py::_apply_retreat_from_tilt.
#
# Named constant per Phase 6 user decision: this is NOT a magic number
# embedded in send_command. Tuning happens here, ONE site.
ROVER_BOTTOM_STOP_THRESHOLD: float = 0.85
```

### send_command override block (ros2_rover.py:144-145)

```python
if (safety_ctx.bbox_bottom_norm is not None
        and safety_ctx.bbox_bottom_norm >= ROVER_BOTTOM_STOP_THRESHOLD):
    twist.linear.x = 0.0
```

(Placed AFTER `twist.linear.x = cmd.forward_m_s` and `twist.angular.z = cmd.yaw_rate`, immediately BEFORE `self._publisher.publish(twist)`.)

### TestBottomEdgeNaturalStop signatures (test_ros2_rover_adapter.py:340-)

```
340: class TestBottomEdgeNaturalStop:
368:     def test_rover_bottom_stop_threshold_constant_value(self, rclpy_mock)
383:     def test_below_threshold_forward_unchanged(self, rclpy_mock)
403:     def test_at_threshold_forward_zeroed(self, rclpy_mock)
423:     def test_above_threshold_forward_zeroed(self, rclpy_mock)
436:     def test_above_threshold_yaw_preserved(self, rclpy_mock)
458:     def test_none_bbox_bottom_norm_forward_passes_through(self, rclpy_mock)
```

### Test results

```
$ python -m pytest robot_follow/tests/test_ros2_rover_adapter.py::TestBottomEdgeNaturalStop -v
test_rover_bottom_stop_threshold_constant_value         PASSED
test_below_threshold_forward_unchanged                  PASSED
test_at_threshold_forward_zeroed                        PASSED
test_above_threshold_forward_zeroed                     PASSED
test_above_threshold_yaw_preserved                      PASSED
test_none_bbox_bottom_norm_forward_passes_through       PASSED
============================== 6 passed in 0.36s ===============================
```

### Architectural locks (all empty across both 06-04 commits)

- `robot_follow/robot_api/adapters/mavsdk_drone.py`: empty diff
- `robot_follow/follow_api/`: empty diff (06-01 already landed all needed changes)
- `robot_follow/pipeline_adapter/`: empty diff (06-03's territory)
- `robot_follow/robot_follow_app.py`: empty diff (06-03's territory)
- `sim/rover/`: empty diff (Phase 5 architectural lock)
- `sim/bridge/video_bridge.py`: empty diff (RSIM-06 architectural lock)

### Capabilities axes-only contract held

No new fields added to `Capabilities` — the threshold lives in the rover adapter only.

## Next Phase Readiness

- **Plan 06-05 (RINT-06):** `TestBottomEdgeNaturalStop` is the new class in `test_ros2_rover_adapter.py`; Plan 06-05 will add `TestSigintShutdown` as a separate class to the same file (Wave 3).
- **Plan 06-06 (RINT-04):** Rover sim E2E test will exercise this override path implicitly when the actor walks close enough that `bbox_bottom_norm` crosses 0.85; the close-approach window assertion (`n_id_w / n_w >= 0.8`) is sufficient to detect a regression of the override (a runaway rover crashing into the actor would tank the close-frame ratio).
- **No blockers** for Wave 3.

---
*Phase: 06-sim-integration*
*Completed: 2026-05-20*
