---
status: partial
phase: 06-sim-integration
source: [06-VERIFICATION.md, 06-07-SUMMARY.md]
started: 2026-05-20T16:00:00Z
updated: 2026-05-20T16:00:00Z
---

## Current Test

[awaiting human testing on a wired-up rover sim box]

## Tests

### 1. TestRoverWalkAcrossThenApproach passes on the wired-up box
expected: `RUN_SIM_TESTS=1 python -m pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v` reports PASSED (not SKIPPED, not FAILED). Requires gz Garden + ROS 2 Humble + Hailo accelerator.
result: [pending]

### 2. Rover physically drives + tracks the actor in gz GUI
expected: After `./sim/rover/start_rover_sim.sh --world walk_across_then_approach` + `robot-follow --robot rover --input udp://0.0.0.0:5600 --config configs/rover_simulation.json --webui`, the rover model in the gz GUI follows the walking actor through the walk pattern. Web UI status bar shows "Following: ID <N>".
result: [pending]

### 3. RINT-02 bottom-edge slow-down fires at the right moment
expected: As the actor enters the lowest 15% of the frame (bbox_bottom_norm ≥ 0.85), `gz topic -e -t /cmd_vel` shows `twist.linear.x` drop to 0.0 while `twist.angular.z` may stay non-zero (yaw preserved). Rover stops driving forward but can still rotate to recenter.
result: [pending]

### 4. RINT-06 live SIGINT cleanly stops the rover
expected: After Ctrl+C on `robot-follow`, `gz topic -e -t /cmd_vel --duration 3` returns no further messages within ~1 s. `pgrep robot-follow` returns empty within ~5 s.
result: [pending]

### 5. No FATAL/ERROR in robot-follow console scrollback
expected: After a full ~90s rover sim run, `grep -E "FATAL|ERROR"` against the captured robot-follow scrollback returns no entries beyond established pre-Phase-6 baseline.
result: [pending]

### 6. Drone path not regressed (separate PX4 SITL run)
expected: `sim/start_sim.sh --bridge --world person_in_front` + `robot-follow --robot drone --takeoff-landing` — drone still acquires + follows a person. Confirms the controller_config wiring change in 06-03 didn't regress the drone path (already proven byte-identical in code; this is the eyes-on confirmation).
result: [pending]

## Summary

total: 6
passed: 0
issues: 0
pending: 6
skipped: 0
blocked: 0

## Gaps
