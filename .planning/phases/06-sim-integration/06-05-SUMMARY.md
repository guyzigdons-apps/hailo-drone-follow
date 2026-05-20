---
phase: 06-sim-integration
plan: 05
subsystem: testing
tags: [phase-6, rover, sigint, shutdown, RINT-06, wave-3]

requires:
  - phase: 04-rover-adapter
    provides: Ros2RoverAdapter.shutdown contract (idempotent, bounded duration, post-shutdown publisher cleared)
provides:
  - TestSigintShutdown class (3 tests) pinning RINT-06 silence + timing contract
affects: [06-07]

tech-stack:
  added: []
  patterns:
    - "Contract-pinning test pattern: when production code from a prior phase already implements a contract, add tests that pin it so future refactors can't silently regress."

key-files:
  created: []
  modified:
    - robot_follow/tests/test_ros2_rover_adapter.py

key-decisions:
  - "Test-only plan, zero production code change: the adapter shutdown contract is already correct from Phase 4; this plan pins it with tests."
  - "Three RINT-06-specific tests (post-shutdown send_command silence + post-shutdown send_zero silence + <1.0s timing); existing TestLifecycle tests for idempotency/event-set/destroy-ordering NOT duplicated"

patterns-established: []

requirements-completed: [RINT-06]

duration: ~5min
completed: 2026-05-20
---

# Phase 6 Plan 05: RINT-06 SIGINT Shutdown Tests

**TestSigintShutdown pins the Phase-4 shutdown contract: post-shutdown send_command/send_zero are no-ops (publisher cleared), and shutdown completes in < 1.0 s.**

## Performance

- **Duration:** ~5 min (inline execution after background executor returned without Bash access)
- **Completed:** 2026-05-20
- **Tasks:** 1 / 1
- **Files modified:** 1 (test file only)

## Accomplishments

- **RINT-06 contract pinned:** New `TestSigintShutdown` class with 3 tests asserting (a) `send_command` after shutdown does not publish (publisher is None → early-return guard fires), (b) `send_zero` after shutdown does not publish, (c) `shutdown()` completes in < 1.0 s with the mocked executor.
- **Production code untouched:** `ros2_rover.py` byte-identical; the adapter's shutdown was already correct from Phase 4.
- **Drone path untouched:** `mavsdk_drone.py` byte-identical.

## Task Commits

1. **Task 1: TestSigintShutdown** — `f065123` (test)

## Files Created/Modified

- `robot_follow/tests/test_ros2_rover_adapter.py` — appended `TestSigintShutdown` class with 3 tests; also added `import time` to the imports block (used by the timing test)

## Decisions Made

None beyond what the plan locked. Inline execution (not worktree) used because the background executor agent returned without Bash access before creating its worktree.

## Deviations from Plan

None — three tests added exactly as specified.

## Issues Encountered

- **Background executor agent failed without Bash access** on the first attempt (returned message "I need Bash access to execute this plan"). No worktree was created. Switched to inline execution on the main working tree.

## Verification Evidence

```
$ python -m pytest robot_follow/tests/test_ros2_rover_adapter.py::TestSigintShutdown -v
test_post_shutdown_send_command_does_not_publish    PASSED
test_post_shutdown_send_zero_does_not_publish       PASSED
test_shutdown_completes_within_1s                   PASSED
============================== 3 passed in 0.38s ===============================
```

Adapter unchanged:
```
$ git diff --name-only HEAD~1 HEAD -- robot_follow/robot_api/adapters/ros2_rover.py robot_follow/robot_api/adapters/mavsdk_drone.py
(empty)
```

## Next Phase Readiness

- **Plan 06-07 (operator gate):** RINT-06 is now formally pinned by automated tests; the operator gate can rely on `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestSigintShutdown` as the structural witness.
- **No blockers.**

---
*Phase: 06-sim-integration*
*Completed: 2026-05-20*
