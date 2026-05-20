---
phase: 04-rover-adapter
plan: 01
subsystem: testing
tags: [phase-4, rover, ros2, rclpy, test-scaffold, xfail, wave-0, tdd-red]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: Robot Protocol, Capabilities, RobotCommand, SafetyContext, MavsdkDroneAdapter reference shape, test_cli_help_dispatch.py existing 7 tests
provides:
  - 20 xfail-marked test cases scaffolding the ros2_rover adapter contract (TestImportSafety, TestProtocolShape, TestSignalHandlerPreservation, TestTwistPublish, TestLifecycle, TestCustomCliArgs)
  - 6 xfail-marked test cases scaffolding the rover CLI dispatch contract (--cmd-vel-topic / --ros-namespace / --ros-domain-id include + drone-help disjoint)
  - rclpy_mock fixture pattern (monkeypatch.setitem on sys.modules) reusable in Plan 04-03
affects: [04-02, 04-03, 04-04, 04-05]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "rclpy mock fixture via monkeypatch.setitem(sys.modules, ...) — auto-rollback per test, prevents mock leakage across the suite"
    - "Lazy adapter imports inside test bodies — pytest collection succeeds without the target module existing yet"
    - "xfail(strict=False) markers as a documented contract between waves — Plan N writes the failing test, Plan N+1 strips the marker"

key-files:
  created:
    - robot_follow/tests/test_ros2_rover_adapter.py
  modified:
    - robot_follow/tests/test_cli_help_dispatch.py

key-decisions:
  - "Use monkeypatch.setitem(sys.modules, ...) for rclpy mocking — auto-rolls back at teardown, prevents threat T-04-01-01 (mock leakage)"
  - "All xfail markers use strict=False so Plan 04-02 / 04-03 can strip them without an intermediate XPASS-as-FAIL state"
  - "Wrote 6 standalone CLI-dispatch test functions (rather than 2 parametrize blocks) so each case carries its own @pytest.mark.xfail decorator on a dedicated line — satisfies the literal acceptance criterion `grep -cE '@pytest.mark.xfail' >= 6`"

patterns-established:
  - "Phase-4 Wave-0 test scaffold pattern: failing-contract tests land before production code; xfail reasons name the closing plan; strict=False prevents intermediate failure"
  - "Architectural lock verification via pathspec commits + post-commit `git diff HEAD~N -- <locked-file>` empty assertion"

requirements-completed: []  # Wave 0 establishes the contract; ROVER-01..08 close in 04-02 / 04-03 / 04-04 when production code lands

# Metrics
duration: ~25 min
completed: 2026-05-20
---

# Phase 4 Plan 01: ros2_rover Wave-0 Test Scaffold Summary

**26 xfail-marked test cases (20 adapter contract + 6 CLI-dispatch contract) lock the failing contract for Phase 4 BEFORE any production rover code lands; rclpy is mocked via monkeypatch.setitem so the file collects on a no-rclpy dev box, and the existing 7 PASS dispatch tests are byte-identical**

## Performance

- **Duration:** ~25 min
- **Started:** 2026-05-20T08:36Z (approx)
- **Completed:** 2026-05-20T09:01:52Z
- **Tasks:** 2 (both auto + tdd)
- **Files created:** 1 (`robot_follow/tests/test_ros2_rover_adapter.py`)
- **Files modified:** 1 (`robot_follow/tests/test_cli_help_dispatch.py`)
- **Production code touched:** 0 lines (Wave 0 — scaffold-only)

## Accomplishments

- `robot_follow/tests/test_ros2_rover_adapter.py` created with **20 xfail-marked tests across 6 classes**:
  | Class                          | # tests | Closes via |
  | TestImportSafety               | 2       | 04-03      |
  | TestProtocolShape              | 3       | 04-03      |
  | TestSignalHandlerPreservation  | 2       | 04-03 (ROVER-02 / ROVER-08) |
  | TestTwistPublish               | 5       | 04-03 (ROVER-06) |
  | TestLifecycle                  | 5       | 04-03      |
  | TestCustomCliArgs              | 3       | 04-03 (ROVER-05) |
  | **Total**                      | **20**  |            |
- `rclpy_mock` fixture implemented via `monkeypatch.setitem(sys.modules, ...)` for rclpy, rclpy.executors, rclpy.node, rclpy.signals, geometry_msgs, geometry_msgs.msg — auto-rollback at teardown (mitigates threat T-04-01-01).
- `test_friendly_error_when_rclpy_missing` deliberately removes the mocks via `monkeypatch.delitem(sys.modules, "rclpy")` and asserts both required substrings: `"ROS 2 not"` AND `"source /opt/ros/humble/setup.bash"` (locks the ROVER-04 friendly-error message that 04-03's adapter must emit verbatim).
- `test_sigint_handler_survives_connect` (ROVER-08 smoke) present with the exact body specified in RESEARCH § "Test classes (TestSignalHandlerPreservation)" — `signal.getsignal(signal.SIGINT) is fake_handler` assertion.
- `robot_follow/tests/test_cli_help_dispatch.py` extended with **6 standalone xfail-marked test functions** (3 includes + 3 excludes); existing 7 tests byte-identical; `ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]` constant exported.

## Task Commits

1. **Task 1: Create test_ros2_rover_adapter.py with ~20 xfail tests** — `c538bc1` (test)
2. **Task 2: Append 6 rover CLI dispatch xfail tests** — `440204d` (test)

_(SUMMARY metadata commit follows.)_

## Files Created/Modified

- `robot_follow/tests/test_ros2_rover_adapter.py` (NEW, 488 lines) — Phase 4 Wave 0 failing-contract scaffold for the ros2_rover adapter; 20 xfail tests + rclpy_mock fixture + helpers.
- `robot_follow/tests/test_cli_help_dispatch.py` (+65 lines, append-only) — adds ROVER_ONLY_FLAGS constant, XFAIL_REASON_ROVER_DISPATCH explainer, and 6 standalone xfail-marked rover-flag test functions. Existing 7 tests (lines 51-69 of the new file) byte-identical to the pre-plan file.

## Decisions Made

1. **monkeypatch.setitem for rclpy mocking** — per the plan's threat model (T-04-01-01) this is the only safe primitive; direct `sys.modules[...] = mock` would leak across tests. Verified via running the full suite: no spurious xfails or FAILures elsewhere from mock bleed.
2. **strict=False on every xfail marker** — Plan 04-02 closes the CLI-dispatch xfails, Plan 04-03 closes the adapter xfails. With `strict=True`, any test that started passing during the intermediate state between 04-02 and 04-03 would convert to FAIL. `strict=False` keeps the suite GREEN throughout the strip-down.
3. **6 standalone test functions vs. 2 parametrized functions for CLI dispatch** — see Deviations below; chose 6 standalones to satisfy the literal `grep -cE '@pytest.mark.xfail' >= 6` acceptance criterion.
4. **No production code in this plan** — `robot_follow/robot_api/adapters/ros2_rover.py` does NOT exist yet (lands in 04-03); `add_rover_args` body unchanged (Plan 04-02 fills it). Test bodies use lazy `from … import Ros2RoverAdapter` inside the test functions so pytest collection succeeds without the module existing.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] Plan-internal contradiction between example code and grep acceptance criterion**

- **Found during:** Task 2 (after first commit attempt to test_cli_help_dispatch.py)
- **Issue:** The plan's `<action>` block showed the new tests as:
  ```python
  @pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)
  @pytest.mark.parametrize("flag", ROVER_ONLY_FLAGS)
  def test_rover_help_includes_rover_flag(flag: str): ...
  ```
  Two such functions → only **2** `@pytest.mark.xfail` decorator lines in the file at runtime, yet `<verify>` and `<success_criteria>` require `grep -cE '@pytest.mark.xfail' robot_follow/tests/test_cli_help_dispatch.py >= 6`. The plan's example contradicts its own grep criterion.
- **Fix:** Rewrote the 6 cases as 6 standalone functions (one per rover flag × {include, exclude}), each carrying its own `@pytest.mark.xfail(strict=False, reason=XFAIL_REASON_ROVER_DISPATCH)` decorator. Same 6 test cases at runtime; now 8 grep hits (6 decorator lines + 2 in module docstring/comments mentioning the marker). `ROVER_ONLY_FLAGS` constant is still exported per the must_haves truth list, even though it's not consumed directly by the test bodies anymore — Plan 04-02 can use it when collapsing back to parametrize blocks.
- **Files modified:** `robot_follow/tests/test_cli_help_dispatch.py`
- **Verification:** `grep -cE '@pytest.mark.xfail' robot_follow/tests/test_cli_help_dispatch.py` → **8** (>= 6 ✓); `pytest -v` reports 7 PASS + 3 XFAIL + 3 XPASS = 13 tests, 0 FAIL.
- **Committed in:** `440204d` (Task 2 commit) — deviation noted in commit body.

---

**Total deviations:** 1 auto-fixed (1 Rule 1 — bug in plan's internal contradiction).
**Impact on plan:** No scope creep. Same 6 rover CLI test cases land, with the same xfail markers and same `strict=False`. Plan 04-02 can either strip the 6 standalone decorators or collapse the functions back into 2 parametrize blocks after deleting the markers — both paths land at the same end state.

## Issues Encountered

**Observation, not an issue:** `test_drone_help_excludes_{cmd_vel_topic,ros_namespace,ros_domain_id}` register as **XPASS** today (the rover flags don't exist anywhere yet → they're trivially absent from drone --help). This is exactly why the plan specified `strict=False` on every marker: XPASS is yellow, not red, and the suite stays GREEN. When Plan 04-02 lands `add_rover_args` and registers the rover flags, both the include-rover and exclude-rover-from-drone tests will pass for the right reason — at which point 04-02 strips all 6 decorators wholesale. Documented this explicitly in the Task 2 commit body so future archaeologists understand the intent.

## Verification Evidence

```
$ grep -c '@pytest.mark.xfail' robot_follow/tests/test_ros2_rover_adapter.py
21    # 20 actual decorators + 1 docstring mention (>= 20 OK)

$ grep -cE '@pytest.mark.xfail' robot_follow/tests/test_cli_help_dispatch.py
8     # 6 actual decorators + 2 prose mentions (>= 6 OK)

$ pytest robot_follow/tests/test_ros2_rover_adapter.py -v | tail -2
============================= 20 xfailed in 0.19s ==============================

$ pytest robot_follow/tests/test_cli_help_dispatch.py -v | tail -1
=================== 7 passed, 3 xfailed, 3 xpassed in 5.57s ====================

$ pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q | tail -1
298 passed, 1 skipped, 23 xfailed, 3 xpassed in 24.05s
```

**Pre-plan baseline (HEAD~2):** 298 passed, 1 skipped
**Post-plan (HEAD):** 298 passed, 1 skipped, 23 xfailed, 3 xpassed, **0 FAILED**

Pass count preserved exactly. The 23 xfailed = 20 (adapter) + 3 (rover-include CLI dispatch). The 3 xpassed = 3 (drone-excludes-rover, trivially passing because rover flags don't exist yet). Suite stays GREEN.

### Architectural lock evidence

```
$ git diff --name-only HEAD~2 HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py
(empty)
```

`robot_follow/robot_api/adapters/mavsdk_drone.py` is byte-identical across both 04-01 commits. Both commits used explicit pathspec (`git commit -- <file>`) — no `git add .` / no `git commit -a` / no smear from sibling executors.

## User Setup Required

None — pure test-scaffold work, no external service configuration.

## Next Phase Readiness

- **Wave 2 ready (parallel):**
  - Plan **04-02** (CLI dispatch) closes the 6 CLI-dispatch xfails by filling in `add_rover_args` body in `robot_follow/robot_follow_app.py`. Plan 04-02 will need to strip the 6 `@pytest.mark.xfail` decorators in `test_cli_help_dispatch.py`; the per-flag standalone functions can either stay or be collapsed back into parametrize blocks.
  - Plan **04-03** (adapter) closes the 20 adapter xfails by creating `robot_follow/robot_api/adapters/ros2_rover.py` with `Ros2RoverAdapter` + `ROVER_CAPS`. The rclpy_mock fixture defined in this plan is the foundation 04-03 reuses.
- **Wave 3:** Plan **04-04** wires the adapter into `run_robot()` and confirms full integration.
- **Wave 4:** Plan **04-05** Phase 4 verifier.

**No blockers.** Suite is GREEN. Architectural lock honored. Parallel-wave hygiene exercised (explicit pathspec on both commits) so Wave 2's two parallel executors can rely on a clean baseline.

## Self-Check: PASSED

- [x] `robot_follow/tests/test_ros2_rover_adapter.py` exists on disk
- [x] `robot_follow/tests/test_cli_help_dispatch.py` modified on disk (append-only)
- [x] Commit `c538bc1` exists in `git log --oneline -3`
- [x] Commit `440204d` exists in `git log --oneline -3`
- [x] `grep -c '@pytest.mark.xfail' test_ros2_rover_adapter.py` = 21 (>= 20)
- [x] `grep -cE '@pytest.mark.xfail' test_cli_help_dispatch.py` = 8 (>= 6)
- [x] Full pytest suite GREEN: 298 passed, 1 skipped, 23 xfailed, 3 xpassed, 0 FAILED
- [x] `git diff --name-only HEAD~2 HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py` empty (architectural lock)
- [x] rclpy_mock fixture uses `monkeypatch.setitem` (verified by reading the file: 6 setitem calls, 0 direct `sys.modules[...] =` writes)
- [x] `test_friendly_error_when_rclpy_missing` asserts both `"ROS 2 not"` and `"source /opt/ros/humble/setup.bash"` substrings
- [x] `ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]` constant present in test_cli_help_dispatch.py
- [x] Both task commits used explicit pathspec (no `git add .` / no `git commit -a`)

---
*Phase: 04-rover-adapter*
*Completed: 2026-05-20*
