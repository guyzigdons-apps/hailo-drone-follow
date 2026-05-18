---
phase: 03-abstraction
plan: 02
subsystem: testing
tags: [pytest, xfail, scaffolds, abs-01, abs-02, abs-03, abs-09, abs-10]

# Dependency graph
requires:
  - phase: 02-cleanup
    provides: green baseline test suite (176 PASSED, 0 xfailed)
provides:
  - test_robot_protocol_shape.py (ABS-01 gate scaffold; xfail until 03-04/03-06)
  - test_robot_command_shape.py (ABS-02 gate; renamed from test_velocity_command_shape.py; xfail until 03-03/03-07)
  - test_layout_smoke.py (ABS-03 file-layout gate; xfail until 03-05 + 03-09)
  - test_cli_help_dispatch.py (ABS-09 two-pass argparse gate; xfail until 03-08)
  - test_setup_env_sh.py (ABS-10 ROS-sourcing gate; xfail until 03-09; 1 sanity test passes today)
affects: [03-03, 03-04, 03-05, 03-06, 03-07, 03-08, 03-09]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - XFAIL_REASON_* module-top constants for grep-driven xfail-strip discipline
    - subprocess-driven CLI/shell smoke tests with timeout guards
    - Conditional skip vs xfail based on environment (ROS install status)
    - Explicit-pathspec commits to survive parallel-wave staging-area collisions

key-files:
  created:
    - robot_follow/tests/test_robot_protocol_shape.py
    - robot_follow/tests/test_robot_command_shape.py
    - robot_follow/tests/test_layout_smoke.py
    - robot_follow/tests/test_cli_help_dispatch.py
    - robot_follow/tests/test_setup_env_sh.py
  modified: []
  deleted:
    - robot_follow/tests/test_velocity_command_shape.py (renamed via git mv)

key-decisions:
  - "git mv preserves filesystem-level rename but git log --follow does NOT trace history because the file was substantially rewritten in the same commit (old=18 lines, new=46 lines), exceeding git's rename-detection similarity threshold even at 20%. Acceptable: rename intent is recorded in commit message + module docstring."
  - "strict=False on every @pytest.mark.xfail — wave-0 scaffolds may xpass coincidentally (e.g. drone-help tests pass today because no --robot dispatch yet means argparse short-circuits on --help with all flags visible). Strict semantics belong in the strip plans (03-03..09)."
  - "test_setup_env_sh_exists is the only non-xfail test in this plan — gives the file a green anchor today and bumps suite count from 176 → 175+1=176 (175 because rename removed 2 legacy tests + 1 new passing test)."
  - "Explicit-pathspec commits required every time (parallel-wave hygiene). Plan 03-01 had unstaged work in tests/cases/ + test_robot_command_snapshot.py during all 3 of this plan's commits; pathspec prevented cross-contamination."

patterns-established:
  - "XFAIL_REASON_*: module-top string constants that name the strip plan; future agents `git grep XFAIL_REASON` to find strip sites mechanically"
  - "Two-import-attempt pattern: `try: from x import Y; except ImportError: pytest.skip(...)` lets the test collect cleanly even before the production module exists"
  - "Subprocess CLI smoke via `python -m robot_follow.robot_follow_app` (path-independent — works regardless of console-script PATH state in the subprocess env)"

requirements-completed: [ABS-01, ABS-02, ABS-03, ABS-09, ABS-10]

# Metrics
duration: 4min
completed: 2026-05-18
---

# Phase 03 Plan 02: Wave-0 Test Scaffolds Summary

**5 xfail-marker test scaffolds (ABS-01/02/03/09/10) committed atomically with parallel-wave-safe pathspec commits; full suite stays green at 175 passed + 6 xfailed + 6 xpassed + 4 skipped (= 191 collected, 0 failed)**

## Performance

- **Duration:** 4 min
- **Started:** 2026-05-18T19:59:00Z
- **Completed:** 2026-05-18T20:03:36Z
- **Tasks:** 3 (each → 1 atomic commit)
- **Files created:** 5
- **Files renamed:** 1 (via `git mv`)
- **Files deleted:** 1 (the rename source)

## Accomplishments

- ABS-01 (Robot protocol shape) gate scaffolded — 2 xfail tests will pass once 03-04 lands `robot_api.robot.Robot` and 03-06 makes `MavsdkDroneAdapter` implement it.
- ABS-02 (RobotCommand field shape) gate scaffolded — test file renamed from `test_velocity_command_shape.py` so the legacy assertion lives alongside the new one until 03-07 strips it.
- ABS-03 (file-layout: `robot_api.adapters.mavsdk_drone` present, `drone_api/` absent) gate scaffolded — 2 xfail tests will pass once 03-05 moves the file and 03-09 deletes `drone_api/`.
- ABS-09 (two-pass argparse: `--robot {drone,rover}` dispatch) gate scaffolded — 7 xfail tests (3 parametrized drone-includes + 3 parametrized rover-excludes + 1 common) will pass once 03-08 lands the pre-parser.
- ABS-10 (`setup_env.sh` conditional ROS sourcing) gate scaffolded — 1 sanity test passes today, 3 xfail tests will pass once 03-09 lands the conditional source block (gated on `/opt/ros/humble/setup.bash` existence).

## Task Commits

Each task was committed atomically with explicit `--pathspec` to avoid parallel-wave staging collisions:

1. **Task 1: Rename test_velocity_command_shape.py + scaffold test_robot_protocol_shape.py** — `feef586` (test)
2. **Task 2: Scaffold test_layout_smoke.py + test_cli_help_dispatch.py** — `2ce6203` (test)
3. **Task 3: Scaffold test_setup_env_sh.py** — `655d08c` (test)

**Parallel agent's commit (03-01) interleaved:** `2bf59d4` (between tasks 2 and 3 — zero file overlap).

## Files Created/Modified

- `robot_follow/tests/test_robot_command_shape.py` — ABS-02 gate (renamed from test_velocity_command_shape.py; 2 xfail tests: legacy VelocityCommand shape + new RobotCommand shape).
- `robot_follow/tests/test_robot_protocol_shape.py` — ABS-01 gate (2 xfail tests: Robot protocol 6-methods+caps + MavsdkDroneAdapter implements Robot).
- `robot_follow/tests/test_layout_smoke.py` — ABS-03 gate (2 xfail tests: `robot_api.adapters.mavsdk_drone` imports + `drone_api/` raises ModuleNotFoundError).
- `robot_follow/tests/test_cli_help_dispatch.py` — ABS-09 gate (7 xfail tests, parametrized over `DRONE_ONLY_FLAGS = ["--takeoff-landing", "--target-altitude", "--serial"]`).
- `robot_follow/tests/test_setup_env_sh.py` — ABS-10 gate (1 PASS + 3 xfail tests: setup_env.sh exists + ROS_DISTRO-iff-installed + venv-first-PYTHONPATH + conditional-ROS-block-static-grep).
- `robot_follow/tests/test_velocity_command_shape.py` — DELETED (renamed via `git mv` to `test_robot_command_shape.py`).

## Decisions Made

- **`strict=False` on every xfail marker.** Several wave-0 tests xpass today coincidentally — e.g. `test_drone_help_includes_drone_flag` xpasses because today `robot-follow --help` prints all flags (no `--robot` dispatch yet, so argparse short-circuits on `--help` before complaining about the unknown `--robot drone`). Locking these as strict would break the suite on parallel-wave merges. Strict semantics land with the xfail-strip plans (03-03..09).
- **`git mv` history preservation has a similarity-threshold caveat.** `git log --follow robot_follow/tests/test_robot_command_shape.py` shows only the rename commit, not the pre-rename history, because the same commit substantially rewrote the file (18 → 46 lines, well below git's default 50% similarity threshold; even `--find-renames=20%` doesn't help). The rename intent is documented in the commit message and the module docstring; for archaeological queries use `git log -- robot_follow/tests/test_velocity_command_shape.py` (which still shows the legacy history under the old path).
- **One non-xfail test per scaffold file where safely possible** — `test_setup_env_sh_exists` is the anchor. Prevents the scaffold file from being a pure xfail container that could be silently emptied of its purpose during refactors.
- **Subprocess CLI tests via `python -m`, not `robot-follow`.** Subprocess env doesn't reliably inherit the venv's `bin/` on PATH; `sys.executable -m robot_follow.robot_follow_app` is path-independent and works under both `pip install -e .` (dev) and bare-source-checkout scenarios.

## Deviations from Plan

None — plan executed exactly as written. The plan's `<verify>` blocks and the explicit-pathspec commit instructions matched reality 1:1, including parallel-wave staging collisions with 03-01 (which the explicit pathspec contained cleanly).

## Issues Encountered

- **Parallel-wave staging collision (expected and handled):** During Task 2's commit step, plan 03-01's `tests/cases/__init__.py` and `tests/cases/drone_command_baseline.py` were already staged in the index (the parallel agent had run `git add` but not yet committed). Explicit `git commit -m "..." -- <my-files-only>` excluded them. By the end of Task 2, the parallel agent's commit `2bf59d4` had landed those files, so they vanished from my `git status`. No cross-contamination — `git show 2ce6203 --stat` lists only my 2 files.
- **`git log --follow` does not trace the rename history** (see Decisions Made). Documented in the test file's module docstring so future archaeologists know to query the old path.

## Verification

**This plan's tests only (17 collected):**

```
1 passed, 4 skipped, 6 xfailed, 6 xpassed in 3.42s
```

- 1 passed: `test_setup_env_sh_exists`
- 4 skipped: tests gated on production modules that don't exist yet (RobotCommand, Robot protocol, MavsdkDroneAdapter) or ROS install
- 6 xfailed: tests that fail-as-expected today (layout, rover-excludes, ROS-block-static-grep)
- 6 xpassed: tests that pass-as-not-strictly-expected today (legacy VelocityCommand still around, drone-help includes flags because no dispatch yet)

**Full suite (`pytest robot_follow/tests --ignore=test_sim_worlds.py`):**

```
175 passed, 4 skipped, 6 xfailed, 6 xpassed in 16.39s
```

- 191 total collected (176 baseline + 17 from this plan - 2 legacy renamed = net +15 from this plan)
- 0 unexpected failures
- 0 errors at collection

Baseline math: 176 passed (pre-plan) - 2 legacy tests (lived in deleted file) + 1 new pass (`test_setup_env_sh_exists`) = 175 passed. Pass count delta is correct.

## Next Phase Readiness

- All 5 ABS-* gates have observable scaffolds; plans 03-03..09 can strip xfail markers as they land each piece.
- `XFAIL_REASON_*` constants in each new file make strip sites grep-discoverable: `git grep "XFAIL_REASON" robot_follow/tests/`.
- No blockers introduced; parallel-wave hygiene (pathspec commits) verified to work under live contention with plan 03-01.

## Self-Check: PASSED

- File `robot_follow/tests/test_robot_command_shape.py` — FOUND
- File `robot_follow/tests/test_robot_protocol_shape.py` — FOUND
- File `robot_follow/tests/test_layout_smoke.py` — FOUND
- File `robot_follow/tests/test_cli_help_dispatch.py` — FOUND
- File `robot_follow/tests/test_setup_env_sh.py` — FOUND
- File `robot_follow/tests/test_velocity_command_shape.py` — ABSENT (correctly deleted via `git mv`)
- Commit `feef586` — FOUND in git log
- Commit `2ce6203` — FOUND in git log
- Commit `655d08c` — FOUND in git log

---

*Phase: 03-abstraction*
*Completed: 2026-05-18*
