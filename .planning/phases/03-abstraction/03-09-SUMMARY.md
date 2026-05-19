---
phase: 03-abstraction
plan: 09
subsystem: infra
tags: [ros2, setup-env, shim-deletion, xfail-strip, bash, pytest]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: "03-05 shim landing, 03-07 atomic controller→Robot migration (deleted VelocityCommandAPI / run_live_drone), 03-08 retargeted add_drone_args to canonical path (no Phase-3-internal caller of drone_api/ remains)"
provides:
  - "Conditional ROS 2 Humble source block in setup_env.sh (venv-first ordering, idempotent on no-ROS box)"
  - "robot_follow/drone_api/ directory DELETED — legacy shim removed (ABS-03 final)"
  - "test_setup_env_sh.py: 3 xfail markers stripped + XFAIL_REASON constant gone"
  - "test_layout_smoke.py: 1 xfail marker stripped + XFAIL_REASON_DRONE_API_DELETED constant gone"
  - "Phase 3 closes with 0 xfailed in the full suite (mirrors Phase 2 02-07 close)"
affects: [04-rover-adapter, 05-rover-sim, future-ros2-callers]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Conditional ROS source block with single-file guard (`[ -f /opt/ros/humble/setup.bash ]`)"
    - "Venv-first sourcing — ROS source line lands AFTER the venv activation"

key-files:
  created: []
  modified:
    - "setup_env.sh — appended 17-line conditional ROS source block"
    - "robot_follow/tests/test_setup_env_sh.py — stripped 3 xfail markers + XFAIL_REASON + docstring rewrite"
    - "robot_follow/tests/test_layout_smoke.py — stripped 1 xfail marker + XFAIL_REASON_DRONE_API_DELETED + docstring rewrite"
  deleted:
    - "robot_follow/drone_api/__init__.py (and the empty parent directory)"

key-decisions:
  - "Two atomic commits (not one): Task 1 = ROS block + test_setup_env_sh xfail strip (ABS-10); Task 2 = drone_api/ deletion + test_layout_smoke xfail strip (ABS-03 final). Each commit is independently bisectable — a future regression in either ABS-10 or ABS-03-final can be located mechanically by `git bisect` without intermixing concerns."
  - "ROS env-leak verification deferred to a ROS-equipped box (per DESIGN-NOTES risk #10). On this no-ROS dev box, the `if [ -f ... ]` guard skips the source line by construction; the conditional is dead code on this host."

patterns-established:
  - "Pattern: ROS conditional source ordering — venv-first. The block sits AFTER hailo-apps' setup_env.sh activates the venv, so venv site-packages stays ahead of /opt/ros/humble/lib/.../site-packages on sys.path (PITFALLS.md Pitfall 2)."
  - "Pattern: shim deletion preceded by xfail flip — when the shim's only known external surface (boot service) is already retargeted at the canonical path (03-08), and no Phase-3-internal caller is grep-detectable, the shim can be deleted in a single commit and the matching xfail marker stripped in the same commit."

requirements-completed: [ABS-03, ABS-10]

# Metrics
duration: 5min
completed: 2026-05-19
---

# Phase 03 Plan 09: Wave-7 cleanup Summary

**Conditional ROS 2 Humble source block landed in setup_env.sh + robot_follow/drone_api/ shim deleted; Phase 3 closes with 0 xfailed in the full suite.**

## Performance

- **Duration:** ~5 min
- **Started:** 2026-05-19T16:46Z (worktree spawn)
- **Completed:** 2026-05-19T16:52Z
- **Tasks:** 2
- **Files modified:** 3 (1 created/edited bash, 2 test edits)
- **Files deleted:** 1 (`robot_follow/drone_api/__init__.py` + empty parent dir)

## Accomplishments

- **ABS-10 closed.** setup_env.sh now ends with a 17-line conditional ROS-source block (`if [ -f /opt/ros/humble/setup.bash ]; then source it`). Idempotent: no-ROS box → no change; rover users on a ROS-equipped box get ROS for free; drone users on a ROS-equipped box source ROS but the drone path is unaffected (ROS env vars don't conflict with venv Python or MAVSDK).
- **ABS-03 final closed.** `robot_follow/drone_api/` directory deleted (`__init__.py` was the only file post-03-05 move). `python -c "import robot_follow.drone_api"` now raises `ModuleNotFoundError`. No legacy importers remain anywhere in `robot_follow/`.
- **4 xfail markers stripped** in two test files; docstrings rewritten to post-fix world; XFAIL_REASON constants removed.
- **Phase 3 closes with 0 xfailed** in the full suite (mirrors Phase 2 02-07 close).

## Task Commits

Each task was committed atomically:

1. **Task 1: Append ROS-source block to setup_env.sh + strip test_setup_env_sh.py xfails** — `f08b7e5` (feat)
2. **Task 2: Delete robot_follow/drone_api/ directory + strip test_layout_smoke.py xfail** — `2e2dc78` (refactor)

## Files Created/Modified/Deleted

- **Modified** `setup_env.sh` — appended 17-line conditional ROS source block (comment + `if [ -f /opt/ros/humble/setup.bash ]` guard + `source` line).
- **Modified** `robot_follow/tests/test_setup_env_sh.py` — stripped 3 `@pytest.mark.xfail` decorators + `XFAIL_REASON` constant + docstring rewrite.
- **Modified** `robot_follow/tests/test_layout_smoke.py` — stripped 1 `@pytest.mark.xfail` decorator + `XFAIL_REASON_DRONE_API_DELETED` constant + docstring rewrite.
- **Deleted** `robot_follow/drone_api/__init__.py` — the legacy shim (re-exported `add_drone_args` + `_reap_mavsdk_server` from the canonical path; no remaining Phase-3-internal caller after 03-08).
- **Deleted** `robot_follow/drone_api/` directory (filesystem cleanup of the now-empty `__pycache__` + dir).

## Test Counts (cumulative xfail strip ledger)

Plan target tests, before vs. after:

| Test | Before (baseline) | After (this plan) |
|------|---|---|
| `test_setup_env_sh_exists` | PASSED | PASSED |
| `test_ros_distro_iff_ros_installed` | XPASS (xfail strict=False) | PASSED |
| `test_venv_first_in_pythonpath` | SKIPPED (no ROS on dev box) | SKIPPED |
| `test_setup_env_sh_contains_conditional_ros_block` | XFAIL | PASSED |
| `test_robot_api_adapters_mavsdk_drone_imports` | PASSED | PASSED |
| `test_drone_api_module_is_gone` | XFAIL | PASSED |

Full suite (`pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q`):

- **Baseline (pre-plan):** 277 passed, 1 skipped, 2 xfailed, 1 xpassed, 5 failed (pre-existing worktree-isolation `test_install_smoke.py` failures — see Deviations).
- **After this plan:** 280 passed, 1 skipped, **0 xfailed, 0 xpassed**, 5 failed (same pre-existing worktree-isolation failures, unchanged).
- Delta: **+3 passing** (2 from Task 1 xfail→pass + xpass→pass; 1 from Task 2 xfail→pass). xfailed 2→0; xpassed 1→0.

Per-file xfail count post-strip (Phase-2 "nothing left behind" discipline):
- `grep -c xfail robot_follow/tests/test_setup_env_sh.py` → **0**
- `grep -c xfail robot_follow/tests/test_layout_smoke.py` → **0**

**Confirmation: `0 xfailed` in the full suite (mirrors Phase 2 02-07 close).**

Cumulative xfail strips since Phase 3 start (per PLAN's verification block):
- 03-06: `test_mavsdk_drone_adapter.py` + `test_robot_protocol_shape.py` markers stripped → 0 xfail
- 03-07: `test_robot_command_snapshot.py` + `test_robot_command_shape.py` markers stripped → 0 xfail
- 03-08: `test_cli_help_dispatch.py` markers stripped → 0 xfail (7 tests pass)
- 03-09: `test_setup_env_sh.py` (3) + `test_layout_smoke.py` (1) → 0 xfail (this plan, +4 strips)

## No-ROS-env-leak confirmation (this dev box)

`/opt/ros/humble/setup.bash` does not exist on this dev box, so the `if [ -f ... ]` guard skips the `source` line by construction. The block is dead code on this host. By definition, no ROS_* env vars are introduced — verified by inspection of the conditional + the absence of the file.

**Deferred operator verification (ROS-equipped box):** Per DESIGN-NOTES risk #10, a future operator verifier should `env | grep -E '^(ROS|AMENT|CMAKE_PREFIX)'` on a ROS-equipped box before and after sourcing setup_env.sh to confirm the diff matches `/opt/ros/humble/setup.bash`'s expected env additions and nothing else leaks. This is recorded as a deferred manual smoke for Phase 3 verifier / Phase 4 rover-adapter executor.

## Decisions Made

- **Two atomic commits, not one.** Plan didn't mandate a single combined commit; per the Phase 3 03-05 split-commit precedent, keeping ABS-10 (setup_env.sh + ROS) and ABS-03-final (drone_api/ deletion) on separate commits makes `git bisect` clean — a future regression isolated to either concern can be located mechanically without intermixing.
- **Followed plan exactly otherwise** — no Rule-2/Rule-3 deviations encountered; both Task action steps applied verbatim from `<action>` blocks.

## Deviations from Plan

**None — plan executed exactly as written.**

The plan's `<interfaces>` block flagged a possible deviation if any Phase-3-internal caller still imported `from robot_follow.drone_api`. Grep confirmed empty (no callers); no deviation was needed.

## Issues Encountered

- **Pre-existing `test_install_smoke.py` 5 failures (out-of-scope per Scope Boundary).** All 5 `test_install_smoke.py` tests fail with `shutil.which("robot-follow")` returning `None`. Root cause: the editable `robot-follow` install is registered to `Editable project location: /home/guyz/code/guyz/hailo-drone-follow/.claude/worktrees/agent-a1992049610eccea4` (a different parallel-wave executor's worktree), so the console scripts on `$PATH` resolve to that worktree's bin, not this one's. Failures present at baseline before any edits in this plan; deletion of `drone_api/` does NOT touch the console-script registration. Out-of-scope (per Scope Boundary: pre-existing, not caused by current task's changes). Manual `robot-follow --help` invocation via `python -c "from robot_follow.robot_follow_app import main; ..."` exits 0, confirming the CLI is functionally unaffected.

  Recommended action: the orchestrator's worktree-merge step or a Phase-3 close-out fix should re-run `pip install -e .` against the merged main repo so console scripts re-register to the canonical path. Logged for orchestrator visibility, not blocking this plan.

## User Setup Required

None — no external service configuration required. The conditional ROS source block is a no-op on hosts without ROS 2 Humble installed at `/opt/ros/humble/setup.bash`.

## Self-Check

- `setup_env.sh` exists and contains `/opt/ros/humble/setup.bash` → FOUND
- `robot_follow/drone_api/` directory absent → CONFIRMED (`test ! -d` returns true)
- Commit `f08b7e5` exists in `git log` → FOUND
- Commit `2e2dc78` exists in `git log` → FOUND

## Self-Check: PASSED

## Next Phase Readiness

- **Phase 3 closes with 0 xfailed** (mirrors Phase 2 02-07 close discipline).
- **ABS-03 fully closed** — `drone_api/` deletion completes the legacy-symbol cleanup; no shim layer remains.
- **ABS-10 closed** — setup_env.sh conditionally sources ROS 2 Humble when present, with venv-first ordering preserved.
- **Phase 3's only remaining work** is 03-10 (operator-witnessed SITL gate, checkpoint plan) closing ABS-11.
- **Ready for Phase 4** (rover adapter): the ROS source block now lands cleanly on a rover host without operator intervention; the package-rename + shim-deletion is final.

---
*Phase: 03-abstraction*
*Plan: 09*
*Completed: 2026-05-19*
