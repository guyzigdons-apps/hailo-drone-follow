---
phase: 02-cleanup
plan: 01
subsystem: refactor
tags: [dead-code, cleanup, pipeline-adapter, sim]

# Dependency graph
requires:
  - phase: 02-cleanup
    provides: "Wave 0 xfail scaffolds (02-00) anchoring CLEAN-15/CLEAN-16 gates"
provides:
  - "sim/world_loader.py deleted (CLEAN-01)"
  - "scripts/bench_reid_callback.py deleted (CLEAN-02)"
  - "strip_tiles_and_highlight_target alias removed from vision_branches.py (CLEAN-06)"
  - "create_app(controller_config=) kwarg removed; post-construction attach preserved (CLEAN-10)"
affects: ["02-04", "02-05", "02-06", "02-07", "03-drone-adapter"]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Post-construction attribute attach for callback config (preserved at robot_follow_app.py:340)"

key-files:
  created: []
  modified:
    - "robot_follow/pipeline_adapter/hailo_drone_detection_manager.py (CLEAN-10 — Task 3, committed in 0b40abd)"
    - "robot_follow/pipeline_adapter/vision_branches.py (CLEAN-06 — Task 2, swept into f923870 by parallel plan 02-02)"
    - "sim/world_loader.py (CLEAN-01, deleted — Task 1, swept into cd26780 by parallel plan 02-02)"
    - "scripts/bench_reid_callback.py (CLEAN-02, deleted — Task 1, swept into cd26780 by parallel plan 02-02)"

key-decisions:
  - "DroneFollowUserData.controller_config attribute kept (initialised to None) so callback at line 278 still works; only the constructor kwarg path was dead and got removed."
  - "robot_follow_app.py:340 post-construction attach (`app.user_data.controller_config = controller_config`) is the surviving wiring path — not touched by this plan."

patterns-established:
  - "Pattern 1: Parallel-plan working-tree race — when multiple Wave 1 plans run concurrently in a single working tree, `git add .` in any plan's commit can sweep up other plans' unstaged edits. Per-plan agents that use targeted `git add <file>` (as 02-01 Task 3 did) preserve attribution. Tasks 1 and 2 here were swept into 02-02's commits."

requirements-completed: [CLEAN-01, CLEAN-02, CLEAN-06, CLEAN-10]

# Metrics
duration: 5min
completed: 2026-05-14
---

# Phase 02-cleanup Plan 01: Dead-code deletions Summary

**Four edit-isolated dead-code deletions landed (2 file removals + 1 alias + 1 kwarg); production wiring unchanged; pytest at DEFER-02-00-A baseline.**

## Performance

- **Duration:** 5 min
- **Started:** 2026-05-14T17:05:58Z
- **Completed:** 2026-05-14T17:10:21Z
- **Tasks:** 3
- **Files modified:** 4 (2 deleted, 2 edited)

## Accomplishments

- **CLEAN-01:** `sim/world_loader.py` deleted (zero callers — `start_sim.sh` uses `PX4_GZ_WORLD` env var instead).
- **CLEAN-02:** `scripts/bench_reid_callback.py` deleted (imported nonexistent `reid_worker` module; broken pre-milestone, zero external references in docs).
- **CLEAN-06:** `strip_tiles_and_highlight_target` alias deleted from `vision_branches.py:397` plus the legacy-callers comment above it; `highlight_target` is now the sole public name.
- **CLEAN-10:** `controller_config=None` kwarg removed from `create_app(...)` and from `DroneFollowUserData.__init__`; the in-class `self.controller_config = None` initialisation kept so the callback site at line 278 (`config = user_data.controller_config`) still works via the post-construction attach in `robot_follow_app.py:340`.

## Task Commits

Each task was committed atomically:

1. **Task 1: Delete sim/world_loader.py and scripts/bench_reid_callback.py (CLEAN-01, CLEAN-02)** — swept into `cd26780` (`refactor(02-02): delete unused vfov field/flag/wiring (CLEAN-03)`) by the parallel 02-02 agent. The two deletions appear in that commit's `--stat` (`scripts/bench_reid_callback.py | 186 ---`, `sim/world_loader.py | 139 ---`). Success criteria satisfied; attribution drifted due to working-tree race (see Deviations).
2. **Task 2: Remove strip_tiles_and_highlight_target alias (CLEAN-06)** — swept into `f923870` (`refactor(02-02): tighten state.update() signature, no default (CLEAN-07)`) by the parallel 02-02 agent. That commit's `--stat` includes `robot_follow/pipeline_adapter/vision_branches.py | 4 ----` covering the alias + its preceding comment.
3. **Task 3: Remove create_app(controller_config=) kwarg (CLEAN-10)** — `0b40abd` (`refactor(02-01)`) — committed cleanly via targeted `git add <file>`.

## Files Created/Modified

- `sim/world_loader.py` — DELETED (CLEAN-01)
- `scripts/bench_reid_callback.py` — DELETED (CLEAN-02)
- `robot_follow/pipeline_adapter/vision_branches.py` — line 397 alias + preceding 1-line comment removed (CLEAN-06)
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` — `create_app` signature, `DroneFollowUserData.__init__` signature, `self.controller_config = controller_config` body, and `DroneFollowUserData(...)` kwarg call all stripped (CLEAN-10)

## Decisions Made

- **Keep `self.controller_config = None` initialisation inside `DroneFollowUserData.__init__`** even after removing the constructor kwarg. The callback at line 278 reads `user_data.controller_config`; if the attribute didn't exist, the reader would `AttributeError` before `robot_follow_app.py:340` had a chance to attach the real value. Initialising to `None` preserves the contract while removing the dead kwarg.
- **Treat parallel-plan sweeping as Rule-3 (blocking issue auto-resolved).** When parallel agents 02-02/02-03 committed their unstaged work and inadvertently included my deletions and alias removal, the success criteria were already satisfied — re-doing the work would have created merge conflicts on their SUMMARY commits. Recorded the attribution drift as a deviation rather than fighting it.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Parallel-plan working-tree race] Tasks 1 and 2 committed by parallel agents**

- **Found during:** Task 1 (and again at Task 2)
- **Issue:** The parallel Wave-1 plans (02-02 and 02-03) ran concurrently in the same working tree. Their commit steps used staging patterns that picked up my unstaged deletions and the `vision_branches.py` alias edit alongside their own work. Result: Task 1's file deletions (`sim/world_loader.py`, `scripts/bench_reid_callback.py`) appear in commit `cd26780` (02-02), and Task 2's `vision_branches.py` alias removal appears in commit `f923870` (02-02). My 02-01 plan only owns commit `0b40abd` (Task 3).
- **Fix:** Verified the success criteria on disk (files gone, grep gates clean, alias gone). Recorded the attribution drift here so the SUMMARY accurately reflects who shipped what. Did NOT attempt to revert or re-attribute — that would have broken 02-02's atomic commits and SUMMARY.
- **Files modified:** None additional — work landed correctly, just under the wrong commit prefix.
- **Verification:** All four CLEAN-01/02/06/10 success-criteria commands pass (see Verification section below).
- **Committed in:** `cd26780` (Task 1), `f923870` (Task 2), `0b40abd` (Task 3).

---

**Total deviations:** 1 auto-fixed (parallel-plan working-tree race — attribution-only, no code impact)
**Impact on plan:** Zero code-level impact. The phase's pytest gate is satisfied. Attribution drift documented here. Lesson for future parallel waves: each plan agent should use a dedicated worktree (`git worktree add`) when more than one Wave-N plan can touch the working tree simultaneously, or stagger commits more tightly.

## Issues Encountered

- **Transient `vfov` test flake during Task 1 quick-suite run.** After the `git rm` calls and before the parallel 02-02 commit landed, an in-flight `vfov` removal lived as unstaged changes in `config.py`, causing one quick-suite run of `test_install_smoke.py::test_robot_follow_help_exits_zero` to fail. The next run (≤ 30 s later) was green — the parallel agent had committed cleanly meanwhile. Not caused by my changes; resolved by re-running the suite.

## Verification

All four CLEAN items confirmed against the post-plan tree:

- `test ! -f sim/world_loader.py` → exit 0
- `test ! -f scripts/bench_reid_callback.py` → exit 0
- `grep -rE "world_loader|bench_reid" ... --exclude-dir=.planning ...` → 0 lines
- `grep -n 'strip_tiles_and_highlight_target' robot_follow/pipeline_adapter/vision_branches.py` → 0 lines
- `python -c "import inspect; from robot_follow.pipeline_adapter.hailo_drone_detection_manager import create_app; assert 'controller_config' not in inspect.signature(create_app).parameters"` → `OK`

Pytest summary:

| Suite | Before plan | After plan |
|-------|-------------|------------|
| Quick (`test_install_smoke + test_shared_state + test_config_persistence + test_velocity_command_shape`) | n/a | 36 passed, 0 failed |
| Full (`robot_follow/tests --ignore=test_sim_worlds.py`) | 151 passed, 2 failed, 21 xfailed (DEFER-02-00-A baseline) | 152 passed, 2 failed, 21 xfailed |

The 2 failures (`TestDistanceForward::test_center_y_is_ignored`, `TestDistanceForward::test_clamped_to_max_forward`) are the DEFER-02-00-A baseline carried over from Wave 0 — not a regression. The +1 passed delta vs. baseline reflects the new `test_vfov_field_removed` test landed by parallel plan 02-02.

## Next Phase Readiness

- Wave 1 dead-code deletions complete (CLEAN-01/02/06/10 for this plan; CLEAN-03/05/07/08/09 from parallel plans 02-02 and 02-03 already on the branch).
- Wave 2 plans (02-04 onward) can proceed; their inputs (`vfov` gone, `controller_config` kwarg gone, alias gone) are met.
- Pre-existing DEFER-02-00-A controller failures persist as the floor; should be resolved before Phase 3 starts.

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*

## Self-Check: PASSED

Verified post-write:

- `sim/world_loader.py` MISSING (deleted as expected)
- `scripts/bench_reid_callback.py` MISSING (deleted as expected)
- `robot_follow/pipeline_adapter/vision_branches.py` FOUND (alias stripped — `strip_tiles_and_highlight_target` not present)
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` FOUND (`controller_config` kwarg stripped from `create_app`)
- Commit `cd26780` FOUND in `git log` (Task 1)
- Commit `f923870` FOUND in `git log` (Task 2)
- Commit `0b40abd` FOUND in `git log` (Task 3)
- `.planning/phases/02-cleanup/02-01-SUMMARY.md` FOUND (this file)
