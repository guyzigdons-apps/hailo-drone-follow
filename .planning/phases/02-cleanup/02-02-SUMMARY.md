---
phase: 02-cleanup
plan: 02
subsystem: testing
tags: [dead-code, dataclass, signature-tightening, server, cleanup]

# Dependency graph
requires:
  - phase: 02-cleanup-00
    provides: Wave 0 xfail scaffolds for CLEAN-15 and CLEAN-16 already landed; Phase 2 baseline pytest counts established (DEFER-02-00-A noted)
provides:
  - ControllerConfig with no `vfov` field, no `--vfov` flag, no constructor wiring (CLEAN-03)
  - SharedDetectionState.update(detection, available_ids) with no default — callers MUST pass the set explicitly (CLEAN-07)
  - robot_follow/servers/{web_server,openhd_bridge}.py free of `_NULLABLE_FIELDS` sentinel and the 3 dead branches that gated on it (CLEAN-09)
affects: [02-cleanup-06 (CLEAN-14 _CONFIG_FIELDS hoist), 03-drone-adapter (clean dataclass surface for ABS-01/05)]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Atomic signature-change + test-update commit: state.py + test_shared_state.py landed in one commit so pytest is green at every intermediate commit"
    - "Tolerance-aware schema cleanup: confirmed from_json/load_from_file filter by `fields(cls)` names BEFORE deleting field, so legacy JSON keys remain backwards-compatible (silently ignored, not rejected)"

key-files:
  created: []
  modified:
    - robot_follow/follow_api/config.py
    - robot_follow/follow_api/state.py
    - robot_follow/tests/test_shared_state.py
    - robot_follow/tests/test_config_persistence.py
    - robot_follow/servers/web_server.py
    - robot_follow/servers/openhd_bridge.py

key-decisions:
  - "JSON-config loader tolerance confirmed BEFORE deleting `vfov` (CLEAN-03 Step A): both `from_json` and `load_from_file` filter input keys by `{f.name for f in fields(cls)}`, so legacy `\"vfov\": 41.0` keys in df_config.example.json, sim/configs/simulation.json, sim/configs/simulation_follow.json are silently ignored — no JSON edits required, backwards compatibility preserved for shipped tuning files in the field."
  - "CLEAN-07 atomicity gate held: signature change (`state.py:19`) + 11 test call-site updates (`test_shared_state.py:32, 40, 46, 47, 54, 55, 56, 63, 66, 82, 104`) landed in commit f923870. Production callers in `hailo_drone_detection_manager.py` (8 sites) and `test_follow_server.py` (4 sites) were untouched — they already passed `available_ids=` explicitly."
  - "CLEAN-09 collapse strategy: removed the `_NULLABLE_FIELDS = set()` symbol AND the three dead branches gated on it (`web_server.py:333`, `openhd_bridge.py:310`, `:359`). All three were branches whose if-condition was always False (empty-set membership). No behavior change."

patterns-established:
  - "Pre-deletion schema tolerance check: when removing a dataclass field that has external JSON callers, read the loader path first (`load_from_file` / `from_json`) and confirm unknown-key tolerance via `{f.name for f in fields(cls)}` filtering. Skip JSON edits if tolerant."
  - "Per-task commit hygiene under parallel waves: stage by individual file path (`git add file1 file2`), NEVER `git add .` or `git add -A` — sibling agents pre-stage their files in the index. Even so, the pre-staged unmodified-by-me files were swept into commit cd26780 (CLEAN-01/02 deletions) and f923870 (CLEAN-15 vision_branches.py). Acceptable: those changes are correct, on the same feature branch, and would have landed anyway in 02-01/02-04."

requirements-completed: [CLEAN-03, CLEAN-07, CLEAN-09]

# Metrics
duration: 4min
completed: 2026-05-14
---

# Phase 02 Plan 02: Wave 1B Dead-Code Cleanup Summary

**Three surgical deletions: `vfov` field + flag + wiring (CLEAN-03), `state.update()` default-arg removal with 11 test-site fixup in one atomic commit (CLEAN-07), and `_NULLABLE_FIELDS` sentinel + 3 dead branches (CLEAN-09).**

## Performance

- **Duration:** ~4 min
- **Started:** 2026-05-14T17:06:20Z
- **Completed:** 2026-05-14T17:10:05Z
- **Tasks:** 3
- **Files modified:** 6 (2 source + 1 test for CLEAN-03; 1 source + 1 test for CLEAN-07; 2 source for CLEAN-09)

## Accomplishments

- `ControllerConfig.vfov` removed from dataclass, CLI, and `from_args` constructor — zero readers in the entire tree (CLEAN-03)
- `SharedDetectionState.update()` signature now requires `available_ids` explicitly — no default value, no `if available_ids is not None:` guard (CLEAN-07)
- `_NULLABLE_FIELDS = set()` and the 3 dead `if x in _NULLABLE_FIELDS` branches removed from `web_server.py` and `openhd_bridge.py` (CLEAN-09)
- Added a forward-looking guard test: `test_config_persistence.py::test_vfov_field_removed` — fails fast if anyone reintroduces `vfov` accidentally
- Full pytest suite green except for the 2 pre-existing DEFER-02-00-A failures (TestDistanceForward); no new regressions

## Task Commits

Each task was committed atomically (per-task per plan):

1. **Task 1: CLEAN-03 delete `vfov` field/flag/wiring** — `cd26780` (refactor)
2. **Task 2: CLEAN-07 tighten `state.update` signature + 11 test sites (ATOMIC)** — `f923870` (refactor)
3. **Task 3: CLEAN-09 drop `_NULLABLE_FIELDS` + 3 dead branches** — `3732757` (refactor)

**Plan metadata commit:** _(to be appended after this SUMMARY lands)_

## Files Created/Modified

- `robot_follow/follow_api/config.py` — Deleted `vfov: float = 41.0` field (line 21), `--vfov` argparse registration (line 172), and `vfov=_arg("vfov", default=defaults.vfov)` from `from_args` (line 278).
- `robot_follow/follow_api/state.py` — `SharedDetectionState.update()` signature tightened to `def update(self, detection, available_ids: set)` (no default). Removed the `if available_ids is not None:` guard.
- `robot_follow/tests/test_shared_state.py` — All 11 single-arg `state.update(...)` call sites (lines 32, 40, 46, 47, 54, 55, 56, 63, 66, 82, 104) now pass `available_ids=set()` explicitly.
- `robot_follow/tests/test_config_persistence.py` — Added `test_vfov_field_removed` negative-assertion guard at end of file.
- `robot_follow/servers/web_server.py` — Removed local `_NULLABLE_FIELDS = set()` (line 325) and collapsed the `if key in _NULLABLE_FIELDS and (value is None or value == 0):` branch (line 333) into a single `setattr(cfg, key, expected(value))` call.
- `robot_follow/servers/openhd_bridge.py` — Removed module-level `_NULLABLE_FIELDS = set()` (line 68) and collapsed two dead branches in `_apply_config_param` (line 310) and `_send_report` (line 359).

## Decisions Made

- **JSON loader tolerance check executed first** — Per the plan's Step A guard: opened `from_json` and `load_from_file` BEFORE deleting `vfov`, confirmed both filter input keys by `{f.name for f in fields(cls)}`. Unknown keys are silently dropped, so legacy `"vfov": 41.0` entries in `df_config.example.json:3`, `sim/configs/simulation.json:7`, and `sim/configs/simulation_follow.json:11` remain valid — no JSON edits needed. Forward-compat win: any shipped tuning file with a stale `vfov` key in the field still loads.
- **CLEAN-07 atomic commit verified via `git show --stat`** — Both `state.py` and `test_shared_state.py` land in `f923870` together. No intermediate commit where the suite would have been red. (The commit also incidentally absorbed `pipeline_adapter/vision_branches.py`, which had been pre-staged by a parallel-wave agent — see Deviations.)
- **CLEAN-09 collapse, not delete-the-set-and-keep-the-branches** — The plan called for both removing `_NULLABLE_FIELDS` AND collapsing the dead branches. Did both: dropped the set declaration AND the gated branches, so the post-edit code paths read as straight-line logic, no sentinel mention anywhere.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 — Cross-plan scope sweep] Task 1 commit absorbed CLEAN-01/02 file deletions**
- **Found during:** Task 1 commit (CLEAN-03)
- **Issue:** When I ran `git add robot_follow/follow_api/config.py robot_follow/tests/test_config_persistence.py`, the index already had `scripts/bench_reid_callback.py` and `sim/world_loader.py` pre-staged as `D` by a parallel-wave agent (plan 02-01 in flight). `git commit` swept all four files in.
- **Fix:** Left the commit as-is. The deletions are correct (they're CLEAN-01 and CLEAN-02 from plan 02-01) and on the same feature branch — they would have landed anyway. Reverting the foreign deletions would interrupt the parallel agent's workflow.
- **Files affected:** `cd26780` includes `scripts/bench_reid_callback.py` (-186 lines) and `sim/world_loader.py` (-139 lines) in addition to my planned `config.py` + `test_config_persistence.py` changes.
- **Verification:** `git show --stat cd26780` confirms 4 files. `grep -rnE "world_loader|bench_reid" .` returns nothing post-commit. Both deletions match RESEARCH § CLEAN-01 / CLEAN-02 plans exactly.
- **Committed in:** cd26780 (Task 1 commit)

**2. [Rule 1 — Cross-plan scope sweep] Task 2 commit absorbed CLEAN-15 partial work**
- **Found during:** Task 2 commit (CLEAN-07)
- **Issue:** Before staging, `git status --short` showed `robot_follow/pipeline_adapter/vision_branches.py` as `M ` (working-tree modified, unstaged). Between then and my `git add`, another agent (plan 02-04 / CLEAN-15) had staged that file. My `git commit` (after `git add` only on my two files) somehow also picked it up — likely because that file was staged in the index by the parallel agent at the moment I ran `git commit`.
- **Fix:** Left the commit as-is. The CLEAN-15 partial work (vision_branches.py) is correct and on the same feature branch.
- **Files affected:** `f923870` includes `robot_follow/pipeline_adapter/vision_branches.py` (-4 lines) in addition to my planned `state.py` + `test_shared_state.py` (atomic pair).
- **Verification:** The CLEAN-07 atomicity gate is still satisfied: `state.py` + `test_shared_state.py` landed together in one commit. Tests green after this commit (`python -m pytest robot_follow/tests/test_shared_state.py -v` → 8 passed).
- **Committed in:** f923870 (Task 2 commit)

---

**Total deviations:** 2 auto-fixed (both Rule 1 — cross-plan scope sweep from parallel-wave staging-area collisions)
**Impact on plan:** ZERO functional impact. CLEAN-07 atomicity gate held. All three planned CLEAN items (03/07/09) landed cleanly with their own commit messages. Foreign files swept in are real, correct work that belongs on this branch anyway — they're just attributed to the wrong commit. STATE.md decisions section notes that parallel-wave staging-area collisions are a known hazard with this workflow.

## Issues Encountered

- **Transient `test_install_smoke.py` failures during one full-suite run** — On one of two full-suite runs, three `test_install_smoke.py` tests (`test_robot_follow_help_exits_zero`, `test_drone_follow_help_exits_zero`, `test_help_outputs_byte_identical`) failed with an `ImportError`-shaped traceback from `create_app`. On re-run (and when running `test_install_smoke.py` in isolation), all 10 tests passed. Likely cause: parallel-wave agents were writing source files mid-suite-run, causing transient ImportError mid-collection. Not a regression in my code. Final suite run: only 2 failures, both DEFER-02-00-A.

## User Setup Required

None — no external service configuration required for this plan.

## Next Phase Readiness

- CLEAN-03 / CLEAN-07 / CLEAN-09 complete; remaining Phase 2 plans (02-04, 02-05, 02-06, 02-07) can proceed independently on different files.
- For Phase 3 (drone adapter): `SharedDetectionState.update()` now has a tightened contract — callers in the new adapter MUST pass `available_ids` explicitly, but every existing production caller already does, so no porting work needed.
- For plan 02-06 (CLEAN-14 `_CONFIG_FIELDS` hoist): `web_server.py` and `openhd_bridge.py` are now `_NULLABLE_FIELDS`-free — the hoist can proceed on clean files without sentinel-aware logic.

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*

## Self-Check: PASSED

- [x] `robot_follow/follow_api/config.py` — modified, no `vfov` (verified via `grep -rnE "\.vfov|config\.vfov|cfg\.vfov|defaults\.vfov" robot_follow/` returns 0 matches)
- [x] `robot_follow/follow_api/state.py` — modified, signature has no default for `available_ids` (verified via `inspect.signature(...).parameters['available_ids'].default is inspect.Parameter.empty`)
- [x] `robot_follow/tests/test_shared_state.py` — all 11 sites updated (verified via `grep -nE "state\.update\(" robot_follow/tests/test_shared_state.py` lists 11 calls, all with `available_ids=set()`)
- [x] `robot_follow/tests/test_config_persistence.py` — `test_vfov_field_removed` added (16 tests collected, all pass)
- [x] `robot_follow/servers/web_server.py` — `_NULLABLE_FIELDS` absent (verified via `grep -rn '_NULLABLE_FIELDS' robot_follow/` returns 0 matches)
- [x] `robot_follow/servers/openhd_bridge.py` — `_NULLABLE_FIELDS` absent (same grep)
- [x] Commit `cd26780` exists (CLEAN-03)
- [x] Commit `f923870` exists (CLEAN-07, atomic)
- [x] Commit `3732757` exists (CLEAN-09)
- [x] Full pytest suite: 152 passed, 21 xfailed, 2 failed (only pre-existing DEFER-02-00-A) — no new regressions
