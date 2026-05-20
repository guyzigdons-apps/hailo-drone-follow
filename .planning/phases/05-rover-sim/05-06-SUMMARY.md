---
phase: 05-rover-sim
plan: 06
subsystem: testing
tags: [phase-5, rover-sim, smoke-test, pytest, skip-on-no-tool, RSIM-01, RSIM-02, RSIM-03, RSIM-04, RSIM-05, RSIM-06, RSIM-07, wave-3]

requires:
  - phase: 05-rover-sim
    provides: "Plans 05-01..05-05 — rover.sdf, worlds/, README, install.sh --rover, start_rover_sim.sh"
provides:
  - "robot_follow/tests/test_rover_sim_smoke.py — 10 parse-only structural tests (RSIM-01..07 regression guard)"
  - "Architectural lock for RSIM-06: byte-identity check vs HEAD on sim/bridge/video_bridge.py"
  - "Fortress-bridge exclusion guard: install.sh cannot regress to ros-humble-ros-gz-bridge silently"
affects: [phase-6, RINT-04, future rover-sim refactors]

tech-stack:
  added: []  # no new deps; pure stdlib + pytest already in venv
  patterns:
    - "Smoke-as-grep: stdlib subprocess + shutil.which + pytest.skip for tool-detected gates"
    - "Byte-identity HEAD lock: subprocess('git show HEAD:<path>') vs Path.read_bytes() for architectural-lock files"

key-files:
  created:
    - "robot_follow/tests/test_rover_sim_smoke.py (265 lines, 10 tests)"
  modified: []

key-decisions:
  - "Deviation Rule 1 fix: test_sdfs_lint_clean_with_gz filters out SDFs containing <uri>model://...</uri> because gz sdf -k has no sdf::findFile() callback (that's gz sim runtime only). Without the filter, world SDFs fail Lint with `Tried to use callback in sdf::findFile()` regardless of GZ_SIM_RESOURCE_PATH. Include validity stays covered by test_world_sdfs_include_rover_model."
  - "Imported `re` at module top (stdlib-only constraint preserved) to strip XML comments before scanning for <uri>model://, since rover.sdf has a comment that mentions the include pattern."
  - "Kept test_sdfs_lint_clean_with_gz as an optional 10th test — not in must_haves.truths required name list — but valuable for catching XML-malformed SDFs in future commits."

patterns-established:
  - "Phase 5 smoke ethos: skip-on-no-tool > hard-fail-on-no-tool. shutil.which() gates every external-tool invocation."
  - "RSIM-06 lock pattern: any file declared 'do not edit as part of work X' gets a byte-identity-vs-HEAD test as a regression tripwire."

requirements-completed: [RSIM-01, RSIM-02, RSIM-03, RSIM-04, RSIM-05, RSIM-06, RSIM-07]

duration: 8min
completed: 2026-05-20
---

# Phase 05 Plan 06: Rover-Sim Smoke Tests Summary

**Phase 5's CI-friendly regression guard — 10 parse-only pytest assertions locking the RSIM-01..07 invariants against future drift, with skip-on-no-tool gates so the file runs cleanly on no-gz / no-shellcheck contributor boxes.**

## Performance

- **Duration:** 8 min
- **Started:** 2026-05-20T13:30:00Z (approx)
- **Completed:** 2026-05-20T13:38:26Z
- **Tasks:** 1 / 1
- **Files modified:** 1

## Accomplishments

- Created `robot_follow/tests/test_rover_sim_smoke.py` (265 lines, 10 tests, pure stdlib + pytest)
- All 10 tests PASS on this dev box (gz, shellcheck, git all available); skip-gates exist for every external tool
- Suite goes from **326 passed / 1 skipped → 336 passed / 1 skipped** (+10, zero regressions)
- RSIM-06 architectural lock is now enforced: `sim/bridge/video_bridge.py` byte-identity vs git HEAD is checked on every test run

## Task Commits

1. **Task 1: test_rover_sim_smoke.py with 10 parse-only RSIM-01..07 tests** — `eb3f5bf` (test)

**Plan metadata:** (this commit, after SUMMARY.md write)

## Files Created/Modified

- `robot_follow/tests/test_rover_sim_smoke.py` (NEW, 265 lines) — 10 parse-only smoke tests covering Phase 5's structural invariants

## Verification

### Collect

```
$ python -m pytest robot_follow/tests/test_rover_sim_smoke.py --collect-only -q
...
10 tests collected in 0.03s
```

### Verbose run (this dev box, gz + shellcheck + git all installed)

```
$ python -m pytest robot_follow/tests/test_rover_sim_smoke.py -v
robot_follow/tests/test_rover_sim_smoke.py::test_sdf_no_ignition_prefix PASSED            [ 10%]
robot_follow/tests/test_rover_sim_smoke.py::test_rover_sdf_uses_gz_diff_drive PASSED      [ 20%]
robot_follow/tests/test_rover_sim_smoke.py::test_rover_sdf_cmd_vel_topic_override PASSED  [ 30%]
robot_follow/tests/test_rover_sim_smoke.py::test_world_sdfs_include_rover_model PASSED    [ 40%]
robot_follow/tests/test_rover_sim_smoke.py::test_install_sh_rover_lists_garden_bridge PASSED [ 50%]
robot_follow/tests/test_rover_sim_smoke.py::test_install_sh_rover_excludes_fortress_bridge PASSED [ 60%]
robot_follow/tests/test_rover_sim_smoke.py::test_start_rover_sim_passes_shellcheck PASSED [ 70%]
robot_follow/tests/test_rover_sim_smoke.py::test_video_bridge_byte_identical_to_head PASSED [ 80%]
robot_follow/tests/test_rover_sim_smoke.py::test_readme_documents_eol_and_smoke PASSED    [ 90%]
robot_follow/tests/test_rover_sim_smoke.py::test_sdfs_lint_clean_with_gz PASSED           [100%]

============================== 10 passed in 0.22s ==============================
```

### Per-test outcome breakdown

| Test | Outcome | RSIM IDs | Notes |
|---|---|---|---|
| test_sdf_no_ignition_prefix | PASSED | RSIM-01, RSIM-03 | All 4 SDFs free of `ignition::` |
| test_rover_sdf_uses_gz_diff_drive | PASSED | RSIM-01 | Garden DiffDrive class + filename present |
| test_rover_sdf_cmd_vel_topic_override | PASSED | RSIM-02 | Exactly 1 `<topic>cmd_vel</topic>` in rover.sdf |
| test_world_sdfs_include_rover_model | PASSED | RSIM-03 | All 3 worlds include `model://rover` |
| test_install_sh_rover_lists_garden_bridge | PASSED | RSIM-05 | Garden bridge + apt-cache preflight present |
| test_install_sh_rover_excludes_fortress_bridge | PASSED | RSIM-05 + Pitfall 5 | Fortress form absent after stripping Garden/Harmonic forms |
| test_start_rover_sim_passes_shellcheck | PASSED | RSIM-04 | shellcheck 0.8.0 returns rc=0 |
| test_video_bridge_byte_identical_to_head | PASSED | RSIM-06 | Working tree bytes == `git show HEAD:sim/bridge/video_bridge.py` bytes (architectural lock) |
| test_readme_documents_eol_and_smoke | PASSED | RSIM-07 | "November 2024" + "gz topic -l" + Harmonic apt-name all present |
| test_sdfs_lint_clean_with_gz | PASSED | RSIM-01..03 | After deviation fix (filter SDFs with `<uri>model://`); 1 standalone SDF (rover.sdf) lints rc=0 |

### Full suite regression

```
$ python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q
...
336 passed, 1 skipped in 24.98s
```

Baseline before plan: 326 passed / 1 skipped. After plan: 336 passed / 1 skipped (+10, exactly the 10 new smoke tests; pre-existing tests are unchanged).

### Architectural locks evidence

```
$ git diff --name-only HEAD~1 HEAD
robot_follow/tests/test_rover_sim_smoke.py

$ git diff --name-only HEAD~1 HEAD -- robot_follow/ | grep -v '^robot_follow/tests/test_rover_sim_smoke.py$'
(empty)

$ git diff --name-only HEAD~1 HEAD -- sim/ install.sh
(empty)
```

Only the new smoke file is in the diff. `sim/`, `install.sh`, and the rest of `robot_follow/` are untouched.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Bug] `test_sdfs_lint_clean_with_gz` failed verbatim because `gz sdf -k` cannot resolve `<include><uri>model://...</uri></include>`**

- **Found during:** Task 1 verification (`pytest -v` showed 1 FAILED).
- **Root cause:** The plan's `<interfaces>` block claimed: *"sdf -k lints generic SDFormat semantics and should accept the rover SDFs even though Garden runtime resolution is separate."* This is incorrect on Harmonic gz-sim8 (and on Garden too): `gz sdf -k` calls the SDFormat library directly without registering a `sdf::setFindCallback()`, so any `<include><uri>model://rover</uri></include>` fails with `Tried to use callback in sdf::findFile()` regardless of `GZ_SIM_RESOURCE_PATH`. Only `gz sim` itself registers the find-callback. Verified empirically on this box (gz-tools2, gz-sim8 Harmonic):
  - `gz sdf -k sim/rover/rover.sdf` → rc=0 (standalone, no `<include>`)
  - `gz sdf -k sim/rover/worlds/circle_around.sdf` → rc=255 (has `<include><uri>model://rover</uri></include>`)
- **Fix:** Modified `test_sdfs_lint_clean_with_gz` to filter the SDF list to "standalone" SDFs only — those whose XML body (with comments stripped) contains no `<uri>model://`. This means the lint only runs against `rover.sdf` (the only SDF in the tree that doesn't include another model), which is exactly what `gz sdf -k` can actually validate. The include validity of the 3 worlds is already covered by `test_world_sdfs_include_rover_model` (grep-style `<uri>model://rover</uri>` assertion).
- **Code change:** Added `import re` at module top (stdlib-only, no new deps). Added `re.compile(r"<!--.*?-->", re.DOTALL)` to strip XML comments before checking for `<uri>model://` (necessary because `rover.sdf` has a comment that mentions the include pattern). Updated the test docstring to explain the constraint.
- **Files modified:** `robot_follow/tests/test_rover_sim_smoke.py` (single file; the change is internal to the test).
- **Commit:** `eb3f5bf` (the entire plan landed in one commit; the fix is part of that commit, not a separate one).

No Rule 2/3/4 deviations.

## Phase 5 Status

Phase 5 (RSIM-01..07) is now **STRUCTURALLY COMPLETE**:

| Requirement | Implementing plan | This plan's regression guard |
|---|---|---|
| RSIM-01 (rover.sdf Garden DiffDrive + camera) | 05-01 | `test_sdf_no_ignition_prefix`, `test_rover_sdf_uses_gz_diff_drive` |
| RSIM-02 (cmd_vel topic override) | 05-01 | `test_rover_sdf_cmd_vel_topic_override` |
| RSIM-03 (3 actor worlds with rover include) | 05-02 | `test_world_sdfs_include_rover_model` |
| RSIM-04 (start_rover_sim.sh launcher) | 05-05 | `test_start_rover_sim_passes_shellcheck` |
| RSIM-05 (install.sh --rover apt list) | 05-04 | `test_install_sh_rover_lists_garden_bridge`, `test_install_sh_rover_excludes_fortress_bridge` |
| RSIM-06 (video_bridge.py reuse) | 05-05 (CLI flag invocation) | `test_video_bridge_byte_identical_to_head` (byte-identity-vs-HEAD lock) |
| RSIM-07 (README EOL + smoke step) | 05-03 | `test_readme_documents_eol_and_smoke` |

**Next gate:** Phase 6 RINT-04 (E2E "rover follows actor through Gazebo Garden world end-to-end") is under operator-witnessed checkpoint (Phase 3 03-10 / 03-12 pattern). Phase 5 does NOT block on it — the structural lock provided by this smoke file is sufficient closure for Phase 5.

## Threat Flags

None. The plan introduced no new endpoints, auth paths, file access patterns, or schema changes. All subprocess invocations are static commands with no operator-supplied data, no shell=True, no string interpolation of arguments. T-05-06-SC (no new package installs) is satisfied — the file uses only stdlib + the existing pytest already in the venv.

## Self-Check: PASSED

- File `robot_follow/tests/test_rover_sim_smoke.py` exists at 265 lines (≥130 required) — VERIFIED via `wc -l`.
- Commit `eb3f5bf` exists — VERIFIED via `git log --oneline -1`.
- All 9 required test functions present — VERIFIED via per-test `grep -c "^def $T"` (each returned 1).
- 10 tests collected, 10 passed, 0 failed, 0 skipped (on this dev box) — VERIFIED via `pytest -v`.
- Suite count went from 326→336 passed (+10) — VERIFIED via `pytest robot_follow/tests --ignore=test_sim_worlds.py -q`.
- `git diff --name-only HEAD~1 HEAD` shows ONLY the smoke file — VERIFIED.
- `git diff --name-only HEAD~1 HEAD -- sim/ install.sh` is empty — VERIFIED.
