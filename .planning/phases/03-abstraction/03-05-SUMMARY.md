---
phase: 03-abstraction
plan: 05
subsystem: refactoring
tags: [git-mv, file-move, backward-compat-shim, mavsdk, robot_api, drone_api]

# Dependency graph
requires:
  - phase: 03-abstraction
    provides: "robot_api/ package scaffold with adapters/ directory (03-04)"
provides:
  - "MAVSDK drone code physically relocated to robot_api/adapters/mavsdk_drone.py with git history preserved (--follow works)"
  - "Backward-compat shim at drone_api/__init__.py re-exports the 4 public symbols so legacy callers keep working"
  - "robot_follow_app.py uses canonical robot_api.adapters.mavsdk_drone import path"
affects:
  - "03-06 (wraps the relocated file as MavsdkDroneAdapter)"
  - "03-07/08/09 (orchestrator wiring + drone_api/ shim deletion in 03-09)"

# Tech tracking
tech-stack:
  added: []  # no new deps; pure relocation
  patterns:
    - "git mv preserves --follow history (vs Write+delete which loses it)"
    - "Backward-compat package-shim pattern: __init__.py re-exports from new module location during migration windows"

key-files:
  created:
    - "robot_follow/robot_api/adapters/mavsdk_drone.py (via git mv; content byte-identical to old location)"
  modified:
    - "robot_follow/drone_api/__init__.py (rewritten as deprecation shim)"
    - "robot_follow/robot_follow_app.py (imports updated to canonical new path)"
  deleted:
    - "robot_follow/drone_api/mavsdk_drone.py (gone via git mv; recorded as R-status rename)"

key-decisions:
  - "Two-commit shape: commit 1 is pure rename (0 bytes changed), commit 2 adds the shim + retargets robot_follow_app.py. This bisects cleanly: HEAD~1 has the move only; HEAD has the working shim."
  - "Test files (test_controller.py, test_velocity_api_and_smoother.py) intentionally NOT updated to the canonical path — they exercise the shim, which is exactly what the shim exists for. 03-09 will batch-migrate them when the shim is deleted."

patterns-established:
  - "Backward-compat shim during file moves: rewrite the old package __init__.py to re-export from the new module location with a deprecation docstring; delete the shim in a later plan once all callers are migrated."

requirements-completed: [ABS-03]

# Metrics
duration: 3min
completed: 2026-05-19
---

# Phase 03 Plan 05: MAVSDK Adapter File Move + Shim Summary

**Bisectable git-mv of `drone_api/mavsdk_drone.py` to `robot_api/adapters/mavsdk_drone.py` with a re-exporting deprecation shim left at the old location.**

## Performance

- **Duration:** 3m 8s
- **Started:** 2026-05-19T10:47:45Z
- **Completed:** 2026-05-19T10:50:53Z
- **Tasks:** 2
- **Files modified:** 3 (1 moved, 1 rewritten, 1 import-updated)

## Accomplishments

- File relocated via `git mv`; staged diff shows `rename robot_follow/{drone_api => robot_api/adapters}/mavsdk_drone.py (100%)` (0 insertions, 0 deletions). Code body byte-identical.
- `git log --follow robot_follow/robot_api/adapters/mavsdk_drone.py` shows the full history through to pre-move commits (verified: top of follow log shows `e6ba475 refactor(03-05): git mv ...` then `fc111c4 refactor(02-04): merge telemetry position+altitude tasks (CLEAN-13)` — pre-move history preserved).
- Backward-compat shim re-exports the 4 public symbols (`VelocityCommandAPI`, `run_live_drone`, `add_drone_args`, `_reap_mavsdk_server`) from the new location; `VelocityCommandAPI is V2` identity check confirms both import paths resolve to the same object.
- `robot_follow_app.py` consolidated to a single `from robot_follow.robot_api.adapters.mavsdk_drone import (run_live_drone, _reap_mavsdk_server, add_drone_args)` (was previously two lines, one of which used the now-deleted `drone_api.mavsdk_drone` submodule path).
- Full suite: **175 passed, 4 skipped, 86 xfailed, 32 xpassed in 16.21s** — matches Phase 2 baseline; zero behavioral drift.
- `robot-follow --help` and `drone-follow --help` both exit 0.

## Task Commits

1. **Task 1: `git mv` mavsdk_drone.py to robot_api/adapters/** — `e6ba475` (refactor)
2. **Task 2: Shim drone_api + retarget app to robot_api.adapters** — `bc8c87e` (refactor)

**Plan metadata:** _(this commit)_ (docs: complete plan)

## Files Created/Modified

- `robot_follow/robot_api/adapters/mavsdk_drone.py` — NEW location (via `git mv`); 37267 bytes; CODE BODY UNCHANGED.
- `robot_follow/drone_api/mavsdk_drone.py` — DELETED via `git mv` (R-status rename, recorded by git).
- `robot_follow/drone_api/__init__.py` — Rewritten as a deprecation shim that re-exports from the new location; same 4 public symbols in `__all__`; deprecation docstring directs new callers to `robot_follow.robot_api.adapters.mavsdk_drone`.
- `robot_follow/robot_follow_app.py` — Lines 31–32 consolidated into a single import from the canonical new path. No behavior change.

## Decisions Made

- **Two atomic commits, not one.** The plan's verification block suggested a single combined commit; I split into (1) the pure `git mv` (zero-content rename) and (2) shim + caller retarget. Rationale: the pure rename commit is an ideal bisect target — if a future regression appears, `git bisect` between Task 1 and Task 2 instantly reveals whether the symptom comes from the move itself or from the shim/caller wiring. This is consistent with the plan's stated bisectability goal in the `<verification>` block ("at HEAD of this plan, `robot-follow --help` works, the drone path is unchanged"). Two commits cost nothing and double the diagnostic resolution.
- **Test files (test_controller.py, test_velocity_api_and_smoother.py) intentionally NOT updated** to the canonical path. The plan frontmatter's `files_modified` lists only `robot_follow_app.py` (not test files). The shim's whole purpose is to keep legacy callers working through the migration window — tests use `from robot_follow.drone_api import VelocityCommandAPI` (package-level), which the shim covers. 03-09 will batch-migrate test imports when the shim is deleted.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 — Bug] Updated `robot_follow_app.py:32` (extra legacy import the plan did not enumerate)**
- **Found during:** Task 2 (grep for `from robot_follow.drone_api` callers)
- **Issue:** The plan's `<interfaces>` block listed only one importer at `robot_follow_app.py:31` (`from robot_follow.drone_api import run_live_drone, _reap_mavsdk_server`). In reality, line 32 also imported from the now-deleted submodule path: `from robot_follow.drone_api.mavsdk_drone import add_drone_args`. After the `git mv`, that submodule path no longer resolves (the shim re-exports at the package level but does NOT keep a `mavsdk_drone` submodule under `drone_api/`). Without retargeting line 32, `robot_follow_app` would have raised `ModuleNotFoundError: robot_follow.drone_api.mavsdk_drone` at import time.
- **Fix:** Consolidated lines 31–32 into a single import from the canonical new path: `from robot_follow.robot_api.adapters.mavsdk_drone import (run_live_drone, _reap_mavsdk_server, add_drone_args)`.
- **Files modified:** `robot_follow/robot_follow_app.py`
- **Verification:** `python -c "from robot_follow.robot_follow_app import main"` exits 0; `robot-follow --help` exits 0.
- **Committed in:** `bc8c87e` (Task 2 commit)

### Plan-verify-block expectation that did not match reality

**Task 2's automated verify block** runs `grep -rn "from robot_follow.drone_api\|import robot_follow.drone_api" robot_follow/ --include='*.py' | grep -v "robot_follow/drone_api/__init__.py"` and asserts the output is empty (`test ! -s /tmp/legacy_imports.txt`). After Task 2, two legitimate hits remain:

- `robot_follow/tests/test_controller.py:13:from robot_follow.drone_api import VelocityCommandAPI`
- `robot_follow/tests/test_velocity_api_and_smoother.py:13:from robot_follow.drone_api import VelocityCommandAPI`

These are NOT bugs — they exercise the shim, which is exactly what the shim is for. The user's `additional_context` for this plan explicitly listed `files_modified` containing only `robot_follow_app.py` (not test files), confirming the intent: `robot_follow_app.py` uses canonical paths; tests stay on the legacy path until 03-09's batch migration. The plan-author's grep predicate was overly strict for this milestone. The full test suite (175 passed) is the more meaningful gate, and it passes.

---

**Total deviations:** 1 auto-fixed (Rule 1, undocumented additional importer at line 32 surfaced during grep step of Task 2).
**Impact on plan:** Zero scope creep; the fix was strictly necessary to keep `robot_follow_app` importable. Two-commit shape is a stylistic improvement, not a deviation.

## Issues Encountered

- **`source setup_env.sh` cannot persist across `Bash` tool calls** (each call starts a fresh shell). Worked around by invoking the venv's Python directly at `/home/guyz/code/guyz/hailo-drone-follow/hailo-apps/venv_hailo_apps/bin/python` and prepending the venv `bin/` to `$PATH` for pytest runs that need `robot-follow` / `drone-follow` console scripts on PATH (`test_install_smoke.py`).

## Layout-Smoke XFail Status

Both layout-smoke tests **remain xfailed** after this plan, exactly as the plan's `<verification>` block predicted:

| Test | Status | Reason |
|------|--------|--------|
| `test_robot_api_adapters_mavsdk_drone_imports` | **XFAIL** | Module imports cleanly, but `hasattr(mod, "MavsdkDroneAdapter")` is still False — the wrapper class lands in 03-06. Will flip to `XPASS` after 03-06. |
| `test_drone_api_module_is_gone` | **XFAIL** | `drone_api` package still imports cleanly (the shim is intentionally still there). Will flip to `XPASS` after 03-09 deletes the shim. |

`strict=False` on both xfail markers means coincidental flips would not break the build; both will deliberately convert to `XPASS` in their respective plans.

## Git History Verification

```
$ git log --follow --oneline robot_follow/robot_api/adapters/mavsdk_drone.py | head -3
e6ba475 refactor(03-05): git mv drone_api/mavsdk_drone.py to robot_api/adapters/
fc111c4 refactor(02-04): merge telemetry position+altitude tasks (CLEAN-13)
d0d4afb refactor(02-04): extract _reap_mavsdk_server helper (CLEAN-11)
```

History preserved through the move; bisection across the rename works.

## Import-Path Smoke Tests

| Import expression | Result | Notes |
|---|---|---|
| `from robot_follow.robot_api.adapters.mavsdk_drone import VelocityCommandAPI` | OK | canonical path |
| `from robot_follow.drone_api import VelocityCommandAPI` | OK | via shim (same object identity) |
| `from robot_follow.drone_api.mavsdk_drone import VelocityCommandAPI` | `ModuleNotFoundError` | expected; submodule file moved away. Tests do not exercise this path. |
| `from robot_follow.robot_follow_app import main` | OK | composition root imports cleanly |
| `robot-follow --help` / `drone-follow --help` | exit 0 | both console scripts intact |

## Internal-importer count updated

Plan expected: 1 (only `robot_follow_app.py:31`).
Actual: 1 file, **2 lines** (`robot_follow_app.py:31` AND `robot_follow_app.py:32`). Both consolidated into a single new import. See deviation Rule 1 above.

## Next Phase Readiness

- 03-06 can now wrap the file as `MavsdkDroneAdapter` (signature change + new class) without colliding with the move history.
- The shim stays in place until 03-09; any future caller that imports `from robot_follow.drone_api` keeps working until that deletion.
- No blockers; phase budget on track.

## Self-Check: PASSED

- File at new location exists: `robot_follow/robot_api/adapters/mavsdk_drone.py`
- Old location removed: `robot_follow/drone_api/mavsdk_drone.py` does NOT exist
- Shim file exists: `robot_follow/drone_api/__init__.py`
- Caller updated: `robot_follow/robot_follow_app.py` (imports from canonical new path)
- Task 1 commit `e6ba475` in `git log`
- Task 2 commit `bc8c87e` in `git log`
- Full suite: 175 passed
- `git log --follow` resolves through the move

---
*Phase: 03-abstraction*
*Completed: 2026-05-19*
