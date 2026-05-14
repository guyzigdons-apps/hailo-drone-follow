---
phase: 02-cleanup
plan: 04
subsystem: drone-api
tags: [mavsdk, telemetry, asyncio, refactor, dedup]

# Dependency graph
requires:
  - phase: 02-cleanup
    provides: "02-03 mavsdk_drone.py surgical edits (removed CLEAN-04/05/08 dead code at different line ranges)"
provides:
  - "Single _reap_mavsdk_server helper in robot_follow.drone_api.mavsdk_drone (re-exported via drone_api package)"
  - "Single position-stream subscription in mavsdk_drone.py — _telemetry_position_task writes both telemetry_cache and altitude_cache"
  - "altitude_cache initialised eagerly at run_live_drone entry (no longer lazy via deleted _start_altitude_telemetry)"
affects: ["02-cleanup phase gate", "phase-03-robot-abstraction (touches the adapter boundary; cleaner deduped surface)"]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Module-level helper for cross-call-site duplication (CLEAN-11 pattern: extract helper into adapter module, re-export from package __init__)"
    - "Shape A cache-merge (CLEAN-13 pattern: keep dict APIs, fan out from single stream subscriber) — defers cache consolidation (Shape B) to Phase 3"

key-files:
  created: []
  modified:
    - "robot_follow/drone_api/mavsdk_drone.py — helper added, _telemetry_altitude_task and _start_altitude_telemetry removed, merged _telemetry_position_task signature, spawn-site hoist"
    - "robot_follow/drone_api/__init__.py — re-export _reap_mavsdk_server"
    - "robot_follow/robot_follow_app.py — finally-block uses helper; subprocess import dropped"

key-decisions:
  - "Keep time.sleep(0.3) settle inline in DetachedMavsdkServer.__enter__ — it's specific to the respawn flow (release UDP 14540 / TCP 50051 before binding), not the finally-block path"
  - "Drop subprocess import from robot_follow_app.py — line 448 was its only user; os import stays (used by os.path / os.environ elsewhere)"
  - "Shape A for CLEAN-13 (keep both dict caches, fan out from one task) per RESEARCH § CLEAN-13 recommendation — Shape B (drop altitude_cache, read telemetry_cache['rel_alt']) deferred to Phase 3"
  - "altitude_cache hoisted to run_live_drone entry alongside telemetry_cache (was lazy-created inside the deleted _start_altitude_telemetry). Both --takeoff-landing branches now share the same eagerly-created cache; live_control_loop's altitude-hold P-loop continues to gate on its own conditions, so early population is harmless"
  - "_start_altitude_telemetry deleted entirely (not retained as a no-op) — its lifecycle role was 'late-spawn altitude task and swap-and-cancel the telem-log task'; both are obsolete now"
  - "Docstring of merged task says 'the MAVSDK position stream' instead of 'drone.telemetry.position()' — keeps the grep gate (1 occurrence) clean without making the docstring less informative"

patterns-established:
  - "Atomic per-task commits via explicit pathspec (`git commit -m '...' -- <files>`) instead of `git add` + `git commit` — survives parallel-wave staging-area collisions"
  - "Module-level mavsdk-process helpers belong in mavsdk_drone.py (not robot_follow_app.py) — keeps subprocess + mavsdk imports co-located"

requirements-completed: [CLEAN-11, CLEAN-13]

# Metrics
duration: 5min
completed: 2026-05-14
---

# Phase 02 Plan 04: Duplication Merges (CLEAN-11 + CLEAN-13) Summary

**Single `_reap_mavsdk_server` helper replaces inlined `pkill -9 mavsdk_server` at two sites, and `_telemetry_position_task` now writes both telemetry + altitude caches from one MAVSDK position-stream subscriber.**

## Performance

- **Duration:** 5 min
- **Started:** 2026-05-14T17:15:27Z
- **Completed:** 2026-05-14T17:20:10Z
- **Tasks:** 2
- **Files modified:** 3

## Accomplishments

- **CLEAN-11:** Extracted `_reap_mavsdk_server()` helper to `mavsdk_drone.py`; both call sites (DetachedMavsdkServer.__enter__ at line 252 + robot_follow_app.py finally-block at line 445) delegate to it. Whole-package grep for `pkill.*mavsdk_server` returns 1 hit (the helper body).
- **CLEAN-13:** Merged `_telemetry_altitude_task` (deleted) and `_telemetry_position_task` (signature widened to accept both caches) into a single subscriber. Whole-file grep for `drone.telemetry.position()` returns 1 hit; `_telemetry_altitude_task` symbol gone (0 occurrences).
- **Lifecycle simplification:** `_start_altitude_telemetry` helper deleted — its swap-and-cancel telem-log upgrade is obsolete now that altitude_cache exists from the start. `alt_task` local + its finally-block cleanup also removed.
- Drone behaviour preserved: `live_control_loop`'s altitude-hold P-loop continues reading `altitude_cache.get("m")` (cache is just populated earlier now). Telemetry-log task continues reading both `telemetry_cache` and `altitude_cache`.

## Task Commits

Each task was committed atomically using explicit pathspec to avoid parallel-wave staging collisions:

1. **Task 1: Extract _reap_mavsdk_server helper (CLEAN-11)** — `d0d4afb` (refactor)
   - Files: `robot_follow/drone_api/mavsdk_drone.py`, `robot_follow/drone_api/__init__.py`, `robot_follow/robot_follow_app.py`
2. **Task 2: Merge telemetry tasks, Shape A (CLEAN-13)** — `fc111c4` (refactor)
   - Files: `robot_follow/drone_api/mavsdk_drone.py`

**Plan metadata commit:** (to be created after this SUMMARY lands)

Commands used (mirroring the Wave 2 lessons-learned in `.claude/memory/feedback_parallel_wave_worktree_isolation.md`):
```
git commit -m "..." -- robot_follow/drone_api/mavsdk_drone.py robot_follow/drone_api/__init__.py robot_follow/robot_follow_app.py
git commit -m "..." -- robot_follow/drone_api/mavsdk_drone.py
```
Sibling unstaged work on `follow_api/config.py`, `servers/web_server.py`, `servers/openhd_bridge.py`, `tests/test_config_persistence.py` (parallel plan 02-06) was never staged or referenced.

## Files Created/Modified

- `robot_follow/drone_api/mavsdk_drone.py` — Added `_reap_mavsdk_server()` (lines 55-78), refactored `DetachedMavsdkServer.__enter__` to delegate, merged `_telemetry_position_task` signature `(drone, telemetry_cache, altitude_cache, shutdown)`, deleted `_telemetry_altitude_task` and `_start_altitude_telemetry`, hoisted `altitude_cache = {}` to `run_live_drone` entry, dropped `alt_task` local + finally cleanup. Net: +43 / -32 across the two commits.
- `robot_follow/drone_api/__init__.py` — Re-exports `_reap_mavsdk_server` (added to import block and `__all__`).
- `robot_follow/robot_follow_app.py` — finally-block calls `_reap_mavsdk_server()` instead of inlined `subprocess.run([...pkill...])`; dropped now-unused `subprocess` import (top-of-file).

## Decisions Made

See `key-decisions` in frontmatter. The substantive choices were:

1. **Shape A over Shape B for CLEAN-13.** Both `altitude_cache["m"]` and `telemetry_cache["rel_alt"]` carry the same `position.relative_altitude_m` value; collapsing to a single dict (Shape B) would touch `live_control_loop`'s signature at lines 438, 512, 553 and the spawn sites at the lines that pass `altitude_cache` through. RESEARCH § CLEAN-13 recommends Shape A for Phase 2 because cache consolidation is a Phase-3 concern (Phase 3 already moves altitude fields to `Optional` behind the adapter boundary).
2. **Eager altitude_cache init at `run_live_drone` entry.** The pre-edit code lazy-created `alt_cache = {}` inside `_start_altitude_telemetry` and re-bound it into the caller via `nonlocal alt_task` + `return alt_cache`. With the merge, the single position task spawns at startup, so the cache must exist before that spawn. Hoisting it to the same point as `telemetry_cache = {}` makes the two consumers symmetric and removes the closure dance.
3. **`_start_altitude_telemetry` deleted entirely.** Per the plan's three-case dispatch, this is the "function does nothing useful after the merge" case — both its side effects (spawn altitude task, swap-and-cancel telem-log task to include altitude_cache) are obsolete. No callers remain after the spawn-site rewrite.
4. **Drop `subprocess` import from `robot_follow_app.py`.** Grep confirmed line 448 was the only user; `os.getuid()` was likewise only used in that single pkill invocation. `os` stays (heavy use of `os.path` / `os.environ` elsewhere).

## Deviations from Plan

None — plan executed exactly as written. The plan's three-case dispatch for `_start_altitude_telemetry` correctly anticipated the "delete entirely" outcome (case 1: "its only side effects were spawning `_telemetry_altitude_task` and returning the task handle... DELETE the function").

One minor in-task adjustment: the merged task's docstring originally said `drone.telemetry.position()` verbatim, which would have made the grep gate report 2 hits (1 docstring + 1 call site). Rephrased to "the MAVSDK position stream" so the grep gate cleanly checks the call site count — same meaning, zero behavioural impact. Logged here for honesty rather than as a deviation.

## Issues Encountered

**Pre-existing failures (DEFER-02-00-A, unchanged by this plan):**
- `test_controller.py::TestDistanceForward::test_center_y_is_ignored` — fails with `cmd_bot.forward_m_s == -0.75` vs expected `0.0`
- `test_controller.py::TestDistanceForward::test_clamped_to_max_forward` — fails with `1.4970000000000003` vs expected `1.5`

Both failures predate this plan (logged in 02-00-SUMMARY.md and `.planning/phases/02-cleanup/deferred-items.md`). Verified by inspection: this plan's diff is entirely within `mavsdk_drone.py` + `drone_api/__init__.py` + `robot_follow_app.py`, none of which is exercised by `test_controller.py`. Recommended action remains "fold into CLEAN-19 before Phase 3 starts".

**Suite result with this plan:** 153 passed, 2 failed (pre-existing), 21 xfailed (expected). Zero regressions.

## User Setup Required

None — no external service configuration required.

## Next Phase Readiness

- **02-04 closed.** CLEAN-11 + CLEAN-13 landed. The mavsdk_drone.py surface used by Phase 3's `MavsdkDroneAdapter` is now deduped: single reaper helper, single position-stream subscriber.
- **No new blockers introduced.** Phase 3 (`MavsdkDroneAdapter`) will inherit a cleaner `run_live_drone` that hoists `altitude_cache` eagerly and routes both caches through one task — the adapter boundary work in Phase 3 doesn't need to wrestle with the swap-and-cancel telem-log pattern that's now gone.
- **Recommended follow-up (not blocking):** Shape B (cache consolidation — drop `altitude_cache` and have `live_control_loop` read `telemetry_cache["rel_alt"]`) fits naturally inside the Phase-3 adapter cut, since the adapter will already touch `live_control_loop`'s signature.

## Self-Check

Verifications:

| Claim | Verification | Result |
|-------|--------------|--------|
| `_reap_mavsdk_server` helper exists in mavsdk_drone.py | `grep -n "def _reap_mavsdk_server" robot_follow/drone_api/mavsdk_drone.py` | FOUND at line 59 |
| Helper re-exported via drone_api package | `python -c "from robot_follow.drone_api import _reap_mavsdk_server"` | OK |
| Both call sites delegate | grep `_reap_mavsdk_server(` returns 2 call-site hits (lines 252 + 445) | OK |
| Whole-package `pkill.*mavsdk_server` count = 1 | `grep -rnE 'pkill.*mavsdk_server' robot_follow/ --include='*.py'` | 1 (helper only) |
| Single position subscription | `grep -c "drone.telemetry.position()" robot_follow/drone_api/mavsdk_drone.py` | 1 |
| `_telemetry_altitude_task` symbol removed | `grep -c "_telemetry_altitude_task" robot_follow/drone_api/mavsdk_drone.py` | 0 |
| Merged task accepts both caches | `inspect.signature(_telemetry_position_task)` shows `(drone, telemetry_cache, altitude_cache, shutdown)` | OK |
| Task 1 commit exists with intended files only | `git show --stat d0d4afb` shows 3 files (mavsdk_drone.py, __init__.py, robot_follow_app.py) | FOUND |
| Task 2 commit exists with intended files only | `git show --stat fc111c4` shows 1 file (mavsdk_drone.py) | FOUND |
| Full pytest suite green except DEFER-02-00-A baseline | `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py` → 153 passed, 2 failed (pre-existing), 21 xfailed | OK |

## Self-Check: PASSED

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*
