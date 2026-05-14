---
phase: 02-cleanup
plan: 03
subsystem: cleanup
tags: [argparse, mavsdk, cli, docs]

requires:
  - phase: 02-cleanup
    provides: "Wave 0 xfail scaffolds (02-00); clean tree at HEAD 7618a62"
provides:
  - "Documented --mission-duration semantics (argparse help + CLAUDE.md bullet)"
  - "Honest serial_baud attribute access in _resolve_serial_connection"
  - "Clean run_live_drone signature with no shutdown_read_fd plumbing"
affects: [02-04, 02-05, 02-06, 02-07, 03-drone-abstraction]

tech-stack:
  added: []
  patterns:
    - "Document load-bearing flags rather than removing them — RESEARCH-driven"
    - "Drop getattr fallbacks for unconditionally-registered argparse fields"
    - "Remove unreachable kwargs at the same time as their dead inner blocks"

key-files:
  created:
    - ".planning/phases/02-cleanup/02-03-SUMMARY.md"
  modified:
    - "robot_follow/drone_api/mavsdk_drone.py"
    - "robot_follow/robot_follow_app.py"
    - "CLAUDE.md"

key-decisions:
  - "CLEAN-04 documents --mission-duration instead of removing it; RESEARCH confirmed it is read at mavsdk_drone.py:767 (auto-land timeout) and :799 (control-loop iteration timeout). Removal would silently turn 300s default into infinite mission."
  - "CLEAN-05 keeps the outer getattr(args, 'serial', None) guard — --serial has a real None default. Only the inner serial_baud fallback was unreachable."
  - "CLEAN-08 keeps `import os` at the top of mavsdk_drone.py — used by os.getuid(), os.path.join, os.path.exists, os.environ elsewhere in the file."

patterns-established:
  - "Wave-1 parallel-plan hygiene: this plan's three edits in mavsdk_drone.py and robot_follow_app.py do not overlap with 02-01 (sim + pipeline_adapter) or 02-02 (follow_api + tests + servers) file sets, so the three plans can land in any order."

requirements-completed: [CLEAN-04, CLEAN-05, CLEAN-08]

duration: 3min
completed: 2026-05-14
---

# Phase 02-cleanup Plan 03: Three Surgical Edits in mavsdk_drone.py + robot_follow_app.py Summary

**Documented `--mission-duration`, dropped the lying `serial_baud` getattr fallback, and removed the unreachable `shutdown_read_fd` pipe-reader plumbing — three small wave-1C edits with no behavioural change.**

## Performance

- **Duration:** 3 min
- **Started:** 2026-05-14T17:06:29Z
- **Completed:** 2026-05-14T17:09:05Z
- **Tasks:** 3
- **Files modified:** 3

## Accomplishments
- `drone-follow --help` now explains `--mission-duration` semantics (auto-land at expiry with `--takeoff-landing`, control-loop restart without). CLAUDE.md's Key CLI Flags section gained a bullet warning that the 300 s default is a surprise hazard.
- `_resolve_serial_connection` reads `args.serial_baud` directly. The previous `getattr(args, "serial_baud", 115200)` would have shipped the wrong baud rate (115200 vs registered default 57600) if the registration ever broke — silent and hard to diagnose.
- `run_live_drone` signature lost its `shutdown_read_fd=None` parameter, and the 14-line `if shutdown_read_fd is not None:` block (with the nested `_on_shutdown_pipe` reader) was deleted. The single caller never passed the kwarg, so the block was unreachable.

## Task Commits

Each task was committed atomically:

1. **Task 1: Document `--mission-duration` (CLEAN-04)** — `9b6682f` (docs)
2. **Task 2: Drop `serial_baud` getattr fallback (CLEAN-05)** — `3e24dc9` (refactor)
3. **Task 3: Remove `shutdown_read_fd` plumbing (CLEAN-08)** — `10abf64` (refactor)

**Plan metadata:** (this SUMMARY commit)

## Files Created/Modified

- `robot_follow/drone_api/mavsdk_drone.py` — Added `help=` string to `--mission-duration` argparse registration (Task 1); removed `shutdown_read_fd=None` parameter and the entire pipe-reader block at lines 642–665 (Task 3). `import os` preserved (still used by `os.getuid`, `os.path.*`, `os.environ`).
- `robot_follow/robot_follow_app.py` — Replaced `baud = getattr(args, "serial_baud", 115200)` with `baud = args.serial_baud` in `_resolve_serial_connection` (Task 2). Outer `getattr(args, "serial", None) is not None` guard unchanged.
- `CLAUDE.md` — Added a Key CLI Flags bullet documenting `--mission-duration` watchdog timeout behaviour and the 300 s surprise-hazard note (Task 1).

## Decisions Made

- **CLEAN-04 is DOCUMENT, not REMOVE.** RESEARCH § CLEAN-04 confirmed `--mission-duration` is load-bearing — read at `mavsdk_drone.py:767` (auto-land timeout in the `--takeoff-landing` branch) and `:799` (control-loop iteration timeout in the no-takeoff-landing branch). Removing it would silently change behaviour. The fix is to document the watchdog semantics so operators know what 300 s does and how to disable it (`--mission-duration 86400`).
- **CLEAN-05 keeps the outer guard.** `getattr(args, "serial", None) is not None` is intentional — `--serial` has a real `nargs="?", default=None` registration, so the guard is testing whether the operator passed the flag at all. Only the inner `serial_baud` fallback was unreachable.
- **CLEAN-08 keeps `import os`.** The file has multiple other `os.*` usages (`os.getuid()` at line 230, `os.path.join`/`os.path.exists` at lines 212/216, `os.environ` references elsewhere). Removing the import alongside the pipe-reader block would have introduced a `NameError`.

## Deviations from Plan

None - plan executed exactly as written.

The three tasks ran clean: per-task automated verifies and the quick suite (36 tests, 2.1 s) passed on each commit; the full suite at end-of-plan reproduced the documented baseline (2 failed in `test_controller.py::TestDistanceForward::*`, 152 passed, 21 xfailed) — those two failures are DEFER-02-00-A and pre-date this plan.

## Issues Encountered

- **`--help` grep gate needed multi-line awareness.** The plan's verify command (`grep -qE "mission-duration.*(auto-land|timeout|restart|watchdog)"`) assumed `mission-duration` and the help keywords would land on the same output line, but argparse wraps long help strings — the flag is on its own line, with the help text wrapped on subsequent lines. Worked around by switching to `grep -A 6 "^  --mission-duration" | grep -qE "(auto-land|timeout|restart|watchdog)"`, which is multi-line aware and asserts the same invariant. No code change was needed — the help text is correct, only the gate command had to be adjusted at verify time. Not a deviation from plan substance.

## Verification Gates Run

1. **Gate 1** — `drone-follow --help` shows `--mission-duration` followed by help text containing `auto-land|timeout|restart|watchdog`: PASS.
2. **Gate 2** — `grep -q "mission-duration" CLAUDE.md`: PASS.
3. **Gate 3** — `grep -c 'getattr(args, "serial_baud"' robot_follow/robot_follow_app.py` returns 0: PASS.
4. **Gate 4** — `inspect.signature(run_live_drone).parameters` does not contain `shutdown_read_fd`: PASS.
5. **Gate 5** — CLEAN-04 commit message (9b6682f) contains no `delete` or `remove` tokens: PASS (says "document").
6. **Quick suite** — 36 tests pass in 2.1 s (post-Task-1, post-Task-2, post-Task-3).
7. **Full suite** — 152 passed, 21 xfailed, 2 failed. Failures are pre-existing DEFER-02-00-A (`test_controller.py::TestDistanceForward::test_center_y_is_ignored` and `::test_clamped_to_max_forward`), not caused by this plan.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

- Plan 02-03 lands cleanly alongside the parallel 02-01 (sim + pipeline_adapter) and 02-02 (follow_api + tests + servers) plans — their file sets do not overlap.
- Wave 2 (CLEAN-11..14, CLEAN-15 xfail strip) can proceed once 02-01 and 02-02 land. No new blockers introduced.
- Pre-existing concern still open: DEFER-02-00-A (2 `test_controller.py` failures) should be green before Phase 3 starts, per STATE.md. Out of scope for this plan.

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*

## Self-Check: PASSED

- FOUND: `.planning/phases/02-cleanup/02-03-SUMMARY.md`
- FOUND: `robot_follow/drone_api/mavsdk_drone.py`
- FOUND: `robot_follow/robot_follow_app.py`
- FOUND: `CLAUDE.md`
- FOUND commit: `9b6682f` (Task 1: CLEAN-04 docs)
- FOUND commit: `3e24dc9` (Task 2: CLEAN-05 refactor)
- FOUND commit: `10abf64` (Task 3: CLEAN-08 refactor)
