---
phase: 01-rename
plan: 01
subsystem: testing
tags: [pytest, smoke-test, rename, forward-compatible]

# Dependency graph
requires:
  - phase: 01-rename
    provides: Phase 1 plan set (01-01..01-03) defining Wave 0 → Wave 1 → Wave 2 ordering
provides:
  - "Phase 1 verification gate: drone_follow/tests/test_install_smoke.py rewritten with 10-test contract"
  - "Forward-compatible skip-guard pattern via _robot_follow_installed() / _skip_if_pre_rename()"
  - "Always-on console-script alias contract (drone-follow must work pre- and post-rename)"
affects: [01-02-atomic-rename, 01-03-rename-followups]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Forward-compatible test pattern: importlib.util.find_spec + pytest.skip lets a single test file gate two tree states (pre- and post-rename)"
    - "pip metadata isolation: subprocess [sys.executable, '-m', 'pip', 'show', ...] binds checks to the test interpreter's venv"
    - "Help-output equivalence via difflib.unified_diff on stdout+stderr concatenation"

key-files:
  created: []
  modified:
    - drone_follow/tests/test_install_smoke.py

key-decisions:
  - "Skip-guards land in 01-01 and get stripped in 01-02 (rename commit). Wave 1 mutates the same file to flip it from forward-compatible (skips) to strict-mode (asserts)."
  - "drone-follow console-script alias is always-on (no skip-guard) — pre-rename it's the real script, post-rename it's the alias entry point. Same contract either way."
  - "pip show binds via sys.executable -m pip, not bare pip, so the test reports against the interpreter that actually owns the distribution (avoids false positives from a different venv on PATH)."

patterns-established:
  - "Forward-compatible gating: a test file that lands BEFORE a structural change can still be exercised pre-change by skip-guarding the post-change assertions. Wave N+1 strips the guards."
  - "Single-file verification gate per phase: one test file per atomic structural commit, asserting the full success-criteria contract from ROADMAP."

requirements-completed: [RENAME-01, RENAME-02]

# Metrics
duration: 1 min
completed: 2026-05-14
---

# Phase 1 Plan 1: Forward-compatible install smoke test for rename gate Summary

**10-test forward-compatible install smoke test that skip-guards `robot_follow` assertions on the pre-rename tree and enforces the full Phase 1 success-criteria contract (package imports, console scripts, --help byte-equivalence, pip metadata exclusivity) once `robot_follow` is installed.**

## Performance

- **Duration:** 1 min
- **Started:** 2026-05-14T14:04:46Z
- **Completed:** 2026-05-14T14:05:58Z
- **Tasks:** 1
- **Files modified:** 1

## Accomplishments

- Rewrote `drone_follow/tests/test_install_smoke.py` (56 → 218 lines, 10 tests) as Phase 1 verification gate.
- 8 of 10 tests skip-guarded against the pre-rename tree via `_skip_if_pre_rename()`.
- 2 always-on tests preserve the existing `drone-follow --help` contract through the rename (legacy script pre-rename, alias entry point post-rename).
- Pre-rename run on the current `feature/rover-support` tree: **2 PASSED, 8 SKIPPED, 0 FAILED** — matches the plan's expected pre-rename profile exactly.
- Wave 1 (01-02) inherits a working test file it can flip to strict mode by stripping the `_skip_if_pre_rename()` calls.

## Task Commits

1. **Task 1: Rewrite test_install_smoke.py as forward-compatible Phase 1 gate** — `0869454` (test)

**Plan metadata:** pending (post-summary commit)

## Files Created/Modified

- `drone_follow/tests/test_install_smoke.py` — Replaced 3-test legacy file with 10-test forward-compatible Phase 1 verification gate. Adds module-level `_robot_follow_installed()` detector and `_skip_if_pre_rename()` helper, `difflib`-based diff message on `--help` byte-equivalence failure, and `sys.executable -m pip show` binding for pip metadata tests.

## Decisions Made

- **Skip-guard pattern over duplicate files.** Could have left the legacy file untouched and added `tests/test_install_smoke_renamed.py` for Wave 1 to land. Rejected: two files for one contract is sprawl, and a duplicate forces Wave 1 to delete one in the rename commit anyway. Single forward-compatible file with `_skip_if_pre_rename()` guards is one-commit-now / one-edit-in-Wave-1 (strip guards), which is cleaner.
- **Always-on `drone-follow --help` test (no skip).** The console-script alias is the user-facing invocation contract (`scripts/start_air.sh`, boot service, muscle memory). Pre-rename it's the real entry point; post-rename it's a pyproject.toml alias mapping to the same `main()`. Either way the contract is "`drone-follow --help` exits 0 and mentions `--input`". A skip-guard would be a regression risk window.
- **`sys.executable -m pip show` over bare `pip`.** Pytest runs under `/usr/bin/python3` (system pytest at `~/.local/bin/pytest`) while the venv is on PATH only for executable resolution. Bare `pip` would resolve to the venv's pip — usually correct, but brittle if PATH ordering changes. Binding via `sys.executable` reports against the interpreter that actually owns the test, which matches what the assertion semantics want.

## Deviations from Plan

None - plan executed exactly as written.

The plan's `<behavior>` block was followed item-by-item for all 10 tests. The implementation notes in `<action>` (use `find_spec` not try/except import, use `pytest.skip` at function granularity, concatenate stdout+stderr for byte-identical check, use `difflib.unified_diff` for failure messages, use `sys.executable -m pip`, preserve `env={**os.environ}` pattern) were all applied.

One minor note: the file landed at 218 lines vs the planner's "~150 lines" soft target. Reason: per-test docstrings (requested in `<behavior>`) plus the `difflib` diff-message branch plus the module docstring describing the forward-compatibility design. The planner's hard floor (`min_lines: 90`) is satisfied 2.4×; the 150 was a guideline, not a constraint.

---

**Total deviations:** 0 auto-fixed
**Impact on plan:** None — plan executed verbatim.

## Issues Encountered

None.

## User Setup Required

None - no external service configuration required.

## Next Phase Readiness

**Ready for 01-02 (Wave 1: atomic rename).** Wave 1 will:

1. Perform the atomic `drone_follow/` → `robot_follow/` rename via `git mv` (one commit).
2. Update `pyproject.toml` to add the `robot-follow` console script + `drone-follow` alias entry point pointing at the same `main()`.
3. Strip the `_skip_if_pre_rename()` calls from this same `test_install_smoke.py` file (now `robot_follow/tests/test_install_smoke.py` after the dir rename) so the post-rename assertions become hard gates.
4. Re-run `pytest robot_follow/tests/test_install_smoke.py -v` — expect 10 PASSED, 0 SKIPPED.

The Wave-1 strip-guard step is the verification handshake: if any of the 8 currently-skipped assertions fails post-rename, the rename commit is incomplete and must be amended before merge.

### Self-Check

- File `drone_follow/tests/test_install_smoke.py` exists on disk and contains 10 tests (verified via `pytest --collect-only`).
- Commit `0869454` exists on `feature/rover-support` (`git log --oneline | grep 0869454`).
- Pre-rename test run: 2 PASSED + 8 SKIPPED + 0 FAILED (verified above).
- No `from drone_follow` / `import drone_follow` introduced outside the negative-assertion test (`test_drone_follow_import_raises_after_rename`), which uses `import_module("drone_follow")` inside a `pytest.raises` block — semantically a runtime probe, not a static dependency.

## Self-Check: PASSED

---
*Phase: 01-rename*
*Completed: 2026-05-14*
