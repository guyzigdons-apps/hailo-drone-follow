---
phase: 02-cleanup
plan: 05
subsystem: pipeline-adapter
tags: [argparse, vision-branches, decide_branches, branch-decision, openhd, webui, display, record, xfail-flip]

# Dependency graph
requires:
  - phase: 02-cleanup
    provides: "Plan 02-00 landed test_vision_branches.py with 18 xfail tests; plans 02-01/02-03/02-04 touched vision_branches.py + robot_follow_app.py (different lines) and left them at HEAD = 9f62401"
provides:
  - "vision_branches.decide_branches(*, openhd, webui, display, record) -> BranchDecision (single source of truth for output-branch policy)"
  - "BranchDecision (frozen dataclass: display, record_branch_enabled, webui, openhd)"
  - "robot_follow_app.py: single argparse.ArgumentParser(add_help=False) pre-parser (ui+reid+tracker collapsed) + one decide_branches() call"
  - "test_vision_branches.py with 18 PASSING tests (xfail markers stripped)"
affects: [02-07, phase-3-abstraction]

# Tech tracking
tech-stack:
  added: []  # No new libraries — dataclass + argparse already on PATH
  patterns:
    - "Single-source-of-truth helper: branch-decision policy (mutex + implicit-display + record-gating) consolidated into one pure function that returns a frozen dataclass; consumers read the resolved fields instead of re-deriving them."
    - "Pre-parser writeback: the pre-parse argparse Namespace is mutated in place to carry the resolved display flag forward; the post-create_app full-parser Namespace gets the same value propagated via decision.display capture (no second decide_branches call)."
    - "xfail-strip protocol: when the production helper lands, replace the lazy _decide() shim with a module-level import and delete every @pytest.mark.xfail decorator in one commit — no markers should leak into the post-fix world."

key-files:
  created:
    - ".planning/phases/02-cleanup/02-05-SUMMARY.md"
  modified:
    - "robot_follow/robot_follow_app.py — 3 pre-parsers collapsed into 1; decide_branches() called once; duplicate mutex + duplicate implicit-display rule removed"
    - "robot_follow/pipeline_adapter/vision_branches.py — added BranchDecision + decide_branches()"
    - "robot_follow/pipeline_adapter/hailo_drone_detection_manager.py — removed the third implicit-display site inside get_pipeline_string()"
    - "robot_follow/tests/test_vision_branches.py — stripped @pytest.mark.xfail decorators; replaced lazy _decide() shim with module-level import; 18 tests now PASS"

key-decisions:
  - "decide_branches lives in vision_branches.py (not a new branch_policy.py module) — it's a 10-line dataclass + helper; co-locating with assemble_output_stage keeps the branch-policy boundary inside the module that already owns the GStreamer launch-string assembly. RESEARCH § Open Questions Q4 picked this; PLAN 02-05 confirmed."
  - "ValueError from decide_branches() is surfaced as SystemExit at the call site (robot_follow_app.main). The helper raises a pure Python exception (testable in isolation); the CLI layer converts it to a non-zero exit. Both layers are correct in isolation."
  - "Pre-parser writeback writes both pre_args.display AND args.display (after create_app). The pre_args copy is what the recording-branch + UI-state setup reads BEFORE the full parser runs; args.display is what ControllerConfig.from_args + the pipeline-string builder read AFTER. decision is captured in closure, so no second decide_branches() call is needed."
  - "tracker_factory import hoisted next to the consolidated pre-parser (was late at the old line 311). The module imports unconditionally either way; hoisting it makes the pre-parser block self-contained and the 'why is this import here?' question obvious."

patterns-established:
  - "Branch-decision helper pattern: a pure function with keyword-only args returning a frozen dataclass. Pre-parser writes back; the dataclass also captured in closure for post-full-parse propagation. No state, no I/O, trivially unit-testable across the 16-combo input matrix."
  - "Sanity grep for centralisation: after consolidating a duplicated rule, run grep -rnE for the rule's syntactic form across the package to confirm only ONE call site remains. False positives (different rule, same regex shape — e.g., the 'delegate to upstream default' guard) get triaged manually and documented."

requirements-completed: [CLEAN-12, CLEAN-15]

# Metrics
duration: 17 min
completed: 2026-05-14
---

# Phase 2 Plan 05: Pre-parser collapse + decide_branches single source of truth Summary

**Three throwaway pre-parsers in `robot_follow_app.py` collapsed into one; `decide_branches()` helper added to `vision_branches.py` as the single source of truth for the `--openhd`/`--webui` mutex + implicit-display rule + record-branch gating; 3 implicit-display duplications + 1 duplicate mutex check removed; xfail markers stripped from `test_vision_branches.py` (18 tests flipped from xfailed -> passed).**

## Performance

- **Duration:** 17 min
- **Started:** 2026-05-14T17:35:06Z
- **Completed:** 2026-05-14T17:51:48Z
- **Tasks:** 2
- **Files modified:** 4 (production code) + 1 (test) — 5 total

## Accomplishments

- **CLEAN-12: single pre-parser.** `ui_pre` (7 flags), `reid_pre` (12 flags), `tracker_pre` (1 flag) all merged into ONE `argparse.ArgumentParser(add_help=False)` with a single `parse_known_args()`. `grep -c 'ArgumentParser(add_help=False)' robot_follow/robot_follow_app.py` returns `1`. All 20 downstream references rewritten (`ui_pre_args.X` / `reid_pre_args.X` / `tracker_pre_args.X` → `pre_args.X`). install_smoke 10/10 PASS; full suite unchanged.
- **CLEAN-15: `decide_branches()` single source of truth.** `BranchDecision` (frozen dataclass: `display`, `record_branch_enabled`, `webui`, `openhd`) + `decide_branches(*, openhd, webui, display, record) -> BranchDecision` exported from `vision_branches.py`. Called once in `robot_follow_app.main()` after the pre-parser. Implicit-display rule appears in exactly ONE place (inside the helper body).
- **4 duplication sites removed:**
  1. `robot_follow_app.py:226-227` (implicit-display, pre-parse): replaced with `pre_args.display = decision.display`.
  2. `robot_follow_app.py:325-326` (DUPLICATE mutex check, post-full-parse): deleted entirely — `decide_branches` already raised.
  3. `robot_follow_app.py:330-331` (DUPLICATE implicit-display, post-full-parse): replaced with `args.display = decision.display` (decision captured in closure).
  4. `hailo_drone_detection_manager.py:1166-1167` (third implicit-display site inside `get_pipeline_string()`): deleted — `options_menu.display` is now authoritative from the pre-parser writeback.
- **xfail flip: `test_vision_branches.py` 18 xfailed -> 18 passed.** Lazy `_decide()` shim replaced with module-level `from robot_follow.pipeline_adapter.vision_branches import decide_branches`. Direct calls to `decide_branches(...)` throughout. `XFAIL_REASON` constant removed.
- **Test deltas:** Per-plan target test goes from `18 xfailed` -> `18 passed`. Full suite goes from `2 failed, 153 passed, 21 xfailed` (post-02-04) -> `2 failed, 171 passed, 3 xfailed` (post-02-05) — the 18 xfails flipped to passes, the 2 baseline failures (DEFER-02-00-A controller frame-edge tests) are unchanged, the 3 remaining xfails are all in `test_web_server_sse.py` and close in plan 02-07.

## Task Commits

Each task was committed atomically:

1. **Task 1: Collapse 3 pre-parsers into one (CLEAN-12)** — `11a69c4` (refactor)
   - `robot_follow/robot_follow_app.py | 133 +++++++++++++++++++--------------------`
   - 1 file changed, 64 insertions(+), 69 deletions(-)
2. **Task 2: decide_branches() single source of truth (CLEAN-15)** — `54b6ddb` (refactor)
   - `robot_follow/pipeline_adapter/vision_branches.py | +46`
   - `robot_follow/robot_follow_app.py | 58 +++++++++++-----------`
   - `robot_follow/tests/test_vision_branches.py | 40 ++++-----------`
   - `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py | 8 +--`
   - 4 files changed, 91 insertions(+), 61 deletions(-)

**Plan metadata:** _to be filled by git_commit_metadata step_

## decide_branches Signature

```python
@dataclass(frozen=True)
class BranchDecision:
    display: bool
    record_branch_enabled: bool
    webui: bool
    openhd: bool


def decide_branches(*, openhd: bool, webui: bool, display: bool,
                    record: bool) -> BranchDecision:
    """Single source of truth for the display/record/webui/openhd branch decision."""
    if openhd and webui:
        raise ValueError(
            "--openhd and --webui are mutually exclusive "
            "(only one network encoder may run at a time)"
        )
    if not openhd and not webui:
        display = True
    record_branch_enabled = record or webui or openhd
    return BranchDecision(
        display=display,
        record_branch_enabled=record_branch_enabled,
        webui=webui,
        openhd=openhd,
    )
```

## Files Created/Modified

- `robot_follow/robot_follow_app.py` — 3 pre-parsers → 1; `decide_branches()` call once after pre-parse; pre-parser writeback (`pre_args.display = decision.display`); duplicate mutex + duplicate implicit-display removed after `create_app`; `args.display = decision.display` propagation; tracker_factory import hoisted next to the consolidated pre-parser; 122 line delta across the two tasks.
- `robot_follow/pipeline_adapter/vision_branches.py` — added `from dataclasses import dataclass` import, `BranchDecision` frozen dataclass, and `decide_branches()` helper at the top of the module (after the module docstring, before the `select_h264_encoder` block). +46 lines.
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` — removed the 2-line `if not openhd and not webui: display = True` block inside `get_pipeline_string()`; replaced with a comment naming `options_menu.display` as authoritative. -2 lines + comment +5 = +3 net.
- `robot_follow/tests/test_vision_branches.py` — stripped 7 `@pytest.mark.xfail(...)` decorators (6 named tests + 1 parametrized); removed `XFAIL_REASON` constant; replaced `def _decide(): ... return decide_branches` lazy shim with `from robot_follow.pipeline_adapter.vision_branches import decide_branches` module-level import; rewrote `_decide()(...)` call-sites as `decide_branches(...)`. -15 lines.

## Sanity Grep — Implicit-display rule centralisation

```
$ grep -rnE "not (\w+\.)?openhd and not (\w+\.)?webui" robot_follow/ --include='*.py' --exclude-dir=__pycache__
robot_follow/pipeline_adapter/vision_branches.py:81:    if not openhd and not webui:    # canonical site (inside decide_branches)
robot_follow/tests/test_vision_branches.py:68:    if not openhd and not webui:           # test assertion verifying the rule
robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:1175: if not display and not openhd and not webui and not record \   # different rule: "delegate to upstream default pipeline" guard
```

Implicit-display rule appears in exactly ONE place (vision_branches.py:81 inside `decide_branches`). The test asserts that rule (correct). The third hit is a structurally similar but semantically distinct condition (the "all-default → delegate to upstream" early-return guard), not a duplication.

## pytest Counts Before/After

| Suite | Before plan 02-05 (HEAD=9f62401) | After plan 02-05 (HEAD=54b6ddb) |
|-------|----------------------------------|---------------------------------|
| `test_vision_branches.py` (target) | 18 xfailed | **18 passed** |
| Full (`--ignore=test_sim_worlds.py`) | 2 failed, 153 passed, 21 xfailed | 2 failed, 171 passed, 3 xfailed |
| `test_install_smoke.py` | 10 passed | 10 passed |

The 2 baseline failures (DEFER-02-00-A: `test_center_y_is_ignored`, `test_clamped_to_max_forward`) are unchanged — out of scope for this plan. The 3 remaining xfails are all in `test_web_server_sse.py` and close in plan 02-07 (CLEAN-16).

## Decisions Made

- **`decide_branches` lives in `vision_branches.py`**, not a new `branch_policy.py` module. The helper is 10 lines + a dataclass and co-locates with `assemble_output_stage` (which already owns the GStreamer launch-string assembly for the same branches). A new module would over-engineer the boundary for one phase's worth of consolidation.
- **`ValueError` raised by the helper, `SystemExit` raised by the caller.** The helper is a pure function — it raises a regular Python exception that the unit test asserts via `pytest.raises(ValueError, ...)`. The CLI layer (`robot_follow_app.main`) catches the `ValueError` and re-raises as `SystemExit("error: ...")` so users see a clean CLI error instead of a traceback. Both layers are correct in isolation; the test exercises the helper, not the CLI shim.
- **Pre-parser writeback + closure capture (no second `decide_branches()` call).** `pre_args.display` is written before `create_app` runs (so recording-branch wiring + UI-state setup see the resolved flag). `args.display = decision.display` is written after `create_app` populates `args = app.options_menu` (so `ControllerConfig.from_args(args)` and the pipeline-string builder see the same resolved flag). `decision` is captured in the enclosing scope; calling `decide_branches` twice would split the mutex/implicit-rule logic across two sites again, defeating the consolidation.
- **`tracker_factory` import hoisted to top of `main()` body next to the pre-parser.** Previously was late at the old line 311 (just above the `tracker_pre` parser). The module imports unconditionally either way — the hoist makes the pre-parser block self-contained and the import location semantically meaningful ("import the constants I'm about to wire into the parser").
- **xfail strip discipline: nothing left behind.** When the production helper lands, the lazy `_decide()` shim is GONE, every `@pytest.mark.xfail` decorator is GONE, and `XFAIL_REASON` is GONE. Module docstring updated to describe the POST-fix world (no "marked xfail until plan X" language). A single grep for `xfail` or `_decide` in the test file returns zero hits.

## Deviations from Plan

None — plan executed exactly as written. No Rule 1-3 auto-fixes were needed during execution.

Minor implementation detail not specified in PLAN but consistent with the plan's intent: the PLAN block at lines 245-246 said "REMOVE the old mutex check at lines 222-224 — `decide_branches` raises ValueError inside on the same condition. KEEP a guard if you want to catch the ValueError and re-raise as `argparse.ArgumentError`..." I chose to catch and re-raise as `SystemExit("error: ...")` (matching the original CLI error format) rather than `argparse.ArgumentError` — the original code raised `SystemExit` directly. This preserves byte-identical CLI exit behavior for users.

## Issues Encountered

None — both tasks landed without iteration. The single pre-parser smoke (`drone-follow --help`) passed first try because the consolidated parser registers the EXACT same flag set the three throwaways registered (no defaults drift versus the RESEARCH example, which was a known caveat per the plan's Step A instruction).

## User Setup Required

None — no external service configuration required. Plan is pure refactoring within the existing CLI surface.

## Next Phase Readiness

- **CLEAN-12 + CLEAN-15 closed.** ROADMAP success criterion 5 ("Branch-decision tree is defined in one place in `vision_branches`; implicit-display rule appears exactly once") is now satisfied at the code level — phase gate manual smoke (UI flags in different combinations against a running pipeline) will confirm end-to-end behaviour at phase close.
- **Plan 02-07 unblocked.** The remaining incomplete plan in this phase (CLEAN-16 SSE race fix) does not touch `robot_follow_app.py`, `vision_branches.py`, or `hailo_drone_detection_manager.py` in any of the lines consolidated by this plan. 02-07 closes the 3 remaining `test_web_server_sse.py` xfails.
- **No new blockers.** Pre-existing DEFER-02-00-A (controller frame-edge failures) tracked from plan 02-00 — should be green before Phase 3 starts but doesn't block Phase 2 close.

## Self-Check: PASSED

- `robot_follow/pipeline_adapter/vision_branches.py` — FOUND (contains `def decide_branches` at line 70)
- `robot_follow/robot_follow_app.py` — FOUND (contains `decide_branches` import + call)
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` — FOUND (implicit-display rule removed at former line 1166-1167)
- `robot_follow/tests/test_vision_branches.py` — FOUND (zero `@pytest.mark.xfail` decorators; 18 PASSING tests)
- `.planning/phases/02-cleanup/02-05-SUMMARY.md` — FOUND (this file)
- Commit `11a69c4` (Task 1 — CLEAN-12 pre-parser collapse) — FOUND
- Commit `54b6ddb` (Task 2 — CLEAN-15 decide_branches + xfail strip) — FOUND
- Per-plan target test `python -m pytest robot_follow/tests/test_vision_branches.py -v` — 18 passed, 0 xfail, 0 fail ✅
- Full suite `python -m pytest robot_follow/tests --ignore=test_sim_worlds.py` — 2 failed (DEFER-02-00-A baseline), 171 passed, 3 xfailed ✅

---
*Phase: 02-cleanup*
*Completed: 2026-05-14*
