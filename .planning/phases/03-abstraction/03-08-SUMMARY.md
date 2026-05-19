---
phase: 03-abstraction
plan: 08
subsystem: cli
tags: [argparse, two-pass-dispatch, --robot, run_robot, composition-root, ABS-08, ABS-09]

requires:
  - phase: 03-07
    provides: "run_drone() wraps run_robot_loop in args.mission_duration deadline (B1 fix); add_drone_args lives in robot_api.adapters.mavsdk_drone"
  - phase: 03-02
    provides: "test_cli_help_dispatch.py with 7 xfail-marked tests pending --robot dispatch"
  - phase: 01-rename
    provides: "parser.prog='robot-follow' policy so robot-follow / drone-follow aliases produce byte-identical --help"

provides:
  - "robot_follow_app.add_common_args(parser): registers ControllerConfig.add_args + _add_app_args + add_tracker_args (all robot-agnostic)"
  - "robot_follow_app.add_rover_args(parser): empty stub (just the argument group); Phase 4 ROVER-05 fills in --cmd-vel-topic, --ros-namespace, --ros-domain-id"
  - "robot_follow_app._build_parser(): two-pass argparse — pre-parser extracts --robot, full parser dispatches to add_drone_args or add_rover_args"
  - "robot_follow_app.run_robot(): renamed from run_drone(); body dispatches by args.robot value; PRESERVES 03-07's mission_duration deadline wrap"
  - "robot_thread = threading.Thread(target=run_robot, name='robot-follow-control', daemon=True) (was drone_thread)"
  - "test_cli_help_dispatch.py: 7 PASSING tests (xfail markers stripped); locks dispatch behavior"

affects: [03-09, 03-10, Phase-4-rover]

tech-stack:
  added: []
  patterns:
    - "Two-pass argparse: tiny pre-parser with add_help=False + parse_known_args extracts --robot, full parser is built robot-conditionally"
    - "Composition root rename without runtime drift: run_drone → run_robot preserves the asyncio.wait + getattr(args, 'mission_duration', math.inf) deadline wrap mechanically"
    - "Rover-branch stub raises NotImplementedError with a forward-looking comment pointing at Phase 4's ROVER-04 friendly RuntimeError replacement"
    - "Common-flags helper (add_common_args) operates on a parser already populated by upstream get_pipeline_parser() — pipeline flags are not duplicated"

key-files:
  created: []
  modified:
    - "robot_follow/robot_follow_app.py — _build_parser two-pass + add_common_args + add_rover_args + run_drone→run_robot rename + drone_thread→robot_thread rename"
    - "robot_follow/tests/test_cli_help_dispatch.py — 4 @pytest.mark.xfail decorators + XFAIL_REASON constant removed; docstring updated to post-fix world"

key-decisions:
  - "add_drone_args stays in robot_api.adapters.mavsdk_drone (Q3 inertia per 03-RESEARCH § Open Question 3): the app-side _build_parser just imports it. No wrapper introduced; one canonical add_drone_args."
  - "Empty add_rover_args body in Phase 3: Phase 4 fills in --cmd-vel-topic etc. Per DESIGN-NOTES line 128, no flag may be in both add_drone_args and add_rover_args."
  - "Rover-branch stub uses NotImplementedError (not RuntimeError) in Phase 3. Phase 4 ROVER-04 replaces it with the friendly RuntimeError('ROS 2 not sourced — run sudo apt install ros-humble-ros-base then re-source setup_env.sh') path. The docstring + inline comment in run_robot tell the next reader where the replacement will land."
  - "B1 carry-through done mechanically: the asyncio.wait/FIRST_COMPLETED + getattr(args, 'mission_duration', math.inf) wrap is moved from run_drone to run_robot verbatim (only outer log prefix changed from [drone] to [robot]). Watchdog stays in the composition root, NOT in run_robot_loop — keeps the orchestrator robot-agnostic so the rover path (no --mission-duration) gets math.inf naturally."
  - "_build_parser() calls get_pipeline_parser() FIRST to create the parser (upstream factory creates the parser with --input, --tiles-x, etc. pre-installed), then add_common_args + the robot-specific add_*_args register the rest. add_common_args does NOT call get_pipeline_parser internally because that would return a fresh parser, losing all the work."
  - "parser.prog = 'robot-follow' pinned at the end of _build_parser so the byte-identical alias requirement (Phase 1 01-02) survives the parser rebuild."

patterns-established:
  - "Two-pass argparse with pre-parser: parse_known_args on a tiny add_help=False parser to extract the dispatch key, then build the full parser conditionally on the dispatch value. Re-add the dispatch key (--robot) to the full parser at the end so --help renders it and final parse_args accepts it."
  - "Composition-root rename: rename the runtime function (run_drone → run_robot) AND the thread variable (drone_thread → robot_thread, with name='robot-follow-control') in a single atomic commit so bisect survives and grep finds the new symbols at every point in history."

requirements-completed: [ABS-08, ABS-09]

duration: ~25min
completed: 2026-05-19
---

# Phase 3 Plan 08: Composition root + CLI dispatch Summary

**Two-pass argparse with `--robot {drone,rover}` dispatch landed; `run_drone()` renamed to `run_robot()` with body that dispatches by `args.robot`; 03-07's `args.mission_duration` deadline wrap preserved mechanically; 7 xfail markers in `test_cli_help_dispatch.py` stripped; `drone-follow` alias still byte-identical with `robot-follow`.**

## Performance

- **Duration:** ~25 min (Task 1 implementation + verification + Task 2 xfail strip + verification)
- **Tasks:** 2 (Task 1 atomic commit for parser refactor; Task 2 separate commit for xfail strip)
- **Files modified:** 2 (robot_follow_app.py, test_cli_help_dispatch.py)
- **Files created:** 0
- **Files deleted:** 0

## Verification Evidence

### `--robot drone --help` vs `--robot rover --help`

Per-flag count via `grep -c -E '(--takeoff-landing|--target-altitude|--serial)'`:

| Help variant            | Hit count | Expected         |
| ----------------------- | --------- | ---------------- |
| `--robot drone --help`  | 20        | nonzero (3 flags appear in usage + their own descriptions + cross-references in other flags' help text) |
| `--robot rover --help`  | 0         | 0 (drone flags filtered out by add_rover_args branch) |

The plan's verify line counts lines (not unique flags), and `--takeoff-landing` appears once in the usage block plus several times in help text of other drone flags that cross-reference it (e.g. `--target-altitude` mentions `--takeoff-landing`); same story for `--serial`. The relevant property is **drone = nonzero, rover = exactly 0**, which holds.

Common flags (`--webui`, `--display`) appear in both:

```
$ python -m robot_follow.robot_follow_app --robot drone --help | grep -E '(--webui|--display)' | head -2
                    [--webui] [--webui-port WEBUI_PORT]
                    [--display] [--record]
$ python -m robot_follow.robot_follow_app --robot rover --help | grep -E '(--webui|--display)' | head -2
                    [--webui] [--webui-port WEBUI_PORT]
                    [--display] [--record]
```

### Alias byte-identity (Phase 1 01-02 carry-over)

```
$ diff <(robot-follow --help) <(drone-follow --help) && echo OK
OK
```

### Xfail strip count

```
$ grep -c "xfail" robot_follow/tests/test_cli_help_dispatch.py
0
```

7 xfail markers stripped (4 `@pytest.mark.xfail` decorators — 3 parametrized + 1 non-parametrized; 1 `XFAIL_REASON` constant; plus the "xfail until ..." line in the module docstring).

### Test-suite delta

| Counter             | Baseline (pre-plan) | After Task 1 | After Task 2 (final) |
| ------------------- | ------------------- | ------------ | -------------------- |
| `passed`            | 275                 | 275          | **282 (+7)**         |
| `xfailed`           | 5                   | 2            | 2                    |
| `xpassed`           | 5                   | 8 (+3 rover) | 1                    |
| `skipped`           | 1                   | 1            | 1                    |
| Unexpected failures | 0                   | 0            | 0                    |

The net of +7 passed matches the 7 stripped xfail markers. The drop in xpassed from 8 → 1 is because the 7 cli_help_dispatch tests turned from xpassed (was 7 with markers but passing) to real passed (markers gone). The 1 remaining xpassed is unrelated (a holdover xpass in another test — left for a future plan to clean up).

### B1 carry-through (mission_duration deadline)

Locked via `inspect.getsource`:

```
$ python -c "
import inspect
from robot_follow.robot_follow_app import main
src = inspect.getsource(main)
assert 'def run_robot' in src
assert 'mission_duration' in src
assert 'FIRST_COMPLETED' in src
assert 'asyncio.wait' in src
assert 'getattr(args, \"mission_duration\", math.inf)' in src
print('B1 carry-through OK')
"
B1 carry-through OK
```

The relevant 4-6 lines from `run_robot()`'s body (in `robot_follow/robot_follow_app.py`):

```python
async def _main():
    duration = getattr(args, "mission_duration", math.inf)
    loop_task = asyncio.create_task(
        run_robot_loop(adapter, shared_state, controller_config, shutdown)
    )
    deadline_task = asyncio.create_task(asyncio.sleep(duration))
    try:
        done, pending = await asyncio.wait(
            [loop_task, deadline_task],
            return_when=asyncio.FIRST_COMPLETED,
        )
```

Identical to 03-07's `run_drone` body except for the log-prefix change (`[drone] → [robot]`) on the warning lines that fire on `Exception`.

### Rover branch stub

```python
elif args.robot == "rover":
    # Phase 4 replaces this stub with ROVER-04's friendly
    # ``RuntimeError("ROS 2 not sourced — ...")`` path.
    raise NotImplementedError(
        "Rover adapter lands in Phase 4. "
        "Run with --robot drone (default) until then."
    )
```

The inline comment explicitly points at Phase 4 ROVER-04 for the next reader; the docstring on `run_robot()` repeats the pointer with the full friendly message text so `pydoc robot_follow.robot_follow_app.main` (or grep) finds it.

## Commits

| # | Hash      | Type | Message                                                                                                                            |
| - | --------- | ---- | ---------------------------------------------------------------------------------------------------------------------------------- |
| 1 | `cafdcc1` | feat | two-pass argparse with --robot dispatch + run_robot() composition root (preserves 03-07 mission_duration deadline)                |
| 2 | `e8e2ec1` | test | strip xfail markers in test_cli_help_dispatch.py (7 tests pass)                                                                    |

Per Phase 3 commit-shape policy: 2 commits, each bisect-safe (suite green at HEAD of each task).

## Deviations from Plan

None — the plan was executed exactly as written. A handful of mechanical adjustments to match the plan's intent against the existing code:

- The plan's interface sketch showed `add_common_args` calling `get_pipeline_parser(parser=parser)` to install pipeline flags. The upstream `get_pipeline_parser()` does NOT accept a parser kwarg (it always creates a fresh `argparse.ArgumentParser` — see `hailo-apps/hailo_apps/python/core/common/parser.py:106`). The plan's note "Strategy: leave most of the existing parser-building code in place but factor it into add_common_args" allows for this: `_build_parser()` now calls `get_pipeline_parser()` FIRST to create the parser, then `add_common_args(parser)` registers everything else on top. The pipeline flags still appear in --help for both robots — they're just installed by the factory, not by `add_common_args`. This honors the plan's stated invariant (pipeline flags are common to both robots) without changing the upstream signature.

This adjustment was a Rule 3 fix (would have been a TypeError otherwise) and is documented in the Task 1 commit message + `add_common_args` docstring.

## Known Stubs

- `add_rover_args(parser)` body is intentionally empty (just adds `parser.add_argument_group("rover-ros2 (Phase 4)")`) — Phase 4 ROVER-05 fills it with `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id`. This is the plan-checker invariant for "rover-only flags exist but are empty in Phase 3" and is locked by the `test_rover_help_excludes_drone_flag` parametrize (rover help has 0 drone flags AND 0 rover-specific flags right now — only common flags).
- `run_robot()`'s rover branch raises `NotImplementedError("Rover adapter lands in Phase 4. ...")`. Phase 4 ROVER-04 replaces this with the friendly `RuntimeError("ROS 2 not sourced — run sudo apt install ros-humble-ros-base then re-source setup_env.sh")` path that detects whether `import rclpy` works and gives the operator the exact recovery command.

Both stubs are documented in the `run_robot` docstring and the inline comment, and surfaced here so the verifier and the Phase 4 author know where to land their changes.

## Self-Check: PASSED

- Created files exist:
  - `.planning/phases/03-abstraction/03-08-SUMMARY.md` — this file (in progress; will exist after Write completes)
- Commits exist:
  - `cafdcc1` (feat 03-08 two-pass argparse): present in `git log --oneline`
  - `e8e2ec1` (test 03-08 strip xfail): present in `git log --oneline`
- Verification:
  - `robot-follow --help` exits 0
  - `--robot drone --help` shows drone-only flags; `--robot rover --help` does not
  - `robot-follow --help` and `drone-follow --help` are byte-identical (diff is empty)
  - Full suite: 282 passed, 1 skipped, 2 xfailed, 1 xpassed, 0 unexpected failures
  - B1 carry-through: `mission_duration` + `FIRST_COMPLETED` + `getattr(args, "mission_duration", math.inf)` all present in `run_robot()` source
  - `grep -c xfail robot_follow/tests/test_cli_help_dispatch.py` → 0
