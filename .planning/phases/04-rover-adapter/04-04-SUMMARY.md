---
phase: 04-rover-adapter
plan: 04
subsystem: robot_follow_app / composition-root
tags: [phase-4, rover, composition-root, run_robot, smoke-test, ROVER-08, wave-3]
requires:
  - 04-02  # add_rover_args body — args.cmd_vel_topic / ros_namespace / ros_domain_id exist
  - 04-03  # Ros2RoverAdapter + ROVER_CAPS — the lazy import resolves to a real class
provides:
  - run_robot() rover branch wired to Ros2RoverAdapter (replaces NotImplementedError stub)
  - Lazy-import boundary that keeps drone path runnable on no-rclpy boxes
  - ROVER-04 friendly-error logging + clean return (no crash, pipeline continues)
  - Integration-layer ROVER-08 SIGINT preservation smoke (composition-root level)
affects:
  - robot_follow/robot_follow_app.py (run_robot() rover branch only)
  - robot_follow/tests/test_ros2_rover_adapter.py (+1 class, 2 PASS)
tech-stack:
  added: []
  patterns:
    - Lazy import inside dispatch branch (not module top) for optional-dep boundary
    - Narrow `except RuntimeError` for friendly-error catch (NOT widened to ConnectionError — preserves post-init SIGINT-clobber detection per plan-checker watch-list #8)
    - LOGGER.error + early return mirrors the drone path's outer try/except wrap
key-files:
  created: []
  modified:
    - robot_follow/robot_follow_app.py
    - robot_follow/tests/test_ros2_rover_adapter.py
key-decisions:
  - "Lazy import of Ros2RoverAdapter inside the rover dispatch branch — NOT at module top — so `import robot_follow.robot_follow_app` and `--robot drone --help` continue to work on a no-rclpy dev box"
  - "RuntimeError catch is narrow (not Exception, not RuntimeError|ConnectionError) — ConnectionError stays surfaced to the outer try/except wrap so post-init SIGINT-clobber detection is not masked"
  - "Phase 3 carry-throughs are mechanical: B1 mission_duration deadline wrap + 03-11 ui_state=ui_state kwarg preserved byte-identical in the rover path because run_robot's outer machinery is shared between drone + rover branches"
  - "Integration smoke does NOT invoke run_robot() directly via subprocess — that would require GStreamer / shared_state / pipeline-thread mocks far heavier than the contract under test. The lazy import + adapter construction are exercised by the test calling the same statements run_robot calls"
requirements-completed: [ROVER-08]
duration: ~95 min
completed: 2026-05-20
---

# Phase 4 Plan 04: Composition-Root Wiring Summary

Replaces the `NotImplementedError("Rover adapter lands in Phase 4…")` stub at `robot_follow/robot_follow_app.py` (formerly lines 537-543 in `main()`) with real `Ros2RoverAdapter(args, controller_config)` construction wrapped in a narrow `RuntimeError` catch. The import of `Ros2RoverAdapter` stays **lazy** — inside the rover dispatch branch, NOT at module top — so the drone path continues to work on this no-rclpy dev box. After this plan lands, `drone-follow --robot rover` on a real-ROS box constructs the adapter and publishes Twist on `/cmd_vel`; on a no-rclpy box it logs the ROVER-04 friendly message and exits `run_robot()` cleanly, with the pipeline continuing without robot control.

Net suite delta: **324 → 326 PASS** (+2, exactly as planned — both new `TestCompositionRootIntegration` tests PASS); 1 skipped (unchanged); 0 xfail across the rover adapter test file.

## Tasks executed (2)

| # | Type | Commit    | Files                                            |
| - | ---- | --------- | ------------------------------------------------ |
| 1 | feat | `f541440` | `robot_follow/robot_follow_app.py`               |
| 2 | test | `5d0b439` | `robot_follow/tests/test_ros2_rover_adapter.py`  |

## The replaced rover branch

```python
elif args.robot == "rover":
    # Phase 4 Plan 04-04: replace the NotImplementedError stub
    # from Phase 3 03-08 with real Ros2RoverAdapter construction.
    #
    # The import is LAZY (inside this branch, not at module top)
    # for two reasons:
    #   1. So `--robot drone` runs on a no-rclpy dev box continue
    #      to work without surfacing rover-only errors.
    #   2. So the ROVER-04 friendly RuntimeError (raised by
    #      Ros2RoverAdapter.__init__ when rclpy/geometry_msgs are
    #      missing) only fires when the user actually selects
    #      --robot rover. The catch below logs + returns so the
    #      outer pipeline continues without robot control —
    #      mirroring the drone branch's outer try/except at the
    #      bottom of run_robot.
    try:
        from robot_follow.robot_api.adapters.ros2_rover import (
            Ros2RoverAdapter,
        )
        adapter = Ros2RoverAdapter(args, controller_config)
    except RuntimeError as e:
        LOGGER.error(
            "[rover] adapter construction failed: %s\n"
            "Pipeline continues without robot control.",
            e,
        )
        return
```

The drone branch (`adapter = MavsdkDroneAdapter(args, controller_config)`), the `_main()` body, the `getattr(args, "mission_duration", math.inf)` deadline wrap, the `asyncio.FIRST_COMPLETED` race, the outer try/except wrap, and the `ui_state=ui_state` kwarg passed to `run_robot_loop` are all **byte-identical** to their pre-04-04 state. Verified by inspection (grep + line-by-line audit of `git diff HEAD~2 HEAD -- robot_follow/robot_follow_app.py`).

## Carry-through evidence

**B1 mission_duration deadline wrap (Phase 3 plan 03-07/03-08):**

```
robot_follow_app.py:568          duration = getattr(args, "mission_duration", math.inf)
robot_follow_app.py:578              return_when=asyncio.FIRST_COMPLETED,
```

For rover: `args.mission_duration` is not registered by `add_rover_args` (Plan 04-02), so `getattr` falls back to `math.inf` → rover has effectively no deadline. The FIRST_COMPLETED race still wraps `run_robot_loop` to support cancellation on EOS / shutdown.

**03-11 ui_state kwarg (gap closure):**

```
robot_follow_app.py:572              run_robot_loop(adapter, shared_state, controller_config, shutdown, ui_state=ui_state)
```

Rover adapter is UI-agnostic (axes-only contract); `run_robot_loop` itself is robot-agnostic and publishes `mode` + `velocity` to the web UI via `ui_state` regardless of which adapter is in use. No separate rover UI wiring needed.

**Carry-through gate (all 5 assertions passed):**

```
$ python -c "
import inspect
from robot_follow.robot_follow_app import main
src = inspect.getsource(main)
assert 'mission_duration' in src
assert 'FIRST_COMPLETED' in src
assert 'Ros2RoverAdapter' in src
assert 'NotImplementedError(' not in src
assert 'ui_state=ui_state' in src
print('Plan 04-04 carry-through gates: OK')
"
Plan 04-04 carry-through gates: OK
```

> **Note on the gate command:** the PLAN's verification snippet imports `run_robot` directly. `run_robot` is a closure defined inside `main()` (line 512 in the post-04-04 source), so the actual executed gate uses `inspect.getsource(main)` instead. All substring assertions still hold — `run_robot`'s entire body is contained inside `main`'s source — and this matches the source structure that has been stable since Phase 3 plan 03-08. No semantic deviation; tooling adjustment only.

## No-rclpy safety evidence

```
$ grep -cE '^from robot_follow.robot_api.adapters.ros2_rover' robot_follow/robot_follow_app.py
0

$ python -c "import robot_follow.robot_follow_app; print('module imports cleanly on no-rclpy box: OK')"
module imports cleanly on no-rclpy box: OK
```

The grep returns 0 — there is exactly ONE import of `Ros2RoverAdapter` in the file and it lives at column 17 inside the rover dispatch branch, not at column 0 at module top.

## --help paths (all exit 0)

```
$ robot-follow --help >/dev/null && echo "OK ($?)"                  → OK (0)
$ robot-follow --robot drone --help >/dev/null && echo "OK ($?)"    → OK (0)
$ robot-follow --robot rover --help >/dev/null && echo "OK ($?)"    → OK (0)
$ drone-follow --help >/dev/null && echo "OK ($?)"                  → OK (0)
```

The `drone-follow` alias (preserved permanently per v1.1 rename memory) continues to work identically to `robot-follow`.

## Test results

**Rover adapter file (22 tests, all PASS, 0 xfail):**

```
robot_follow/tests/test_ros2_rover_adapter.py
  TestImportSafety              2 PASS
  TestProtocolShape             3 PASS
  TestSignalHandlerPreservation 2 PASS  (adapter unit-level ROVER-08)
  TestTwistPublish              5 PASS
  TestLifecycle                 5 PASS
  TestCustomCliArgs             3 PASS
  TestCompositionRootIntegration 2 PASS  ← NEW in this plan (integration-level ROVER-08)
                              = 22 PASS, 0 XFAIL, 0 FAIL
```

**Full suite:**

```
Baseline (post-04-03): 324 passed, 1 skipped
Post-04-04:            326 passed, 1 skipped  (Δ = +2 PASS, exactly as planned)
```

Both `TestCompositionRootIntegration` tests are NOT xfail-marked — they PASS on first run (Plan 04-04 is the closer, not a scaffold).

## Architectural locks

| Lock                                                        | Status                                                                |
| ----------------------------------------------------------- | --------------------------------------------------------------------- |
| `robot_follow/robot_api/adapters/mavsdk_drone.py` untouched | ✓ `git diff --name-only HEAD~2 HEAD -- mavsdk_drone.py` returns empty |
| `robot_follow/robot_api/adapters/ros2_rover.py` untouched   | ✓ `git diff --name-only HEAD~2 HEAD -- ros2_rover.py` returns empty   |
| `follow_api/*` untouched (axes-only contract)               | ✓ no follow_api files modified                                        |
| `add_rover_args` body (Plan 04-02) untouched                | ✓ no edits to `add_rover_args`                                        |
| Lazy import of Ros2RoverAdapter                             | ✓ 0 hits for `^from robot_follow.robot_api.adapters.ros2_rover`       |
| Narrow `except RuntimeError` (not widened)                  | ✓ only one `except` line in rover branch — catches `RuntimeError` only|

## Deviations from Plan

**None — semantic.** Two minor tooling adaptations worth recording:

1. **Carry-through gate runs against `main()` instead of `run_robot`.** The PLAN's command `from robot_follow.robot_follow_app import run_robot` would have failed because `run_robot` is a closure inside `main()`. Substituting `inspect.getsource(main)` exercises exactly the same substring contract (run_robot's entire body lives inside main's source). All 5 substring assertions still pass.

2. **The plan refers to lines 509-515 for the stub location;** in the working tree the stub was at lines 537-543 (Phase 3 03-08 / 03-11 added some lines above `run_robot`). The replacement landed at the correct semantic location (the rover branch of the `args.robot == "rover"` elif) regardless of exact line numbers.

Neither deviation changes behavior. Recorded here for traceability per Rule 1-3 convention.

## Explicit choice: do NOT widen the RuntimeError catch

The catch is `except RuntimeError as e:` — narrow on purpose. **It is NOT widened to `except (RuntimeError, ConnectionError):`** because doing so would mask post-init `ConnectionError`s that the adapter intentionally raises when SIGINT-clobber is detected (the `assert signal.getsignal(SIGINT) is before` post-init guard inside `Ros2RoverAdapter.__init__` per Plan 04-03 contract). Plan-checker watch-list item #8 calls this out explicitly. ConnectionError continues to propagate to the outer try/except wrap at the bottom of `run_robot()` where it lands in the `LOGGER.warning("[robot] Control loop failed …")` branch — exactly where ROVER-08 detection signals belong.

## Deferred items

- **Real-rclpy SITL exercise** — Plan 04-05 territory (operator gate vs. autonomous verifier).
- **Phase 6 RINT-01..06** — rover-specific tuning (`max_yawspeed=90` in `ControllerConfig` defaults is deg/s, tuned for drone; rover uses rad/s so the default 90 rad/s is wildly unsafe). Phase 6 RINT-01 ships `configs/rover_simulation.json` with rad/s-appropriate tuning. Operators MUST use that config when running `--robot rover` against the sim or a real rover.
- **Phase 6 RINT-02** — bottom-edge slow-near-edge for rover (currently `on_target_lost` publishes all-zero Twist; drone's last-seen-side spin behavior is intentionally NOT mirrored for the rover).

## Self-Check: PASSED

- ✓ `robot_follow/robot_follow_app.py` exists and was modified in commit `f541440`
- ✓ `robot_follow/tests/test_ros2_rover_adapter.py` exists and was modified in commit `5d0b439`
- ✓ Commit `f541440` present in `git log`
- ✓ Commit `5d0b439` present in `git log`
- ✓ Both commits used explicit pathspec (only their declared file appears in `git show --stat`)
- ✓ `robot_follow/robot_api/adapters/mavsdk_drone.py` byte-identical across the plan
- ✓ `robot_follow/robot_api/adapters/ros2_rover.py` byte-identical across the plan
- ✓ Full suite 326 passed, 1 skipped, 0 xfail (rover adapter file); +2 PASS net vs baseline
- ✓ No top-level `from robot_follow.robot_api.adapters.ros2_rover` import
- ✓ Module imports cleanly on no-rclpy box
- ✓ All 4 `--help` paths (default / drone / rover / `drone-follow` alias) exit 0
