---
phase: 03-abstraction
plan: 11
subsystem: orchestrator, ui-telemetry, hailo-detection-manager
tags: [ui-telemetry, orchestrator, gap-closure, axes-only, F1, F2, ABS-11]
requirements: [ABS-11]
dependency_graph:
  requires: [03-09]
  provides: [orchestrator ui_state publish, pre-smoothing pin, F2 demotion]
  affects: [robot_follow_app.py, orchestrator.py, hailo_drone_detection_manager.py]
tech_stack:
  added: []
  patterns: [duck-typed ui_state protocol, SpyUIState test pattern, FakeRobot stub]
key_files:
  created:
    - robot_follow/tests/test_orchestrator_ui_update.py
  modified:
    - robot_follow/robot_api/orchestrator.py
    - robot_follow/robot_follow_app.py
    - robot_follow/pipeline_adapter/hailo_drone_detection_manager.py
decisions:
  - Pre-smoothing cmd published from orchestrator (not adapter) — axes-only contract
  - Mode strings robot-agnostic: AUTO/LOCKED/LOST/IDLE
  - LOCKED/AUTO distinction deferred (FollowTargetState not wired — see below)
metrics:
  duration: ~15min
  completed: "2026-05-19"
  tasks_completed: 3
  tasks_total: 3
  files_created: 1
  files_modified: 3
---

# Phase 3 Plan 11: Orchestrator UI Telemetry + F2 Log Demotion Summary

ABS-11 gap closure: wire `ui_state.update_velocity(forward, down, yawspeed, mode)` into `run_robot_loop` at the orchestrator layer, restoring the operator velocity status bar broken by 03-07's deletion of `live_control_loop`. Demote the per-tick INFO log spam to DEBUG (F2).

## What Was Built

### F1 Fix — Orchestrator UI publish (orchestrator.py)

`run_robot_loop` gained an optional 5th parameter `ui_state: Optional[Any] = None`. After each tick branch, if `ui_state is not None`, it calls `ui_state.update_velocity(forward, down, yawspeed, mode)`:

| Branch | Mode published | Values |
|---|---|---|
| Detection present | `"AUTO"` | `(cmd.forward_m_s, cmd.down_m_s, cmd.yaw_rate)` |
| Hold (within search_enter_delay_s) | `"IDLE"` | `(0.0, 0.0, 0.0)` |
| Lost (past delay) | `"LOST"` | `(0.0, 0.0, 0.0)` |
| Shutdown finally | `"IDLE"` | `(0.0, 0.0, 0.0)` |

### F2 Fix — Log demotion (hailo_drone_detection_manager.py)

`LOGGER.info("ctrl: bh=%.3f ...")` at line ~533 demoted to `LOGGER.debug(...)`. Single token change; format string and arguments preserved verbatim. Eliminates ~10 Hz INFO log noise during active follow.

### Composition root wiring (robot_follow_app.py)

`asyncio.create_task(run_robot_loop(adapter, shared_state, controller_config, shutdown))` extended to `run_robot_loop(adapter, shared_state, controller_config, shutdown, ui_state=ui_state)`. `ui_state` is already in scope at this call site (constructed at line 377 in the enclosing `run_robot()` closure).

### New tests (test_orchestrator_ui_update.py)

5 tests — all passing in 0.22s wall-clock:

1. `test_publishes_auto_mode_with_non_zero_velocity_on_detection` — detection present → AUTO + non-zero yaw
2. `test_publishes_lost_mode_after_search_enter_delay` — no detection past delay → LOST + zeros
3. `test_publishes_idle_on_shutdown` — LAST spy call is `(0.0, 0.0, 0.0, "IDLE")`
4. `test_no_publish_when_ui_state_is_none` — no AttributeError when ui_state=None
5. `test_publish_uses_pre_smoothing_cmd` — spy forward == controller.compute forward (regression guard)

## Key Decisions

### Pre-smoothing cmd (architectural decision — pinned by test 5)

Published values reflect the controller's PRE-smoothing `cmd`, not the drone adapter's post-smoothed output. Rationale:

1. **Axes-only Capabilities contract** — `_apply_smoothing` is a drone-specific EMA + slew cap inside `MavsdkDroneAdapter`. Pushing `ui_state` into the adapter to publish post-smoothed values would violate the axes-only contract (see `feedback_robot_abstraction_axes_only.md`).
2. **Undefined for rover** — the future `Ros2RoverAdapter` has no smoother; post-smoothing values would be undefined at that layer.
3. **Pre-Phase-3 parity** — `live_control_loop` (deleted in 03-07) published pre-smoothing values. This restores the operator's existing mental model.
4. **Test 5 is the regression guard** — if a future refactor accidentally moves the publish call inside the adapter (post-smoothing), `test_publish_uses_pre_smoothing_cmd` will fail because smoothed ≠ raw controller output.

### LOCKED/AUTO distinction (deferred, documented)

`run_robot_loop` publishes `"AUTO"` unconditionally in the detection-present branch. The pre-Phase-3 `live_control_loop` distinguished LOCKED from AUTO via `FollowTargetState`. Wiring `FollowTargetState` through to `run_robot_loop` would require a 6th parameter and was judged out of scope for this gap closure. The mode string `"LOCKED"` is reserved in the allowed vocabulary (`AUTO / LOCKED / LOST / IDLE`). A future plan (post-03-12) can add `target_state: Optional[FollowTargetState] = None` and emit `"LOCKED"` when `target_state.is_explicit_lock()` is True.

### Mode strings are robot-agnostic

The four allowed values (`AUTO`, `LOCKED`, `LOST`, `IDLE`) carry no drone-specific semantics. Phase-4's `Ros2RoverAdapter` will use the same strings from the same orchestrator without any adapter-side changes.

### MavsdkDroneAdapter untouched (axes-only contract verified)

```
git diff --stat 3bedcf9..HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py
(empty — zero lines changed)
```

`MavsdkDroneAdapter.__init__` contains no `ui_state` parameter. Zero references to `SharedUIState` or `update_velocity` anywhere in `mavsdk_drone.py`.

## Test Suite Results

| Metric | Before 03-11 | After 03-11 |
|---|---|---|
| Tests passing | 275 | 280 |
| Tests skipped | 7 | 7 |
| New tests added | 0 | 5 |
| Wall-clock (new tests) | — | 0.22s |

Pre-existing failure: `test_install_smoke.py::test_robot_follow_console_script_on_path` — requires `source setup_env.sh` to put `robot-follow` on PATH; fails in CI without the venv activation. Pre-existing; not introduced by this plan.

## F2 Demotion Confirmation

```
grep -c 'LOGGER.info.*ctrl: bh' hailo_drone_detection_manager.py  → 0 (multi-line call; info call gone)
grep 'LOGGER.debug' + context shows "ctrl: bh=%.3f" on the next line  → confirmed
```

The `LOGGER.debug(` call at line 533 followed by `"ctrl: bh=%.3f ..."` on line 534 is the demoted form.

## Commits

| # | Hash | Message |
|---|---|---|
| 1 | `47f796c` | `fix(03-11): publish (fwd,down,yaw,mode) from run_robot_loop + demote F2 log` |
| 2 | `2e4c9d7` | `test(03-11): pin orchestrator ui_state publish behavior (FakeRobot + spy)` |
| 3 | `abba4cf` | `fix(03-11): wire ui_state into run_robot_loop call site (composition root)` |

## Deviations from Plan

None. Plan executed exactly as written. LOCKED/AUTO distinction was documented as deferred in the plan itself — treated as expected behavior, not a deviation.

## Recommended Next Step

**Plan 03-12: Operator SITL re-run (ABS-11 gate)**

The 03-10 gate failure documented the F1 regression. This plan (03-11) closes F1+F2 in code. The operator must re-run the SITL gate to confirm:
1. Web UI status bar updates at ≥5 Hz during active follow (non-zero velocity values visible)
2. Mode string transitions correctly (AUTO during follow, IDLE on hold, LOST after delay, IDLE on shutdown)
3. INFO log spam eliminated (no more `ctrl: bh=...` flooding the console)

Phase 3 does NOT close until the operator approves plan 03-12's SITL re-run.

## Self-Check

- [x] orchestrator.py modified — `run_robot_loop` has `ui_state` param + 4 publish call sites
- [x] hailo_drone_detection_manager.py modified — `LOGGER.debug` at line 533
- [x] test_orchestrator_ui_update.py created — 5 tests pass
- [x] robot_follow_app.py modified — `ui_state=ui_state` at call site
- [x] 3 task commits with explicit pathspec
- [x] MavsdkDroneAdapter unchanged
- [x] 280 tests pass (pre-existing PATH smoke test excluded from count)
