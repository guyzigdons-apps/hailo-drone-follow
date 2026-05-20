---
phase: 06-sim-integration
plan: 03
subsystem: pipeline-adapter
tags: [phase-6, sim-integration, bytetracker, tracker, config-wiring, RINT-03, wave-2]

requires:
  - phase: 06-sim-integration
    plan: 01
    provides: ControllerConfig.bytetracker_{track_thresh,track_buffer,match_thresh,frame_rate} fields (drone-default byte-identical)
provides:
  - create_app() signature extended with controller_config kwarg (RINT-03 wiring)
  - Path A wiring: robot_follow_app.py passes ControllerConfig.from_args(pre_args) to create_app
  - test_bytetracker_config.py: 6 unit tests locking the RINT-03 contract
affects: [06-04, 06-06]

tech-stack:
  added: []
  patterns:
    - "Path A wiring: build ControllerConfig from pre-parsed args BEFORE create_app to seed tracker; keep full-args ControllerConfig as source of truth for rest of app"
    - "Lazy import pattern: from robot_follow.follow_api.config import ControllerConfig as _CC inside create_app body to avoid circular import"
    - "argparse duplicate registration: --config on pre-parser + full parser; both default=None, same dest — argparse tolerates silently"

key-files:
  created:
    - robot_follow/tests/test_bytetracker_config.py
  modified:
    - robot_follow/pipeline_adapter/hailo_drone_detection_manager.py
    - robot_follow/robot_follow_app.py

key-decisions:
  - "Path A wiring chosen over Path B: build ControllerConfig from pre_args BEFORE create_app; keep existing line-448 ControllerConfig.from_args(args) as full-app source of truth"
  - "Duplicate --config registration: pre.add_argument added without conflict_handler='resolve' — argparse tolerates duplicate registrations with same dest + default=None silently (verified at import)"
  - "6 tests in test_bytetracker_config.py vs plan's 4 — extra test for pre_parser_registers_config_flag (Pitfall A) and test_create_app_signature_has_controller_config_param (belt-and-suspenders)"

requirements-completed: [RINT-03]

duration: ~25min
completed: 2026-05-20
---

# Phase 6 Plan 03: ByteTracker Config Wiring (RINT-03)

**RINT-03 atomic refactor: ByteTracker construction reads from controller_config.bytetracker_* fields instead of hardcoded literals; drone defaults byte-identical; rover override path wired via Path A composition root**

## Performance

- **Duration:** ~25 min
- **Completed:** 2026-05-20
- **Tasks:** 3 / 3
- **Files modified:** 2 modified, 1 new test file

## Accomplishments

- **RINT-03 shipped:** `create_app()` in `hailo_drone_detection_manager.py` reads ByteTracker knobs from `controller_config.bytetracker_*` instead of the legacy hardcoded literals `(0.4, 90, 0.5, 30)`.
- **Path A wiring:** `robot_follow_app.py` pre-parser registers `--config` (Pitfall A guard); `create_app` invocation passes `controller_config=ControllerConfig.from_args(pre_args)`; existing line-448 `ControllerConfig.from_args(args)` call unchanged.
- **6 unit tests** in `test_bytetracker_config.py` lock the RINT-03 contract: drone defaults, rover override, source-grep regression guard, Path A wiring, Pitfall A, signature inspection.
- **Architectural locks held:** `mavsdk_drone.py`, `ros2_rover.py`, `controller.py`, `types.py`, `config.py`, `sim/rover/`, `sim/bridge/video_bridge.py` all byte-identical across all 3 commits.

## Task Commits

1. **Task 1: Extend create_app signature + replace hardcoded literals** — `d82afe0` (refactor)
2. **Task 2: Wire Path A controller_config into create_app + --config on pre-parser** — `398b61a` (feat)
3. **Task 3: test_bytetracker_config.py with 6 RINT-03 wiring tests** — `354f704` (test)

## Files Created/Modified

- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` — `create_app` signature extended with `controller_config: "Optional[ControllerConfig]" = None`; normalization line `_cfg = controller_config if controller_config is not None else _CC()`; `create_tracker` call reads from `_cfg.bytetracker_*`; legacy literals removed from call site; docstring updated.
- `robot_follow/robot_follow_app.py` — `pre.add_argument("--config", type=str, default=None)` added to pre-parser; `create_app` invocation gains `controller_config=ControllerConfig.from_args(pre_args)` as LAST kwarg; existing `controller_config = ControllerConfig.from_args(args)` at line 448 preserved unchanged.
- `robot_follow/tests/test_bytetracker_config.py` (NEW) — 6 tests covering the full RINT-03 contract.

## Key Implementation Details

### New create_app signature line

```python
def create_app(shared_state, target_state=None, eos_reached=None, ui_state=None, ui_fps=10,
               parser: Optional[argparse.ArgumentParser] = None,
               record_enabled=False, record_dir=None, reid_manager=None,
               reid_search_timeout: float = 20.0,
               tracker_name=None, log_perf=False,
               controller_config: "Optional[ControllerConfig]" = None):
```

### Normalization + read-from-config lines (inside create_app body, before create_tracker)

```python
from robot_follow.follow_api.config import ControllerConfig as _CC
_cfg = controller_config if controller_config is not None else _CC()
_tracker_name = tracker_name or "byte"
_t0 = time.monotonic()
_inner_tracker = create_tracker(
    _tracker_name,
    track_thresh=_cfg.bytetracker_track_thresh,
    track_buffer=_cfg.bytetracker_track_buffer,
    match_thresh=_cfg.bytetracker_match_thresh,
    frame_rate=_cfg.bytetracker_frame_rate,
)
```

### Legacy literal removal verification

```
$ grep -nE 'track_thresh=0\.4, track_buffer=90, match_thresh=0\.5, frame_rate=30' \
    robot_follow/pipeline_adapter/hailo_drone_detection_manager.py | grep -v '^[[:space:]]*#'
(no output — empty)
```

### Composition root snippet (robot_follow_app.py)

```python
# --config is registered here on the pre-parser so ControllerConfig.from_args(pre_args)
# honors --config at the Path A wiring point (RINT-03 / RESEARCH Pitfall A guard).
pre.add_argument("--config", type=str, default=None)
...
# RINT-03 (Path A wiring): first-pass ControllerConfig built from pre-parsed args,
# passed into create_app for ByteTracker init.
app = create_app(shared_state, target_state=target_state, eos_reached=eos_reached,
                 ui_state=ui_state, ui_fps=pre_args.webui_fps, parser=parser,
                 record_enabled=record_branch_enabled, record_dir=recordings_dir,
                 reid_manager=reid_manager,
                 reid_search_timeout=pre_args.reid_timeout,
                 tracker_name=pre_args.tracker,
                 log_perf=pre_args.log_perf,
                 controller_config=ControllerConfig.from_args(pre_args))
```

### argparse duplicate --config registration

No `conflict_handler='resolve'` needed. argparse tolerates duplicate `--config` registrations when both have `default=None` and same dest. Verified at import: `import robot_follow.robot_follow_app` raises no errors; `_build_parser()` builds cleanly.

## Deviations from Plan

### Plan documented 4 tests; implementation ships 6

The plan's `<behavior>` section mentioned 4 tests; the final file has 6:
- `test_create_app_does_not_use_hardcoded_bytetracker_literals` (plan test 3)
- `test_robot_follow_app_passes_controller_config_to_create_app` (plan test 4)
- `test_pre_parser_registers_config_flag` (added: Pitfall A guard, explicitly called out in plan's threat model T-06-03-02 but not counted in the 4-test list)
- `test_create_app_uses_default_bytetracker_when_controller_config_none` (plan test 1)
- `test_create_app_uses_rover_overridden_track_buffer` (plan test 2)
- `test_create_app_signature_has_controller_config_param` (added: belt-and-suspenders signature check)

All 6 pass; no regressions. The plan's `<verification>` section notes "the exact count is whatever this task produces; what matters is full coverage of the contract."

## Verification Evidence

```
$ python -m pytest robot_follow/tests/test_bytetracker_config.py -v
test_create_app_does_not_use_hardcoded_bytetracker_literals PASSED
test_robot_follow_app_passes_controller_config_to_create_app PASSED
test_pre_parser_registers_config_flag                        PASSED
test_create_app_uses_default_bytetracker_when_controller_config_none PASSED
test_create_app_uses_rover_overridden_track_buffer           PASSED
test_create_app_signature_has_controller_config_param        PASSED
============================= 6 passed in 0.32s ================================

$ python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q
2 failed (pre-existing: rclpy-missing + ROS distro), 346 passed in 24.15s
(+6 net new PASS vs prior plan state — all from test_bytetracker_config.py)
```

Architectural locks (empty diff across all 3 commits):
```
$ git diff --name-only HEAD~3 HEAD -- robot_follow/robot_api/ robot_follow/follow_api/ \
    sim/rover/ sim/bridge/video_bridge.py
(no output — empty)
```

Drone-not-regressed evidence across Phase 6 (all plans to date):
```
$ git diff --name-only 6be7c04 HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py
(no output — byte-identical)
```

## Known Stubs

None — all wiring is functional.

## Threat Surface Scan

No new network endpoints, auth paths, file access patterns, or schema changes at trust boundaries introduced by this plan. The changes are confined to constructor wiring and test assertions.

## Next Phase Readiness

- **Plan 06-04 (RINT-02):** Rover adapter is ready to read `SafetyContext.bbox_bottom_norm` (added in 06-01) — that plan will wire the rover `send_command` to threshold at 0.85.
- **Plan 06-06 (RINT-04):** RINT-04 E2E test will exercise the full Path A flow with `--config configs/rover_simulation.json` — drive the tracker with `track_buffer=30` and verify the rover sim path end-to-end.

---
*Phase: 06-sim-integration*
*Completed: 2026-05-20*
