---
phase: 06-sim-integration
plan: 01
subsystem: config
tags: [phase-6, rover, config, bytetracker, safety-context, RINT-01, RINT-02, RINT-03, wave-1]

requires:
  - phase: 04-rover-adapter
    provides: SafetyContext + ControllerConfig + Ros2RoverAdapter — the data-shape this plan extended
provides:
  - configs/rover_simulation.json (rover-safe defaults, 18 keys, RINT-01)
  - ControllerConfig.bytetracker_{track_thresh,track_buffer,match_thresh,frame_rate} (RINT-03 prep — read site in 06-03)
  - SafetyContext.bbox_bottom_norm: Optional[float] (RINT-02 prep — read site in 06-04)
affects: [06-03, 06-04, 06-05, 06-06]

tech-stack:
  added: []
  patterns:
    - "Non-breaking dataclass extension: new optional field with default sits at end of field block; existing positional callers unaffected"
    - "Drone-default byte-identity lock: bytetracker_* defaults BYTE-IDENTICAL to the legacy hardcoded call at hailo_drone_detection_manager.py:1281 so Plan 06-03's wiring change is a no-op for the drone path"

key-files:
  created:
    - configs/rover_simulation.json
  modified:
    - robot_follow/follow_api/config.py
    - robot_follow/follow_api/types.py
    - robot_follow/tests/test_config_persistence.py

key-decisions:
  - "bytetracker_* fields NOT added to tunable_fields() — tracker is constructed once at pipeline init; live web-UI mutation would silently no-op (RESEARCH A3)"
  - "bytetracker_* fields NOT added to add_args() — operators tune via JSON config; 4 CLI flags would clutter --help"
  - "SafetyContext.bbox_bottom_norm is a NEW field (Optional[float] = None), not a rename of bbox_bottom_normalized — both coexist; drone path keeps reading bbox_bottom_normalized"
  - "rover JSON OMITS altitude knobs (target_altitude/min/max, kp_alt_hold, max_climb/down_speed) — fall through to dataclass defaults which are self-consistent (2.0 ≤ 3.0 ≤ 4.0); validate(caps=None) passes"

patterns-established:
  - "Pure-data foundation pattern: when an upcoming refactor needs new config fields, land them with drone-default-byte-identical values FIRST so the wiring change becomes a behavior-preserving substitution"

requirements-completed: [RINT-01]

duration: ~60min (across multiple sessions)
completed: 2026-05-20
---

# Phase 6 Plan 01: Sim-Integration Config Foundations

**configs/rover_simulation.json (18 keys) + ControllerConfig.bytetracker_* fields (drone-default byte-identical) + SafetyContext.bbox_bottom_norm: Optional[float] — the pure-data foundations for Wave 2's behavioral changes**

## Performance

- **Duration:** ~60 min (spread across 2 sessions; current session closed out task 3)
- **Completed:** 2026-05-20
- **Tasks:** 3 / 3
- **Files modified:** 4 (1 new JSON, 2 .py extended, 1 test file extended)

## Accomplishments

- **RINT-01 shipped:** `configs/rover_simulation.json` at repo root with 18 keys (yaw_only=false, kp_yaw=3.0, max_forward=1.0, max_forward_accel=0, bytetracker_track_buffer=30, etc.). Loads cleanly via `ControllerConfig.from_json` and produces rover-safe values; altitude knobs fall through to defaults so `validate(caps=None)` passes.
- **RINT-03 prep:** `ControllerConfig` gained 4 `bytetracker_*` fields with drone-default values byte-identical to the legacy hardcoded call at `hailo_drone_detection_manager.py:1281` (`track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30`). Plan 06-03 wires the read site.
- **RINT-02 prep:** `SafetyContext` gained `bbox_bottom_norm: Optional[float] = None`. Populated in `from_detection` from `cy + bh/2`; left None in `lost()` per Q6 lock. Existing `bbox_bottom_normalized` field is untouched (drone path keeps reading it). Plan 06-04 wires the rover-adapter read + threshold logic.
- **Architectural locks held:** `mavsdk_drone.py`, `ros2_rover.py`, `controller.py`, `Capabilities`, `sim/rover/`, `sim/bridge/video_bridge.py`, `sim/configs/` all byte-identical to pre-plan HEAD.

## Task Commits

1. **Task 1: ControllerConfig + 4 bytetracker_* fields + drone-default test** — `6be7c04` (feat)
2. **Task 2: SafetyContext.bbox_bottom_norm + from_detection/lost tests** — `46e7801` (feat)
3. **Task 3: configs/rover_simulation.json + round-trip load test** — `41292a5` (feat)

## Files Created/Modified

- `configs/rover_simulation.json` (NEW) — 18 keys; rover-safe ControllerConfig overrides + 4 bytetracker_* keys with track_buffer=30 (1 s @ 30 fps)
- `robot_follow/follow_api/config.py` — 4 new bytetracker_* fields appended after `log_verbosity`; drone-default byte-identical to legacy hardcoded call
- `robot_follow/follow_api/types.py` — `SafetyContext.bbox_bottom_norm: Optional[float] = None`; populated in `from_detection`; left None in `lost()`
- `robot_follow/tests/test_config_persistence.py` — 4 new tests covering all three additions

## Decisions Made

None beyond what the plan locked. All locked decisions (`bytetracker_*` NOT in tunable_fields/add_args; `bbox_bottom_norm` as NEW field not rename; rover JSON omits altitude knobs) preserved.

## Deviations from Plan

None — plan executed exactly as written across two sessions. Task 3 was paused mid-execution (file written + test added but uncommitted); orchestrator resumed via `safe_resume_gate` and committed task 3 atomically.

## Issues Encountered

- **Pre-existing test failure (unrelated):** `test_friendly_error_when_rclpy_missing` in `test_ros2_rover_adapter.py` fails on this dev machine because `rclpy` IS installed (the test expects `RuntimeError` when missing). Not introduced by this plan — verified by stashing 06-01's changes and re-running. To investigate in Phase 6 verification (not a 06-01 regression).

## Verification Evidence

```
$ python -m pytest robot_follow/tests/test_config_persistence.py -v -k "bytetracker or bbox_bottom_norm or rover_simulation"
test_bytetracker_defaults_match_drone_hardcoded                     PASSED
test_rover_simulation_json_loads_with_rover_safe_defaults           PASSED
test_safety_context_from_detection_populates_bbox_bottom_norm       PASSED
test_safety_context_lost_has_bbox_bottom_norm_none                  PASSED
======================= 4 passed, 17 deselected in 0.28s =======================

$ python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q
1 failed (pre-existing rclpy-missing test, unrelated), 340 passed in 23.60s

$ grep -nE 'bytetracker_' robot_follow/follow_api/config.py | head
120:    bytetracker_track_thresh: float = 0.4
121:    bytetracker_track_buffer: int = 90
122:    bytetracker_match_thresh: float = 0.5
123:    bytetracker_frame_rate: int = 30

$ grep -nE 'bbox_bottom_norm\b' robot_follow/follow_api/types.py
115:    bbox_bottom_norm: Optional[float] = None
125:            bbox_bottom_norm=det.center_y + det.bbox_height / 2,
```

Architectural locks: empty diff across all 3 commits for `mavsdk_drone.py`, `ros2_rover.py`, `controller.py`, `sim/rover/`, `sim/bridge/video_bridge.py`, `sim/configs/`.

## Next Phase Readiness

- **Plan 06-03 (RINT-03):** `controller_config.bytetracker_{track_thresh,track_buffer,match_thresh,frame_rate}` ready to read; wire `create_app`'s signature next.
- **Plan 06-04 (RINT-02):** `SafetyContext.bbox_bottom_norm` is populated by `from_detection`; rover adapter can branch on `safety_ctx.bbox_bottom_norm >= 0.85` to zero `twist.linear.x` next.
- **No blockers** for Wave 2.

---
*Phase: 06-sim-integration*
*Completed: 2026-05-20*
