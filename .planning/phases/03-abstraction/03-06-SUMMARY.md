---
phase: 03-abstraction
plan: 06
subsystem: robot-abstraction
tags: [robot-protocol, mavsdk, adapter, pure-functions, ema, tdd]

# Dependency graph
requires:
  - phase: 03-abstraction-05
    provides: file-moved robot_api/adapters/mavsdk_drone.py + drone_api shim
  - phase: 03-abstraction-04
    provides: Robot @runtime_checkable Protocol
  - phase: 03-abstraction-03
    provides: RobotCommand, Capabilities, SafetyContext, Axis in follow_api/types.py
  - phase: 03-abstraction-01
    provides: xfail scaffold for test_mavsdk_drone_adapter.py
provides:
  - MavsdkDroneAdapter class implementing the Robot protocol (dormant — composition root unchanged)
  - DRONE_CAPS module-level constant (axes=FORWARD/YAW/ALTITUDE, yaw_unit="deg/s")
  - SmoothingState dataclass (per-axis EMA + slew-rate filter state)
  - Four R5 pure-function extracts at module level (_apply_altitude_p, _apply_retreat_from_tilt, _apply_smoothing, _compute_search_yawspeed)
  - 37 passing unit tests across 5 classes in test_mavsdk_drone_adapter.py
affects: [03-abstraction-07, 03-abstraction-08, 03-abstraction-09]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - "Robot-protocol adapter: 6 async methods + caps; pure functions co-located at module level for testability"
    - "Pure-function extraction discipline: behavior byte-equivalent to source-line counterparts (live_control_loop:509-524, controller._apply_frame_edge_safety, VelocityCommandAPI.send, search-direction branch)"
    - "Option-(b) lock for legacy interop: adapter constructs a throwaway VelocityCommandAPI inside start_session for _start_offboard's signature; filter state stays local to handshake, NOT shared with adapter's SmoothingState"
    - "Q6 lock: SafetyContext.target_lost short-circuits both send_command (adapter) and _apply_retreat_from_tilt (pure function); sentinel bbox values intentionally not inspected"
    - "Lazy asyncio.Event init in start_session (not __init__) so the adapter is constructable without a running event loop — required for unit tests"

key-files:
  created: []
  modified:
    - robot_follow/robot_api/adapters/mavsdk_drone.py
    - robot_follow/tests/test_mavsdk_drone_adapter.py
    - robot_follow/tests/test_robot_protocol_shape.py
    - robot_follow/tests/test_layout_smoke.py
    - robot_follow/tests/test_velocity_api_and_smoother.py

key-decisions:
  - "MavsdkDroneAdapter is INSTANTIABLE but DORMANT — composition root (robot_follow_app.py / run_live_drone / live_control_loop / VelocityCommandAPI) is UNCHANGED. 03-08 wires the adapter; 03-07 deletes the legacy path."
  - "Option (b) locked for _start_offboard integration: throwaway VelocityCommandAPI for the legacy helper signature, NOT a new send_raw method on the adapter and NOT an inline rewrite of _start_offboard. Minimal migration diff."
  - "DRONE_CAPS lives in robot_api/adapters/mavsdk_drone.py per Q8 lock — follow_api/types.py stays types-only; per-adapter constants live with their adapter."
  - "Pure functions live at MODULE LEVEL, not as @staticmethods of MavsdkDroneAdapter — easier to unit-test in isolation; promote to staticmethods only if a subclass needs to override."
  - "test_robot_protocol_shape.py::test_robot_protocol_has_six_methods_plus_caps was still xfailed despite 03-04 landing the protocol — opportunistically stripped here while editing the same file (both xfails in the file flip to passes in one commit)."
  - "test_velocity_api_and_smoother.py kept AS-IS (with new deprecation note) — its 21 tests still exercise the legacy VelocityCommandAPI production path that lives through 03-06; migration of those 21 cases happens in 03-07 atomically with VelocityCommandAPI deletion."

patterns-established:
  - "Adapter byte-equivalence pattern: pure-function extracts mirror their source-line counterparts in math, ordering, and clamp/branch logic — guarantees 03-07's atomic legacy-path deletion is a no-op for wire behavior."
  - "Direct-mutation test pattern for unreachable validate() states: ControllerConfig.__post_init__ enforces target_altitude ∈ [min_altitude, max_altitude], but the runtime UI live-edit path can move target outside that range — exercise these branches via post-construction direct assignment (config.target_altitude = ...)."

requirements-completed: [ABS-04, ABS-05, ABS-06]

# Metrics
duration: 105min
completed: 2026-05-19
---

# Phase 3 Plan 06: Wrap relocated MAVSDK code as MavsdkDroneAdapter implementing Robot protocol; extract 4 R5 pure functions; fill 37 tests

**MavsdkDroneAdapter implementing the @runtime_checkable Robot protocol + DRONE_CAPS + 4 pure-function extracts (altitude P, retreat-from-tilt, smoothing, search yawspeed) — all instantiable, all tested, but dormant: live_control_loop / VelocityCommandAPI continue to handle production drone control until 03-07's atomic swap.**

## Performance

- **Duration:** 105 min
- **Started:** 2026-05-19T10:56:15Z
- **Completed:** 2026-05-19T12:41:55Z
- **Tasks:** 3
- **Files modified:** 5

## Accomplishments

- MavsdkDroneAdapter class lands with all 6 Robot protocol methods + caps attribute; `isinstance(adapter, Robot)` returns True via @runtime_checkable
- Four pure-function R5 extracts live at module level — testable without instantiating the adapter or talking to MAVSDK
- DRONE_CAPS module-level constant (axes=FORWARD/YAW/ALTITUDE, yaw_unit="deg/s") declared in the adapter module per Q8 lock
- SmoothingState dataclass formalises the per-axis EMA + slew filter state previously held as instance attributes on VelocityCommandAPI
- 37 passing unit tests across 5 classes in test_mavsdk_drone_adapter.py (scaffold's 5 xfails stripped); test_robot_protocol_shape (2 tests) and test_layout_smoke (half) flip from xfail to pass
- Production drone control wire behavior is byte-unchanged — live_control_loop and VelocityCommandAPI co-exist; the new adapter is dormant code waiting for 03-08 to wire it in

## Task Commits

1. **Task 1: Extract 4 pure functions + DRONE_CAPS + SmoothingState** — `73cc682` (feat)
2. **Task 2: Add MavsdkDroneAdapter + strip xfails in protocol_shape + layout_smoke** — `029aff8` (feat)
3. **Task 3: Fill 37 tests in test_mavsdk_drone_adapter.py + deprecate-note legacy smoother tests** — `b9a5c65` (test)

## Files Created/Modified

- `robot_follow/robot_api/adapters/mavsdk_drone.py` — Added DRONE_CAPS constant, SmoothingState dataclass, 4 R5 pure functions (`_apply_altitude_p`, `_apply_retreat_from_tilt`, `_apply_smoothing`, `_compute_search_yawspeed`), and the MavsdkDroneAdapter class (6 Robot protocol methods + caps). VelocityCommandAPI, live_control_loop, run_live_drone, all helpers UNCHANGED.
- `robot_follow/tests/test_mavsdk_drone_adapter.py` — Replaced 5-test xfail scaffold with 37 real passing tests across TestApplyAltitudeP (8), TestApplyRetreatFromTilt (11), TestApplySmoothing (6), TestComputeSearchYawspeed (4), TestMavsdkDroneAdapterIntegration (8). Uses MockDrone / MockOffboard pattern to assert set_velocity_body calls without real MAVSDK.
- `robot_follow/tests/test_robot_protocol_shape.py` — Stripped both xfail markers (the 03-04 protocol-shape test was still xfailed too despite landing in 03-04); both tests now pass with explicit module-top imports replacing the lazy try/except ImportError pattern.
- `robot_follow/tests/test_layout_smoke.py` — Stripped xfail on test_robot_api_adapters_mavsdk_drone_imports (the import works now that MavsdkDroneAdapter exists); kept xfail on test_drone_api_module_is_gone (drone_api shim removal is 03-09).
- `robot_follow/tests/test_velocity_api_and_smoother.py` — Added top-of-file deprecation note pointing at TestApplySmoothing as the successor. All 21 existing tests stay green (VelocityCommandAPI is still in production).

## Decisions Made

- **Dormant adapter, unchanged composition root.** Per the migration commit-shape rationale in 03-RESEARCH § Wave 4 commit 5, the adapter lands as testable code but the production hot path is byte-unchanged. 03-07 will atomically swap live_control_loop OUT and the adapter IN as part of the controller-signature change.
- **Option (b) for `_start_offboard` integration (already locked in plan).** Adapter constructs a throwaway `VelocityCommandAPI(self._drone, self._config)` inside `start_session()` and passes it to the unchanged `_start_offboard(drone, vel_api, shutdown)` helper. The throwaway is discarded; smoothing state lives on `self._smoothing` and is NOT shared. Locked in plan must_haves; no executor decision required.
- **DRONE_CAPS lives in the adapter module, NOT in follow_api/types.py** (Q8 lock from CONTEXT). follow_api stays types-only; per-adapter constants belong with their adapter.
- **Pure functions are module-level free functions, NOT @staticmethods.** Easier to unit-test in isolation without instantiating the adapter; promote to staticmethods only if a future subclass needs per-class override behavior.
- **Two xfail strips in test_robot_protocol_shape.py instead of one.** The protocol-shape test was still xfailed after 03-04 landed the Robot protocol (presumably overlooked). Stripped opportunistically while editing the same file.
- **Lazy asyncio.Event in `start_session`, not `__init__`.** Constructing an asyncio.Event in `__init__` binds it to whatever loop is current at construction time (often None for unit tests), which then mismatches the loop the adapter actually runs on. Lazy-init in start_session keeps construction loop-agnostic and lets unit tests instantiate the adapter with `asyncio.run` per-call without loop-binding errors.

## Deviations from Plan

None — plan executed exactly as written. The two minor adjustments below are well within the plan's spirit:

1. The plan said "~30 tests"; the actual count is 37 (8 + 11 + 6 + 4 + 8). The additional cases cover edge behaviors (e.g., `test_send_command_when_drone_none_is_silent`, `test_isinstance_robot_protocol`, `test_returns_new_robot_command_does_not_mutate_input`) that fell naturally out of the test scaffolding. No scope creep — all extras stay within the must_have boundary "test_mavsdk_drone_adapter.py has ~30 real tests".

2. The plan estimated 46 cases in test_velocity_api_and_smoother.py; the actual count is 21. Confirmed in the on-disk file (`grep -c "def test_" test_velocity_api_and_smoother.py == 21`); the plan's 46 estimate was likely off-by-line-counting or copy-pasted from a different file. All 21 stay green — the actual outcome (tests stay green during the migration) is correct; only the count was inflated in the plan estimate.

## Issues Encountered

- **ControllerConfig.validate() forbids the at-floor-descending / at-ceiling-climbing test scenarios.** `__post_init__` enforces `min_altitude <= target_altitude <= max_altitude`, but the runtime UI live-edit path can mutate `target_altitude` outside that range. To exercise these branches, the tests construct a valid config first, then directly mutate `config.target_altitude = <out-of-range>` to bypass `__post_init__`. The pattern is documented inline so future readers don't think it's a test bug.
- **One small initialisation cleanup.** The first draft of `MavsdkDroneAdapter.__init__` had a placeholder `self._shutdown_event = asyncio.Event() if False else None  # lazy` that was an obvious code smell. Cleaned up to `self._shutdown_event: Optional[asyncio.Event] = None` with an explanatory comment, and the actual `asyncio.Event()` is constructed inside `start_session` (which runs under the right loop).

## User Setup Required

None — no external service configuration required.

## Next Phase Readiness

- **Ready for 03-07** (atomic controller-signature swap + VelocityCommandAPI deletion + 21-test migration). The adapter's `send_command` signature is `(cmd: RobotCommand, safety_ctx: SafetyContext) -> None`; the controller currently returns `VelocityCommand`. 03-07 will change `compute_velocity_command`'s return type and wire the adapter, then atomically delete VelocityCommandAPI and live_control_loop and migrate test_velocity_api_and_smoother.py into TestApplySmoothing.
- **Ready for 03-08** (composition-root wire-up in robot_follow_app.py). The adapter is fully instantiable today; 03-08 only needs to construct it in place of the current `run_live_drone` flow.

## Self-Check: PASSED

All claimed files exist on disk and all claimed task commits exist in git history:

- `robot_follow/robot_api/adapters/mavsdk_drone.py` — FOUND
- `robot_follow/tests/test_mavsdk_drone_adapter.py` — FOUND
- `robot_follow/tests/test_robot_protocol_shape.py` — FOUND
- `robot_follow/tests/test_layout_smoke.py` — FOUND
- `robot_follow/tests/test_velocity_api_and_smoother.py` — FOUND
- `.planning/phases/03-abstraction/03-06-SUMMARY.md` — FOUND
- Commit `73cc682` (Task 1: pure functions + DRONE_CAPS) — FOUND
- Commit `029aff8` (Task 2: MavsdkDroneAdapter + xfail strips) — FOUND
- Commit `b9a5c65` (Task 3: 37 tests) — FOUND

Suite verification: `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -q` → `215 passed, 1 skipped, 82 xfailed, 31 xpassed`. Baseline was 175 passed; net +40 (37 new in adapter test + 3 from xfail strips: 2 in test_robot_protocol_shape + 1 in test_layout_smoke). Zero failures.

`grep -c xfail robot_follow/tests/test_mavsdk_drone_adapter.py` → `0`.

`robot-follow --help` exits 0. `drone-follow --help` (alias) exits 0.

---
*Phase: 03-abstraction*
*Completed: 2026-05-19*
