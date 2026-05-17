# Phase 3: Abstraction - Research

**Researched:** 2026-05-17
**HEAD verified:** `9ba4ec7` (`feature/rover-support`)
**Domain:** Actuator-boundary refactor — introduce `Robot` protocol, axes-only `Capabilities`, `MavsdkDroneAdapter`, shared orchestrator, two-pass argparse with `--robot`, conditional ROS sourcing in `setup_env.sh`.
**Confidence:** HIGH (every claim below is grounded in a verified file:line reference on the current tree; no speculative library research required).

---

## Summary

Phase 3 is a structural refactor of one slim seam (the boundary between `follow_api/controller.py` and `drone_api/mavsdk_drone.py`) into the four-layer architecture described in `03-DESIGN-NOTES.md`: `follow_api` (pure types + controller) → `robot_api/robot.py` (protocol) → `robot_api/adapters/mavsdk_drone.py` (drone behaviors) → MAVSDK. The orchestrator (`robot_api/orchestrator.py`) becomes the shared per-tick driver; the controller becomes axes-aware (only emits `down_m_s` when `Axis.ALTITUDE` is in `caps.axes`); the drone adapter absorbs all drone-specific behaviors (altitude P, retreat-from-tilt overlay, smoothing/clamping, telemetry, offboard handshake, yaw-spin search).

Drone behavior at the MAVSDK wire must be byte-identical pre- and post-refactor — that's the whole gate. The snapshot test (`test_robot_command_snapshot.py`) is the CI guard; the operator SITL run is the wire-behavior guard. R5's pure-function extracts (`_apply_altitude_p`, `_apply_retreat_from_tilt`, `_apply_smoothing`, `_compute_search_yawspeed`) are the adapter's standalone safety net for future refactors.

**Primary recommendation:** Plan this as 6–8 small bisectable commits keeping the existing 176-test suite green at every intermediate state. The snapshot test is the ONLY new automated gate; everything else is a relocation of code that already has tests. Land the controller signature change + test fixups (~5 sites, much smaller than CLEAN-07's 11) in one atomic commit. Do NOT try to land the whole phase as one mega-commit — bisect-survival demands the wave shape.

---

## User Constraints (from CONTEXT.md)

### Locked Decisions

**Types live in `follow_api/types.py`** (R1 — adversarial-review reversal):
- `Detection` (already there; unchanged)
- `Axis` enum: `FORWARD`, `YAW`, `ALTITUDE` — body-frame velocity axes only. Drone-shaped; rename `Robot` → `BodyVelocityRobot` in v1.2 if a non-velocity actuator surfaces.
- `Capabilities = {axes: frozenset[Axis], yaw_unit: Literal["deg/s", "rad/s"]}`. **Axes-only — no behavioral flags.** Launch-time fixed (no runtime degradation path).
- `RobotCommand(forward_m_s=0.0, yaw_rate=0.0, down_m_s=0.0)`.
- `SafetyContext(bbox_bottom_normalized, bbox_size_normalized, target_lost, last_target_x)` — controller derives once from `Detection`; adapter never reads `Detection` directly. (R4).

In `robot_api/robot.py` (types-free except for protocol):
- `Robot` protocol: `connect`, `start_session`, `send_command`, `send_zero`, `on_target_lost`, `shutdown`. Plus `caps: Capabilities` attribute.
- **`send_zero()` and `on_target_lost(last_detection)` are SPLIT** (R2). `send_zero()` is shutdown/quiescent (no arg); `on_target_lost(last_detection)` is the per-tick search behavior.

All robot-specific behaviors live **inside the adapter** (retreat-from-tilt, slow-near-edge, takeoff/land, yaw-spin, offboard handshake). The controller stays robot-agnostic.

**Orchestrator state machine** (R3 — explicit pseudocode in `03-CONTEXT.md` lines 65–103): fixed 10 Hz tick, hold-velocity for the first `search_enter_delay_s` after target-lost, then `on_target_lost`. `send_zero()` runs once in `finally` on shutdown only.

**`send_command(cmd, safety_ctx)`** — adapter receives `SafetyContext`, not `Detection`. (R4).

**Adapter unit-test plan (R5)** — pure-function extracts (`_apply_altitude_p`, `_apply_retreat_from_tilt`, `_apply_smoothing`, `_compute_search_yawspeed`) tested in isolation via `test_mavsdk_drone_adapter.py`.

**Snapshot test is a Phase-3-only artifact** — archive after verifier passes; replace with property-based invariants.

**CLI two-pass argparse** — pre-parser parses `--robot` first; `add_common_args` / `add_drone_args` / `add_rover_args` register robot-specific flags. Rover users see no drone flags in `--help`.

**`setup_env.sh`** — always source `/opt/ros/humble/setup.bash` if the file exists. Sourcing order: venv FIRST, then ROS. No `DF_SKIP_ROS=1` escape hatch.

**Connection failure** — `adapter.connect()` raises on hard failure; orchestrator unwinds via `try/finally`. No retry in v1.1.

### Claude's Discretion

- `Robot.send_zero` exact docstring + whether `last_detection` is keyword or positional.
- Internal helper function names inside `MavsdkDroneAdapter` (e.g., `_translate_to_mavsdk`, `_apply_altitude_p`, `_apply_retreat_from_tilt`).
- Exact wording of the `RuntimeError` for missing ROS.
- File layout inside `robot_api/`: `robot.py` (protocol + types) + `adapters/__init__.py` + `adapters/mavsdk_drone.py` + `orchestrator.py`. Planner can refine.
- Whether `test_robot_command_snapshot.py` uses pytest parametrize or a single test iterating over cases. Either works.
- Whether the snapshot fixture is `.json` (data-only) or `.py` (Python module with named constants). Slight preference for `.py` for type hinting.

### Deferred Ideas (OUT OF SCOPE)

- Hot-reload `Capabilities` — robot type is fixed at launch via `--robot`. Web UI must NOT expose a robot-type selector. Future v1.2+.
- ArduRover via MAVSDK alternative (HW-09).
- `Robot.reconnect()` semantics for MAVSDK disconnections — current behavior is "log + continue".
- Property-based testing of controller invariants (Hypothesis) — snapshot covers regression; property-based is deferred enrichment.
- `Capabilities` discovery from the wire (e.g., probe MAVSDK to confirm altitude axis available) — v1.1 keeps capabilities static per adapter class.

---

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| ABS-01 | `robot_api/robot.py` defines `Axis` (in `follow_api/types.py` per R1) + `Capabilities` (mechanical only) + `Robot` protocol with 6 methods + `caps` attribute | Concern-mapping + adapter scaffolding sections below |
| ABS-02 | `RobotCommand(forward_m_s, yaw_rate, down_m_s)` replaces `VelocityCommand`; controller and adapter both honor `caps.axes` | Controller signature migration + adapter-method-mapping below |
| ABS-03 | `drone_api/mavsdk_drone.py` moved to `robot_api/adapters/mavsdk_drone.py`; behavior wrapped behind `MavsdkDroneAdapter.start_session()` | Concern-mapping table; migration commit shape |
| ABS-04 | Altitude-hold P-loop gates on `Axis.ALTITUDE in caps.axes` (rover never sees `down_m_s`) | Altitude-P extract in pure-function section |
| ABS-05 | Controller emits `forward_m_s=0` on bbox-bottom edge; per-robot reactions live in adapter | Retreat-from-tilt extract in pure-function section |
| ABS-06 | Controller emits zero/hold on target-lost; yaw-spin search lives in adapter | `_compute_search_yawspeed` extract + `on_target_lost` |
| ABS-07 | `ControllerConfig` altitude fields become `Optional[float]`; `validate()` skips altitude checks when `Axis.ALTITUDE` absent | Migration commit shape + open question 3 |
| ABS-08 | `run_drone()` renamed to `run_robot()`; dispatches on `--robot` flag | Argparse splits + migration commit shape |
| ABS-09 | `--robot drone\|rover` CLI with two-pass argparse — rover users see no drone-only flags | Argparse splits section |
| ABS-10 | `setup_env.sh` auto-sources `/opt/ros/humble/setup.bash` if present; venv-first ordering | `setup_env.sh` ROS-source block section |
| ABS-11 | Full SITL drone follow-the-person test passes with `--robot drone` (or default) | Validation Architecture section — snapshot test (auto) + operator SITL (manual gate) |

---

## Concern-mapping table for `live_control_loop`

Source: `robot_follow/drone_api/mavsdk_drone.py:438-580` (`live_control_loop` function body) and `:667-847` (`run_live_drone` surrounding lifecycle).

| Current concern | Source location | Post-Phase-3 home |
|-----------------|-----------------|-------------------|
| `VelocityCommandAPI` construction | `mavsdk_drone.py:449` | `MavsdkDroneAdapter.__init__` — adapter owns `_filtered_yaw/forward/down`, `_prev_forward` |
| `_log` helper (LOGGER + ui_state push) | `mavsdk_drone.py:451-456` | Orchestrator-side `_log_with_ui` helper; `ui_state` plumbed via orchestrator |
| Tick period `period = 1.0 / control_loop_hz` | `mavsdk_drone.py:458` | Orchestrator `tick_dt` (lines 73 of CONTEXT pseudocode) |
| `last_detection_time` + age-of-detection check (`detection_timeout_s`) | `mavsdk_drone.py:459, 477-483` | Orchestrator — owns `last_seen_t`, applies `detection_timeout_s` check |
| `last_valid_detection` cache | `mavsdk_drone.py:460, 483` | Orchestrator — owns `last_detection`; passed to `on_target_lost(last_detection)` |
| `_prev_target_alt` (logging change-detection) | `mavsdk_drone.py:461, 498-500` | Orchestrator (debug-log only; not behavior) |
| `_prev_cmd` (hold-velocity bridge) | `mavsdk_drone.py:462, 547` | Orchestrator `last_cmd` — used for hold path before `on_target_lost` |
| IDLE mode (`target_state.is_paused()`) gate | `mavsdk_drone.py:486-488` | Orchestrator — when paused, treat `detection=None` and freeze `last_seen_t` so search timers don't advance |
| Search timeout → `shutdown.set()` ("Landing...") | `mavsdk_drone.py:491-495` | Orchestrator — fires the timeout, then sets shutdown; adapter's `shutdown()` lands |
| `compute_velocity_command(detection, config, last_detection, search_active, hold_velocity)` | `mavsdk_drone.py:502-507` | **Replaced** by orchestrator state machine + simplified `controller.compute(detection, caps, config)` |
| Altitude-hold P-loop (read `altitude_cache["m"]`, compute `alt_err`, clamp to climb/down, floor/ceiling) | `mavsdk_drone.py:512-524` | **Moved into adapter** — `MavsdkDroneAdapter._apply_altitude_p(down_m_s, altitude_cache, config) → float` |
| Min/max altitude floor/ceiling clamps | `mavsdk_drone.py:519-522` | Inside `_apply_altitude_p` |
| Throttled `[CTRL]` log (DEBUG every 0.5 s) | `mavsdk_drone.py:466, 470, 527-531` | Drop or move to adapter's send_command; not behavior |
| `vel_api.send(cmd)` (smoothing + clamping + MAVSDK send) | `mavsdk_drone.py:533` | `MavsdkDroneAdapter.send_command(cmd, safety_ctx)` — calls `_apply_smoothing`, `_translate_to_mavsdk` |
| `drone is None` print-only TRACK/SEARCH log | `mavsdk_drone.py:534-538` | **Delete** — no headless print path in post-Phase-3 adapter; was a dev-only echo |
| `ui_state.update_velocity(forward, down, yaw, mode)` | `mavsdk_drone.py:539-546` | Orchestrator (after `send_command` returns the clamped cmd) |
| Periodic 1 Hz UI status log (TRACK/SEARCH-WAIT/SEARCH banners with alt + Vd + hSpd) | `mavsdk_drone.py:549-572` | Orchestrator — pure logging concern; reads `altitude_cache` + `telemetry_cache` through public adapter accessors (`adapter.altitude_m()`, `adapter.velocity_ned()`) OR passes through SafetyContext extras (lightest-weight: planner picks) |
| `CancelledError` → `send_zero()` then re-raise | `mavsdk_drone.py:575-580` | Orchestrator `finally` block → `await robot.send_zero()` + `await robot.shutdown()` |
| `DetachedMavsdkServer` context manager (mavsdk_server subprocess) | `mavsdk_drone.py:679` | **Moved inside `MavsdkDroneAdapter.connect()`** — adapter owns mavsdk_server lifecycle; `connect()` enters the context, `shutdown()` exits via stored ref |
| `_wait_for_connection` (with `_CONNECTION_TIMEOUT_S=15`) | `mavsdk_drone.py:659-664, 699-706` | `MavsdkDroneAdapter.connect()` — raises `ConnectionError` on timeout per CONTEXT lifecycle semantics |
| 3 telemetry tasks (`_telemetry_velocity_task`, `_telemetry_position_task`, `_telemetry_log_task`) | `mavsdk_drone.py:369-435, 720-728` | `MavsdkDroneAdapter.start_session()` spawns them; `shutdown()` cancels them. Caches (`telemetry_cache`, `altitude_cache`) become private adapter attrs |
| `_wait_for_offboard_mode` keep-alive zero-setpoint streamer | `mavsdk_drone.py:288-327` | `MavsdkDroneAdapter.start_session()` — drone-only handshake |
| `_watch_offboard_mode` background task | `mavsdk_drone.py:330-340, 795-796` | `MavsdkDroneAdapter` — internal watchdog; surfaces "offboard lost" through an internal asyncio.Event or via `send_command` raising a tagged exception that orchestrator treats as pause (open question 4) |
| `--takeoff-landing` branch: `set_takeoff_altitude` → arm-retry → takeoff → 5 s sleep → `_start_offboard` | `mavsdk_drone.py:730-757` | `MavsdkDroneAdapter.start_session()` when `args.takeoff_landing=True`. The arm-retry loop, the 5 s climb sleep, and the offboard handshake all live inside the adapter |
| No-takeoff branch: outer `while not shutdown` loop with offboard-lost re-entry | `mavsdk_drone.py:780-834` | Orchestrator owns the outer loop **only** because shutdown is a global event; offboard-lost re-entry is adapter-internal — when `start_session()` detects offboard lost, the adapter can either re-handshake silently OR raise back to orchestrator. CONTEXT punts on this; recommend the adapter re-handshakes internally so orchestrator stays robot-agnostic. (Open question 4) |
| `mission_duration` deadline | `mavsdk_drone.py:772, 802` | Orchestrator — owns the `asyncio.wait` deadline against `args.mission_duration`. Drone-specific by virtue of being CLEAN-04's documented drone watchdog; `add_drone_args` registers the flag |
| `_land_safely` + SIGINT-ignore-during-landing | `mavsdk_drone.py:354-362, 619-637, 845-846` | `MavsdkDroneAdapter.shutdown()` — adapter's idempotent shutdown does the landing if `manage_takeoff_landing and armed` |
| `_reap_mavsdk_server` fallback for hung drone_thread | `robot_follow_app.py:442` | **Stay in `robot_follow_app.py`** — composition root owns the "drone thread join timed out, kill the leftover" path. `_reap_mavsdk_server` continues to be re-exported from `robot_api.adapters.mavsdk_drone` |

**Net result:** `live_control_loop` becomes the 30-line `run_robot_loop` pseudocode in CONTEXT lines 65-103. The 142-line current `live_control_loop` body shrinks to roughly:
- 30 lines orchestrator (`run_robot_loop`)
- ~25 lines of adapter `send_command` (smoothing + altitude P + retreat-from-tilt + MAVSDK send)
- ~10 lines of adapter `on_target_lost` (search yaw direction from `last_detection.center_x`)
- ~15 lines of `start_session` (offboard handshake + telemetry task spawn)

---

## `VelocityCommandAPI` method-mapping table

Source: `robot_follow/drone_api/mavsdk_drone.py:98-203`.

| Current method | Lines | Behavior | Post-Phase-3 home | Notes |
|----------------|-------|----------|-------------------|-------|
| `__init__(drone, config)` | `:108-123` | Stores `drone`, `config`, init filter state (`_filtered_yaw`, `_filtered_forward`, `_filtered_down`, `_prev_forward`) | `MavsdkDroneAdapter.__init__` — adapter owns the filter state directly (no separate `VelocityCommandAPI` class) |
| `_ema(raw, prev, alpha)` static | `:125-128` | First-order EMA `α·raw + (1-α)·prev` | Inline pure helper inside `_apply_smoothing` |
| `send(cmd)` | `:130-181` | Clamp → EMA → slew-limit → MAVSDK send | Becomes `MavsdkDroneAdapter.send_command(cmd, safety_ctx)`. Body: `_apply_altitude_p(cmd.down_m_s)` → `_apply_retreat_from_tilt(cmd.forward_m_s, safety_ctx)` → `_apply_smoothing(cmd)` → `_translate_to_mavsdk(cmd)` → `drone.offboard.set_velocity_body(...)` |
| `send_zero()` | `:183-191` | Reset all filter state, send `VelocityBodyYawspeed(0,0,0,0)` | `MavsdkDroneAdapter.send_zero()` — same body, drops the filter-reset (caller invokes only on shutdown; filter state about to be discarded) OR keeps reset for safety. Recommend: keep the reset — idempotency win. |
| `send_raw(cmd)` | `:193-196` | Bypass clamp + filter, raw MAVSDK send | Private helper `MavsdkDroneAdapter._send_raw_to_mavsdk(rc)` — used only by `start_session`'s offboard zero-setpoint stream pre-handoff |
| `reset_filters()` | `:198-203` | Zero all filter state | Private `MavsdkDroneAdapter._reset_filters()` — called from `start_session` re-entry after offboard-lost (matches current `mavsdk_drone.py:791`) |

**Pure-function extracts per R5** — see Pure-Function Extracts section. The four extracts plus the lifecycle methods give us roughly:

```python
class MavsdkDroneAdapter:
    caps: Capabilities  # axes={FORWARD, YAW, ALTITUDE}, yaw_unit="deg/s"

    def __init__(self, args, config: ControllerConfig): ...
    async def connect(self) -> None: ...                # DetachedMavsdkServer + drone.connect + wait_for_connection
    async def start_session(self) -> None: ...          # offboard handshake + telemetry spawn + optional arm/takeoff
    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None: ...
    async def send_zero(self) -> None: ...
    async def on_target_lost(self, last_detection: Optional[Detection]) -> None: ...
    async def shutdown(self) -> None: ...               # cancel telemetry, land if armed, exit DetachedMavsdkServer

    # Pure helpers (testable without MAVSDK)
    @staticmethod
    def _apply_altitude_p(down_m_s, altitude_cache, config) -> float: ...
    @staticmethod
    def _apply_retreat_from_tilt(forward_m_s, safety_ctx, config) -> float: ...
    @staticmethod
    def _apply_smoothing(cmd, smoothing_state, config) -> RobotCommand: ...   # mutates smoothing_state dict
    @staticmethod
    def _compute_search_yawspeed(last_detection, config) -> float: ...
```

---

## Controller signature migration

Current signature (`robot_follow/follow_api/controller.py:120-126`):
```python
def compute_velocity_command(
    detection: Optional[Detection],
    config: ControllerConfig,
    last_detection: Optional[Detection] = None,
    search_active: bool = True,
    hold_velocity: Optional[VelocityCommand] = None,
) -> VelocityCommand:
```

Target signature:
```python
def compute(
    detection: Detection,                # NEVER None — orchestrator handles target-lost
    caps: Capabilities,
    config: ControllerConfig,
) -> RobotCommand:
```

Changes:
1. **`detection` becomes non-Optional.** Orchestrator state machine handles `detection is None` (hold-velocity / `on_target_lost`).
2. **`last_detection`, `search_active`, `hold_velocity` dropped.** Owned by orchestrator (see CONTEXT pseudocode lines 70-99).
3. **`caps` added.** Controller gates `down_m_s` emission on `Axis.ALTITUDE in caps.axes` (per ABS-04); gates emergency-retreat on `Axis.FORWARD in caps.axes` (per CONTEXT lines 59-60 — "controller's emergency-retreat is gated on `Axis.FORWARD in caps.axes`").
4. **Internal changes:** `_apply_frame_edge_safety` is gutted — controller no longer applies the fade/retreat math. Per ABS-05, controller only sets `forward = 0` if `bbox_bottom_normalized > 1 - bottom_margin_safety` (the edge detection). The fade-zone math (`controller.py:88-101`) and the retreat push (`controller.py:101`) move to the adapter as `_apply_retreat_from_tilt`. Top-margin behavior: ditto for the top edge (controller emits `forward = 0` in the edge zone; adapter applies the corresponding forward push if the robot supports it).

### Caller list (sites that call `compute_velocity_command`)

| File | Lines | Site type |
|------|-------|-----------|
| `robot_follow/drone_api/mavsdk_drone.py` | `:502` | Production — moves to orchestrator `controller.compute(detection, robot.caps, config)` |
| `robot_follow/tests/test_controller.py` | 49 invocations (lines 36, 40, 50, 55, 60, 65, 66, 71, 79, 80, 89, 90, 101, 112, 123, 134, 135, 145, 146, 250, 256, 262, 271, 274, 283, 288, 302, 308, 317, 336, 337, 352, 364, 374, 401, 408, 418, 427, 436, 442, 449, 456, 463, 477, 487, 497, 507, 516) | Test |

**Test fixup count:** 49 invocations in `test_controller.py` need updating. The fix is small (each call gains a `DRONE_CAPS` arg) but mechanical.

### Tests that construct `VelocityCommand`

| File | Lines | Construction count |
|------|-------|-------------------|
| `robot_follow/tests/test_controller.py` | `:211, 213, 217, 228, 231, 233, 236` | 7 (inside `Smoothing` tests that exercise `VelocityCommandAPI` directly) |
| `robot_follow/tests/test_velocity_api_and_smoother.py` | `:44, 49, 54, 55, 60, 64, 78, 79, 91, 101, 105, 108, 121, 134, 141, 164, 171, 177, 178, 183, 202, 207, 214, 216, 222, 238, 249` | 46 |
| `robot_follow/tests/test_velocity_command_shape.py` | `:15` | 1 |

**Total `VelocityCommand(...)` constructions to rewrite to `RobotCommand(...)`:** 54. Mechanical search-and-replace once `VelocityCommand` → `RobotCommand` is locked.

### Tests that read `last_detection`, `search_active`, `hold_velocity`

Only `test_controller.py:134, 135, 146-150` exercise these args. The hold-velocity / search-direction tests move to orchestrator coverage (Phase 3 may keep these tests by retargeting them at the orchestrator's state machine — `test_orchestrator.py` is a new file).

### Comparison to CLEAN-07 fixup pattern

CLEAN-07 was 11 single-arg call sites for `state.update(detection)` → `state.update(detection, available_ids=set())`. Phase 3's controller migration is bigger: **49 controller test invocations + 54 `VelocityCommand` constructions = 103 mechanical edits across 3 test files**. Land in a single atomic commit per the CLEAN-07 pattern (`02-RESEARCH § CLEAN-07`).

---

## Argparse splits: `add_common_args` / `add_drone_args` / `add_rover_args`

### Current flag inventory

**`ControllerConfig.add_args` (`follow_api/config.py:227-320`)** — 29 controller flags. All robot-agnostic; goes to `add_common_args`.

| Flag | Type | Default | Classification |
|------|------|---------|----------------|
| `--config` | JSON path | None | common |
| `--save-config` | JSON path | None | common |
| `--hfov` | float | 66.0 | common |
| `--target-bbox-height` | float | None | common |
| `--distance-gain` (`kp_distance`) | float | 0.8 | common |
| `--distance-gain-back` (`kp_distance_back`) | float | 2.5 | common |
| `--dead-zone-bbox-percent` | float | 10.0 | common |
| `--max-climb-speed` | float | 1.0 | **drone-only** (altitude axis) |
| `--kp-alt-hold` | float | 0.5 | **drone-only** |
| `--min-altitude` | float | 2.0 | **drone-only** |
| `--max-altitude` | float | 4.0 | **drone-only** |
| `--control-loop-hz` | float | 10.0 | common |
| `--yaw-gain` (`kp_yaw`) | float | 4 | common |
| `--yaw-only` / `--no-yaw-only` | bool | True | common (rover semantics: forward-disable) |
| `--auto-select` / `--no-auto-select` | bool | True | common |
| `--search-enter-delay` | float | 2.0 | common |
| `--search-timeout` | float | 60.0 | common |
| `--smooth-forward` / `--no-smooth-forward` | bool | True | common |
| `--forward-alpha` | float | 0.15 | common |
| `--forward-velocity-deadband` | float | 0.05 | common |
| `--smooth-down` / `--no-smooth-down` | bool | True | **drone-only** |
| `--down-alpha` | float | 0.2 | **drone-only** |
| `--max-forward` | float | 1.5 | common |
| `--max-backward` | float | 1.5 | common |
| `--max-forward-accel` | float | 1.5 | common (rover analog: wheel-slip slew cap) |
| `--max-bbox-height-safety` | float | 0.8 | common |
| `--top-margin-safety` | float | 0.10 | common (robot interprets) |
| `--bottom-margin-safety` | float | 0.25 | common (robot interprets) |
| `--log-verbosity` | choice | normal | common |

**Drone-only controller flags:** `--max-climb-speed`, `--kp-alt-hold`, `--min-altitude`, `--max-altitude`, `--smooth-down`, `--down-alpha` (6 flags — altitude-axis-related). These move to `add_drone_args` (or stay in `add_common_args` but get `Optional[float]` defaults per ABS-07; controller's `validate()` skips altitude checks when `Axis.ALTITUDE not in caps.axes`). **Recommendation:** keep the 6 altitude flags registered in `add_common_args` so `df_config.example.json` loads cleanly under both robot types, but document them as drone-effective-only. The `ABS-07` Optional refactor handles the rover-doesn't-care semantics.

**`add_drone_args` (`drone_api/mavsdk_drone.py:31-52`)** — 6 drone-connection flags. All drone-only.

| Flag | Type | Default | Classification |
|------|------|---------|----------------|
| `--connection` | str | `udpin://0.0.0.0:14540` | drone-only |
| `--serial` | str | None (const `/dev/ttyACM0`) | drone-only |
| `--serial-baud` | int | 57600 | drone-only |
| `--takeoff-landing` | bool flag | False | drone-only |
| `--target-altitude` | float | 3.0 | drone-only |
| `--mission-duration` | float | 300.0 | drone-only |

**`_add_app_args` (`robot_follow_app.py:59-156`)** — 24 app/UI/ReID flags. All common.

| Flag | Type | Default | Classification |
|------|------|---------|----------------|
| `--follow-server-port` | int | 8080 | common |
| `--webui` / `--webui-port` / `--webui-fps` | mixed | False/5001/10 | common |
| `--openhd` / `--openhd-port` / `--openhd-bitrate` | mixed | False/5500/3917 | common |
| `--display` / `--record` / `--record-output` / `--record-bitrate` | mixed | mixed | common |
| `--log-perf` / `--test-log` | mixed | False/None | common |
| `--reid-model` / `--no-reid` / `--update-interval` / `--reid-threshold` / `--reid-timeout` / `--reid-drift-threshold` / `--reid-duplicate-threshold` / `--reid-refresh-every` / `--reid-min-gallery-for-drift-check` / `--reid-bootstrap-consistency` / `--reid-dump-embeddings` | mixed | mixed | common (all 11 ReID flags) |

**`add_tracker_args` (`pipeline_adapter/tracker_factory.py:47+`)** — tracker config flags. All common.

**`get_pipeline_parser()` (upstream `hailo-apps/hailo_apps/python/core/common/core.py:90`)** — `--input`, `--input-codec`, tile sizes, etc. All common (camera + Hailo pipeline plumbing).

### Proposed splits

```python
# robot_follow/robot_follow_app.py (new)

def add_common_args(parser: argparse.ArgumentParser) -> None:
    """All flags shared by every Robot type.

    Includes: pipeline (--input, --tiles-x, ...), tracker (--tracker, ...),
    web UI (--webui, --webui-port, --webui-fps), OpenHD (--openhd, ...),
    local outputs (--display, --record, ...), ReID, controller gains,
    framing, smoothing, safety. Altitude flags (--min-altitude,
    --max-altitude, --kp-alt-hold, --max-climb-speed, --smooth-down,
    --down-alpha) stay registered here per ABS-07 (controller treats them
    as Optional and validate() skips altitude checks when ALTITUDE absent).
    """
    # Delegates to upstream + project helpers:
    from hailo_apps.python.core.common.core import get_pipeline_parser
    # ... existing pipeline parser + ControllerConfig.add_args + add_tracker_args + _add_app_args

def add_drone_args(parser: argparse.ArgumentParser) -> None:
    """Drone-only: MAVLink connection + flight lifecycle.

    --takeoff-landing, --target-altitude, --serial, --serial-baud,
    --connection, --mission-duration.
    """
    # Body moves from drone_api/mavsdk_drone.py:31-52 verbatim.

def add_rover_args(parser: argparse.ArgumentParser) -> None:
    """Rover-only: ROS 2 cmd_vel publisher config.

    Phase 3 lands the dispatch infrastructure; Phase 4 fills in the
    rover-specific flags. For Phase 3 the body MAY be empty (rover
    selection still works; --help shows only common flags). Recommend
    leaving a placeholder group so plan-checker can confirm it's wired.
    """
    group = parser.add_argument_group("rover-ros2")  # placeholder
```

### Two-pass argparse: pre-parser extension

The existing single pre-parser at `robot_follow_app.py:213-247` already collects UI/ReID/tracker flags via `parse_known_args`. Extend it minimally:

```python
# Pre-parser pass 1: extract --robot
pre_robot = argparse.ArgumentParser(add_help=False)
pre_robot.add_argument("--robot", choices=("drone", "rover"), default="drone")
robot_args, remaining = pre_robot.parse_known_args()

# Pass 2: full parser with robot-conditional dispatch
parser = argparse.ArgumentParser()
parser.prog = "robot-follow"
add_common_args(parser)
if robot_args.robot == "drone":
    add_drone_args(parser)
else:
    add_rover_args(parser)
# Re-add --robot to the full parser so --help and final argv parse cleanly
parser.add_argument("--robot", choices=("drone", "rover"), default="drone")

args = parser.parse_args()  # raises on bad input
```

**ABS-09 verification:**
- `drone-follow --robot drone --help | grep -E '(--takeoff-landing|--target-altitude|--serial)'` → 3 hits.
- `drone-follow --robot rover --help | grep -E '(--takeoff-landing|--target-altitude|--serial)'` → 0 hits.
- `drone-follow --robot rover --help | grep -E '(--webui|--openhd|--display|--record)'` → ≥4 hits (common flags visible to both).

**Trap to avoid:** the existing pre-parser at `:213` registers many flags (UI/ReID/tracker) so it can `parse_known_args` and downstream code (`web_server`, `ReIDManager` construction) can read pre-parse state. Keep that pre-parser AS-IS; the new `pre_robot` pre-parser is its own small parser that runs first and reads only `--robot`. Don't fold them into one — they have different jobs (pre_robot is dispatch; the existing pre is "decide branches before full parser runs"). The two pre-parsers can coexist.

**Plan-checker rule:** no flag may be in both `add_drone_args` and `add_rover_args`. CONTEXT line 128 of DESIGN-NOTES already calls this out.

---

## `setup_env.sh` ROS-source block

Current `setup_env.sh` (37 lines total). The conditional ROS source block goes AFTER the venv activation (after the `cd "$_DF_ORIG_PWD"` at line 36, before the trailing `unset`). Per PITFALLS.md Pitfall 2 (`.planning/research/PITFALLS.md:29-43`) and DESIGN-NOTES § "Why 'always source ROS if installed'" (line 184-186), the order is venv-first, ROS-second so the venv's site-packages take priority on `sys.path` while ROS's `LD_LIBRARY_PATH` and `AMENT_PREFIX_PATH` are appended.

### Exact bash block to insert at end of `setup_env.sh`

```bash
# Conditionally source ROS 2 Humble after the venv is active.
#
# Rationale (CONTEXT 03-abstraction § setup_env.sh ROS sourcing,
# PITFALLS.md Pitfall 2):
# - Venv-first sourcing keeps the hailo-apps venv's site-packages ahead
#   of /opt/ros/humble/lib/python3.10/site-packages on sys.path, so any
#   pure-Python shim in the venv that shadows an rclpy submodule does NOT
#   hide rclpy's _rclpy_pybind11 .so.
# - Idempotent: drone-only users on a non-ROS box see no change; rover
#   users get ROS for free; drone users on a ROS-equipped box source ROS
#   but the drone path is unaffected (ROS env vars don't conflict with
#   the venv's Python or MAVSDK).
# - Single file check (no `command -v ros2` — chicken-and-egg before
#   sourcing).
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
```

### Sourcing-order rationale

Per PITFALLS.md line 35-38 ("`setup_env.sh` exports `PYTHONPATH` for hailo-apps. If that path comes before the ROS site-packages, any pure-Python stub in the venv that shadows an rclpy submodule will hide the `.so`"):
- The hailo-apps `setup_env.sh` (sourced at line 35 of drone-follow's wrapper) sets `PYTHONPATH` to put hailo-apps venv first.
- `source /opt/ros/humble/setup.bash` then **appends** `/opt/ros/humble/lib/python3.10/site-packages` to `PYTHONPATH` and sets `LD_LIBRARY_PATH=/opt/ros/humble/lib:...`.
- Python's import system resolves venv first; `rclpy` is only found via the ROS path; the `.so` finds its sibling C extensions via `LD_LIBRARY_PATH` — both arrangements correct.

### Verification commands (operator-runnable on a ROS-equipped dev box)

```bash
# Before:
env > /tmp/env_before.txt
# After:
source setup_env.sh
env > /tmp/env_after.txt
diff /tmp/env_before.txt /tmp/env_after.txt | grep -E '(ROS|AMENT|LD_LIBRARY|PYTHONPATH|CMAKE)'
# Expected diff: ROS_DISTRO=humble, AMENT_PREFIX_PATH set, LD_LIBRARY_PATH
# appended with /opt/ros/humble/lib, PYTHONPATH appended with
# /opt/ros/humble/lib/python3.10/site-packages.

# Drone smoke: confirm the drone path runs cleanly:
robot-follow --robot drone --help > /dev/null
# Exit 0 expected.

# Rover smoke (before Phase 4 lands the adapter):
robot-follow --robot rover --help > /dev/null
# Exit 0 expected; common flags visible, no drone flags.
```

**This dev box** does NOT have `/opt/ros/humble` installed (`test -d /opt/ros/humble` → false). The block above is safe-by-default — on this machine it's a no-op (test fails, block skipped). ROS-env-leak verification is necessarily a deferred-to-ROS-host smoke step, per DESIGN-NOTES risk #10 (line 213).

---

## `Ros2RoverAdapter` paper sketch (validates protocol fit before locking)

**Purpose:** confirm the `Robot` protocol fits a ROS 2 rclpy adapter before Phase 3 lands. Not for Phase 3 to implement.

```python
# robot_api/adapters/ros2_rover.py (PAPER SKETCH — Phase 4 implements)

from typing import Optional
import signal
import threading

from robot_follow.follow_api.types import (
    Axis, Capabilities, Detection, RobotCommand, SafetyContext,
)

# Capabilities for any ROS 2 cmd_vel rover (ROVER-07):
ROVER_CAPS = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW}),
    yaw_unit="rad/s",
)


class Ros2RoverAdapter:
    """ROS 2 cmd_vel publisher adapter. Implements Robot protocol."""

    caps = ROVER_CAPS  # class-level for type-checker; CONTEXT instance-attr
                      # rule means __init__ ALSO sets self.caps = ROVER_CAPS

    def __init__(self, args, config):
        # Defer rclpy import to allow drone-only users to skip ROS install
        # (ROVER-04 friendly error). Raise RuntimeError, not ImportError,
        # so the CLI emits a clean message instead of a traceback.
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from geometry_msgs.msg import Twist
        except ImportError as e:
            raise RuntimeError(
                "ROS 2 not sourced — run: "
                "sudo apt install ros-humble-ros-base ros-humble-geometry-msgs "
                "then re-source setup_env.sh"
            ) from e

        self._rclpy = rclpy
        self._Twist = Twist
        self._Node = Node
        self._SignalHandlerOptions = SignalHandlerOptions
        self._SingleThreadedExecutor = SingleThreadedExecutor

        self._topic = getattr(args, "cmd_vel_topic", "/cmd_vel")
        self._namespace = getattr(args, "ros_namespace", "")
        self._domain_id = getattr(args, "ros_domain_id", 0)
        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread = None
        self._shutdown_event = threading.Event()

        self.caps = ROVER_CAPS  # CONTEXT instance-attr rule

    async def connect(self) -> None:
        """ROVER-02: rclpy.init with SignalHandlerOptions.NO. Verify the
        SIGINT handler is still drone-follow's on_signal after init."""
        before_sigint = signal.getsignal(signal.SIGINT)
        self._rclpy.init(
            args=None,
            signal_handler_options=self._SignalHandlerOptions.NO,
        )
        after_sigint = signal.getsignal(signal.SIGINT)
        assert before_sigint is after_sigint, (
            "rclpy.init replaced SIGINT handler despite "
            "SignalHandlerOptions.NO — abort before publishing setpoints"
        )

    async def start_session(self) -> None:
        """Create node + publisher + spin-thread. Effectively instantaneous
        (no analog of the drone's offboard handshake)."""
        self._node = self._Node("robot_follow_rover", namespace=self._namespace)
        self._publisher = self._node.create_publisher(self._Twist, self._topic, 10)
        self._executor = self._SingleThreadedExecutor()
        self._executor.add_node(self._node)

        def _spin_loop():
            while not self._shutdown_event.is_set():
                self._executor.spin_once(timeout_sec=0.05)  # ROVER-03

        self._executor_thread = threading.Thread(target=_spin_loop, daemon=True)
        self._executor_thread.start()

    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None:
        # ROVER-06: yaw_rate is in caps.yaw_unit ("rad/s" for rover).
        # Controller emitted yaw_rate in caps.yaw_unit at the boundary, so
        # NO conversion needed here — the controller did it. But CONTEXT
        # says the rover adapter converts from deg/s to rad/s at the wire
        # because the controller emits in deg/s. Open question 5: which is
        # the contract?
        twist = self._Twist()
        twist.linear.x = cmd.forward_m_s
        twist.angular.z = cmd.yaw_rate  # already in caps.yaw_unit ("rad/s")
        # Rover does NOT touch cmd.down_m_s — Axis.ALTITUDE not in caps.axes.
        # Bottom-edge safety overlay (RINT-02): slow if person too low.
        if safety_ctx.bbox_bottom_normalized > 1.0 - 0.10:  # config-driven in Phase 4
            twist.linear.x = 0.0
        self._publisher.publish(twist)

    async def send_zero(self) -> None:
        twist = self._Twist()  # all zeros by default
        self._publisher.publish(twist)

    async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
        # Rover doesn't yaw-spin on target loss (unlike drone). Just stop.
        twist = self._Twist()
        self._publisher.publish(twist)

    async def shutdown(self) -> None:
        # Idempotent (CONTEXT lifecycle). Always called in orchestrator finally.
        self._shutdown_event.set()
        if self._executor_thread is not None:
            self._executor_thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()  # MUST come before try_shutdown
                                       # (PITFALLS.md "Looks Done But Isn't")
        try:
            self._rclpy.try_shutdown()
        except Exception:
            pass
```

### Does the `Robot` protocol fit?

**Yes**, with one caveat surfaced for the planner:

| Protocol method | Drone fit | Rover fit | Notes |
|-----------------|-----------|-----------|-------|
| `connect` | TCP/UDP connect + 15 s timeout | `rclpy.init` with `SignalHandlerOptions.NO` | Both raise on failure cleanly |
| `start_session` | Offboard handshake (10–60 s) | Node + publisher creation (sub-second) | Adapter-specific lifecycle. Fits. |
| `send_command(cmd, safety_ctx)` | Smoothing + altitude P + retreat-from-tilt + `set_velocity_body` | `Twist` publish, optionally slowed by bottom-edge | `SafetyContext` decouples both adapters from `Detection` shape. Fits. |
| `send_zero()` | `VelocityBodyYawspeed(0,0,0,0)` + reset filters | `Twist()` (all-zeros) | Fits. |
| `on_target_lost(last_detection)` | Yaw spin in direction of last bbox side | `Twist()` (stop) | Fits. |
| `shutdown()` | Cancel telemetry, land if armed, exit `DetachedMavsdkServer` | Stop spin thread, destroy node, `try_shutdown` | Fits. |
| `caps` attribute | `{FORWARD, YAW, ALTITUDE}, yaw_unit="deg/s"` | `{FORWARD, YAW}, yaw_unit="rad/s"` | Fits. |

**Caveat surfaced — yaw_unit conversion location.** CONTEXT line 168 says "yaw_rate is in caps.yaw_unit (drone: deg/s; rover: rad/s). Adapter normalizes at the wire." ROVER-06 says "At adapter boundary, `RobotCommand.yaw` (deg/s) is converted to rad/s before assignment to `Twist.angular.z`." These contradict — first says controller emits in caps.yaw_unit (so no conversion at adapter), second says controller emits in deg/s always (so adapter converts). See Open Question 5 below.

---

## Snapshot fixture design

**File:** `robot_follow/tests/cases/drone_command_baseline.py` (preferred — `.py` allows typed constants per CONTEXT line 159) or `.json` (if planner prefers data-only).

**Cases:** ~100 representative `Detection` inputs covering:

### Category breakdown (~100 cases)

| Category | Count | Example case |
|----------|-------|--------------|
| **Target-centered, at-target-bbox** | 5 | `cy=0.5, cx=0.5, bh=0.25` → all-zero command |
| **Target-offset-left** | 8 | `cx ∈ {0.10, 0.20, 0.30, 0.40, 0.45, 0.48, 0.49, 0.50}, cy=0.5, bh=0.25` → yaw sweep negative-to-zero |
| **Target-offset-right** | 8 | symmetric `cx ∈ {0.51, 0.52, 0.55, 0.60, 0.70, 0.80, 0.90, 1.00}` → yaw sweep zero-to-positive |
| **Bbox-too-small (approach)** | 10 | `bh ∈ {0.05, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.22, 0.23, 0.24}, cx=cy=0.5` → forward sweep |
| **Bbox-too-large (retreat)** | 8 | `bh ∈ {0.26, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.79}` → backward sweep up to safety threshold |
| **Emergency safety (bbox > 0.8)** | 4 | `bh ∈ {0.81, 0.85, 0.90, 0.95}` → `forward_m_s == -max_backward` |
| **Bbox at top margin (cy small)** | 8 | `cy ∈ {0.04, 0.08, 0.10, 0.12, 0.15, 0.18, 0.20, 0.22}, bh=0.20` → top-edge fade then push |
| **Bbox at bottom margin (cy large)** | 12 | `cy ∈ {0.65, 0.70, 0.75, 0.80, 0.85, 0.88, 0.90, 0.92, 0.93, 0.95, 0.97, 0.99}, bh=0.20` → bottom-edge fade then push |
| **Bbox in fade zone** | 8 | Mixed `cy/bh` placing `bbox_bottom ∈ [0.50, 0.75]` (bottom fade zone for `bottom_margin=0.25`) |
| **`yaw_only=True` config** | 5 | Various detections with `yaw_only=True` — confirm `forward = 0` always |
| **Dead-zone holds zero** | 4 | `cx ∈ {0.495, 0.500, 0.505, 0.510}` (inside `dead_zone_deg`) |
| **Forward velocity deadband** | 3 | `bh` placing |natural forward| < 0.05 → snapped to 0 |
| **Asymmetric retreat (`kp_distance_back`)** | 4 | Verify retreat 3× more aggressive than approach for same `\|factor\|` |
| **Search direction from `last_detection`** | 8 | `detection=None`, `last_detection.center_x ∈ {0.1, 0.2, 0.3, 0.4, 0.6, 0.7, 0.8, 0.9}` → `_compute_search_yawspeed` direction sweep |
| **Hold velocity (search-enter delay not yet reached)** | 5 | Orchestrator-level: `detection=None, search_active=False, last_cmd=...` |
| **Total** | **~100** | |

### Data layout

```python
# robot_follow/tests/cases/drone_command_baseline.py

"""Phase-3-only snapshot fixture.

Captures the OLD compute_velocity_command output for ~100 detections at
HEAD = 9ba4ec7 (pre-rewrite). Phase 3 verifier asserts that the new
controller.compute(detection, DRONE_CAPS, config) + MavsdkDroneAdapter
internal pipeline (altitude_p + retreat_from_tilt + smoothing +
translate_to_mavsdk) produces equivalent VelocityBodyYawspeed outputs.

ARCHIVE this file after Phase 3 verifier passes. Future tuning changes
(rover kp_yaw in Phase 6, etc.) WILL break the snapshot — by design,
this is intentional, the snapshot is not a permanent invariant. See
CONTEXT § Regression test strategy.
"""

from dataclasses import dataclass
from typing import Optional
from robot_follow.follow_api.types import Detection

@dataclass(frozen=True)
class BaselineCase:
    name: str                       # human-readable, e.g. "target_centered_at_target_bbox"
    config_overrides: dict          # ControllerConfig kwargs (e.g. {"yaw_only": False})
    detection: Optional[Detection]  # current detection
    last_detection: Optional[Detection]  # for search-direction cases
    # Expected outputs (filled by pre-rewrite capture)
    expected_velocity_command: tuple[float, float, float]  # (fwd, down, yaw) from compute_velocity_command
    # NOTE: expected_robot_command will be filled post-rewrite for the
    # equivalence assertion; the snapshot test calls both pipelines and
    # asserts they match.

DRONE_CAPS = ...  # frozenset({Axis.FORWARD, Axis.YAW, Axis.ALTITUDE}), deg/s

CASES: list[BaselineCase] = [
    BaselineCase(
        name="target_centered_at_target_bbox",
        config_overrides={"yaw_only": False},
        detection=Detection(label="person", confidence=0.9,
                            center_x=0.5, center_y=0.5, bbox_height=0.25,
                            timestamp=0.0),
        last_detection=None,
        expected_velocity_command=(0.0, 0.0, 0.0),
    ),
    # ... ~99 more
]
```

### Capture procedure (pre-rewrite)

```bash
# 1. Author the case list (BaselineCase entries without expected_*).
# 2. Pre-rewrite, run a one-shot script that calls compute_velocity_command
#    for each case and fills in expected_velocity_command.
# 3. Commit the populated fixture.
# 4. Post-rewrite, the test asserts new pipeline produces the same tuple.
```

### Test shape (`test_robot_command_snapshot.py`)

```python
# Reuse pytest.parametrize over CASES for clean per-case failure messages.

@pytest.mark.parametrize("case", CASES, ids=lambda c: c.name)
def test_snapshot(case: BaselineCase):
    config = ControllerConfig(**case.config_overrides)

    # New pipeline:
    if case.detection is not None:
        cmd = controller.compute(case.detection, DRONE_CAPS, config)
        safety_ctx = SafetyContext.from_detection(case.detection)
    else:
        # Replicate orchestrator's lost-target path
        cmd = controller.compute_lost(case.last_detection, DRONE_CAPS, config)  # OR adapter.on_target_lost
        safety_ctx = SafetyContext.lost(last_target_x=case.last_detection.center_x if case.last_detection else None)

    # Apply adapter's pre-MAVSDK pipeline (altitude_p + retreat_from_tilt + smoothing):
    cmd = MavsdkDroneAdapter._apply_altitude_p(...)  # mock altitude_cache as needed
    cmd = MavsdkDroneAdapter._apply_retreat_from_tilt(...)
    # (Smoothing is stateful — exclude from snapshot or seed deterministically.)

    # Assert equivalence
    actual = (cmd.forward_m_s, cmd.down_m_s, cmd.yaw_rate)
    assert actual == pytest.approx(case.expected_velocity_command, abs=1e-6)
```

**Smoothing caveat:** EMA + slew-limiter are stateful across calls. Either (a) the snapshot replicates filter state in the expected tuple per-case sequenced over multiple ticks, or (b) the snapshot disables smoothing (`smooth_yaw=False, smooth_forward=False, smooth_down=False, max_forward_accel=0`) and tests smoothing separately in `test_mavsdk_drone_adapter.py`. **Recommend (b)** — smoothing equivalence is `test_velocity_api_and_smoother.py`'s job today, and migrating those 46 tests to the new `_apply_smoothing` pure function is mechanical.

---

## Pure-function extracts (R5)

All four extracts are static/module-level so they can be unit-tested without instantiating `MavsdkDroneAdapter`. Each takes only the inputs it needs (no `self` reference).

### `_apply_altitude_p`

**Source:** `mavsdk_drone.py:509-524` (altitude P-loop body).

```python
# robot_api/adapters/mavsdk_drone.py

def _apply_altitude_p(
    down_m_s: float,
    altitude_cache: dict,           # {"m": float | None}
    config: ControllerConfig,
) -> float:
    """Altitude-hold P correction on the down axis.

    - If `altitude_cache["m"]` is None (telemetry not yet received), pass
      `down_m_s` through unchanged.
    - If `config.yaw_only`, keep the controller-emitted `down_m_s` (which
      will be 0 from a yaw_only config).
    - Otherwise compute `down = config.kp_alt_hold * (current_alt -
      config.target_altitude)` clamped to [-max_climb_speed,
      max_down_speed], then apply min/max altitude floor/ceiling.
    """
    current_alt = altitude_cache.get("m")
    if current_alt is None:
        return down_m_s
    if config.yaw_only:
        return down_m_s  # controller already emitted 0 for yaw_only
    alt_err = current_alt - config.target_altitude
    down = config.kp_alt_hold * alt_err
    down = max(-config.max_climb_speed, min(config.max_down_speed, down))
    if current_alt <= config.min_altitude and down > 0:
        down = 0.0  # at floor
    elif current_alt >= config.max_altitude and down < 0:
        down = 0.0  # at ceiling
    return down
```

**Test cases (~8):** alt_cache empty → passthrough; yaw_only=True → passthrough; well below target → climb (down<0); well above target → descend (down>0); at floor descending → clamp to 0; at ceiling climbing → clamp to 0; saturate at max_climb_speed; saturate at max_down_speed.

### `_apply_retreat_from_tilt`

**Source:** `controller.py:58-117` (`_apply_frame_edge_safety` body — fade-zone + safety-push math).

```python
def _apply_retreat_from_tilt(
    forward_m_s: float,
    safety_ctx: SafetyContext,
    config: ControllerConfig,
) -> float:
    """Drone-specific bottom/top frame-edge fade + retreat overlay.

    Operates on adapter-side bbox geometry (carried in safety_ctx). The
    controller has already zeroed forward when bbox is in the edge zone
    (per ABS-05); this adapter overlay then applies the fade-zone scale-
    to-zero and the safety push.

    Equivalent to today's _apply_frame_edge_safety
    (follow_api/controller.py:58-117) but with bbox_top / bbox_bottom
    read from safety_ctx instead of recomputed from Detection.
    """
    if config.yaw_only:
        return forward_m_s
    bbox_bottom = safety_ctx.bbox_bottom_normalized
    bbox_top = bbox_bottom - safety_ctx.bbox_size_normalized
    forward = forward_m_s

    if config.bottom_margin_safety > 0:
        margin = config.bottom_margin_safety
        if forward > 0:
            fade_depth = bbox_bottom - (1.0 - 2.0 * margin)
            if fade_depth > 0:
                fade = max(0.0, 1.0 - fade_depth / margin)
                forward *= fade
        depth = bbox_bottom - (1.0 - margin)
        if depth > 0:
            ratio = min(depth / margin, 1.0)
            forward = min(forward, -ratio * config.max_backward)

    if config.top_margin_safety > 0:
        margin = config.top_margin_safety
        if forward < 0:
            fade_depth = (2.0 * margin) - bbox_top
            if fade_depth > 0:
                fade = max(0.0, 1.0 - fade_depth / margin)
                forward *= fade
        depth = margin - bbox_top
        if depth > 0:
            ratio = min(depth / margin, 1.0)
            forward = max(forward, ratio * config.max_forward)

    return forward
```

**Test cases (~10):** mirror today's `TestFrameEdgeSafety` cases in `test_controller.py:378-518`. The existing 14 tests migrate verbatim with `_apply_frame_edge_safety` swapped for `_apply_retreat_from_tilt(...)`.

### `_apply_smoothing`

**Source:** `mavsdk_drone.py:130-181` (`VelocityCommandAPI.send` body, lines 143-175 specifically).

```python
@dataclass
class SmoothingState:
    filtered_yaw: float = 0.0
    filtered_forward: float = 0.0
    filtered_down: float = 0.0
    prev_forward: float = 0.0  # for slew-rate limiter

def _apply_smoothing(
    cmd: RobotCommand,
    state: SmoothingState,
    config: ControllerConfig,
) -> RobotCommand:
    """Clamp + per-axis EMA + forward-axis slew-rate cap.

    Mutates `state` in place; returns the clamped/filtered RobotCommand.
    Behavior is byte-identical to VelocityCommandAPI.send body lines
    143-175 (drone_api/mavsdk_drone.py).
    """
    forward = max(-config.max_backward, min(config.max_forward, cmd.forward_m_s))
    down = max(-config.max_down_speed, min(config.max_down_speed, cmd.down_m_s))
    yaw_raw = max(-config.max_yawspeed, min(config.max_yawspeed, cmd.yaw_rate))

    if config.smooth_forward:
        state.filtered_forward = config.forward_alpha * forward + (1.0 - config.forward_alpha) * state.filtered_forward
        forward = state.filtered_forward
    else:
        state.filtered_forward = forward

    if config.smooth_down:
        state.filtered_down = config.down_alpha * down + (1.0 - config.down_alpha) * state.filtered_down
        down = state.filtered_down
    else:
        state.filtered_down = down

    if config.smooth_yaw:
        state.filtered_yaw = config.yaw_alpha * yaw_raw + (1.0 - config.yaw_alpha) * state.filtered_yaw
        yaw_out = state.filtered_yaw
    else:
        state.filtered_yaw = yaw_raw
        yaw_out = yaw_raw

    # Forward-axis slew-rate cap
    if config.max_forward_accel > 0 and config.control_loop_hz > 0:
        max_step = config.max_forward_accel / config.control_loop_hz
        delta = forward - state.prev_forward
        if delta > max_step:
            forward = state.prev_forward + max_step
        elif delta < -max_step:
            forward = state.prev_forward - max_step
    state.prev_forward = forward
    state.filtered_forward = forward

    return RobotCommand(forward_m_s=forward, yaw_rate=yaw_out, down_m_s=down)
```

**Test cases (~15):** All 46 existing tests in `test_velocity_api_and_smoother.py` migrate. The test signature changes from `api.send(VelocityCommand(...))` to `_apply_smoothing(RobotCommand(...), state, config)` with state initialized per-test.

### `_compute_search_yawspeed`

**Source:** `controller.py:138-142` (search-direction-from-last-detection).

```python
def _compute_search_yawspeed(
    last_detection: Optional[Detection],
    config: ControllerConfig,
) -> float:
    """Yaw speed for the drone's search-mode spin.

    Direction follows the last seen side: `last_detection.center_x > 0.5`
    → spin right (+); else spin left (−). Magnitude is
    `config.search_yawspeed_slow`. No last detection → default right.
    """
    if last_detection is None:
        return config.search_yawspeed_slow
    sign = 1.0 if last_detection.center_x > 0.5 else -1.0
    return sign * config.search_yawspeed_slow
```

**Test cases (~3):** last_detection=None → positive; last_detection.center_x=0.8 → positive; last_detection.center_x=0.2 → negative.

---

## `SafetyContext.from_detection` derivation

```python
# follow_api/types.py

from dataclasses import dataclass
from typing import Optional

@dataclass(frozen=True)
class SafetyContext:
    """Mininal struct the controller derives from Detection and passes
    to the adapter alongside RobotCommand. Decouples adapter from
    Detection-shape changes (v1.2: depth, multi-camera, multi-target).
    """
    bbox_bottom_normalized: float       # 0..1, for bottom-margin checks
    bbox_size_normalized: float         # bbox_height, for size-based safety
    target_lost: bool                   # convenience flag for adapter
    last_target_x: Optional[float]      # for on_target_lost search direction

    @classmethod
    def from_detection(cls, det: "Detection") -> "SafetyContext":
        """Construct from a current real detection."""
        return cls(
            bbox_bottom_normalized=det.center_y + det.bbox_height / 2,
            bbox_size_normalized=det.bbox_height,
            target_lost=False,
            last_target_x=det.center_x,
        )

    @classmethod
    def lost(cls, last_target_x: Optional[float] = None) -> "SafetyContext":
        """Construct for the orchestrator's hold-velocity path when no
        current detection exists. bbox fields are placeholders (1.0, 0.0)
        chosen so the adapter's bottom-edge safety overlay can't fire
        spuriously: bbox_bottom_normalized=1.0 means 'at frame bottom',
        bbox_size_normalized=0.0 means 'no bbox' — the
        _apply_retreat_from_tilt depth math evaluates to 0 (with a 0-size
        bbox, depth = 1.0 - (1-margin) = margin, ratio = 1.0, push =
        -max_backward) — actually this WOULD fire. So use placeholder
        values that put the bbox safely OUTSIDE the edge zones.
        """
        return cls(
            bbox_bottom_normalized=0.5,     # center of frame — safely inside
            bbox_size_normalized=0.25,      # target bbox size — neutral
            target_lost=True,
            last_target_x=last_target_x,
        )
```

### Verification mapping (every adapter use is covered)

| Adapter site | Reads from `SafetyContext` | Today reads from `Detection` |
|--------------|----------------------------|------------------------------|
| `_apply_retreat_from_tilt` bottom margin fade | `bbox_bottom_normalized` | `det.center_y + det.bbox_height / 2` (controller.py:87) |
| `_apply_retreat_from_tilt` top margin fade | `bbox_bottom_normalized - bbox_size_normalized` (= bbox_top) | `det.center_y - det.bbox_height / 2` (controller.py:86) |
| Emergency-safety threshold check (still controller-side per CONTEXT line 59) | n/a (controller reads `det.bbox_height` directly) | n/a |
| `on_target_lost` search direction | `last_target_x` (via SafetyContext.lost) OR direct `last_detection.center_x` (adapter receives both) | `last_detection.center_x` (controller.py:141) |

**Confirmed:** all adapter uses of `Detection` map to two SafetyContext fields (`bbox_bottom_normalized`, `bbox_size_normalized`) plus `last_target_x` for search direction. **Recommend:** `on_target_lost` takes `last_detection: Optional[Detection]` directly (per CONTEXT line 42), NOT a SafetyContext — the search-direction case is the only place the adapter needs the full Detection, and keeping the protocol signature `on_target_lost(last_detection)` is what CONTEXT locked.

---

## Connection-failure flow trace

### Today's failure handling

`run_live_drone` (`robot_follow_app.py:386-398` calling `mavsdk_drone.py:667+`):

1. **`with DetachedMavsdkServer(args.connection) as connection_url`** (`mavsdk_drone.py:679`):
   - Subprocess spawn of `mavsdk_server` with `start_new_session=True`.
   - Reaps stale server at line 252.
   - Sleeps 1.0 s and checks `process.poll()`. If exited, logs warning and returns `connection_url` unchanged (falls back to default System() behavior).
   - Failure mode: never raises — silent fallback.

2. **`drone.connect()`** (`mavsdk_drone.py:686 or :689`):
   - Async call to MAVSDK gRPC. Raises on hard gRPC failures (rare; MAVSDK absorbs most).

3. **`_wait_for_connection(drone)`** with `asyncio.wait_for(..., timeout=15)` (`mavsdk_drone.py:699-702`):
   - Subscribes to `drone.core.connection_state()`. Yields when `is_connected`.
   - On `asyncio.TimeoutError`: caught, `connected = False`. Then `raise ConnectionError(f"No drone detected on {args.connection} after 15s...")` at line 704-706.

4. **`raise ConnectionError(...)`** propagates up through the `with DetachedMavsdkServer` (which gracefully terminates the subprocess in `__exit__`).

5. **`run_drone()` wrapper** (`robot_follow_app.py:386-398`) catches `Exception` and logs: "Drone connection failed — pipeline continues without drone control." The pipeline keeps running without drone control. Drone thread exits cleanly.

### Post-Phase-3 flow

Per CONTEXT lines 111-116:

1. **`adapter.connect()`**:
   - Wraps today's `DetachedMavsdkServer` + `drone.connect()` + `_wait_for_connection(timeout=15)`.
   - Raises `ConnectionError` on timeout (same exception type as today).
2. **Orchestrator `run_robot_loop`**:
   ```python
   try:
       await robot.connect()
       await robot.start_session()
       while not shutdown.is_set():
           ...
   except Exception:
       LOGGER.warning("[robot] Connection failed — pipeline continues without robot control.", exc_info=True)
   finally:
       await robot.send_zero()    # no-op if connect failed (adapter guards on self._drone is None)
       await robot.shutdown()      # idempotent
   ```
3. **`adapter.shutdown()` idempotency:** all clean-up paths (DetachedMavsdkServer `__exit__`, telemetry cancel, land-if-armed) must check pre-conditions (`if self._drone is not None`, `if self._telemetry_tasks`) so a connect-fail-then-shutdown sequence doesn't raise on missing state.

### Failure-handling unchanged from today

- 15 s connection timeout: stays.
- Pipeline continues without drone control on connect failure: stays.
- No retry in v1.1: stays.
- `_reap_mavsdk_server` fallback for hung thread: stays in `robot_follow_app.py` finally.

---

## `Capabilities` as instance attribute — confirmation

CONTEXT line 40: "Plus `caps: Capabilities` attribute." Phrased as a protocol attribute, not a static class attribute.

### Python typing pattern

Python's `typing.Protocol` supports both class- and instance-attribute declarations. The cleanest form for adapter implementations:

```python
# robot_api/robot.py
from typing import Protocol
from robot_follow.follow_api.types import Capabilities, RobotCommand, SafetyContext, Detection

class Robot(Protocol):
    caps: Capabilities                          # protocol attribute declaration

    async def connect(self) -> None: ...
    async def start_session(self) -> None: ...
    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None: ...
    async def send_zero(self) -> None: ...
    async def on_target_lost(self, last_detection: Optional[Detection]) -> None: ...
    async def shutdown(self) -> None: ...
```

### Class-level constant + per-instance copy (recommended)

Each adapter declares a module-level `DRONE_CAPS` / `ROVER_CAPS` constant, and the `__init__` assigns `self.caps = DRONE_CAPS` (or `ROVER_CAPS`). This satisfies the Protocol's instance-attribute contract while also exposing the constant for tests:

```python
# robot_api/adapters/mavsdk_drone.py
DRONE_CAPS = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW, Axis.ALTITUDE}),
    yaw_unit="deg/s",
)

class MavsdkDroneAdapter:
    def __init__(self, args, config):
        self.caps = DRONE_CAPS
        ...
```

**Rationale:** static class attribute (`MavsdkDroneAdapter.caps = DRONE_CAPS` outside `__init__`) works for type-checking, but conflicts with the Protocol's instance-attribute reading at runtime (a Protocol can't tell the difference if `caps` is class-level, but tests reading `adapter.caps` expect an instance attribute). Instance-level is the safest pattern. **CONTEXT line 40 locks in instance attribute.**

---

## ROS env-leak verification test

This dev box does NOT have `/opt/ros/humble` installed (`test -d /opt/ros/humble` → false). The test is deferred to a ROS-equipped operator run, but documented as a concrete procedure:

```bash
# ON A ROS-EQUIPPED DEV BOX:

# Test 1 — env diff
fresh_shell -c '
    env | sort > /tmp/env_pre.txt
    source /home/hailo/hailo-drone-follow/setup_env.sh
    env | sort > /tmp/env_post.txt
'
diff /tmp/env_pre.txt /tmp/env_post.txt | head -50
# Expected: differences ONLY in ROS_DISTRO, ROS_VERSION, ROS_PYTHON_VERSION,
# AMENT_PREFIX_PATH, CMAKE_PREFIX_PATH, LD_LIBRARY_PATH (appended),
# PATH (prepended /opt/ros/humble/bin), PKG_CONFIG_PATH, PYTHONPATH
# (appended /opt/ros/humble/lib/python3.10/site-packages),
# COLCON_PREFIX_PATH, ROS_LOCALHOST_ONLY, ROS_DOMAIN_ID. No deletions or
# reorderings of the venv-set variables.

# Test 2 — sys.path order (venv first)
source /home/hailo/hailo-drone-follow/setup_env.sh
python -c "import sys; print('\n'.join(p for p in sys.path if 'site-packages' in p))"
# Expected output:
#   /home/hailo/hailo-drone-follow/hailo-apps/venv_hailo_apps/lib/python3.10/site-packages
#   /usr/lib/python3.10/dist-packages
#   /opt/ros/humble/lib/python3.10/site-packages           ← LAST
# (venv site-packages MUST appear before /opt/ros/humble/...)

# Test 3 — drone CLI smoke (no behavior change)
robot-follow --robot drone --help > /tmp/help_with_ros.txt
diff /tmp/help_with_ros.txt /tmp/help_baseline.txt  # baseline captured pre-ROS-install
# Expected: empty diff. Same flags, same ordering, no leakage of ROS env
# names into argparse help.

# Test 4 — rover CLI smoke (no drone flags, has rover scaffolding)
robot-follow --robot rover --help | grep -E "(--takeoff-landing|--target-altitude|--serial)"
# Expected: zero matches.

# Test 5 — drone runs unchanged (the real gate — SITL behavior)
# (Operator runs the Phase 3 SITL test from CONTEXT line 129.)
```

**If any of Test 1-4 fail:** ROS sourcing introduced a regression. Mitigation per CONTEXT line 148: sourcing order is venv-first; if `sys.path` shows ROS before venv, the order is wrong.

---

## Migration commit shape proposal

Phase 3 is structural — 5 layers (`follow_api` types, `robot_api` package, controller signature, CLI dispatch, `setup_env.sh`) plus tests. Per DESIGN-NOTES § Process risk 14 + CONTEXT line 220, the planner picks single-atomic vs wave-of-commits. Recommend **wave of 8 bisectable commits**, each keeping the existing 176-test suite green (the snapshot test is added incrementally; until commit 7 it's not a CI gate).

### Wave 0 — Test scaffolds

**Commit 1: `test(03): xfail-scaffold the snapshot + adapter unit-test files`**
- Add `robot_follow/tests/test_robot_command_snapshot.py` (xfail marker, skeleton test).
- Add `robot_follow/tests/test_mavsdk_drone_adapter.py` (xfail marker, skeleton).
- Add `robot_follow/tests/cases/__init__.py` and `cases/drone_command_baseline.py` (case list, expected_velocity_command empty for now).
- Modify nothing else. Suite stays green; xfails registered.

### Wave 1 — Types in `follow_api/types.py`

**Commit 2: `feat(03): add Axis + Capabilities + RobotCommand + SafetyContext to follow_api/types.py`**
- Edit `robot_follow/follow_api/types.py`: add the 4 new types. Keep `VelocityCommand` for now.
- Edit `robot_follow/follow_api/__init__.py`: re-export the new types.
- Snapshot test scaffold consumes the new types; xfail flips ok-but-still-incomplete.
- No production code change. Suite green.

### Wave 2 — Scaffolding for `robot_api/`

**Commit 3: `feat(03): scaffold robot_api/ package with Robot protocol + orchestrator stub`**
- Create `robot_api/__init__.py`, `robot_api/robot.py` (Protocol class), `robot_api/orchestrator.py` (skeleton — just `run_robot_loop` signature, body raises `NotImplementedError`), `robot_api/adapters/__init__.py`.
- `pyproject.toml` `[tool.setuptools] packages.include`: add `"robot_api*"`.
- No callers of `robot_api` yet. Suite green.

### Wave 3 — Move drone code into `robot_api/adapters/mavsdk_drone.py`

**Commit 4: `refactor(03): move drone_api/mavsdk_drone.py to robot_api/adapters/mavsdk_drone.py (MAVSDK code unchanged)`**
- `git mv robot_follow/drone_api/mavsdk_drone.py robot_follow/robot_api/adapters/mavsdk_drone.py`.
- Edit `robot_follow/drone_api/__init__.py` to re-export from the new location (backward-compat alias for callers that haven't been migrated yet).
- Update internal imports in the moved file (`from robot_follow.follow_api.types import VelocityCommand` etc.).
- All 176 tests still pass via the alias. Suite green.

### Wave 4 — Add `MavsdkDroneAdapter` wrapper class

**Commit 5: `feat(03): introduce MavsdkDroneAdapter wrapping live_control_loop + VelocityCommandAPI`**
- Edit `robot_follow/robot_api/adapters/mavsdk_drone.py`: add the `MavsdkDroneAdapter` class with all 6 protocol methods + caps. Initially, `send_command` just delegates to `VelocityCommandAPI.send` (no pure-function extracts yet); `start_session` wraps the current `_start_offboard` + telemetry-task spawn logic; `shutdown` wraps `_land_safely` + telem cancel.
- Extract the 4 pure functions (`_apply_altitude_p`, `_apply_retreat_from_tilt`, `_apply_smoothing`, `_compute_search_yawspeed`) as module-level functions, BUT keep `live_control_loop` calling them for now (so the wire behavior is unchanged).
- Add `robot_api/orchestrator.py` `run_robot_loop` body (pseudocode from CONTEXT lines 65-103).
- Update `test_mavsdk_drone_adapter.py` to fill in the ~30 unit tests for the four pure functions. The 46 tests in `test_velocity_api_and_smoother.py` continue passing because `VelocityCommandAPI` still exists.
- Suite stays green; new tests added.

### Wave 5 — Controller signature migration

**Commit 6: `refactor(03): controller.compute(detection, caps, config) → RobotCommand; orchestrator owns last_detection/search/hold`** (atomic per CLEAN-07 pattern)
- Edit `robot_follow/follow_api/controller.py`:
  - Rename function `compute_velocity_command` → `compute`.
  - Drop `last_detection`, `search_active`, `hold_velocity` args.
  - Add `caps: Capabilities` arg.
  - Return `RobotCommand` instead of `VelocityCommand`.
  - Gut `_apply_frame_edge_safety` to just emit `forward = 0` when in edge zone; the fade/retreat math is now in `_apply_retreat_from_tilt` (adapter-side).
- Edit `robot_follow/robot_api/orchestrator.py`: implement the full pseudocode (10 Hz tick, hold-velocity for `search_enter_delay_s`, then `on_target_lost`).
- Edit `robot_follow/robot_api/adapters/mavsdk_drone.py`: `MavsdkDroneAdapter.send_command(cmd, safety_ctx)` now applies all the relocated drone behaviors. `live_control_loop` is replaced (deleted) — `run_robot_loop` is the entry point.
- Edit `robot_follow/follow_api/__init__.py`: add `compute` to the re-exports; keep `compute_velocity_command` as a thin alias for backward-compat one more commit (delete in commit 8).
- Edit ALL 49 test invocations in `robot_follow/tests/test_controller.py`: add `DRONE_CAPS` arg.
- Edit ALL 54 `VelocityCommand(...)` constructions in `test_controller.py`, `test_velocity_api_and_smoother.py`, `test_velocity_command_shape.py`: replace with `RobotCommand(...)`.
- Snapshot test fixture (`drone_command_baseline.py`) gets its `expected_velocity_command` filled by a one-shot script run on the PRE-commit-6 tree.
- Snapshot test xfail removed; now a hard gate.
- This is the BIG commit — bisect-survival is critical.

### Wave 6 — Composition root + CLI

**Commit 7: `feat(03): --robot dispatch + run_robot() composition root + add_common/drone/rover_args split`**
- Edit `robot_follow/robot_follow_app.py`:
  - Add `--robot {drone,rover}` pre-parser (described in Argparse Splits section above).
  - Split `_add_app_args` into `add_common_args`. Keep ReID + UI + tracker flags here.
  - Move `add_drone_args` import from `drone_api` to `robot_api.adapters.mavsdk_drone`.
  - Add `add_rover_args` (placeholder body for Phase 4).
  - Rename `run_drone()` → `run_robot()`. Body instantiates `MavsdkDroneAdapter` (if `args.robot == "drone"`) or raises `NotImplementedError` for rover (Phase 4 fills in).
  - Pass adapter to `robot_api.orchestrator.run_robot_loop`.

### Wave 7 — Environment + cleanup

**Commit 8: `feat(03): setup_env.sh conditional ROS source + drop drone_api/ + delete VelocityCommand`**
- Edit `setup_env.sh`: append the conditional ROS-source block.
- Delete `robot_follow/drone_api/` directory (and the shim that re-exported from `robot_api.adapters.mavsdk_drone`).
- Delete `compute_velocity_command` alias from `follow_api/controller.py`; `follow_api/__init__.py` re-exports `compute`.
- Delete `VelocityCommand` from `follow_api/types.py` and `__init__.py`.
- Update `robot_follow/__init__.py` exports.
- Update `CLAUDE.md` to reference the new layout (drone_api → robot_api/adapters).
- This is the destructive cleanup commit. Snapshot test is the bisect anchor.

### Bisectability check

Every commit 1-8 must leave the working tree with:
- `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py` → 176 (or 176+new) PASS, 0 unexpected fail.
- `robot-follow --help` exits 0.
- `import robot_follow` and `import robot_follow.follow_api` succeed.

Commits 5 and 6 are the highest-risk. Recommend dry-running commit 6's diff once and inspecting the test fixup mechanically (49 `compute_velocity_command(` calls → `compute(` + caps arg) before committing.

---

## Validation Architecture

> Phase 3 requires `workflow.nyquist_validation` semantics: every ROADMAP success criterion maps to an automated command, and Wave 0 lists any gaps.

### Test Framework

| Property | Value |
|----------|-------|
| Framework | pytest (already in use; venv at `./hailo-apps/venv_hailo_apps`) |
| Config file | none — pytest uses default discovery |
| Quick run command | `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -x -q` |
| Full suite command | `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -v` |
| **Mandatory invocation** | `python -m pytest` — NEVER bare `pytest` (Phase 1 footgun documented in `01-VERIFICATION.md` Anti-Patterns section) |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|--------------|
| ABS-01 | `robot_api/robot.py` defines `Robot` protocol; types live in `follow_api/types.py` | unit | `python -c "from robot_follow.follow_api.types import Axis, Capabilities, RobotCommand, SafetyContext; from robot_follow.robot_api.robot import Robot; print('ok')"` | ❌ Wave 0 (add `test_robot_protocol_shape.py`) |
| ABS-02 | `RobotCommand` field shape; `VelocityCommand` removed | unit | `python -m pytest robot_follow/tests/test_robot_command_shape.py -v` | ❌ Wave 0 (rename from `test_velocity_command_shape.py`) |
| ABS-03 | `robot_api/adapters/mavsdk_drone.py` exists; `drone_api/mavsdk_drone.py` does not | unit | `python -c "import robot_follow.robot_api.adapters.mavsdk_drone; import robot_follow.drone_api" 2>&1; ` test that import 1 succeeds AND import 2 raises `ModuleNotFoundError` | ❌ Wave 0 (add `test_layout_smoke.py`) |
| ABS-04 | Altitude P-loop gates on `Axis.ALTITUDE in caps.axes` | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestApplyAltitudeP -v` | ❌ Wave 0 (file scaffolded in commit 1; tests filled in commit 5) |
| ABS-05 | Bottom-edge frame safety: controller emits 0, adapter applies retreat | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestApplyRetreatFromTilt -v` + `python -m pytest robot_follow/tests/test_controller.py::TestFrameEdgeBoundary -v` (renamed) | ❌ Wave 0 |
| ABS-06 | Search-mode yaw-spin lives in adapter; controller emits zero on lost | unit | `python -m pytest robot_follow/tests/test_mavsdk_drone_adapter.py::TestComputeSearchYawspeed -v` | ❌ Wave 0 |
| ABS-07 | `ControllerConfig` altitude fields are Optional; validate skips when ALTITUDE absent | unit | `python -m pytest robot_follow/tests/test_config_persistence.py::TestAltitudeOptional -v` (new) | ❌ Wave 0 (add ~3 tests to existing file) |
| ABS-08 | `run_robot()` exists; `run_drone()` does not | unit | `python -c "from robot_follow.robot_follow_app import run_robot; assert not hasattr(__import__('robot_follow.robot_follow_app', fromlist=['']), 'run_drone')"` | ❌ Wave 0 |
| ABS-09 | `--robot drone --help` shows drone flags; `--robot rover --help` does not | smoke | See "ABS-09 verification" snippet in Argparse Splits section — `robot-follow --robot drone --help \| grep takeoff-landing` returns 1 hit; `robot-follow --robot rover --help \| grep takeoff-landing` returns 0 hits | ❌ Wave 0 (add `test_cli_help_dispatch.py`) |
| ABS-10 | `setup_env.sh` sources ROS when `/opt/ros/humble/setup.bash` exists; sub-shell isolation | smoke | `bash -c 'source setup_env.sh && env'` — assert `ROS_DISTRO=humble` is set IFF `/opt/ros/humble/setup.bash` exists | ❌ Wave 0 (add `test_setup_env_sh.py`) — file-based check possible without ROS installed |
| ABS-11 | Full SITL drone follow-the-person passes with `--robot drone` | **integration (manual-only)** | Operator runs: `bash sim/start_sim.sh --bridge --world walk_across_then_approach` + `source setup_env.sh && drone-follow --robot drone --input udp://0.0.0.0:5600 --takeoff-landing --webui`; observes drone follows the walking actor for the full pattern without losing target. **Cannot be automated in CI** — requires Hailo HW + Gazebo + GPU. | n/a (`human_needed` gate per Phase 1 Verification A pattern) |
| **Snapshot gate (Phase-3-only)** | Drone command sequence byte-equivalent pre/post refactor | unit | `python -m pytest robot_follow/tests/test_robot_command_snapshot.py -v` — all ~100 cases pass | ❌ Wave 0 (scaffold in commit 1; expected values captured in commit 6) |

### Sampling Rate

- **Per task commit:** `python -m pytest robot_follow/tests/test_controller.py robot_follow/tests/test_mavsdk_drone_adapter.py robot_follow/tests/test_robot_command_snapshot.py -x -q` (~3 s)
- **Per wave merge:** `python -m pytest robot_follow/tests --ignore=robot_follow/tests/test_sim_worlds.py -x -q` (~13 s; matches Phase 2 verifier baseline)
- **Phase gate:** Full suite green AND operator SITL run signed off (per CONTEXT line 129) — `human_needed` until operator approves

### Wave 0 Gaps

- [ ] `robot_follow/tests/test_robot_command_snapshot.py` — covers ABS-01..06 controller-output equivalence (~100 cases)
- [ ] `robot_follow/tests/test_mavsdk_drone_adapter.py` — covers ABS-04, ABS-05, ABS-06 via pure-function unit tests (~30 cases) + integration test with mock MAVSDK system
- [ ] `robot_follow/tests/cases/__init__.py` + `cases/drone_command_baseline.py` — fixture file with case definitions (~100 BaselineCase entries)
- [ ] `robot_follow/tests/test_robot_protocol_shape.py` — asserts Robot has 6 methods + caps attribute (ABS-01)
- [ ] `robot_follow/tests/test_layout_smoke.py` — asserts `robot_api.adapters.mavsdk_drone` imports + `drone_api` module is GONE (ABS-03)
- [ ] `robot_follow/tests/test_cli_help_dispatch.py` — runs `subprocess.run(["robot-follow", "--robot", "rover", "--help"])` and greps for drone flag absence (ABS-09)
- [ ] `robot_follow/tests/test_setup_env_sh.py` — sources `setup_env.sh` in a sub-shell, asserts env after sourcing (ABS-10)
- [ ] Rename `test_velocity_command_shape.py` → `test_robot_command_shape.py` (ABS-02)
- [ ] Migration of 46 tests in `test_velocity_api_and_smoother.py` → new `test_mavsdk_drone_adapter.py::TestApplySmoothing` (ABS-05 smoothing path; functional equivalence)
- [ ] Migration of 49 `compute_velocity_command(` calls in `test_controller.py` → `compute(` with `DRONE_CAPS` arg (mechanical edit; happens in commit 6)

Framework already installed (pytest in `./hailo-apps/venv_hailo_apps`). No new install step required.

---

## Open Questions for Planner

1. **`ui_state.update_velocity` / 1 Hz status log: orchestrator or adapter?** Today `live_control_loop` reads `altitude_cache` and `telemetry_cache` for the periodic status banner (`mavsdk_drone.py:550-572`). Post-refactor these caches are adapter-private. Options: (a) orchestrator queries adapter via public methods (`adapter.altitude_m()`, `adapter.velocity_ned()`) — clean but adds protocol surface; (b) adapter handles all logging itself — duplicated for rover; (c) pass a logger callback into the adapter. Recommend (a) for the drone-specific log but only add `altitude_m()` / `velocity_ned()` to `MavsdkDroneAdapter` as extra methods, NOT to the `Robot` protocol. Orchestrator calls `if hasattr(robot, 'altitude_m'): alt = robot.altitude_m()` for the optional drone-info log line.

2. **`mission_duration` watchdog ownership.** Today `live_control_loop` runs inside an outer `asyncio.wait(..., asyncio.sleep(args.mission_duration))` at `mavsdk_drone.py:769-775, 798-805`. Drone-specific (CLEAN-04). Orchestrator-level or adapter-level? Recommend **orchestrator** owns the wait, but the timeout value is sourced from `args.mission_duration` which is registered in `add_drone_args` — rover args register no equivalent. Orchestrator default: `getattr(args, "mission_duration", math.inf)`.

3. **`ControllerConfig` Optional altitude fields (ABS-07).** Current `min_altitude`, `max_altitude`, `target_altitude` are `float` with non-None defaults. Making them `Optional[float]` and gating `validate()` is per ABS-07, but the CLI flags `--min-altitude`, `--max-altitude`, `--target-altitude` would need their defaults removed (else rover users get drone defaults silently set). Recommend: keep CLI defaults for drone-only flags (registered via `add_drone_args` for `--target-altitude` and via `add_common_args` for the rest); when `--robot rover`, `from_args` sets altitude fields to None.

4. **Offboard-lost re-entry: adapter-internal or orchestrator-visible?** Today `mavsdk_drone.py:790-834` runs an inner `while not shutdown` loop that re-handshakes offboard whenever the pilot drops out. Two options: (a) adapter handles re-entry internally inside `send_command` (raises a tagged exception, catches it, re-handshakes, retries; orchestrator never sees it) — clean but adapter has implicit retry state; (b) adapter raises a typed exception, orchestrator catches and re-calls `start_session()` — explicit but adds protocol surface. Recommend (a) — orchestrator stays robot-agnostic.

5. **`yaw_unit` conversion location (rover).** CONTEXT line 168: "Adapter normalizes at the wire" — controller emits in `caps.yaw_unit`. ROVER-06: "RobotCommand.yaw (deg/s) is converted to rad/s before assignment to `Twist.angular.z`" — controller always emits in deg/s. **Contradiction.** Recommend controller emits in `caps.yaw_unit` (CONTEXT-aligned), and ROVER-06 is re-read as "the rover adapter receives `yaw_rate` already in rad/s — no conversion needed at the wire." Planner must lock this before Phase 4 starts.

6. **`SafetyContext.lost()` neutral bbox values.** Proposed `(0.5, 0.25)` keeps the bbox safely inside the frame, but a different sentinel (e.g. `(0.0, 0.0)` plus an adapter-side `if safety_ctx.target_lost: skip retreat overlay` check) is cleaner. The adapter's bottom-edge fade depends on a real bbox position; with `target_lost=True` the fade has no meaning. Recommend the adapter early-return on `target_lost=True` instead of computing fade math against placeholder values.

7. **`asyncio` task isolation: drone thread vs orchestrator main loop.** Today `run_drone()` runs in a daemon thread with its own asyncio event loop (`robot_follow_app.py:386-400`). Post-Phase-3 `run_robot()` could either (a) keep the same daemon-thread shape — minimal change, lowest risk; or (b) hoist into the main async loop — cleaner but touches the SIGINT/Gst pipeline interaction. Recommend (a) — keep the daemon thread; Phase 3's job is the actuator boundary, not the threading model.

8. **`DRONE_CAPS` import location for tests.** The snapshot fixture and the controller tests both need `DRONE_CAPS`. Define it at module level in `robot_api/adapters/mavsdk_drone.py` and import from there, OR in `follow_api/types.py` alongside `Capabilities` (canonical constants module). Slight preference for `mavsdk_drone.py` — keeps `follow_api` types-only, no per-adapter constants.

---

## Sources

### Primary (HIGH confidence) — file:line references verified on HEAD = 9ba4ec7

- `robot_follow/drone_api/mavsdk_drone.py:1-847` — full live_control_loop, VelocityCommandAPI, telemetry tasks, offboard handshake
- `robot_follow/follow_api/controller.py:1-181` — current compute_velocity_command + _apply_frame_edge_safety
- `robot_follow/follow_api/types.py:1-27` — VelocityCommand + Detection
- `robot_follow/follow_api/config.py:1-376` — ControllerConfig + tunable_fields + add_args
- `robot_follow/follow_api/state.py:1-101` — SharedDetectionState + FollowTargetState
- `robot_follow/robot_follow_app.py:1-455` — composition root, single pre-parser at :213, _build_parser at :159
- `robot_follow/tests/test_controller.py:1-518` — 49 compute_velocity_command call sites mapped
- `robot_follow/tests/test_velocity_api_and_smoother.py:1-251` — 46 VelocityCommand constructions + smoothing test patterns
- `robot_follow/tests/test_velocity_command_shape.py` — 1 VelocityCommand construction
- `robot_follow/pipeline_adapter/tracker_factory.py:1-47` — add_tracker_args definition (common-args candidate)
- `setup_env.sh:1-37` — current sourcing flow (venv → no ROS)
- `.planning/research/PITFALLS.md:11-43` — rclpy signal handler + venv/ROS sourcing order pitfalls (Pitfalls 1 and 2)
- `.planning/phases/03-abstraction/03-CONTEXT.md` — locked decisions, R1-R5 revisions
- `.planning/phases/03-abstraction/03-DESIGN-NOTES.md` — architecture diagram, change summary, risks 1-16
- `.planning/REQUIREMENTS.md` — ABS-01..11, ROVER-01..08
- `.planning/phases/02-cleanup/02-VERIFICATION.md` — CLEAN-12 (single pre-parser at :213), CLEAN-13 (telemetry merge), CLEAN-14 (tunable_fields)
- `.planning/phases/02-cleanup/deferred-items.md` — DEFER-02-00-A resolved on 2026-05-17 (test_controller.py is fully green at HEAD)

### Secondary (MEDIUM confidence)

- DESIGN-NOTES § "Why 'always source ROS if installed'" + PITFALLS § Pitfall 2 — cross-verified the venv-first sourcing order rationale
- ROVER-02/03/04/05/06/07/08 requirement statements — used to validate Robot protocol fit for the paper-sketch Ros2RoverAdapter

### Tertiary (LOW confidence)

- None. Every claim in this RESEARCH.md is grounded in a file:line citation against HEAD = 9ba4ec7.

---

## Metadata

**Confidence breakdown:**
- Concern mapping (live_control_loop → home): HIGH — every line of the current loop has a verified destination
- VelocityCommandAPI method mapping: HIGH — 1:1 method → method correspondence verified
- Argparse splits: HIGH — every flag classified against verified file:line source
- setup_env.sh block: HIGH — PITFALLS.md authoritatively covers ordering; ROS not installed on this dev box (verified) so the block is safe-by-default
- Ros2RoverAdapter paper sketch: MEDIUM — protocol fit verified, but yaw_unit contradiction (open question 5) blocks final lock
- Snapshot fixture design: HIGH — case categories verified against today's test_controller coverage
- Pure-function extracts: HIGH — each extract's body lifted verbatim from a verified source location
- SafetyContext derivation: HIGH — every adapter use of Detection mapped
- Connection failure trace: HIGH — current behavior verified line-by-line
- Migration commit shape: MEDIUM — bisectability is plausible but depends on planner discipline in commit 6 (the big one)
- Validation Architecture: HIGH — automated commands runnable today (subject to Wave 0 files existing)

**Research date:** 2026-05-17
**Valid until:** 30 days (stable codebase, no fast-moving external deps in scope)
