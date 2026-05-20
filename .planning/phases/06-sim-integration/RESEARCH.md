# Phase 6: Sim integration — Research

**Researched:** 2026-05-20
**Domain:** wiring rover adapter + rover sim into one end-to-end follow loop with rover-safe defaults
**Confidence:** HIGH — all upstream pieces (Phase 4 adapter, Phase 5 sim) are landed and operator-verified on this dev box; the work is config, a 4-line tracker refactor, a docs append, and two tests

---

## Summary

Phase 6 is **not** new architecture. Phases 3-5 already built the load-bearing pieces:

- `Ros2RoverAdapter` (Phase 4, all 5 plans complete) — implements `Robot`, publishes `Twist` on `/cmd_vel`, `SignalHandlerOptions.NO`, idempotent shutdown.
- Rover sim (Phase 5, operator-verified 2026-05-20) — `sim/rover/{model.sdf, worlds/*.sdf, start_rover_sim.sh}`, `install.sh --rover`, `video_bridge.py` reused verbatim, camera feed on UDP 5600.
- Controller, `RobotCommand`, `SafetyContext`, `run_robot_loop` are already robot-agnostic (Phase 3); the orchestrator dispatches to the right adapter via `--robot rover`.

Phase 6 closes the loop with **six small landings**:

1. **RINT-01** — `configs/rover_simulation.json` (a JSON file mirroring `sim/configs/simulation*.json`).
2. **RINT-02** — bottom-edge safety per adapter: the drone path is already correct; the rover adapter publishes `Twist.linear.x = forward_m_s` and that's already a natural "slow/stop" because the controller emits low/zero forward when the bbox is at the bottom margin. **Net new rover code: zero.** What we add is a test asserting this natural behavior.
3. **RINT-03** — extract 4 hardcoded ByteTracker args (`track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30` at `hailo_drone_detection_manager.py:1281`) into `ControllerConfig` fields. Drone defaults preserve current values byte-identically; rover config overrides `track_buffer=30`.
4. **RINT-04** — deterministic E2E test mirroring `test_sim_worlds.py::test_walk_across_then_approach_holds_target_through_approach`, but driving `start_rover_sim.sh` instead of PX4 SITL and `--robot rover` instead of the default drone.
5. **RINT-05** — append a port-isolation section to `sim/rover/README.md`. (Already partially present — see "Port 5600 conflict with PX4 SITL" — verify it satisfies RINT-05's wording before extending.)
6. **RINT-06** — SIGINT shutdown integration test using the existing `rclpy_mock` fixture from `test_ros2_rover_adapter.py`.

**Primary recommendation:** Treat Phase 6 as a *config + tests + small refactor* phase, not a build phase. The hard system-design work is behind us; the gate is RINT-04 (E2E follows-the-actor) and RINT-06 (clean SIGINT shutdown).

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Rover-safe gains + ByteTracker buffer | `configs/rover_simulation.json` (data) → `ControllerConfig` (load) | controller / detection manager (consume) | Pure tuning; no code logic per-robot. ControllerConfig stays a flat dataclass. |
| Bottom-edge safety reaction | Adapter (`MavsdkDroneAdapter` / `Ros2RoverAdapter`) | Controller (raw output) | ABS-05 lock: per-robot behavior lives in adapter, not controller, not Capabilities. |
| ByteTracker construction | `pipeline_adapter/hailo_drone_detection_manager.py` `create_app()` | `ControllerConfig.bytetracker_*` fields | Read once at app creation; not a tunable_field (no live mutate). |
| E2E follow validation | `robot_follow/tests/test_sim_worlds.py` (rover variant added) | `sim/rover/start_rover_sim.sh` driver | Mirrors drone-side pattern: subprocess sim + drone-follow, capture JSONL via `--test-log`, assert. |
| SIGINT shutdown correctness | `Ros2RoverAdapter.shutdown` (already correct) | New test in `test_ros2_rover_adapter.py` | Adapter is the source of truth; test guards it. |
| Port-isolation docs | `sim/rover/README.md` (append section if not already sufficient) | `.planning/research/PITFALLS.md` (existing integration-gotcha row) | Docs, not code. |

---

## User Constraints (from ROADMAP.md Phase 6 success criteria — no Phase 6 CONTEXT.md exists yet)

> Phase 6 has no `06-CONTEXT.md` (operator did not run `/gsd:discuss-phase` for Phase 6). The
> roadmap's success criteria stand in. Locked decisions below are extracted from CLAUDE.md,
> REQUIREMENTS.md, the design review, and the axes-only memory pin
> (`feedback_robot_abstraction_axes_only.md`).

### Locked decisions

- **`robot_api/adapters/mavsdk_drone.py` is byte-identical at end of phase.** The drone retreat-from-tilt path stays exactly as Phase 3 left it. Verified via `git diff HEAD~ -- robot_follow/robot_api/adapters/mavsdk_drone.py` being limited to ByteTracker config reads (RINT-03), nothing else.
- **`Capabilities` stays axes-only.** No `bottom_edge_policy`, no `slow_or_stop`, no behavioral flags added. Per ABS-05 + the axes-only memory pin.
- **`ControllerConfig` stays a pure-data dataclass.** No methods that import `rclpy`, `gz`, MAVSDK, or any I/O. Added fields are plain `float`/`int`/`Optional[float]`.
- **Phase 5 artifacts (`sim/rover/{model.sdf, worlds/*.sdf, start_rover_sim.sh, install.sh --rover branch}`) are byte-identical** except the `README.md` may receive a RINT-05 append. Verified via `git diff HEAD~ -- sim/rover/` showing only README changes.
- **Drone defaults preserve current ByteTracker values byte-identically:** `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30`. Confirmed against `hailo_drone_detection_manager.py:1281` and `tracker_factory.create_tracker` defaults (`track_buffer=90`, `match_thresh=0.5`, `frame_rate=30`).
- **Controller's contribution is unchanged.** Per RINT-02 wording. The drone adapter's `_apply_retreat_from_tilt` (the actual zeroing/fade implementation) remains untouched.
- **Drone behavior at controller call site unchanged.** RINT-02 success criterion: with `--robot drone` the bottom-edge still triggers retreat-from-tilt with byte-identical fade/push/deadband math.

### Claude's discretion

- Exact JSON key set in `configs/rover_simulation.json` (recommendations in § RINT-01 below).
- Whether RINT-04 lives in `test_sim_worlds.py` (one file, drone + rover side-by-side) or `test_rover_sim_worlds.py` (separate file). See Open Question Q2.
- Whether RINT-06 lives in `test_ros2_rover_adapter.py` (extend existing) or `test_rover_shutdown.py` (new file). See Open Question Q3.
- Whether the new `ControllerConfig.bytetracker_*` fields are exposed in `tunable_fields()` (recommendation: NO — tracker is constructed once at pipeline init, not live-mutated).

### Out of scope (deferred)

- **`configs/drone_simulation.json`** — the roadmap success criterion says "(or equivalent)"; the equivalents are `sim/configs/simulation.json` + `sim/configs/simulation_follow.json` and they already exist and load without errors. No new top-level drone-simulation config needed for Phase 6.
- **`--rover-video-port` flag** to remap the rover sim's UDP 5600 → 5610 — defer to v1.2 per RINT-05; document the constraint and move on.
- **Live ByteTracker re-tuning from the web UI** — out of scope; tracker is constructed once at pipeline init.
- **Rover hardware bring-up** — v1.2 milestone (HW-01..09).
- **Latency shim on `/cmd_vel`** (PITFALLS Pitfall 8) — recommended for v1.2 hardware-bringup phase, NOT for Phase 6 (the v1.1 milestone goal is "rover follows actor end-to-end in sim", not "calibrated to hardware").

---

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| RINT-01 | `configs/rover_simulation.json` with rover-safe defaults, no altitude knobs, lower `kp_yaw`, no `max_forward_accel` slew cap | § RINT-01 below + ControllerConfig field audit |
| RINT-02 | Bottom-edge frame safety per adapter (drone unchanged; rover stop/slow); controller's contribution unchanged | § RINT-02 below + § Open Question Q1 |
| RINT-03 | ByteTracker knobs config-driven; drone defaults preserved; rover `track_buffer` ≈ 30 | § RINT-03 below — exact 4-arg refactor at `hailo_drone_detection_manager.py:1281` |
| RINT-04 | E2E test: rover follows walking actor in `walk_across_then_approach` world for full pattern | § RINT-04 below + reference pattern in `test_sim_worlds.py:434` |
| RINT-05 | Port-isolation docs: PX4 SITL + rover sim cannot coexist on same machine (UDP 5600 clash) | § RINT-05 below — README already has a section; verify wording satisfies the requirement |
| RINT-06 | SIGINT integration test: zero `/cmd_vel` after Ctrl+C within 100 ms; clean rclpy shutdown | § RINT-06 below + `rclpy_mock` fixture pattern from `test_ros2_rover_adapter.py:16-44` |

---

## File-by-File Change Inventory

### NEW: `configs/rover_simulation.json`

Mirror the shape of `sim/configs/simulation.json` (top-level JSON object, `ControllerConfig.from_json` filters unknown keys by `{f.name for f in fields(cls)}`). Verified at `follow_api/config.py:213-217`.

```json
{
  "yaw_only": false,
  "kp_yaw": 3.0,
  "max_forward": 1.0,
  "max_backward": 1.0,
  "max_forward_accel": 0,
  "target_bbox_height": 0.25,
  "bottom_margin_safety": 0.25,
  "top_margin_safety": 0.10,
  "smooth_yaw": true,
  "yaw_alpha": 0.3,
  "smooth_forward": true,
  "forward_alpha": 0.15,
  "search_timeout_s": 60.0,
  "log_verbosity": "normal",
  "bytetracker_track_thresh": 0.4,
  "bytetracker_track_buffer": 30,
  "bytetracker_match_thresh": 0.5,
  "bytetracker_frame_rate": 30
}
```

Notes for the planner:
- **`yaw_only: false`** — rover should actually drive forward (drone sim has yaw-only=true). RINT-04 needs forward motion to follow the actor.
- **`kp_yaw: 3.0`** — lower than the drone's 4.0 / sim's 5.0. Rationale: rovers see narrower yaw bandwidth than tilt-rotor drones, and `Twist.angular.z` units are rad/s (a rover at `kp_yaw=5` with sqrt-error scaling could spin alarmingly fast).
- **`max_forward: 1.0`** — rover-safe walking pace, slower than the drone's 1.5. The rover SDF (`sim/rover/rover/model.sdf`) has DiffDrive `<max_linear_acceleration>` and `<max_linear_velocity>` settings; verify the chosen `max_forward` does not exceed them (out of scope for this RESEARCH — the rover sim runs at default DiffDrive parameters; planner can grep the SDF if uncertain).
- **`max_forward_accel: 0`** — disables the slew-rate cap. The slew cap exists to tame PX4 pitch transients on the drone; rovers don't tilt. The smoothing layer (`_apply_smoothing` in `mavsdk_drone.py:237`) checks `config.max_forward_accel > 0 and config.control_loop_hz > 0`, so `0` cleanly disables it without code changes. **Important:** `_apply_smoothing` is in `robot_api/adapters/mavsdk_drone.py` and runs only for the drone path — the rover adapter doesn't call it (see `ros2_rover.py:112-124` `send_command`). The rover config field exists for completeness / documentation, but is effectively unused on the rover path. [VERIFIED: code grep]
- **`bottom_margin_safety: 0.25, top_margin_safety: 0.10`** — same as drone defaults. The rover doesn't read them today (the fade lives in `_apply_retreat_from_tilt`, which is drone-only). They are inert on the rover path; included so the JSON is shape-complete and so a future rover bottom-edge handler (if RINT-02 grows code) reads them naturally.
- **Altitude knobs (`target_altitude`, `min_altitude`, `max_altitude`, `kp_alt_hold`, `max_climb_speed`, `max_down_speed`)** — OMITTED. They're `Optional[float]` per ABS-07; `ControllerConfig.from_json` will leave them at the dataclass defaults (which are not-None numbers today, but `validate()` skips altitude checks when `caps` is provided AND `ALTITUDE not in caps.axes`). Per `config.py:122-126`.

  **Subtle gotcha to surface in the planner:** if the rover config is loaded via `ControllerConfig.from_json("configs/rover_simulation.json")` with no `caps` argument, `__post_init__` calls `validate()` with `caps=None`, which validates altitude unless any altitude field is None (`config.py:128-131`). The dataclass defaults are `target_altitude=3.0, min_altitude=2.0, max_altitude=4.0` — all non-None — so altitude validation runs and *passes* (3.0 is between 2.0 and 4.0). This is fine on the rover path today, but if a future change tightens the defaults so they're inconsistent, the rover config load would error. **Recommended:** add an `Axis` test that confirms a rover ControllerConfig loaded from JSON validates clean both with `caps=None` (legacy callers) and with `caps=ROVER_CAPS` (explicit).

- **`bytetracker_*` keys** — depend on RINT-03 landing first (or atomic same-plan). They're rover-tuned (`track_buffer=30` = 1 s at 30 fps, vs drone 90 = 3 s; rover loses targets less often through occlusion because ground perspective has fewer occluders).

### MODIFY: `robot_follow/follow_api/config.py`

Add 4 fields to `ControllerConfig` for RINT-03. **Drone-default values preserved byte-identically** from the current hardcoded call at `hailo_drone_detection_manager.py:1281` and the factory defaults at `tracker_factory.py:19-21`.

```python
# --- Tracker (ByteTracker) ---
# Hardcoded in hailo_drone_detection_manager.py:1281 pre-Phase-6;
# extracted to config so rover sim can override (RINT-03).
bytetracker_track_thresh: float = 0.4    # detection conf floor
bytetracker_track_buffer: int = 90       # frames to keep lost tracks (drone: 3 s @ 30 fps)
bytetracker_match_thresh: float = 0.5    # IoU matching threshold
bytetracker_frame_rate: int = 30         # pipeline fps for the tracker
```

`add_args()` registration is optional. Recommendation: **do NOT add CLI flags** for these — they're rarely tuned ad-hoc and registering 4 more `--bytetracker-*` flags clutters `--help`. If a developer needs to A/B test, they edit the JSON config. Keeps the `--help` blast radius zero.

`tunable_fields()` schema (`config.py:148-197`) — **do NOT add** the bytetracker fields. Web UI / OpenHD live-mutate the controller config every tick; the tracker is constructed once at pipeline init and never reads `ControllerConfig` again. Adding them would be misleading (the UI slider would have no effect).

`from_args()` and `_arg()` wiring (`config.py:355-399`) — add the 4 fields to the `_arg(..., default=defaults.bytetracker_*)` block so the existing JSON-load + CLI-override semantics carry through. If no CLI flags are registered, the `getattr(args, "bytetracker_*", None)` returns `None` and the default falls through, so this is mostly cosmetic but kept for consistency.

### MODIFY: `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` line 1281

Current (3 lines):

```python
_inner_tracker = create_tracker(
    _tracker_name,
    track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
)
```

Target shape (RINT-03):

```python
# The composition root attaches controller_config to user_data AFTER create_app
# returns (robot_follow_app.py:452); at THIS point we're still inside create_app,
# so controller_config is not yet bound on user_data. Pass it through create_app's
# signature instead — see "Wiring" below.
_inner_tracker = create_tracker(
    _tracker_name,
    track_thresh=controller_config.bytetracker_track_thresh,
    track_buffer=controller_config.bytetracker_track_buffer,
    match_thresh=controller_config.bytetracker_match_thresh,
    frame_rate=controller_config.bytetracker_frame_rate,
)
```

**Wiring:** `create_app(...)` signature at `hailo_drone_detection_manager.py:696-700` currently has no `controller_config` kwarg (CLEAN-10 removed it because no caller passed it; now Phase 6 needs to re-add it for tracker construction).

The cleanest landing:
1. Add `controller_config: Optional["ControllerConfig"] = None` to `create_app`'s signature.
2. Inside `create_app`, before the `create_tracker` call, fall back to a stock `ControllerConfig()` when None (preserves byte-identical behavior for any legacy caller that doesn't pass it).
3. In `robot_follow_app.py:431`, pass `controller_config=ControllerConfig.from_args(args)`. **But notice:** `from_args(args)` is currently called at line 448 (AFTER `create_app`), because it reads parsed args from `args = app.options_menu`. This is a chicken-and-egg: `create_app` builds the parser and parses args; `controller_config` derives from those parsed args.

   **Two cleanup paths the planner can pick from:**

   - **Path A (minimal):** Construct a *first-pass* `ControllerConfig.from_args(pre_args)` immediately after the pre-parse at `robot_follow_app.py:362-405` (before `create_app`), pass it into `create_app`, and re-derive the final config at line 448. The first-pass config supplies tracker knobs; the final config supplies everything else. Risk: two ControllerConfig objects float around briefly; the one used for tracker init is the first-pass one. Acceptable because tracker init reads only the 4 bytetracker_* fields, all of which can be read from `--config` JSON without conflict.
   - **Path B (cleaner):** Hoist the `ControllerConfig.from_args(args)` call before `create_app` by parsing args with a full parser at the pre-parse stage. Bigger refactor. Defer to a follow-up.

   **Recommendation: Path A.** The atomicity is the tracker init in `create_app` — extracting it from `create_app` would mean reshaping `DroneFollowTilingApp.__init__` (because the tracker is on `user_data`), which is a much larger blast radius.

**Verified preservation:** Drone path (`--robot drone`, no `--config`) → `ControllerConfig()` defaults → tracker constructed with `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30`. Byte-identical to current behavior.

### MODIFY: `robot_follow/robot_api/adapters/ros2_rover.py` — **probably zero changes** (see RINT-02 below)

The existing `send_command` at `ros2_rover.py:112-124` already does the right thing for RINT-02:

```python
async def send_command(self, cmd, safety_ctx):
    if safety_ctx.target_lost:
        return
    if self._publisher is None:
        return
    twist = self._Twist()
    twist.linear.x = cmd.forward_m_s
    twist.angular.z = cmd.yaw_rate
    self._publisher.publish(twist)
```

When the bbox bottom is in the bottom-margin region, the controller's `_compute_forward` (at `controller.py:53-74`) emits a *low* `forward_m_s` (because the person is close = `factor < 0` = retreat gain `kp_distance_back`). If the bbox crosses `max_bbox_height_safety = 0.8`, the controller emits `forward_m_s = -max_backward` (the panic branch). For the rover:
- Low forward → low `Twist.linear.x` → slow rover.
- Negative forward → negative `Twist.linear.x` → rover backs up.
- The drone's fade-and-push gradient (`_apply_retreat_from_tilt` in `mavsdk_drone.py:108-186`) does NOT run on the rover path because the rover adapter doesn't call it. That's correct — fade-and-push is tilt physics, irrelevant to rovers.

**Net new rover-adapter code for RINT-02: zero.** What we add is a test that asserts this natural behavior — see § Test Strategy below.

If a planner reviewer wants explicit "slow-when-bottom-margin" reactive logic in the rover adapter (rather than relying on the controller's natural distance-P retreat), that's net-new behavior beyond the requirement and should be opened as a separate ticket. See Open Question Q1 to lock this with the user before plan creation.

### UNCHANGED: `robot_follow/robot_api/adapters/mavsdk_drone.py` — byte-identical lock

The drone-side retreat-from-tilt path stays exactly as Phase 3 left it. The only file modification in Phase 6 across `robot_api/` is potentially zero (if the planner chooses to read `controller_config.bytetracker_*` in the detection manager only, not in the adapter).

**Verification gate:** at end of Phase 6, `git diff HEAD~N -- robot_follow/robot_api/adapters/mavsdk_drone.py` (where N = number of Phase 6 commits) should show NO changes. Plan-checker should add this as a pre-merge guard.

### NEW (or extended): `robot_follow/tests/test_sim_worlds.py` rover variant — RINT-04

The drone test pattern is already established (lines 313-513 of `test_sim_worlds.py`). The rover variant mirrors it. See § RINT-04 below.

### NEW: Rover shutdown integration test — RINT-06

Recommendation: extend `test_ros2_rover_adapter.py` with a new test class `TestSigintShutdown` (not a new file — keeps the rover-adapter contract surface in one place). See § RINT-06 below.

### APPEND (likely): `sim/rover/README.md` — RINT-05

The existing README already has a "Port 5600 conflict with PX4 SITL" section at lines 126-141. Verify it covers RINT-05's required ground:
- ✅ Both sims bind UDP 5600
- ✅ Cannot run simultaneously
- ✅ Future fix (`--rover-video-port`-style remap) is a v1.2 concern
- ❓ Missing: explicit mention that PX4 SITL also uses MAVLink 14540 (rover doesn't — only `/cmd_vel` over ROS DDS)

Plan should diff the existing section against RINT-05's wording and append a short table of port-usage if the planner deems it insufficient. Likely a 3-5-line append, not a full new section.

---

## RINT-01 — `configs/rover_simulation.json` exact key list

### ControllerConfig fields the rover config MAY set (rover-relevant)

From `config.py:36-107` field audit:

| Field | Rover-relevant? | Suggested rover value | Rationale |
|-------|-----------------|----------------------|-----------|
| `hfov` | yes | omit (66.0 default) | Camera FOV; depends on rover SDF camera config, not robot type |
| `kp_yaw` | YES | `3.0` (lower than drone 4.0) | rad/s on rover via Twist.angular.z |
| `dead_zone_deg` | yes | omit (2.0 default) | Yaw dead-zone OK at default |
| `max_yawspeed` | yes | omit (90.0 default in deg/s — but rover uses rad/s; need to verify) | **OPEN:** `max_yawspeed` is in deg/s in drone path. On rover, the smoothing/clamping at `_apply_smoothing:203` uses it but rover doesn't call `_apply_smoothing`. So this is a drone-side knob that the rover ignores. Setting it in the rover config is inert. Recommendation: OMIT. |
| `max_forward` | YES | `1.0` | Rover-safe top speed in m/s |
| `max_backward` | YES | `1.0` | Rover-safe retreat speed in m/s |
| `max_forward_accel` | YES | `0` (disable) | Slew cap is tilt-transient safety; rovers don't tilt |
| `kp_distance` | yes | omit (0.8 default) | Distance-P approach gain; controller-internal |
| `kp_distance_back` | yes | omit (2.5 default) | Distance-P retreat gain; same shape on rover |
| `target_bbox_height` | YES | `0.25` (same as drone) | Setpoint for distance-P |
| `dead_zone_bbox_percent` | yes | omit (10.0 default) | Distance dead-zone OK at default |
| `top_margin_safety` | yes | `0.10` (same as drone; inert on rover today) | Future RINT-02 rover handler may use it |
| `bottom_margin_safety` | yes | `0.25` (same as drone; inert on rover today) | Same |
| `max_bbox_height_safety` | yes | omit (0.8 default) | Panic-retreat threshold; controller emits `-max_backward` at this size — natural rover stop/reverse |
| `yaw_only` | YES | `false` | Rover must drive forward to follow |
| `auto_select` | yes | omit (true default) | Generic AUTO mode behavior, robot-agnostic |
| `detection_timeout_s` | yes | omit (0.5 default) | |
| `search_enter_delay_s` | yes | omit (2.0 default) | |
| `search_timeout_s` | YES | `60.0` | Match sim configs convention |
| `search_yawspeed_slow` | yes | omit (10.0 default) | deg/s on drone, but rover units are rad/s — inert on rover path (rover's `on_target_lost` publishes zero Twist, not a search-spin) |
| `control_loop_hz` | yes | omit (10.0 default) | Robot-agnostic |
| `smooth_yaw` / `yaw_alpha` | YES (drone-only smoother) | `smooth_yaw: true, yaw_alpha: 0.3` | Inert on rover (no smoother call). Included for shape parity. |
| `smooth_forward` / `forward_alpha` | YES (drone-only smoother) | `smooth_forward: true, forward_alpha: 0.15` | Same |
| `forward_velocity_deadband` | yes | omit (0.05 default) | Drone-only (applied inside `_apply_retreat_from_tilt`) |
| `smooth_down` / `down_alpha` | drone-only | omit | Drone has altitude axis; rover doesn't |
| `target_altitude` | drone-only | OMIT — Optional[float] | Falls through to dataclass default 3.0 |
| `min_altitude` / `max_altitude` / `kp_alt_hold` / `max_climb_speed` / `max_down_speed` | drone-only | OMIT — all Optional[float] | Same |
| `log_verbosity` | yes | `"normal"` | Robot-agnostic |
| `bytetracker_track_thresh` (RINT-03 new) | YES | `0.4` (same as drone for safety) | Detection conf floor; lowering risks more spurious tracks |
| `bytetracker_track_buffer` (RINT-03 new) | YES | `30` (vs drone 90) | 1 s @ 30 fps; rover loses targets less often than airborne drone |
| `bytetracker_match_thresh` (RINT-03 new) | YES | `0.5` (same as drone) | IoU matching — robot-agnostic |
| `bytetracker_frame_rate` (RINT-03 new) | YES | `30` (same as drone) | Pipeline framerate |

### Final `configs/rover_simulation.json`

See § File-by-File above. The JSON has **18 keys**, all of which `ControllerConfig.from_json` accepts (verified against `fields(cls)`).

---

## RINT-02 — Bottom-edge safety per adapter (the carefully-worded one)

The requirement reads:

> Bottom-edge frame safety implementation lands in each adapter (per ABS-05 design — not capability-gated in the controller). `MavsdkDroneAdapter` keeps "person too low → retreat-from-tilt" behavior. `Ros2RoverAdapter` adds "person too low → slow/stop" behavior. The controller's contribution is unchanged: emit `forward_m_s=0` when the bbox bottom is below the safe zone. Drone behaviour at the controller call site unchanged.

**Read carefully:**

1. **"Implementation lands in each adapter"** — past tense for the drone, future tense for the rover. The drone's `_apply_retreat_from_tilt` already exists at `mavsdk_drone.py:108-186`.
2. **"`Ros2RoverAdapter` adds 'person too low → slow/stop' behavior"** — this is the only verbatim "adds" in the requirement.
3. **"The controller's contribution is unchanged: emit `forward_m_s=0` when bbox bottom below safe zone"** — *describes* the controller; doesn't say to add code there.

**Now read the actual current controller** (`controller.py:77-120`): the controller does NOT emit `forward_m_s=0` when bbox-bottom enters the safe zone. The controller emits `forward_m_s = -max_backward` when `bbox_height > max_bbox_height_safety` (panic branch) and otherwise emits the unmodulated distance-P output. The bbox-bottom-in-margin → forward-fade logic lives in the drone adapter's `_apply_retreat_from_tilt`.

This is a **wording gap** between RINT-02's success criterion and the implementation reality. The roadmap success criterion is the authoritative gate:

> Success criterion 2: "Bottom-edge frame safety with `--robot rover` slows/stops the rover when the person is too low in frame; with `--robot drone` the same edge still triggers the retreat-from-tilt behavior."

**Resolution (this is the planner's call to honor):**

- **Drone path:** unchanged. The fade-and-push gradient + final deadband in `_apply_retreat_from_tilt` already produces a smooth retreat as the bbox bottom enters the bottom-margin. Verified by reading `mavsdk_drone.py:152-178`. **Net code change: zero.**
- **Rover path:** the natural consequence of the controller's distance-P retreat when the actor is close is that `forward_m_s` goes negative (or near-zero on the boundary). The rover adapter publishes this verbatim. Rover slows (small forward) or stops/reverses (negative or near-zero forward) → behavior matches "slow/stop". **Net code change: zero.**

  *Note for the planner:* the controller does NOT specifically emit `forward_m_s=0` based on `bbox_bottom`; it emits a forward command driven by `(target_bbox_height / bbox_height) - 1.0`. When the person is close (large bbox = bottom near the bottom edge), the factor is negative, gain is `kp_distance_back`, output saturates to `-max_backward` once `|factor| ≥ max_backward / kp_distance_back ≈ 0.6` → bbox ≈ 1.6× target ≈ 0.4 at default target 0.25. So as the bbox approaches the bottom edge it grows, factor goes more negative, the controller retreats, and the rover stops/backs up naturally.

  **The RINT-02 wording "emit forward_m_s=0 when bbox bottom below safe zone" should be interpreted as a description of the abstract intent ("controller produces non-positive forward when target gets too close"), not a literal code change.** The drone fade-and-push at `_apply_retreat_from_tilt` is what *actually* produces the "slow then zero then negative" curve; the rover gets the simpler "controller distance-P naturally negative" curve.

- **What we add:** a test (in `test_ros2_rover_adapter.py` or `test_sim_worlds.py`) that asserts the rover-adapter natural behavior: feed a Detection whose bbox bottom is in the bottom margin, run `controller.compute → adapter.send_command`, assert the published `Twist.linear.x` is non-positive (slow / zero / negative). This is what locks the behavior contract without adding rover-adapter code.

**Recommendation in the plan:**

1. Update RINT-02's plan summary to record: "Net code in `Ros2RoverAdapter`: zero. RINT-02 closes via a test that asserts the natural slow/stop behavior."
2. The drone-side success criterion (retreat-from-tilt still fires) is covered by the existing test suite (`test_mavsdk_drone_adapter.py` — verify it has retreat-from-tilt unit coverage; it does per Phase 3 plan 03-06).

See Open Question Q1 for the user to confirm this interpretation before the plan is final.

---

## RINT-03 — ByteTracker knobs config-driven

### Current state (verified at `hailo_drone_detection_manager.py:1278-1282`)

```python
_tracker_name = tracker_name or "byte"
_t0 = time.monotonic()
_inner_tracker = create_tracker(
    _tracker_name,
    track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30,
)
```

`create_tracker` factory at `tracker_factory.py:19-44` accepts these 4 kwargs and forwards them to `ByteTrackerAdapter` or `FastTrackerAdapter`. The factory's own defaults are `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30` — byte-identical to the hardcoded call. So the refactor preserves behavior trivially.

### Target shape

1. Add to `ControllerConfig` (already specified above).
2. Replace the hardcoded call with reads from `controller_config.bytetracker_*`.
3. Plumb `controller_config` through `create_app`'s signature (see "Wiring" in § File-by-File).

### Drone regression guard

The drone path is asserted via the existing `test_sim_worlds.py` tests — they implicitly exercise ByteTracker behavior. If those tests stay green with the refactor in place, no regression occurred. **Plan should run the drone SITL sim tests post-refactor as a gate** — they're already marked `@pytest.mark.skipif(not _hailo_available())` so on a no-Hailo CI they skip cleanly; on this dev box (operator verified Phase 5 here) they run.

### Add unit tests

- `test_config_persistence.py`: assert `ControllerConfig()` has the 4 new fields with values `0.4, 90, 0.5, 30`.
- `test_config_persistence.py`: assert `ControllerConfig.from_json(rover_simulation.json)` overrides them to `0.4, 30, 0.5, 30`.
- `test_config_persistence.py`: assert `ControllerConfig.from_json` ignores unknown keys (existing test? Verify and add if missing).
- New test (or extension to a tracker-related test): assert `create_app(...)` passes the 4 ControllerConfig values into `create_tracker` (mock or spy the factory). Optional — the integration test (drone SITL) already covers it.

---

## RINT-04 — End-to-end deterministic test

### Drone-side reference

`robot_follow/tests/test_sim_worlds.py:434-489` — `test_walk_across_then_approach_holds_target_through_approach`. Pattern:

1. `pytestmark = [skipif(RUN_SIM_TESTS != "1"), skipif(no_hailo_device)]` — module-level skips.
2. `sim_run` fixture (lines 148-269) spins up:
   - `sim/start_sim.sh --bridge --world walk_across_then_approach` (PX4 SITL + Gazebo + video_bridge.py)
   - `drone-follow --input udp://0.0.0.0:5600 --webui --no-yaw-only --takeoff-landing --record --test-log <jsonl>`
   - Warmup 3 s, capture 60-90 s, then SIGTERM to the drone-follow process group.
3. Parse the JSONL log (one line per frame) into a list of `{detections, followed_id, ...}` dicts.
4. Assertion shape (from the drone test):
   - Pre-window: `n > 0`, `n_det > MIN_FRAMES_WITH_DETECTION (120)` over a 60-90 s capture.
   - Approach window: contiguous span where the largest bbox height ≥ 0.15 (target close + centred).
   - `n_w ≥ 90` — at least 3 s of close frames.
   - `n_det_w / n_w ≥ 0.9` — detector keeps firing on the large + centred target.
   - `n_id_w / n_w ≥ 0.8` — ByteTracker holds an ID across the approach.

### Rover-side mirror

```python
def test_rover_walk_across_then_approach_follows_actor(rover_sim_run):
    """RINT-04: rover follows actor through the full walk pattern."""
    log = _read_jsonl(rover_sim_run("walk_across_then_approach", run_seconds=90))
    s = _summarize("walk_across_then_approach", log)

    assert s["n"] > 0
    assert s["n_det"] > MIN_FRAMES_WITH_DETECTION
    # Mirror the drone-side approach-window assertion: target stays in frame
    # and tracked through the close approach.
    APPROACH_H = 0.15
    big = [
        i for i, r in enumerate(log)
        if r["detections"] and max(d["bbox"][3] for d in r["detections"]) >= APPROACH_H
    ]
    assert big, "actor never came close enough — rover wiring or world may be off"
    window = log[big[0]:big[-1] + 1]
    n_w = len(window)
    n_det_w = sum(1 for r in window if r["detections"])
    n_id_w = sum(1 for r in window if any(d["id"] is not None for d in r["detections"]))
    assert n_w >= 90
    assert n_det_w / n_w >= 0.9
    assert n_id_w / n_w >= 0.8
```

### `rover_sim_run` fixture (new)

Mirror the drone `sim_run` fixture (lines 148-269) but:
- Launch `sim/rover/start_rover_sim.sh --world walk_across_then_approach` instead of `sim/start_sim.sh --bridge`.
- Invoke drone-follow with `--robot rover --config configs/rover_simulation.json --input udp://0.0.0.0:5600 --webui --test-log <jsonl>`.
- Drop `--takeoff-landing` (rover-irrelevant). Drop `--no-yaw-only` (rover_simulation.json sets it).
- Optional drop `--record` (rover sim doesn't need .mkv evidence; keeps test artifacts smaller).
- Skip conditions: in addition to `RUN_SIM_TESTS=1` and Hailo-device check, add ROS-sourced check (`shutil.which("ros2") is not None` + `/opt/ros/humble/setup.bash` exists). Skip cleanly if rover sim deps missing.

### Deterministic-assertion shape

The drone test uses **coarse-grained, robust assertions** (counts and ratios over a 60-90 s window, not exact-frame matching). Gazebo Garden is not deterministic at the millisecond level (per PITFALLS Pitfall 8 + general sim non-determinism). The rover test should follow the same coarse-grained shape:

- ✅ "Target was visible in N% of frames during the approach window"
- ✅ "ByteTracker held an ID for N% of frames during the approach"
- ✅ "Actor came close enough that bbox height crossed 0.15 at least once"
- ❌ "At frame 1437 the rover's forward velocity was exactly 0.42 m/s"

### File placement

Two viable options — see Open Question Q2. **Recommendation:** put the rover variant in the **same** `test_sim_worlds.py` file (as `test_rover_*` functions sharing the JSONL helpers). Avoids duplicating ~150 lines of fixture and summary helpers. The fixture set could be `sim_run` (drone) and `rover_sim_run` (rover); pytest's `request` arg can switch which sim to launch via a fixture param.

---

## RINT-05 — Port-isolation docs

### Required ground (per requirement wording)

> Port-isolation check: rover sim's MAVLink-side ports (none; just `/cmd_vel`) and video port (5600 UDP) coexist with PX4 SITL ports (14540 UDP MAVLink, 5600 UDP video). Documented that they cannot run simultaneously on the same machine without remapping the video port.

### Existing coverage in `sim/rover/README.md`

Lines 126-141 already say:
- "Both PX4 SITL (`sim/start_sim.sh --bridge`) and rover sim (`sim/rover/start_rover_sim.sh`) bind `udp://0.0.0.0:5600`"
- "They cannot run simultaneously on the same machine"
- Future remap is a v1.2 concern
- Cross-reference to `.planning/research/PITFALLS.md` "Integration Gotchas"

### Gap relative to RINT-05's wording

The existing section does NOT explicitly mention:
- **MAVLink 14540** is used by PX4 SITL but NOT by rover sim (rover sim uses ROS DDS, not MAVLink)
- A small port-comparison table would close this

### Suggested append

Add to the existing "Port 5600 conflict with PX4 SITL" section in `sim/rover/README.md`:

```markdown
### Port usage comparison

| Stack | Video (UDP) | Actuator wire |
|-------|-------------|---------------|
| PX4 SITL (drone) | 5600 | MAVLink on UDP 14540 |
| rover sim | 5600 | ROS DDS (no MAVLink) |

The video port is the only collision. The actuator wires are isolated because
PX4 SITL never speaks ROS and the rover sim never speaks MAVLink.
```

That closes the requirement. **No code changes needed** for RINT-05.

---

## RINT-06 — SIGINT shutdown integration test

### Adapter already does the right thing

`Ros2RoverAdapter.shutdown` (`ros2_rover.py:136-150`) is:

```python
async def shutdown(self) -> None:
    """Idempotent; safe after a partially-failed connect/start_session."""
    self._shutdown_event.set()
    if self._executor_thread is not None:
        self._executor_thread.join(timeout=2.0)
        if self._executor_thread.is_alive():
            LOGGER.warning("[rover] spin thread did not exit within 2s")
    if self._node is not None:
        self._node.destroy_node()
    if self._rclpy is not None:
        self._rclpy.try_shutdown()
    self._node = None
    self._publisher = None
    self._executor = None
    self._executor_thread = None
```

This is correct per PITFALLS § "Looks Done But Isn't":
- Stops the spin thread via `threading.Event` (Pitfall 3 + Pitfall 4 prevented)
- `destroy_node` BEFORE `try_shutdown` (locked by `test_shutdown_orders_node_destroy_before_try_shutdown` at `test_ros2_rover_adapter.py:251-265`)
- Idempotent (locked by `test_shutdown_idempotent` at `test_ros2_rover_adapter.py:244-249`)
- Joins with 2 s timeout, logs warning if thread persists

### What RINT-06 asserts

> Ctrl+C produces zero further `/cmd_vel` messages within 100 ms; rover stops within 1 s; rclpy node cleanly destroyed before `rclpy.try_shutdown()`.

The third clause is already covered by `test_shutdown_orders_node_destroy_before_try_shutdown`. The first two clauses are the gap.

### Test shape (recommended placement: `test_ros2_rover_adapter.py`, new `TestSigintShutdown` class)

```python
class TestSigintShutdown:
    """RINT-06: SIGINT-style shutdown produces zero residual /cmd_vel."""

    def test_zero_cmd_vel_publishes_after_shutdown(self, rclpy_mock):
        """After shutdown(), send_command and on_target_lost are no-ops."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        # Pre-shutdown: a publish lands.
        cmd = RobotCommand(forward_m_s=1.0, yaw_rate=0.0)
        ctx = SafetyContext.from_detection(_det())
        asyncio.run(adapter.send_command(cmd, ctx))
        publish_calls_before = adapter._publisher.publish.call_count

        asyncio.run(adapter.shutdown())

        # Post-shutdown: send_command / on_target_lost / send_zero are
        # no-ops because self._publisher is set to None in shutdown().
        asyncio.run(adapter.send_command(cmd, ctx))
        asyncio.run(adapter.on_target_lost(_det()))
        asyncio.run(adapter.send_zero())
        # Guard: no MagicMock call_count growth (publisher is None, so the
        # branches early-return; the original publisher mock is unchanged).
        # Use the captured mock from before shutdown set the attribute to None.

    def test_shutdown_completes_within_one_second(self, rclpy_mock):
        """RINT-06: rover stops within 1 s of SIGINT."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        t0 = time.monotonic()
        asyncio.run(adapter.shutdown())
        elapsed = time.monotonic() - t0
        assert elapsed < 1.0, f"shutdown took {elapsed:.2f}s, expected < 1.0s"

    def test_spin_thread_exits_after_shutdown_event(self, rclpy_mock):
        """The spin loop checks _shutdown_event and exits — Pitfall 3 guard."""
        adapter = _build_adapter(rclpy_mock)
        asyncio.run(adapter.connect())
        asyncio.run(adapter.start_session())
        thread = adapter._executor_thread
        asyncio.run(adapter.shutdown())
        # Allow the join(2.0) to complete; assert the thread really exited.
        assert not thread.is_alive(), (
            "spin thread still alive after shutdown — Pitfall 3 (blocking spin) "
            "or _shutdown_event signal not honored by _spin_loop"
        )
```

### Why mock-based, not live-rclpy

The `rclpy_mock` fixture (at `test_ros2_rover_adapter.py:16-44`) lets us assert the contract without requiring `/opt/ros/humble` sourced. The CI matrix may not have rclpy; the existing fixture pattern is what makes the rest of the adapter test suite portable. Reuse it for RINT-06.

A live-rclpy variant would be a Phase 6.5 / v1.2 concern (real shutdown timing on Humble). For v1.1, mock-level contract is sufficient and matches the rest of the rover-adapter test suite.

---

## Standard stack (Phase 6 reuses existing)

| Library | Version | Purpose | Why reused |
|---------|---------|---------|------------|
| `rclpy` | Humble (already verified by Phase 4) | ROS 2 Python bindings | Already in adapter |
| `geometry_msgs` | Humble | `Twist` message | Already in adapter |
| `gz sim` | Garden (gz-sim7) | Sim runtime | Already in `sim/rover/` |
| `ros-humble-ros-gzgarden-bridge` | Garden bridge | cmd_vel ↔ gz topic | Already in `install.sh --rover` |
| `pytest` | (project default) | Test framework | Existing tests already use it |
| `pytest-asyncio` (implicit) | n/a — tests use `asyncio.run` | async test harness | Pattern from `test_ros2_rover_adapter.py` |

**No new dependencies.** All packages are already declared and verified.

---

## Architecture patterns

### Pattern 1: Robot-agnostic controller + adapter overlays

**Where it lives:**
- `follow_api/controller.py:77-120` — `controller.compute(detection, caps, config) → RobotCommand`. Robot-agnostic.
- `robot_api/adapters/mavsdk_drone.py:108-186` — drone-specific overlay (`_apply_retreat_from_tilt` + `_apply_altitude_p` + `_apply_smoothing`).
- `robot_api/adapters/ros2_rover.py:112-124` — rover-specific (none; raw `Twist` publish).

**For Phase 6:** The rover adapter does NOT acquire an overlay. RINT-02's "rover slow/stop" is the controller's natural distance-P retreat passed through unmodified.

### Pattern 2: Sim test as subprocess harness

**Where it lives:** `test_sim_worlds.py:148-269` — `sim_run` fixture launches sim + drone-follow via `subprocess.Popen(preexec_fn=os.setsid, ...)`, captures JSONL via `--test-log`, SIGTERMs the process group on teardown.

**For Phase 6:** Mirror this for the rover sim. Same pattern, different sim launcher + different drone-follow invocation.

### Pattern 3: rclpy mock fixture for adapter unit tests

**Where it lives:** `test_ros2_rover_adapter.py:16-44` — `rclpy_mock` injects `MagicMock` rclpy/geometry_msgs into `sys.modules`, exposes a `FakeTwist` class for assertion.

**For Phase 6:** Reuse for RINT-06's shutdown tests. Do not introduce a new fixture.

---

## Don't hand-roll

| Problem | Don't build | Use instead | Why |
|---------|-------------|-------------|-----|
| Detection JSONL writer for the rover test | Custom telemetry | Existing `--test-log` flag (`robot_follow_app.py:454-456` → `DroneFollowUserData.open_test_log`) | Drone test reuses it; rover test follows the same pattern |
| Sim subprocess management | Custom fixture | Reuse `sim_run` fixture shape from `test_sim_worlds.py` | `_kill_group`, `_wait_with_progress`, `_tail` helpers all already exist |
| ROS 2 topic subscriber to count `/cmd_vel` for RINT-06 | Live-rclpy subscriber | `rclpy_mock` fixture + assert on `MagicMock.publish.call_count` | Mock-level contract is what the existing test suite uses |
| Adapter handler for "person too low → stop rover" | New `_apply_rover_slow_stop` function | Nothing — the controller's distance-P + the rover adapter's verbatim `Twist.linear.x = forward_m_s` already produce slow/stop | See § RINT-02 |
| New "drone simulation config" file | `configs/drone_simulation.json` | Existing `sim/configs/simulation.json` + `sim/configs/simulation_follow.json` | Roadmap says "(or equivalent)"; equivalents exist |
| New ByteTracker CLI flags (`--bytetracker-track-buffer` etc.) | 4 new argparse entries | JSON config only | --help blast radius; tracker is init-time only, not live-mutated |

---

## Runtime State Inventory

Phase 6 is **not** a rename/refactor phase. There is no stored runtime state, OS-registered state, or build artifact that embeds Phase 6 strings. The ByteTracker refactor changes function-call shape only; no data migration. Skipping the full inventory.

(For completeness: the `configs/rover_simulation.json` file is new but contains only tuning numbers — no IDs, keys, or state that other systems reference.)

---

## Common pitfalls

### Pitfall A: Two ControllerConfig objects in `create_app` (Path A wiring for RINT-03)

**What goes wrong:** `controller_config` derived from pre-parsed args is passed into `create_app` for tracker init, but `controller_config = ControllerConfig.from_args(args)` is re-derived after `create_app` returns. If the two derivations disagree (e.g. `--bytetracker-track-buffer 60` on CLI but not on pre-parse), the tracker uses one value and the rest of the app uses another.

**Why it happens:** The composition root has a pre-parse stage and a full-parse stage. The pre-parse is restricted to a subset of flags (`--robot`, `--reid-*`, `--tracker`, `--webui`, ...); other args including `--config` may not be on the pre-parser.

**How to avoid:** verify the pre-parse stage at `robot_follow_app.py:362-405` includes `--config` (it does — see the `ControllerConfig.add_args(parser)` call at line 195; pre-parse subset may vary). The simplest fix is to call `ControllerConfig.from_args(pre_args)` once and pass that into both `create_app` and the later config wiring. Single source of truth.

**Warning signs:** unit test where `--config rover.json` and `--bytetracker-track-buffer 60` on the CLI disagree, and the tracker init uses one while the live config reflects the other.

### Pitfall B: Rover sim deps missing → RINT-04 fails confusingly

**What goes wrong:** RINT-04 test runs on a box without `/opt/ros/humble` or without `gz` CLI; the subprocess fails with `command not found` and the test asserts on an empty JSONL.

**How to avoid:** mirror the drone test's skip conditions. Add to the rover fixture:
```python
@pytest.fixture
def rover_sim_run(...):
    if not Path("/opt/ros/humble/setup.bash").exists():
        pytest.skip("ROS 2 Humble not installed at /opt/ros/humble")
    if not shutil.which("gz"):
        pytest.skip("gz CLI not available — install Gazebo Garden")
    ...
```

### Pitfall C: `RUN_SIM_TESTS=1` runs both drone and rover sim back-to-back → port 5600 leak

**What goes wrong:** test ordering: drone test releases UDP 5600 on `_kill_group`; rover test starts and binds 5600 before the kernel has freed the socket → bind error.

**How to avoid:** ensure `_kill_group` SIGKILLs after the SHUTDOWN_S timeout (it does — line 138-141 of `test_sim_worlds.py`). For belt-and-braces, add a `time.sleep(2.0)` in teardown after `_kill_group`. PITFALLS § Integration Gotchas already calls out this exact failure mode.

### Pitfall D: `_apply_smoothing` reads `max_forward_accel` for rover path → AttributeError

**What goes wrong:** if the rover ever calls `_apply_smoothing` (which is in `mavsdk_drone.py`), it would try to read `config.max_forward_accel` and `config.control_loop_hz`. Both exist on `ControllerConfig` for rover loads (defaults survive); no AttributeError.

**How to avoid:** keep `_apply_smoothing` import-local to the drone adapter. The rover adapter at `ros2_rover.py:112-124` does NOT import or call it. Verified.

**Warning sign:** any future plan that suggests "share smoothing between drone and rover" would smuggle drone-specific physics (tilt-transient slew) into the rover path. Reject in plan review.

### Pitfall E: Rover sim launcher exits early without --bridge equivalent

**What goes wrong:** `start_rover_sim.sh` does NOT take a `--bridge` flag (unlike `sim/start_sim.sh`). It always starts the video bridge + cmd_vel bridge. If a future plan adds a `--no-bridge` flag (e.g. for parallel video sources), the RINT-04 fixture's hardcoded invocation breaks.

**How to avoid:** the fixture invokes `start_rover_sim.sh --world walk_across_then_approach` — pin to the documented flags. Don't add `--bridge` (the rover launcher's contract is that the bridge is always on).

---

## Code examples (verified patterns from existing code)

### Pattern: `--test-log` JSONL capture (drone-side, reusable for rover)

```python
# From test_sim_worlds.py:204-209 — drone-follow invocation:
cmd = (
    f"source {SETUP_ENV} && "
    f"drone-follow --input udp://0.0.0.0:5600 --webui{reid_flag} "
    f"--no-yaw-only --takeoff-landing "
    f"--record --test-log {log_path}{extra}"
)
```

Rover variant:

```python
# Expected rover-side invocation:
cmd = (
    f"source {SETUP_ENV} && "
    f"robot-follow --robot rover "
    f"--config configs/rover_simulation.json "
    f"--input udp://0.0.0.0:5600 --webui "
    f"--test-log {log_path}"
)
```

### Pattern: rclpy_mock-based shutdown assertion

```python
# Extends test_ros2_rover_adapter.py — RINT-06:
def test_post_shutdown_send_command_no_op(self, rclpy_mock):
    adapter = _build_adapter(rclpy_mock)
    asyncio.run(adapter.connect())
    asyncio.run(adapter.start_session())
    pub = adapter._publisher  # capture before shutdown nulls it
    asyncio.run(adapter.shutdown())
    asyncio.run(adapter.send_command(
        RobotCommand(forward_m_s=1.0), SafetyContext.from_detection(_det())
    ))
    # adapter._publisher is None post-shutdown → send_command early-returns
    # → the original mock pub gets no new calls.
    # Note: pub.publish was called once during the pre-shutdown sanity-check
    # in start_session smoke flow if any; capture call_count and compare.
```

### Pattern: byte-identical drone-defaults preservation for RINT-03

```python
# In ControllerConfig:
bytetracker_track_thresh: float = 0.4    # was hardcoded in detection_manager.py:1281
bytetracker_track_buffer: int = 90       # was hardcoded
bytetracker_match_thresh: float = 0.5    # was hardcoded
bytetracker_frame_rate: int = 30         # was hardcoded

# In hailo_drone_detection_manager.py — replace lines 1279-1282:
_inner_tracker = create_tracker(
    _tracker_name,
    track_thresh=controller_config.bytetracker_track_thresh,
    track_buffer=controller_config.bytetracker_track_buffer,
    match_thresh=controller_config.bytetracker_match_thresh,
    frame_rate=controller_config.bytetracker_frame_rate,
)
```

---

## State of the art (Phase 6 context)

| Old approach | Current approach | When changed | Impact |
|--------------|------------------|--------------|--------|
| Hardcoded tracker init in detection_manager | Config-driven via ControllerConfig (RINT-03) | Phase 6 | Rover can use shorter `track_buffer` without touching code |
| `bottom_edge_policy` flag in Capabilities | Bottom-edge behavior lives in adapter (ABS-05) | Phase 3 | Axes-only Capabilities locked; this phase verifies the contract |
| `configs/simulation.json` only | + `configs/rover_simulation.json` (RINT-01) | Phase 6 | First per-robot tuning split |

### Deprecated / outdated

- The drone-follow project's earlier "search_active" controller arg path (compute_velocity_command) — removed in Phase 3 03-07. The rover orchestrator path uses `robot.on_target_lost` instead.
- `live_control_loop` — removed in Phase 3 03-07; replaced by `orchestrator.run_robot_loop`.

---

## Assumptions log

| # | Claim | Section | Risk if wrong |
|---|-------|---------|---------------|
| A1 | RINT-02's "controller emits forward_m_s=0" is a description, not a literal code change — interpret as "controller produces low/negative forward via distance-P retreat, and the rover adapter's natural Twist publish converts that to slow/stop" | § RINT-02 | If the user actually wants explicit zero-emit logic in the controller, we need a small controller change (and to update RINT-02's plan accordingly). Resolve via Q1. |
| A2 | `configs/drone_simulation.json` is NOT required — the roadmap success criterion's "or equivalent" is satisfied by `sim/configs/simulation*.json` | § File-by-File | If the user expects a new top-level drone-simulation config, that's a separate small task. Low impact (file creation). |
| A3 | The 4 ByteTracker knobs do NOT need to appear in `tunable_fields()` (live web-UI mutation has no effect because tracker is init-time only) | § File-by-File — ControllerConfig | If a future plan exposes them in `tunable_fields`, the web UI would lie (slider changes nothing). Recommend NOT exposing. |
| A4 | The existing `sim/rover/README.md` "Port 5600 conflict" section roughly satisfies RINT-05; only a small append is needed | § RINT-05 | If the planner judges the existing wording insufficient, append a port-comparison table. Low impact. |
| A5 | Path A for `create_app` config wiring is acceptable (two ControllerConfig objects briefly coexist; tracker reads from one) | § File-by-File — detection manager | If a stricter "single source of truth" rule is required, Path B refactors the composition root. Bigger plan footprint. |
| A6 | RINT-06 mock-level test suffices; live-rclpy SIGINT test is v1.2 | § RINT-06 | If the user wants a live SIGINT test that genuinely sends SIGINT to a real rclpy process, that's a separate harness with a dependency on `/opt/ros/humble` being sourced in CI. |
| A7 | The rover's `kp_yaw=3.0` recommendation is conservative — the actual safe value depends on the rover SDF's `<max_angular_velocity>` setting, which is at DiffDrive plugin defaults | § RINT-01 | If the rover oscillates in RINT-04, lower further (2.0). Tuning, not architecture. |
| A8 | `max_forward_accel=0` cleanly disables the slew cap in `_apply_smoothing` | § RINT-01 | Verified at `mavsdk_drone.py:237` (`if config.max_forward_accel > 0 and config.control_loop_hz > 0`). Plus the rover never calls _apply_smoothing anyway, so the field is doubly inert on the rover path. |
| A9 | `--config configs/rover_simulation.json` works with `robot-follow --robot rover` — config-load path is robot-agnostic | § RINT-04 | Verified — `ControllerConfig.from_args(args)` at `robot_follow_app.py:448` runs the same for drone and rover; `add_rover_args` registers `--cmd-vel-topic` etc. but not `--config` (because `ControllerConfig.add_args` is called regardless). |
| A10 | The `walk_across_then_approach.sdf` world under `sim/rover/worlds/` (Phase 5 RSIM-03) renders the same actor walk pattern as the drone world of the same name | § RINT-04 | Verified by the Phase 5 verification doc: operator confirmed the rover world loaded with the actor in `walk_across_then_approach`. Pattern is mirrored on the ground plane. |
| A11 | `tunable_fields()` doesn't need a new schema entry for `bottom_margin_safety` on the rover path (it's already there, `mavlink_id=None`, web-UI-only) | § File-by-File — config.py | Already correct; no change. |

---

## Open questions

### Q1 — RINT-02 wording ambiguity: rover slow/stop natural or explicit?

**What we know:**
- Controller does NOT emit `forward_m_s=0` based on bbox-bottom today (only the panic branch at `bbox_height > max_bbox_height_safety`).
- The drone fade-and-push gradient lives in `_apply_retreat_from_tilt` (drone-specific).
- Rover adapter publishes `forward_m_s` verbatim → distance-P retreat naturally produces slow/zero/negative when actor is close.
- RINT-02 says "Ros2RoverAdapter ADDS 'person too low → slow/stop' behavior" — verb "adds" implies new code.

**What's unclear:**
- Does the user want explicit slow/stop reactive logic in the rover adapter (a new function analogous to `_apply_retreat_from_tilt` but rover-flavored), or is "natural slow/stop via distance-P + verbatim Twist publish" sufficient?

**Recommendation:**
- Default to "natural slow/stop via distance-P". Add a test asserting it. Net rover code: zero.
- If the user wants explicit reactive logic, scope it as a small follow-up — a `_apply_rover_bottom_safety(forward_m_s, safety_ctx, config) → float` function that clamps `forward_m_s` to <=0 when `bbox_bottom_normalized` is past the margin. Roughly 15 lines of code + 4-5 unit tests.

**Ask the user before plan finalization.** The verb "adds" in RINT-02 is the trigger word.

### Q2 — RINT-04 test file placement: one file or two?

**Recommendation:** keep RINT-04 in **`robot_follow/tests/test_sim_worlds.py`** alongside the drone tests. Two new functions (`test_rover_walk_across_then_approach_follows_actor` + a `rover_sim_run` fixture) live next to the drone-side helpers and reuse `_read_jsonl`, `_summarize`, `_wait_with_progress`, `_kill_group`, `MIN_FRAMES_WITH_DETECTION`. Avoids ~150 lines of duplication.

**Alternative:** new `test_rover_sim_worlds.py` — cleaner separation, but the import + helper duplication is the cost. Reject unless the file grows past ~800 lines.

### Q3 — RINT-06 test file placement

**Recommendation:** extend `robot_follow/tests/test_ros2_rover_adapter.py` with a new `TestSigintShutdown` class. Reuses the `rclpy_mock` fixture, the `_build_adapter` helper, and lives in the same contract-surface file as the other adapter tests.

**Alternative:** new `test_rover_shutdown.py` — would need to import `rclpy_mock` from elsewhere (it's a fixture in `test_ros2_rover_adapter.py`, not a `conftest.py` fixture). Reject — moving the fixture to `conftest.py` is a bigger refactor than RINT-06 warrants.

### Q4 — Should the planner write a `bytetracker_*` integration test that asserts the rover config produces a tracker with `track_buffer=30`?

**Recommendation:** yes, lightweight. Mock `create_tracker` (via `monkeypatch.setattr`) and assert it's called with the expected kwargs when `create_app(...)` runs with a rover-loaded `ControllerConfig`. Covers the wiring without spinning up a real Gazebo. Pairs with the heavyweight RINT-04 sim test.

---

## Environment availability

| Dependency | Required by | Available on this dev box | Version | Fallback |
|------------|------------|---------------------------|---------|----------|
| ROS 2 Humble (`/opt/ros/humble/setup.bash`) | RINT-04 E2E, rover adapter live use | ✓ (operator confirmed in Phase 5 verification) | Humble | — |
| `gz` CLI (Gazebo Garden) | RINT-04 sim launch | ✓ (operator confirmed) | gz-garden | — |
| `ros-humble-ros-gzgarden-bridge` | rover sim launcher | ✓ (installed by Phase 5) | n/a | — |
| Hailo accelerator | drone sim tests (skipped if absent); rover sim test needs detector → needs Hailo too | ✓ on this dev box | — | sim tests skip cleanly |
| Python `rclpy` | adapter tests (mock); RINT-04 live | ✓ via Humble | — | rclpy_mock fixture for unit tests |
| `pytest` | all tests | ✓ | n/a | — |
| `shellcheck` | optional — re-verify start_rover_sim.sh stays clean if touched | ✓ on dev box | n/a | tests skip if missing |

**Missing dependencies with no fallback:** none on this dev box.

**CI consideration:** if a CI runner lacks `/opt/ros/humble` or `gz`, all rover-sim tests skip via the fixture's `pytest.skip` calls. The `@pytest.mark.skipif(RUN_SIM_TESTS != "1")` gate keeps them off by default.

---

## Validation architecture

### Test framework
| Property | Value |
|----------|-------|
| Framework | pytest |
| Config file | `pyproject.toml` (existing) |
| Quick run command | `pytest robot_follow/tests/test_config_persistence.py robot_follow/tests/test_ros2_rover_adapter.py -x` |
| Full suite command | `pytest robot_follow/tests/ -x` (skips sim tests by default) |
| Sim test command | `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py -s -x` |

### Phase requirements → test map

| Req ID | Behavior | Test type | Automated command | File exists? |
|--------|----------|-----------|-------------------|-------------|
| RINT-01 | `configs/rover_simulation.json` loads via `ControllerConfig.from_json` without validation error; specific fields override correctly | unit | `pytest robot_follow/tests/test_config_persistence.py -k rover_simulation` | ❌ Wave 0 — add to existing file |
| RINT-02 (drone) | Drone adapter still applies retreat-from-tilt on bottom-margin | unit (existing) | `pytest robot_follow/tests/test_mavsdk_drone_adapter.py -k retreat_from_tilt` | ✅ existing |
| RINT-02 (rover) | Rover adapter publishes negative or zero `Twist.linear.x` when controller emits `forward_m_s ≤ 0` due to distance-P retreat | unit (new) | `pytest robot_follow/tests/test_ros2_rover_adapter.py -k bottom_edge_natural_stop` | ❌ Wave 0 — add to existing file |
| RINT-03 | `ControllerConfig` exposes 4 `bytetracker_*` fields with correct drone defaults; `create_app` reads them into `create_tracker`; rover config overrides them | unit (new) | `pytest robot_follow/tests/test_config_persistence.py -k bytetracker` + a tracker-init wiring test | ❌ Wave 0 |
| RINT-04 | Rover follows actor in `walk_across_then_approach` for full pattern; close-window assertion mirrors drone test | integration (sim) | `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py -k rover_walk_across` | ❌ — add `test_rover_walk_across_then_approach_follows_actor` + `rover_sim_run` fixture |
| RINT-05 | README documents port-isolation explicitly (PX4 14540 + 5600 vs rover 5600 only) | unit (static) | `pytest robot_follow/tests/test_rover_sim_smoke.py -k port_isolation` (extend existing) | ❌ extend `test_rover_sim_smoke.py` |
| RINT-06 | Post-`shutdown()`, send_command / on_target_lost / send_zero are no-ops (no further publishes); shutdown completes in < 1 s | unit (new) | `pytest robot_follow/tests/test_ros2_rover_adapter.py -k SigintShutdown` | ❌ — add `TestSigintShutdown` class |

### Sampling rate
- **Per task commit:** `pytest robot_follow/tests/test_config_persistence.py robot_follow/tests/test_ros2_rover_adapter.py robot_follow/tests/test_rover_sim_smoke.py -x` (< 10 s)
- **Per wave merge:** full unit suite `pytest robot_follow/tests/ -x --ignore=robot_follow/tests/test_sim_worlds.py` (existing baseline ~180 tests)
- **Phase gate:** `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py -s -x` (the operator runs this with the actor world loaded; equivalent to the RINT-04 operator gate)

### Wave 0 gaps

- [ ] `test_config_persistence.py` — extend with `bytetracker_*` field tests + `rover_simulation.json` load test
- [ ] `test_ros2_rover_adapter.py` — extend with `TestSigintShutdown` class + bottom-edge natural-stop test
- [ ] `test_sim_worlds.py` — add `rover_sim_run` fixture + `test_rover_walk_across_then_approach_follows_actor`
- [ ] `test_rover_sim_smoke.py` — extend with `test_readme_documents_port_isolation_table`
- [ ] `configs/rover_simulation.json` — new file (Wave 1 deliverable, not Wave 0; but the load test in Wave 0 can xfail-skip until the file exists)

---

## Risk register

| # | Risk | Likelihood | Impact | Mitigation |
|---|------|-----------|--------|------------|
| R1 | ByteTracker refactor breaks drone follow (RINT-03) | Low | High (Phase 3 ABS-11 regression) | Drone defaults preserve current values byte-identically; run drone SITL test (`test_walk_across_then_approach_holds_target_through_approach`) post-refactor as a gate. Operator gate: this dev box has Hailo + PX4 SITL infrastructure verified in Phase 3 03-14. |
| R2 | RINT-02 ambiguity surfaces mid-plan: user wants explicit slow/stop code in rover adapter | Medium | Medium (re-plan one task) | Resolve via Q1 BEFORE plan creation. Plan should have a clear branch: "if user confirms natural behavior, RINT-02 = test only; if user wants explicit code, add `_apply_rover_bottom_safety` + 5 tests". |
| R3 | RINT-04 deterministic-assertion shape fails sporadically due to Gazebo non-determinism | Medium | Medium (flaky CI) | Use coarse-grained ratio assertions (>0.8) not exact-frame, mirroring the drone test. If still flaky, increase `run_seconds` from 90 → 120. PITFALLS Pitfall 8 covers the open-loop sim model-reality gap (relevant for v1.2 hardware, not for this sim-only test). |
| R4 | Port 5600 collision if developer runs PX4 SITL + rover sim accidentally | Low | Low (immediate bind error) | Documented in README; bind error is loud and immediate, not silent. No automated guard needed. |
| R5 | SIGINT race: rover's spin thread persists under sustained publish load | Low | Medium (RINT-06 test flakes) | Existing `shutdown` does `_shutdown_event.set() + thread.join(2.0)`. Spin loop calls `spin_once(timeout=0.05)` so it observes the event within ≤50 ms. Mock-level test doesn't exercise live ROS publish load — accept this gap; v1.2 hardware bring-up will exercise live-rclpy SIGINT. |
| R6 | The `--config configs/rover_simulation.json` path on `--robot rover` interaction with two-pass argparse (ABS-09) | Low | Low | `ControllerConfig.add_args` is called regardless of `--robot`; verified at `robot_follow_app.py:195`. The two-pass dispatch only filters drone-specific flags (`--takeoff-landing`, `--serial`, `--target-altitude`). `--config` is universal. |
| R7 | Tracker config passed by Path A wiring picks up a stale value | Low | Medium | See Pitfall A. Mitigation: single `ControllerConfig.from_args(pre_args)` call passed into `create_app`; the post-`create_app` re-derivation should equal it. Plan should add an assertion: `assert controller_config_from_pre == controller_config_from_post`. |

---

## Sources

### Primary (HIGH confidence — code/repo files)

- `robot_follow/follow_api/config.py` — ControllerConfig field audit + `from_json` filter semantics (lines 36-217)
- `robot_follow/follow_api/controller.py` — controller emits, distance-P retreat shape (lines 53-120)
- `robot_follow/follow_api/types.py` — Capabilities, RobotCommand, SafetyContext (lines 1-132)
- `robot_follow/robot_api/robot.py` — Robot protocol contract (lines 1-112)
- `robot_follow/robot_api/orchestrator.py` — `run_robot_loop` state machine (lines 1-135)
- `robot_follow/robot_api/adapters/mavsdk_drone.py` — `_apply_retreat_from_tilt`, `send_command`, `caps` (lines 108-186, 683-755)
- `robot_follow/robot_api/adapters/ros2_rover.py` — Ros2RoverAdapter full body (lines 1-151)
- `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:1278-1282` — hardcoded ByteTracker init (the RINT-03 target)
- `robot_follow/pipeline_adapter/tracker_factory.py` — `create_tracker` signature
- `robot_follow/tests/test_sim_worlds.py` — drone-side E2E test pattern + sim_run fixture (lines 1-513)
- `robot_follow/tests/test_ros2_rover_adapter.py` — rclpy_mock fixture, adapter contract tests (lines 1-337)
- `robot_follow/tests/test_rover_sim_smoke.py` — Phase 5 parse-only smoke tests + README assertions (lines 1-269)
- `sim/rover/README.md` — existing port-isolation section (lines 126-141)
- `sim/configs/simulation.json`, `sim/configs/simulation_follow.json` — shape reference for new rover config
- `.planning/REQUIREMENTS.md` — RINT-01..06 wording (lines 101-106)
- `.planning/ROADMAP.md:128-136` — Phase 6 goal + success criteria
- `.planning/research/PITFALLS.md` — Pitfalls 3 (spin), 4 (asyncio), 5 (gz prefixes), 6 (topic), 8 (open-loop)
- `.planning/research/SUMMARY.md` — milestone-level architectural intent
- `.planning/STATE.md` — Accumulated Context (axes-only Capabilities decision)
- `.planning/phases/05-rover-sim/05-VERIFICATION.md` — operator-confirmed rover sim ground truth
- `.planning/presentations/v1_1_design_review.md:543-559` — Phase 6 design intent (the "controller emits forward=0" wording that needs interpretation)

### Secondary (MEDIUM confidence — derived patterns)

- ByteTracker recommended buffer ratio (3 s drone / 1 s rover) from the milestone goal + design review. No external academic source consulted; the value is empirical engineering judgment.

### No tertiary low-confidence sources used — Phase 6 is internal-evidence-driven.

---

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — all upstream Phase 3/4/5 pieces are landed and operator-verified.
- Architecture: HIGH — no new patterns introduced; Phase 6 is a config + small refactor + tests phase.
- Pitfalls: HIGH (A, D, E verified by code), MEDIUM (B, C are operational/runtime risks).
- RINT-02 interpretation: MEDIUM until user confirms Q1.

**Research date:** 2026-05-20
**Valid until:** 2026-06-20 (30 days; stable codebase, no expected upstream churn before Phase 6 lands)

**Files the planner will write (in PLAN.md files):**
1. `configs/rover_simulation.json` — new
2. `robot_follow/follow_api/config.py` — 4 new fields + from_args wiring
3. `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py` — `create_app` signature + 4-line tracker refactor
4. `robot_follow/tests/test_config_persistence.py` — extend
5. `robot_follow/tests/test_ros2_rover_adapter.py` — extend (TestSigintShutdown + bottom-edge natural-stop)
6. `robot_follow/tests/test_sim_worlds.py` — extend (rover_sim_run + test_rover_walk_across*)
7. `robot_follow/tests/test_rover_sim_smoke.py` — extend (port-isolation README assertion)
8. `sim/rover/README.md` — small append (RINT-05 table) IF gap exists vs requirement wording

**Files explicitly NOT changed (architectural locks):**
- `robot_follow/robot_api/adapters/mavsdk_drone.py` (drone byte-identical)
- `robot_follow/robot_api/adapters/ros2_rover.py` (rover adapter natural-behavior contract — recommended; pending Q1)
- `robot_follow/follow_api/controller.py` (controller robot-agnostic)
- `robot_follow/follow_api/types.py` (Capabilities axes-only)
- `robot_follow/robot_api/robot.py` (protocol stable)
- `robot_follow/robot_api/orchestrator.py` (run_robot_loop robot-agnostic)
- `sim/rover/{model.sdf, model.config, worlds/*.sdf, start_rover_sim.sh, install.sh --rover branch}` (Phase 5 byte-identical)
- `sim/bridge/video_bridge.py` (RSIM-06 architectural lock from Phase 5)
