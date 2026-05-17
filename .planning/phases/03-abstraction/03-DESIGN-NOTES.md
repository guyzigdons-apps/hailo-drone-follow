# Phase 3 — Design Notes

Companion to `03-CONTEXT.md`. Diagram + change summary + risks for the abstraction phase.

---

## Architecture diagram (before / after)

```
   BEFORE (today)                                      AFTER (Phase 3)
   ─────────────                                       ──────────────────────────────

   ┌────────────────────┐                              ┌────────────────────┐
   │   pipeline_adapter │ (unchanged)                  │   pipeline_adapter │ (unchanged)
   │   Hailo + tracker  │                              │   Hailo + tracker  │
   └────────┬───────────┘                              └────────┬───────────┘
            │ Detection                                         │ Detection
            ▼                                                   ▼
   ┌──────────────────────────┐                        ┌──────────────────────────┐
   │  follow_api/             │                        │  follow_api/             │
   │  ├── state.py            │                        │  ├── state.py            │
   │  ├── config.py           │                        │  ├── config.py           │
   │  ├── types.py            │                        │  ├── types.py            │
   │  │   VelocityCommand     │ ◄── DELETED            │  │   (Detection only;   │
   │  │   Detection           │                        │  │    no actuator type) │
   │  └── controller.py       │                        │  └── controller.py       │
   │      ┌─ yaw P            │                        │      ┌─ yaw P            │
   │      ├─ forward P        │                        │      ├─ forward P        │
   │      ├─ altitude P       │ ◄── moves to adapter   │      ├─ emergency safety │
   │      ├─ emergency safety │                        │      └─ edge: emit 0     │
   │      ├─ retreat-from-tilt│ ◄── moves to adapter   │   (no robot knowledge)  │
   │      └─ yaw-spin-on-loss │ ◄── moves to adapter   │                          │
   │   returns VelocityCommand│                        │   returns RobotCommand   │
   └────────┬─────────────────┘                        │   (down_m_s if           │
            │                                          │    Axis.ALTITUDE in caps)│
            │                                          └────────┬─────────────────┘
            ▼                                                   │
   ┌──────────────────────────┐                                 ▼
   │  drone_api/              │                        ┌──────────────────────────┐
   │  └── mavsdk_drone.py     │                        │  robot_api/              │ NEW
   │      ├─ live_control_loop│                        │  ├── robot.py            │ NEW
   │      ├─ telemetry tasks  │                        │  │   Robot protocol      │
   │      ├─ offboard wait    │                        │  │   Capabilities        │
   │      └─ VelocityCommandAPI│ ◄─ → MavsdkDroneAdapter│  │   (axes + yaw_unit)   │
   │          ├─ send         │                        │  │   Axis enum           │
   │          ├─ send_zero    │                        │  │   RobotCommand        │
   │          └─ smoothing    │                        │  ├── orchestrator.py     │ NEW
   └────────┬─────────────────┘                        │  │   run_robot()         │
            │                                          │  │   shared loop:        │
            ▼                                          │  │     det → controller  │
   ┌──────────────────────────┐                        │  │     → robot.send_cmd  │
   │  MAVSDK → PX4 → drone    │                        │  └── adapters/           │ NEW
   └──────────────────────────┘                        │      ├── mavsdk_drone.py │ ← moved from drone_api/
                                                       │      │   MavsdkDroneAdapter
                                                       │      │   ├─ connect       │
                                                       │      │   ├─ start_session │ (offboard + telemetry)
                                                       │      │   ├─ send_command  │ (cmd, detection)
                                                       │      │   │  + altitude P  │
                                                       │      │   │  + retreat-tilt│
                                                       │      │   │  + smoothing   │
                                                       │      │   ├─ send_zero     │ (yaw-spin search)
                                                       │      │   └─ shutdown      │
                                                       │      └── ros2_rover.py    │ ◄── placeholder for Phase 4
                                                       └────────┬─────────────────┘
                                                                │
                                                                ▼
                                                       ┌──────────────────────────┐
                                                       │  MAVSDK → PX4 → drone    │ (unchanged at the wire)
                                                       └──────────────────────────┘

   CLI (today)                                         CLI (Phase 3)
   ────────────                                        ─────────────
   drone-follow --takeoff-landing --serial ...         drone-follow --robot drone --takeoff-landing ...
                                                       drone-follow --robot rover --cmd-vel-topic ...

   Single pre-parser (Phase 2 CLEAN-12)                Pre-parser parses --robot first, then
   builds full parser inline                           dispatches to add_drone_args() or add_rover_args()
                                                       so each --robot's --help is filtered
```

---

## Change summary

| # | Change | Why | Where |
|---|--------|-----|-------|
| 1 | Introduce `robot_api/` package with `robot.py` + `orchestrator.py` + `adapters/` | Establish the actuator-boundary seam Phases 4–6 plug into | new dir |
| 2 | Define `Robot` protocol (`connect`, `start_session`, `send_command`, `send_zero`, `shutdown`, `caps`) | Unified actuator interface for drone today, rover tomorrow | `robot_api/robot.py` |
| 3 | Define `Capabilities = {axes: frozenset[Axis], yaw_unit}` (axes-only) | Controller must not know robot type; only mechanical info (which axes, what units) | `robot_api/robot.py` |
| 4 | Define `RobotCommand(forward_m_s, yaw_rate, down_m_s)` replacing `VelocityCommand` | `yaw_rate` is in `caps.yaw_unit`; `down_m_s` only used if `Axis.ALTITUDE in caps.axes` | `robot_api/robot.py` |
| 5 | Move `drone_api/mavsdk_drone.py` → `robot_api/adapters/mavsdk_drone.py` (as `MavsdkDroneAdapter`) | Drone is one robot type; lives behind the Robot protocol | file move + rename |
| 6 | Move altitude-hold P-loop from `live_control_loop` → `MavsdkDroneAdapter.send_command` | Altitude is drone-specific; adapter reads its own `altitude_cache` | `robot_api/adapters/mavsdk_drone.py` |
| 7 | Move offboard handshake + telemetry tasks → `MavsdkDroneAdapter.start_session()` | Drone lifecycle is adapter-internal; rover's `start_session` is a no-op | `robot_api/adapters/mavsdk_drone.py` |
| 8 | Move retreat-from-tilt + yaw-spin-on-loss out of controller → adapter | Robot-specific behaviors; controller stays robot-agnostic per axes-only memo | `robot_api/adapters/mavsdk_drone.py` |
| 9 | Extract generic control loop → `robot_api/orchestrator.py` | One loop for all robots; calls `robot.send_command(cmd, detection)` per tick | `robot_api/orchestrator.py` |
| 10 | Controller signature: `compute(detection, caps, config) → RobotCommand` | Pure function; takes capabilities so it knows which axes to write | `follow_api/controller.py` |
| 11 | Rename `run_drone()` → `run_robot()`; dispatch on `--robot` flag | Composition root is robot-agnostic; instantiates the right adapter | `robot_follow_app.py` |
| 12 | Two-pass argparse: pre-parse `--robot` → load drone or rover args | Rover users don't see drone-only flags in `--help`; reduces confusion | `robot_follow_app.py` |
| 13 | `setup_env.sh` conditionally sources `/opt/ros/humble/setup.bash` if present | ROS available for rover users; idempotent for drone users; zero new args | `setup_env.sh` |
| 14 | New snapshot test: `test_robot_command_snapshot.py` (~100 detections) | CI gate for "drone behavior unchanged" — catches silent drift in <1 s | `robot_follow/tests/` |
| 15 | Operator-witnessed SITL run before Phase 3 closes | Catches anything snapshot misses (timing, MAVSDK wire behavior) | manual gate |
| 16 | `Capabilities` validation: `validate()` skips altitude checks when `Axis.ALTITUDE not in caps.axes` | `ControllerConfig` altitude fields become `Optional[float]` | `follow_api/config.py` |

---

## Reasons for the bigger calls

### Why "shared orchestrator" beats "per-adapter loop"

If each adapter owns its own loop, Phase 4 (rover) duplicates ~80% of the loop scaffold: shared_state read, detection-lost handling, shutdown event, asyncio.gather supervision. A single `robot_api/orchestrator.py` makes Phase 4 trivially small — the rover adapter just implements 5 methods.

### Why altitude P moves into the adapter

Today `live_control_loop` reads `altitude_cache` (populated by the drone's telemetry task) and computes an altitude P correction. The controller doesn't see this — it just emits the raw `down_m_s` request, and the loop blends in the P correction. That's a drone-specific behavior. Per the axes-only decision, all such behaviors live in the adapter. The controller becomes simpler; the adapter owns its `altitude_cache` as a private attribute.

### Why retreat-from-tilt moves out of controller

Today `_apply_frame_edge_safety` knows about retreat (drone-specific reaction to bbox-in-edge): subtract forward velocity, push backward. Rover doesn't tilt — it just stops. Moving the retreat math into the drone adapter lets the rover adapter cleanly do nothing.

### Why `send_command(cmd, detection)` and not `send_command(cmd)`

The adapter needs to know about edge conditions (bbox in bottom margin → drone retreats). The cleanest pass-through is just to give the adapter the current Detection. Inverting it (controller signals via flags) creates a structured-info channel that few robots need; pass-through is simplest.

### Why "controller returns None on target-lost"

Today the controller has a search-yaw-spin built in. Rover doesn't yaw-spin. Moving the search behavior to the adapter (drone spins, rover sends zero) requires the controller to signal "no detection." Cleanest signal: return `None`. The orchestrator catches it and calls `robot.send_zero(last_detection)`. Each adapter decides what "zero" means.

### Why emergency-safety stays in the controller

A bbox bigger than `max_bbox_height_safety` means the target is dangerously close. Both drone and rover want to back off in that case. It's not robot-shaped — it's target-shaped. Keep it in the controller as a generic emit-retreat.

### Why "always source ROS if installed"

ABS-10 says auto-source ROS when `--robot rover`. But `setup_env.sh` runs BEFORE the app sees its args, so it can't condition on `--robot`. Simplest solution: source ROS unconditionally whenever the `/opt/ros/humble/setup.bash` file exists. Drone users on a non-ROS machine see no change; drone users on a ROS-equipped machine source ROS but the drone path is unaffected (ROS env vars don't conflict with the venv).

### Why a snapshot test (not full SITL) as the primary gate

Phase 3 is a refactor: same inputs → same outputs. The 174-test suite already covers controller correctness. The snapshot adds a CI guard against accidental semantic drift in the controller-output sequence. SITL runs take a minute on Hailo HW; snapshot runs in <1 s.

---

## Risks and downsides

### Architectural

1. **`MavsdkDroneAdapter` becomes a god-class.** Telemetry tasks, offboard handshake, altitude P, retreat-from-tilt, smoothing, MAVSDK send all under one roof. Mitigation: split into multiple files in `robot_api/adapters/mavsdk/` (`adapter.py`, `lifecycle.py`, `safety.py`, `telemetry.py`).
2. **`robot_api → follow_api` dependency direction.** `send_command(cmd, detection)` means `robot_api` imports `Detection` from `follow_api`. That's a one-way dependency, but it means `follow_api` is no longer the pure-domain leaf it is today. Mitigation: keep the import narrow (just `Detection`); never import controller or state from robot_api.
3. **Capabilities frozen at boot.** A drone whose altitude sensor is currently degraded still claims `Axis.ALTITUDE`. No runtime degradation path. Mitigation: out of scope for v1.1; add `Robot.refresh_caps()` in v1.2 if needed.
4. **Two adapters for cross-cutting concerns.** If we later add command-rate limiting or a watchdog, both `MavsdkDroneAdapter` and `Ros2RoverAdapter` need it. Mitigation: a small `BaseAdapter` mixin in `robot_api/adapters/` for cross-cutting behavior — but defer adding it until we have two adapters and a real cross-cut.
5. **Protocol-fit risk for rover.** Phase 3 doesn't actually build the rover adapter — we won't truly know the `Robot` protocol fits Twist/cmd_vel/rclpy until Phase 4. If something doesn't fit, Phase 3 needs revision. Mitigation: review the protocol against a paper sketch of `Ros2RoverAdapter` in `03-RESEARCH.md` before Phase 3 lands.

### Implementation

6. **Migration window has double-implemented retreat.** Until the controller change AND the adapter change both land, the same retreat-from-tilt math exists in two places. Either commit them atomically (risky: big change) or stage with intermediate guards. Mitigation: make the controller-side change a `forward_m_s = 0` (not a delete) and land the adapter's overlay in the same commit; existing test_controller passes for both states.
7. **Snapshot test brittleness.** Intentional improvements to the controller emit different VelocityCommand sequences and break the snapshot. Need a clear "update snapshot" workflow. Mitigation: snapshot-update command in CONTRIBUTING.md; reviewer must confirm intentional change.
8. **`send_zero(last_detection)` is asymmetric with `send_command(cmd, detection)`.** Easy to confuse current vs last detection. Mitigation: explicit kwarg names + docstring.
9. **Altitude P unit-testability decreases.** To test altitude correction you now need a `MavsdkDroneAdapter` instance + a mock `altitude_cache`. Mitigation: extract `_apply_altitude_p(down_m_s, altitude_cache, config) → float` as a pure function for unit testing.

### Operational

10. **ROS env vars leak into drone sessions on ROS-equipped machines.** Always-sourcing ROS exports `PYTHONPATH`, `AMENT_PREFIX_PATH`, `LD_LIBRARY_PATH`. Phase 1 already saw a similar issue (bare `pytest` vs `python -m pytest`). Risk: a ROS-installed dev machine has different env after `source setup_env.sh` than a non-ROS machine. Mitigation: sourcing order (venv first, ROS second) per PITFALLS.md ensures Python resolves venv before ROS bindings. Document in `CLAUDE.md` § Virtual Environment.
11. **Two-pass argparse adds a level back.** Phase 2 CLEAN-12 collapsed three pre-parsers into one; Phase 3 adds robot-dispatch logic on top. Net: still simpler than pre-CLEAN-12 but more than today. Mitigation: single pre-parser parses `--robot` only; dispatch table is a `dict[str, Callable[[ArgumentParser], None]]` — easy to read.
12. **Operator gate delays phase close.** SITL run requires Hailo HW + sim setup. If operator is unavailable, Phase 3 stalls before Phase 4 can start. Mitigation: snapshot test is the "automated gate" that lets verifier mark plans complete; SITL is verifier's `human_needed` step, can be deferred like Phase 1's Verification B/C.
13. **Drone behavior must be byte-identical at the wire.** Any change in MAVSDK packet timing or values breaks the contract. Smoothing + clamping logic relocates from `VelocityCommandAPI.send` into `MavsdkDroneAdapter.send_command` — if the relocation introduces an extra coroutine boundary or changes the order of clamp-then-smooth, behavior shifts. Mitigation: keep the function body byte-equivalent during the move; test the snapshot first, then refactor any internals.

### Process

14. **Phase 3 is a single big commit OR a wave of small ones — both have risks.** Single atomic commit: bisect-survives, but reviewer-unfriendly. Wave: bisectable per task but each intermediate may not pass the snapshot. Mitigation: planner decides commit shape. Suggest: separate plans for (a) introduce protocol + types + tests, (b) move telemetry/lifecycle, (c) move altitude P, (d) move retreat-from-tilt, (e) wire CLI + setup_env.sh. Each plan keeps the snapshot green.
15. **`Capabilities` access from controller becomes a third argument.** `compute_velocity_command(det, config) → compute(det, caps, config)`. Every test that calls the controller needs updating. Mitigation: `caps` is a small object, easy keyword arg; default to `DRONE_CAPS` in tests that don't care.
16. **Plan-checker may flag the `follow_api → robot_api` import direction.** Phase 1 verifier was strict about `follow_api` being pure. After Phase 3, `follow_api/controller.py` imports `Axis`, `Capabilities`, `RobotCommand` from `robot_api/robot.py` (or wherever). That's the reverse of today's direction. Mitigation: explicitly document the new dependency; `robot_api/robot.py` is a "types-only" leaf module with no GStreamer/Hailo/MAVSDK imports, so `follow_api` importing from it is still "no third-party imports."

---

## Honest mitigations summary

| Risk class | Mitigation strategy |
|------------|---------------------|
| God-class | Split `MavsdkDroneAdapter` into a small directory of files when it exceeds ~400 lines |
| Snapshot brittleness | Documented update workflow; reviewer-confirmed intentional changes |
| ROS env leakage | Sourcing order (venv first, ROS second); document in CLAUDE.md |
| Operator gate delay | Snapshot test is the CI gate; SITL is "human_needed" — deferrable |
| Protocol-fit for rover | Paper-sketch `Ros2RoverAdapter` in Phase 3 research before locking |
| Migration window | Atomic-ish commits per concern; snapshot stays green between commits |

---

*Phase: 03-abstraction*
*Design notes captured: 2026-05-17*
