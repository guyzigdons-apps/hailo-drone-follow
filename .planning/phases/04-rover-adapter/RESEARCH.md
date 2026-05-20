# Phase 4: Rover adapter — Research

**Researched:** 2026-05-20
**HEAD verified:** `ac0edcc` (`feature/rover-support`)
**Domain:** ROS 2 Humble rclpy adapter implementing the `Robot` protocol; publishes `geometry_msgs/Twist` on `/cmd_vel`; preserves drone-follow's SIGINT handler; integrates with the asyncio `run_robot_loop` orchestrator that Phase 3 landed.
**Confidence:** HIGH — every claim below is grounded in a verified file:line on the current tree OR cited from `.planning/research/PITFALLS.md` (already-locked research). No new library research required; the rclpy/threading work was done pre-Phase-3 and resulted in the locks the rover adapter must respect.

---

## Summary

Phase 4 is a small, contained adapter implementation. The hard work has already happened: Phase 3 landed the `Robot` protocol (`robot_follow/robot_api/robot.py`), the `MavsdkDroneAdapter` (`robot_follow/robot_api/adapters/mavsdk_drone.py:562`), and the generic `run_robot_loop` orchestrator (`robot_follow/robot_api/orchestrator.py:35`). The composition root (`robot_follow_app.py:484-561`) already dispatches on `--robot`; today the rover branch raises `NotImplementedError` (`robot_follow_app.py:509-515`) — this plan replaces that stub with a real `Ros2RoverAdapter`.

The work is:
1. Create `robot_follow/robot_api/adapters/ros2_rover.py` with `ROVER_CAPS` constant + `Ros2RoverAdapter` class implementing the 6 `Robot` methods + `caps` attribute.
2. Add `add_rover_args(parser)` body in `robot_follow_app.py` (today's body is an empty placeholder at line 200-213).
3. Replace the `NotImplementedError` stub in `run_robot()` at `robot_follow_app.py:509-515` with `Ros2RoverAdapter(args, controller_config)` construction.
4. Add `robot_follow/tests/test_ros2_rover_adapter.py` (unit tests with rclpy mocked) and extend `test_cli_help_dispatch.py` to assert the rover flags are visible under `--robot rover --help`.

Phase 3 already did the deep paper-sketch of `Ros2RoverAdapter` in `03-RESEARCH.md` lines 432-572 and validated that the `Robot` protocol fits. The Q5 yaw-unit-conversion contradiction Phase 3 flagged has since been LOCKED in the protocol docstring (`robot.py:75-78`): **no adapter-side conversion** — the controller emits `yaw_rate` already in `caps.yaw_unit`. The rover adapter assigns it directly to `Twist.angular.z`.

**Primary recommendation:** Plan as ~5 bisectable commits — (1) Wave 0 test scaffold with rclpy mock fixture and xfail markers, (2) `add_rover_args` body + CLI dispatch test pass, (3) `Ros2RoverAdapter` class + unit tests pass, (4) `run_robot()` stub replacement + smoke test of the SIGINT preservation assertion, (5) Phase 4 verifier (clean drone path unaffected + rover --help works + xfails close). The whole phase fits in one wave's worth of work; no parallel tasks needed.

---

## User Constraints (from Phase 3 locks + REQUIREMENTS.md)

> Phase 4 has no `04-CONTEXT.md` yet (this research runs ahead of `/gsd:discuss-phase`). The constraints below are inherited from Phase 3's CONTEXT (already-locked) and from REQUIREMENTS.md ROVER-01..08 (locked at the milestone level).

### Locked Decisions (inherited from Phase 3 + REQUIREMENTS.md)

1. **No adapter-side yaw-unit conversion.** Controller emits `yaw_rate` in `caps.yaw_unit`. Drone: deg/s. Rover: rad/s. The rover adapter assigns `cmd.yaw_rate` directly to `Twist.angular.z`. Source: `robot.py:75-78` (Q5 lock); REQUIREMENTS.md ROVER-06; `types.py:42-43`.

2. **`rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is the only allowed init signature.** Drone-follow's `on_signal` handler at `robot_follow_app.py:564` must survive. Source: PITFALLS Pitfall 1; ROVER-02.

3. **Background spin uses `SingleThreadedExecutor.spin_once(timeout_sec=0.05)` in a `while not shutdown_event.is_set()` loop.** Never `rclpy.spin()` or `executor.spin()`. Source: PITFALLS Pitfall 3; ROVER-03.

4. **Defensive `try: import rclpy except ImportError → raise RuntimeError` with friendly message naming `source /opt/ros/humble/setup.bash`.** Not `ImportError` traceback. Source: PITFALLS Pitfall 2; ROVER-04.

5. **`ROVER_CAPS = Capabilities(axes=frozenset({Axis.FORWARD, Axis.YAW}), yaw_unit="rad/s")`** — rover has no ALTITUDE axis. No offboard/arm/takeoff lifecycle. Source: ROVER-07; `types.py:16-46`.

6. **Per-adapter capability constants live in the adapter module**, not in `follow_api/types.py`. (Q8 lock — `mavsdk_drone.py:42-48` defines `DRONE_CAPS` in the same module.) Source: Phase 3 CONTEXT § Q8 lock; consistency with drone adapter.

7. **Rover-specific CLI flags:** `--cmd-vel-topic` (default `/cmd_vel`), `--ros-namespace` (default `""`), `--ros-domain-id` (default `0`). Source: ROVER-05.

8. **`Robot.send_command` short-circuits when `safety_ctx.target_lost` is True (Q6 lock).** Same rule as drone adapter (`mavsdk_drone.py:692-693`). Source: `robot.py:73`.

9. **`Robot.on_target_lost(last_detection)` is the per-tick search behavior** (split from `send_zero()` per R2). For rover: publish zero Twist (no yaw-spin, unlike drone). Source: `robot.py:89-100`; ROVER-08 implied.

### Claude's Discretion

- Exact wording of the `RuntimeError` for missing ROS — must name `source /opt/ros/humble/setup.bash` per ROVER-04. Suggest also mentioning `sudo apt install ros-humble-ros-base ros-humble-geometry-msgs` so users on a clean Ubuntu can fix it in one step.
- Whether `_shutdown_event` is a module-level `threading.Event` or an instance attribute. Recommend instance attribute (matches `_drone` / `_smoothing` pattern in `MavsdkDroneAdapter`).
- Whether to lazy-import rclpy in `__init__` (lets the module file `import` cleanly even when ROS is missing — useful for tests) or at module-top. **Strong recommendation:** lazy-import in `__init__`. This is the only way unit tests can mock rclpy without hitting the `ImportError` during collection.
- Whether the rclpy spin thread is `daemon=True` or joined explicitly in `shutdown()`. Recommend `daemon=True` AND join with 2 s timeout in `shutdown()` — belt-and-suspenders.
- Test names + fixture organization.

### Deferred Ideas (OUT OF SCOPE for Phase 4)

- `/odom` subscription — HW-03, v1.2 hardware milestone.
- `twist_mux` integration — HW-04, v1.2.
- Bottom-edge "person too low → slow rover" behavior — RINT-02, **Phase 6**. Phase 4's `send_command` passes `cmd.forward_m_s` straight through; the safety overlay lands in Phase 6 once a rover config exists to drive it.
- Bridge / sim plumbing (DiffDrive SDF, gz topic mapping) — Phase 5 (RSIM-*).
- Rover-specific controller config (`configs/rover_simulation.json`) — RINT-01, Phase 6. ROVER-01..08 ship without it; the rover runs against the default `ControllerConfig` and the kp values are tuned in Phase 6.
- `TwistStamped` — explicitly OUT (REQUIREMENTS.md "Out of Scope").

---

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| ROVER-01 | `robot_api/adapters/ros2_rover.py` defines `Ros2RoverAdapter` implementing `Robot`; wraps `rclpy.Node` with a `Publisher[Twist]` | § Concrete file structure; § Robot protocol surface; § Adapter sketch (this doc) |
| ROVER-02 | `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)`; SIGINT survives | § rclpy lifecycle ordering; § Test strategy (SIGINT smoke test) |
| ROVER-03 | `SingleThreadedExecutor.spin_once(timeout_sec=0.05)` in a shutdown-event loop | § rclpy lifecycle ordering; § Adapter sketch |
| ROVER-04 | Friendly `RuntimeError` on missing rclpy, not `ImportError` traceback | § Defensive import; PITFALLS Pitfall 2 |
| ROVER-05 | `add_rover_args(parser)` registers `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id` | § Argparse split (mirror 03-08 pattern) |
| ROVER-06 | `cmd.forward_m_s → Twist.linear.x`; `cmd.yaw_rate → Twist.angular.z`; NO conversion | § send_command → Twist mapping |
| ROVER-07 | `ROVER_CAPS = Capabilities(axes=frozenset({Axis.FORWARD, Axis.YAW}), yaw_unit="rad/s")`; no arm/takeoff | § Robot protocol surface (no-op methods) |
| ROVER-08 | Smoke test: `signal.getsignal(signal.SIGINT) is on_signal` after `start_session()` | § Test strategy (SIGINT preservation test) |

---

## Concrete file structure

### Files to CREATE

```
robot_follow/
└── robot_api/
    └── adapters/
        └── ros2_rover.py            # NEW — Phase 4 deliverable
robot_follow/
└── tests/
    ├── test_ros2_rover_adapter.py   # NEW — Phase 4 unit tests (rclpy mocked)
    └── test_cli_help_dispatch.py    # EDIT — add rover-flag assertions
```

### Files to EDIT

| Path | Change | Lines |
|------|--------|-------|
| `robot_follow/robot_follow_app.py` | (a) Replace `add_rover_args` body (currently empty placeholder); (b) Replace `NotImplementedError` stub in `run_robot()` with `Ros2RoverAdapter(args, controller_config)` construction | (a) `:200-213`, (b) `:509-515` |
| `robot_follow/tests/test_cli_help_dispatch.py` | Add tests asserting `--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id` appear under `--robot rover --help` and do NOT appear under `--robot drone --help` | append new test functions |
| `robot_follow/robot_api/adapters/__init__.py` | OPTIONAL — currently a stub docstring mentioning "Phase 4 adds ros2_rover.py". Update docstring or leave as-is | `:4` |

### Files NOT to touch (deliberately)

| Path | Why |
|------|-----|
| `robot_follow/robot_api/adapters/mavsdk_drone.py` | Phase 3 artifact; reference pattern only. The `add_drone_args` body at `:269-290` is the template `add_rover_args` mirrors. |
| `robot_follow/robot_api/orchestrator.py` | Already robot-agnostic; consumes `Robot` protocol; needs zero changes for rover. |
| `robot_follow/robot_api/robot.py` | Protocol is locked at Phase 3. Adding methods would break drone adapter. |
| `robot_follow/follow_api/*` | Pure domain layer; rover adapter never imports from outside `follow_api.types` + `follow_api.config`. |
| `setup_env.sh` | Conditional ROS source block already landed in Phase 3 plan 03-09 (`setup_env.sh:39-55` — verified on current tree). ROVER-04's prerequisite is met. |

### Where `ROVER_CAPS` lives

**In `robot_follow/robot_api/adapters/ros2_rover.py`**, mirroring how `DRONE_CAPS` lives at the top of `robot_follow/robot_api/adapters/mavsdk_drone.py:42-48`. Per Q8 lock (Phase 3 CONTEXT): per-adapter constants live with their adapter; `follow_api/types.py` stays types-only.

```python
# robot_follow/robot_api/adapters/ros2_rover.py
from robot_follow.follow_api.types import Axis, Capabilities

ROVER_CAPS: Capabilities = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW}),
    yaw_unit="rad/s",
)
```

The name `ROVER_CAPS` (not `RoverCapabilities`) matches the drone constant's casing — `DRONE_CAPS` is an instance, not a class.

### Where `add_rover_args` lives

**Stays in `robot_follow/robot_follow_app.py`** at the current placeholder location (line 200-213). Do NOT move it into `ros2_rover.py`. Rationale: `add_drone_args` lives in `robot_api/adapters/mavsdk_drone.py:269-290` (Phase 3 left it there because `mavsdk_drone.py` is imported unconditionally in `robot_follow_app.py:32-36`). For the rover the import would be conditional (rclpy may be missing), so keeping `add_rover_args` in `robot_follow_app.py` avoids a chicken-and-egg: the `--help` path runs `add_rover_args` and must NOT trigger a defensive rclpy ImportError. Argparse flag registration is pure Python; it has no rclpy dependency.

**One asymmetry the planner should note:** `add_drone_args` is imported FROM `mavsdk_drone.py` INTO `robot_follow_app.py` (line 35). `add_rover_args` is DEFINED IN `robot_follow_app.py` (line 200). This is intentional — see rationale above. The two helpers have the same signature and registration pattern; the location difference is purely about import safety on a no-rclpy box.

### `setup_env.sh` (no work for Phase 4)

The conditional ROS source block landed in Phase 3 plan 03-09 (`setup_env.sh:39-55` verified at HEAD `ac0edcc`). ROVER-04's prerequisite — "ROS sourced after venv" — is already satisfied. `test_setup_env_sh.py:46-56` locks this. Phase 4 does NOT touch `setup_env.sh`.

---

## The `Robot` protocol surface — method-by-method for rover

Read the protocol at `robot_follow/robot_api/robot.py:38-111`. The rover adapter must implement all 6 methods plus the `caps` attribute. Here's exactly what each does on the rover side:

| Protocol method | Drone behavior (reference) | Rover behavior |
|-----------------|---------------------------|----------------|
| `caps` (attribute) | `DRONE_CAPS` (FORWARD, YAW, ALTITUDE; deg/s) | `ROVER_CAPS` (FORWARD, YAW; rad/s) |
| `async connect()` | Open MAVSDK gRPC; raises ConnectionError on 15s timeout | **`rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)`** + post-init `signal.getsignal(SIGINT)` assertion (ROVER-02). Raises ConnectionError if init fails. **No network connect** — rclpy publishes on the ROS_DOMAIN_ID network and discovery is async. |
| `async start_session()` | Spawn telemetry tasks + offboard handshake (10-60s) | Create `Node("robot_follow_rover", namespace=self._namespace)`; create `Publisher[Twist]` on `self._topic` with QoS depth 10; start `SingleThreadedExecutor` + spin thread. **Sub-second.** No arm/takeoff/offboard handshake (ROVER-07). |
| `async send_command(cmd, safety_ctx)` | Apply altitude_p + retreat_from_tilt + smoothing + `set_velocity_body` | **Short-circuit if `safety_ctx.target_lost`** (Q6 lock). Build `Twist()`: `linear.x = cmd.forward_m_s`, `angular.z = cmd.yaw_rate`, all other fields = 0.0. Publish via `self._publisher.publish(twist)`. NO conversion. NO smoothing (Phase 4); RINT-02 bottom-edge slow lands in Phase 6. |
| `async send_zero()` | Reset SmoothingState; publish `VelocityBodyYawspeed(0,0,0,0)` | Publish `Twist()` (all-zero by default ctor). Called once in orchestrator `finally`. |
| `async on_target_lost(last_detection)` | Yaw-spin in last-seen direction (search behavior) | Publish `Twist()` (all zeros — rover does NOT yaw-spin on target loss). `last_detection` is unused for rover. Document explicitly: ROVER-07 + REQUIREMENTS § "Rover adapter" implies "no yaw-spin search; just stop". |
| `async shutdown()` | Cancel telemetry, land if armed, exit DetachedMavsdkServer | Set `self._shutdown_event` (`threading.Event`); `self._executor_thread.join(timeout=2.0)`; `self._node.destroy_node()`; `self._rclpy.try_shutdown()`. **Idempotent** (safe to call multiple times — guard against repeated cleanup). |

### No-op / not-applicable methods

| Protocol method | Rover behavior | Why |
|-----------------|---------------|-----|
| Arm / takeoff / land | **None.** Rover has no flight lifecycle. | ROVER-07: "No offboard/arm/takeoff lifecycle is exposed via Capabilities" — these were never part of the `Robot` protocol surface; they live inside the drone adapter's `start_session()` and `shutdown()`. The rover adapter simply has no analogous code. |
| Mission-duration deadline | **No-op (no flag registered).** `add_rover_args` does NOT register `--mission-duration`. `getattr(args, "mission_duration", math.inf)` in `robot_follow_app.py:520` evaluates to `math.inf` for rover → no deadline. | `add_drone_args` registers `--mission-duration` at `mavsdk_drone.py:285`. By NOT registering it in `add_rover_args` the deadline naturally goes away for rover. |
| `--serial` / `--connection` connection handling | **No-op.** `_resolve_serial_connection(args)` at `robot_follow_app.py:56-61` checks `getattr(args, "serial", None) is not None`. For rover, `args.serial` does not exist → no-op. | Verified: `robot_follow_app.py:427` calls `_resolve_serial_connection(args)` unconditionally, but the body is a getattr guard. |

### Where `on_target_lost` for rover differs from drone

Drone `on_target_lost` (`mavsdk_drone.py:724-731`) does yaw-spin via `_compute_search_yawspeed`. Rover's analogous behavior: **just stop** — publish `Twist()` with all zeros. ROVER-07 implies this ("the rover has no ALTITUDE axis, full stop") and Phase 6 RINT-02 confirms it (rover-specific search behavior is not introduced in v1.1).

Two acceptable implementations:

```python
# Option A — explicit zero
async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
    twist = self._Twist()  # all-zero by default
    self._publisher.publish(twist)

# Option B — delegate to send_zero (same wire effect)
async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
    await self.send_zero()
```

**Recommend Option A** — keeps semantics explicit and lets future Phase 6 RINT-02 modify behavior without re-routing through `send_zero` (which is the shutdown path).

---

## rclpy lifecycle ordering — single checklist

Distilled from PITFALLS Pitfalls 1-3. Hand to the executor verbatim:

### `connect()` order (one-time init)

1. Capture `before_sigint = signal.getsignal(signal.SIGINT)` (this is drone-follow's `on_signal` installed at `robot_follow_app.py:570`).
2. Call `rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)`. **MUST** pass `SignalHandlerOptions.NO`; default is `ALL` and clobbers `on_signal` (PITFALLS Pitfall 1).
3. Assert `signal.getsignal(signal.SIGINT) is before_sigint`. If False, raise — abort before any setpoints fly. This is ROVER-02's smoke assertion *inside* the adapter; ROVER-08's smoke test is the same check from a test harness.

### `start_session()` order (per-session)

4. Create `Node("robot_follow_rover", namespace=self._namespace)`.
5. Create `Publisher[Twist]` on `self._topic` with QoS depth 10 (`node.create_publisher(Twist, self._topic, 10)`). Default reliability is RELIABLE which matches what `ros_gz_bridge` defaults to in Phase 5 — no QoS tuning needed at Phase 4 scope.
6. Construct `SingleThreadedExecutor()` and `executor.add_node(node)`.
7. Start a background `threading.Thread(target=self._spin_loop, daemon=True)` and store reference. The spin loop is:
   ```python
   def _spin_loop(self):
       while not self._shutdown_event.is_set():
           self._executor.spin_once(timeout_sec=0.05)   # ROVER-03
   ```
   **NEVER** call `rclpy.spin(node)` or `executor.spin()` (no timeout) — both block forever and hang shutdown (PITFALLS Pitfall 3).

### `shutdown()` order (must be idempotent)

8. `self._shutdown_event.set()` — unblocks the spin loop's `while` check on next iteration (≤50 ms latency given `timeout_sec=0.05`).
9. `self._executor_thread.join(timeout=2.0)` — spin loop exits because `is_set()` returns True. If join times out, log a warning and continue (cleanup still proceeds; daemon=True means the thread dies with the process).
10. `self._node.destroy_node()` — **MUST come before `try_shutdown`** per PITFALLS "Looks Done But Isn't" checklist (line 197) and the canonical rclpy teardown order.
11. `self._rclpy.try_shutdown()` — `try_shutdown` (NOT `shutdown`) to keep `shutdown()` idempotent across re-entry. Wrap in `try/except Exception: pass` because rclpy can raise on duplicate shutdowns despite the `try_` prefix.
12. NULL-OUT `self._node`, `self._publisher`, `self._executor`, `self._executor_thread`. Subsequent `send_command` / `send_zero` / `on_target_lost` calls early-return on `self._publisher is None`.

### Threading boundary diagram

```
Main thread (GStreamer + signal handlers + on_signal)
  │
  │  signal.signal(SIGINT, on_signal)  ← MUST survive rclpy.init
  │
  ├─ Robot-control thread (asyncio loop, run_robot_loop)
  │     │
  │     ├─ await adapter.connect()        ← rclpy.init runs here, on this thread
  │     ├─ await adapter.start_session()  ← spawns spin thread (below)
  │     ├─ loop: await send_command(...)  ← publishes Twist directly (rclpy publisher is thread-safe)
  │     └─ finally: await send_zero() + shutdown()
  │
  └─ rclpy spin thread (threading.Thread, daemon=True)
        │
        └─ while not shutdown_event:
              executor.spin_once(timeout_sec=0.05)
        ← exits on shutdown_event.set()
```

**Cross-thread invariants:**
- The asyncio control loop thread (`run_robot`) calls `self._publisher.publish(twist)` synchronously. **rclpy publishers are thread-safe** ([sources: ROS Answers #342625, PITFALLS source list line 226]). No lock needed.
- The spin thread services callbacks (we register zero subscribers in v1.1, so spin is effectively a no-op heartbeat — but it MUST exist or rclpy timers and lifecycle would never fire if we add them later).
- `self._shutdown_event` is a `threading.Event` — thread-safe `set()` / `is_set()`. **Do NOT use `asyncio.Event`** for cross-thread (PITFALLS Pitfall 4); we never need to await it from the control loop side anyway.
- The orchestrator's `shutdown: asyncio.Event` (passed into `run_robot_loop`) stays asyncio-only — never touched by the spin thread. The rover adapter's `self._shutdown_event` is its own internal threading.Event distinct from the orchestrator's shutdown signal.

---

## `send_command` → Twist mapping (ROVER-06)

The mapping is dead simple per Q5 lock. Hand-coded here so the planner can verify the executor's diff in one glance:

```python
async def send_command(
    self,
    cmd: RobotCommand,
    safety_ctx: SafetyContext,
) -> None:
    # Q6 lock: short-circuit on target_lost (mirrors mavsdk_drone.py:692-693).
    if safety_ctx.target_lost:
        return
    if self._publisher is None:
        return  # connect/start_session failed; silent skip

    twist = self._Twist()
    twist.linear.x  = cmd.forward_m_s  # m/s direct (both sides m/s)
    twist.linear.y  = 0.0              # rover is non-holonomic (no LATERAL axis)
    twist.linear.z  = 0.0              # no ALTITUDE axis on rover (ROVER-07)
    twist.angular.x = 0.0              # no ROLL axis
    twist.angular.y = 0.0              # no PITCH axis
    twist.angular.z = cmd.yaw_rate     # rad/s direct — Q5 lock, NO conversion
    self._publisher.publish(twist)
```

### Why no conversion

The controller emits `yaw_rate` already in `caps.yaw_unit` — see `robot.py:75-78` and `controller.py:7`. For the rover:
- `caps.yaw_unit == "rad/s"`
- `_compute_yaw` at `controller.py:39-50` uses `config.kp_yaw * sqrt(error_x_deg)` clamped to `config.max_yawspeed`. **The result's unit is whatever unit `kp_yaw` and `max_yawspeed` are in.**
- Phase 6 RINT-01 will ship `configs/rover_simulation.json` with `kp_yaw` / `max_yawspeed` / `search_yawspeed_slow` in rad/s.
- Phase 4 just consumes whatever the controller emits and assigns it directly. **NO unit-aware code in the adapter.**

> **Planner note for Phase 6:** The rover sim test (RINT-04) will fail visually if the operator runs `--robot rover` with the default `ControllerConfig()` (where `max_yawspeed=90.0` was tuned as deg/s for drone). 90 rad/s ≈ 5160 deg/s — the rover will spin uncontrollably. Phase 4 lands without `configs/rover_simulation.json`; the unit tests below all override `kp_yaw` / `max_yawspeed` explicitly so they pass, but the operator-facing "run --robot rover and watch the sim" workflow is gated on Phase 6's config. This is correct sequencing per ROADMAP § Execution Order — Phase 4 ships the wire path; Phase 6 ships the tuning.

### `send_zero()` shape

```python
async def send_zero(self) -> None:
    if self._publisher is None:
        return
    twist = self._Twist()  # all fields default to 0.0
    self._publisher.publish(twist)
```

### `on_target_lost()` shape

```python
async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
    # Rover does NOT yaw-spin; just stop. RINT-02 (slow-near-edge) lands in Phase 6.
    if self._publisher is None:
        return
    twist = self._Twist()
    self._publisher.publish(twist)
```

---

## Defensive import (ROVER-04)

PITFALLS Pitfall 2 identifies two failure modes for "rclpy not available":
1. ROS not installed at all → `ImportError: No module named 'rclpy'`
2. ROS installed but not sourced → `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'` (the C extension lookup fails because `LD_LIBRARY_PATH` is missing `/opt/ros/humble/lib`)

Both must produce a friendly `RuntimeError`, not a stacktrace. The guard goes in `Ros2RoverAdapter.__init__` (lazy import) so the MODULE itself imports cleanly even when rclpy is missing — this is what makes unit-test mocking work.

### Exact shape

```python
# robot_follow/robot_api/adapters/ros2_rover.py

class Ros2RoverAdapter:
    """ROS 2 cmd_vel publisher adapter. Implements Robot protocol."""

    caps: Capabilities = ROVER_CAPS

    def __init__(self, args, config: ControllerConfig):
        # Lazy import so the module file imports cleanly on a no-rclpy box
        # (drone-only users, unit tests). PITFALLS Pitfall 2: handle BOTH
        # "module not installed" and "C-extension missing because ROS not
        # sourced" as the same friendly error.
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from geometry_msgs.msg import Twist
        except ImportError as e:
            raise RuntimeError(
                "ROS 2 not available — the rover adapter requires "
                "rclpy + geometry_msgs.\n"
                "Fix:\n"
                "  1) sudo apt install ros-humble-ros-base "
                "ros-humble-geometry-msgs\n"
                "  2) source /opt/ros/humble/setup.bash\n"
                "  3) re-source setup_env.sh\n"
                f"Underlying error: {e}"
            ) from e

        # Bind imported symbols to instance for use in connect/start_session.
        # Storing on self (not module-level) keeps the module import-free.
        self._rclpy = rclpy
        self._Twist = Twist
        self._Node = Node
        self._SignalHandlerOptions = SignalHandlerOptions
        self._SingleThreadedExecutor = SingleThreadedExecutor

        self.caps = ROVER_CAPS
        self._topic = getattr(args, "cmd_vel_topic", "/cmd_vel")
        self._namespace = getattr(args, "ros_namespace", "")
        self._domain_id = getattr(args, "ros_domain_id", 0)
        # If non-zero domain ID, set via env (rclpy honors ROS_DOMAIN_ID at init).
        # Per ROS 2 Humble docs, the env var is the only way to set domain_id
        # programmatically before rclpy.init.
        if self._domain_id != 0:
            os.environ["ROS_DOMAIN_ID"] = str(self._domain_id)

        # State
        self._node = None
        self._publisher = None
        self._executor = None
        self._executor_thread = None
        self._shutdown_event = threading.Event()
```

### Why `RuntimeError` not `ImportError`

- `RuntimeError` is a "your environment is misconfigured" error; argparse error handling at `robot_follow_app.py` won't try to retry, and the message reaches the user with the full text.
- `ImportError` triggers stacktraces and looks like a Python bug. ROVER-04 explicitly says "raises a friendly RuntimeError".
- `raise ... from e` preserves the chain so `--log-verbosity debug` still shows the underlying ImportError if needed.

### Catch ImportError, not Exception

`ImportError` catches both `ModuleNotFoundError` (rclpy missing entirely) AND the relative-import failure for `_rclpy_pybind11` (which is also `ImportError`, subclass `ModuleNotFoundError`). Other exception types from rclpy import-time would indicate a different bug worth surfacing — let those propagate.

---

## Argparse split for rover (ROVER-05)

Mirror the pattern from `add_drone_args` at `mavsdk_drone.py:269-290`. The rover analog goes in `robot_follow_app.py` (already has the empty placeholder at line 200-213).

### Exact body

```python
# robot_follow/robot_follow_app.py — replace lines 200-213

def add_rover_args(parser: argparse.ArgumentParser) -> None:
    """Register rover-only CLI flags.

    Per ROVER-05 + Phase 4. Mirrors add_drone_args' argument-group pattern
    (mavsdk_drone.py:269-290).

    Plan-checker invariant (DESIGN-NOTES line 128): no flag may be in both
    add_drone_args and add_rover_args. Verified by visual inspection —
    none of these three flags overlap with the 6 drone flags.
    """
    group = parser.add_argument_group("rover-ros2")
    group.add_argument(
        "--cmd-vel-topic", default="/cmd_vel",
        help="ROS topic to publish geometry_msgs/Twist setpoints on "
             "(default: /cmd_vel)",
    )
    group.add_argument(
        "--ros-namespace", default="",
        help="ROS namespace prefix for the rover node (default: empty)",
    )
    group.add_argument(
        "--ros-domain-id", type=int, default=0,
        help="ROS_DOMAIN_ID for network isolation (default: 0). "
             "Non-zero values isolate the rover from other ROS nodes on "
             "the same LAN.",
    )
```

### Verification flags

Mirror `test_cli_help_dispatch.py` (existing — has 7 tests at HEAD). The two new test functions:

```python
# robot_follow/tests/test_cli_help_dispatch.py — append after existing tests

ROVER_ONLY_FLAGS = ["--cmd-vel-topic", "--ros-namespace", "--ros-domain-id"]

@pytest.mark.parametrize("flag", ROVER_ONLY_FLAGS)
def test_rover_help_includes_rover_flag(flag: str):
    out = _help_output("rover")
    assert flag in out, f"expected {flag} in --robot rover --help output"

@pytest.mark.parametrize("flag", ROVER_ONLY_FLAGS)
def test_drone_help_excludes_rover_flag(flag: str):
    out = _help_output("drone")
    assert flag not in out, f"unexpected {flag} in --robot drone --help output"
```

These follow the exact pattern of the existing `test_drone_help_includes_drone_flag` and `test_rover_help_excludes_drone_flag` at `:42-50`. **Do not refactor the helper or the existing tests** — just append. Six new test cases (3 flags × 2 directions).

---

## Test strategy

### File: `robot_follow/tests/test_ros2_rover_adapter.py`

Three test classes mirroring `test_mavsdk_drone_adapter.py`'s shape. All tests use mocked rclpy so they pass on this dev box (no ROS installed at `/opt/ros/humble` — verified) and in CI.

### rclpy mock fixture (the foundation)

Inject mocks into `sys.modules` BEFORE the adapter module is imported. This is the standard pattern when the import itself is what fails. Pytest fixture:

```python
# robot_follow/tests/test_ros2_rover_adapter.py

import sys
from unittest.mock import MagicMock, patch

import pytest


@pytest.fixture
def rclpy_mock(monkeypatch):
    """Inject mock rclpy + geometry_msgs into sys.modules.

    Must be applied BEFORE importing Ros2RoverAdapter — the adapter's
    __init__ does `import rclpy` and `from geometry_msgs.msg import Twist`.
    With these mocks in sys.modules, the lazy imports succeed and the
    adapter constructs cleanly.

    PITFALLS Pitfall 2 friendly-error path is tested SEPARATELY in
    test_friendly_error_when_rclpy_missing (which REMOVES the mocks).
    """
    rclpy = MagicMock(name="rclpy")
    rclpy.signals.SignalHandlerOptions.NO = "NO"  # sentinel
    rclpy.executors.SingleThreadedExecutor = MagicMock(name="SingleThreadedExecutor")
    rclpy.node.Node = MagicMock(name="Node")

    geometry_msgs = MagicMock(name="geometry_msgs")
    geometry_msgs_msg = MagicMock(name="geometry_msgs.msg")

    # Use a real-shaped Twist so attribute access (twist.linear.x = ...) works.
    class FakeTwist:
        def __init__(self):
            self.linear = MagicMock(x=0.0, y=0.0, z=0.0)
            self.angular = MagicMock(x=0.0, y=0.0, z=0.0)
    geometry_msgs_msg.Twist = FakeTwist

    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.executors", rclpy.executors)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy.node)
    monkeypatch.setitem(sys.modules, "rclpy.signals", rclpy.signals)
    monkeypatch.setitem(sys.modules, "geometry_msgs", geometry_msgs)
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geometry_msgs_msg)

    yield {"rclpy": rclpy, "Twist": FakeTwist}
```

### Test classes

**`TestImportSafety`** — module import + friendly error
- `test_module_imports_cleanly_without_rclpy` — `importlib.import_module("robot_follow.robot_api.adapters.ros2_rover")` succeeds even on a no-ROS box (the lazy import is INSIDE `__init__`, not at module top).
- `test_friendly_error_when_rclpy_missing` — Construct adapter without the `rclpy_mock` fixture (or with `sys.modules["rclpy"]` deleted); assert `RuntimeError` whose `str(exc)` contains both `"ROS 2 not"` and `"source /opt/ros/humble/setup.bash"`.

**`TestProtocolShape`** — Robot protocol conformance
- `test_implements_robot_protocol` — `assert isinstance(adapter, Robot)` (uses `@runtime_checkable` from `robot.py:38`).
- `test_caps_is_rover_caps` — `adapter.caps == ROVER_CAPS`; `adapter.caps.yaw_unit == "rad/s"`; `Axis.ALTITUDE not in adapter.caps.axes`.
- `test_has_six_methods` — same shape as `test_robot_protocol_shape.py:39-48`.

**`TestSignalHandlerPreservation`** — ROVER-02 / ROVER-08
- `test_connect_calls_init_with_signal_handler_options_no` — after `await adapter.connect()`, assert `rclpy_mock["rclpy"].init.called_with(args=None, signal_handler_options="NO")`.
- `test_sigint_handler_survives_connect` (**ROVER-08**) — the smoke test:
  ```python
  def fake_handler(*_): pass
  signal.signal(signal.SIGINT, fake_handler)
  asyncio.run(adapter.connect())
  assert signal.getsignal(signal.SIGINT) is fake_handler
  ```
  Because rclpy is mocked, `init` doesn't actually touch signal state — the test then verifies the adapter's own assertion `assert before is after` is satisfied. (For a true end-to-end SIGINT-preservation test with real rclpy, mark as `@pytest.mark.skipif(not Path("/opt/ros/humble/setup.bash").exists(), ...)` — but the mocked version is what runs in CI.)

**`TestTwistPublish`** — ROVER-06 wire format
- `test_publishes_twist_with_forward_and_yaw_rate` — Call `await adapter.send_command(RobotCommand(forward_m_s=1.5, yaw_rate=0.3, down_m_s=0.0), SafetyContext.from_detection(...))`. Assert `adapter._publisher.publish.called_with(<twist>)`, where the published Twist has `linear.x == 1.5`, `angular.z == 0.3`, and `linear.y == linear.z == angular.x == angular.y == 0.0`.
- `test_send_command_no_conversion_yaw_rate` — Send `yaw_rate=0.5` (rad/s); assert published `angular.z == 0.5` (NOT converted via `* π/180`).
- `test_send_command_short_circuits_on_target_lost` (**Q6 lock**) — `send_command(cmd, SafetyContext.lost())`; assert `publisher.publish` was NOT called.
- `test_send_zero_publishes_all_zeros` — assert published Twist has all 6 fields = 0.0.
- `test_on_target_lost_publishes_zero_twist` — same as send_zero (rover doesn't yaw-spin on loss).

**`TestLifecycle`** — connect/start/shutdown ordering
- `test_start_session_creates_node_and_publisher` — assert `Node` was called with `namespace=self._namespace`; assert `create_publisher` was called with `(Twist, "/cmd_vel", 10)`.
- `test_start_session_starts_executor_thread` — assert `adapter._executor_thread` is a Thread; `is_alive()` is True after `start_session()`.
- `test_shutdown_idempotent` — call `shutdown()` twice in a row; second call must not raise.
- `test_shutdown_orders_node_destroy_before_try_shutdown` — record call order via `MagicMock.mock_calls`; assert `destroy_node` index < `try_shutdown` index. PITFALLS "Looks Done But Isn't" line 197.
- `test_shutdown_sets_shutdown_event` — after `shutdown()`, `adapter._shutdown_event.is_set()` is True.

**`TestCustomCliArgs`** — ROVER-05
- `test_custom_cmd_vel_topic` — construct with `args.cmd_vel_topic = "/rover/cmd_vel"`; assert `create_publisher` called with that topic.
- `test_custom_namespace` — construct with `args.ros_namespace = "robot1"`; assert `Node` called with `namespace="robot1"`.
- `test_domain_id_sets_env` — construct with `args.ros_domain_id = 7`; assert `os.environ["ROS_DOMAIN_ID"] == "7"`. (Use `monkeypatch.delenv` then check.)

### Test count summary

| Class | Count | Notes |
|-------|-------|-------|
| TestImportSafety | 2 | One requires rclpy mocks; one removes them |
| TestProtocolShape | 3 | Lock the protocol fit |
| TestSignalHandlerPreservation | 2 | ROVER-02 + ROVER-08 |
| TestTwistPublish | 5 | ROVER-06 wire format |
| TestLifecycle | 5 | Connect/start/shutdown ordering |
| TestCustomCliArgs | 3 | ROVER-05 plumbing |
| **Total** | **~20** | All pass on no-ROS dev box via mocks |

### Plus: CLI dispatch tests in test_cli_help_dispatch.py

| Test | Count | Notes |
|------|-------|-------|
| `test_rover_help_includes_rover_flag` (parametrized over 3 flags) | 3 | ROVER-05 visibility |
| `test_drone_help_excludes_rover_flag` (parametrized over 3 flags) | 3 | Plan-checker disjointness |
| **Total** | **6** | Mirror existing 7 tests at `:18-59` |

---

## Risk register

| # | Risk | Likelihood | Mitigation |
|---|------|-----------|------------|
| R1 | rclpy not installed in CI → tests can't import the adapter module | HIGH | Lazy import in `__init__` (not module-top); `sys.modules` mock fixture in tests. The module file imports cleanly without rclpy; only adapter construction would fail without mocks. Verified pattern: PITFALLS Pitfall 2 mitigation. |
| R2 | `SignalHandlerOptions` enum location varies by rclpy version | LOW-MEDIUM | Use `from rclpy.signals import SignalHandlerOptions` (Humble + Iron + Jazzy stable). The path `rclpy.signals.SignalHandlerOptions` has been stable since rclpy 2.0 (Humble's version). REQUIREMENTS § "Out of Scope" rules out other distros. |
| R3 | Background spin thread hangs on shutdown (drone_thread.join 5s timeout fires) | LOW | `timeout_sec=0.05` on `spin_once` means the shutdown_event check happens at ≤50 ms latency. `executor_thread.join(timeout=2.0)` bounds the wait. `daemon=True` ensures the thread can't outlive the process. |
| R4 | `os.environ["ROS_DOMAIN_ID"]` race if multiple adapters constructed in same process | LOW | Document that only one `Ros2RoverAdapter` should be constructed per process (matches `run_robot` thread model — single robot per CLI invocation). No mitigation code needed at v1.1. |
| R5 | Phase 6 RINT-02 (bottom-edge slow for rover) reaches into `send_command` later | LOW | Document in `send_command` docstring that the bottom-edge overlay is Phase 6 work; leave a `# TODO(Phase 6 / RINT-02)` marker so Phase 6's executor can find the insertion point. |
| R6 | `Capabilities` field is `frozenset[Axis]` — adapter test fixture might construct wrong | LOW | `ROVER_CAPS` is a module-level constant; tests assert `adapter.caps is ROVER_CAPS` to lock identity. Mirror `DRONE_CAPS` pattern. |

---

## Open questions

> Only one item; not manufacturing questions for completeness.

1. **Should the friendly error in ROVER-04 ALSO mention `apt install`?** ROVER-04 only requires "`ROS 2 not sourced — run source /opt/ros/humble/setup.bash`". On a clean Ubuntu without ROS the user will hit `apt install` next anyway. **Recommendation:** include the `apt install` line (see § Defensive import "Exact shape" above) — it's a strictly larger message and costs nothing. Planner can defer to user during `/gsd:discuss-phase`.

---

## Environment Availability

Phase 4 has minimal new dependencies — ROS sourcing was Phase 3's responsibility (already landed in `setup_env.sh`). Verified at HEAD `ac0edcc`:

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| Python 3.10 | All Phase 4 code | ✓ | 3.10 (in hailo-apps venv) | — |
| `/opt/ros/humble/setup.bash` | Operator running `--robot rover` against real ROS | ✗ on this dev box | — | Defensive import raises friendly RuntimeError (ROVER-04) |
| `rclpy` | Adapter runtime | ✗ on this dev box | — | Mocked in unit tests; ROVER-04 friendly error at runtime |
| `geometry_msgs/Twist` | Adapter runtime | ✗ on this dev box | — | Mocked in unit tests |
| pytest | Test framework | ✓ (in hailo-apps venv) | — | — |

**Missing dependencies with no fallback:** None for Phase 4 development. ROS is required at run time but NOT at code-write time or test time. Phase 5 sim work and Phase 6 integration testing will exercise rclpy for real.

**Missing dependencies with fallback:** rclpy + geometry_msgs are mocked in unit tests (`sys.modules` injection); ROVER-04 ensures runtime users get a friendly error if they invoke `--robot rover` without ROS.

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest (existing — `robot_follow/tests/`) |
| Config file | `robot_follow/tests/conftest.py` (path setup only — no shared rclpy fixture, that lives in the new test file) |
| Quick run command | `pytest robot_follow/tests/test_ros2_rover_adapter.py robot_follow/tests/test_cli_help_dispatch.py -x` |
| Full suite command | `pytest robot_follow/tests/ -x` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| ROVER-01 | `Ros2RoverAdapter` implements `Robot` protocol | unit | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestProtocolShape -x` | ❌ Wave 0 |
| ROVER-02 | `rclpy.init(SignalHandlerOptions.NO)` is the only init call | unit (mocked) | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestSignalHandlerPreservation::test_connect_calls_init_with_signal_handler_options_no -x` | ❌ Wave 0 |
| ROVER-03 | `spin_once(timeout_sec=0.05)` in shutdown-checked loop | unit (mocked) | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestLifecycle::test_start_session_starts_executor_thread -x` | ❌ Wave 0 |
| ROVER-04 | Friendly RuntimeError on missing rclpy | unit | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestImportSafety::test_friendly_error_when_rclpy_missing -x` | ❌ Wave 0 |
| ROVER-05 | `--cmd-vel-topic` / `--ros-namespace` / `--ros-domain-id` registered, rover-only | unit (subprocess) | `pytest robot_follow/tests/test_cli_help_dispatch.py -x` | ✅ (edit existing file) |
| ROVER-06 | `forward_m_s → linear.x`; `yaw_rate → angular.z`; no conversion | unit (mocked) | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestTwistPublish -x` | ❌ Wave 0 |
| ROVER-07 | `ROVER_CAPS = (FORWARD, YAW; rad/s)`; no ALTITUDE | unit | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestProtocolShape::test_caps_is_rover_caps -x` | ❌ Wave 0 |
| ROVER-08 | `signal.getsignal(SIGINT) is on_signal` after start_session | unit (mocked) | `pytest robot_follow/tests/test_ros2_rover_adapter.py::TestSignalHandlerPreservation::test_sigint_handler_survives_connect -x` | ❌ Wave 0 |
| **ROADMAP gate 1** | Existing 6 cli_help_dispatch tests stay green | unit (subprocess) | `pytest robot_follow/tests/test_cli_help_dispatch.py -x` | ✅ |
| **ROADMAP gate 2** | Drone path unaffected — Phase 3's 12-plan suite stays green | unit | `pytest robot_follow/tests/ -x -k "not test_ros2_rover_adapter"` | ✅ |

### Sampling Rate
- **Per task commit:** `pytest robot_follow/tests/test_ros2_rover_adapter.py robot_follow/tests/test_cli_help_dispatch.py -x`
- **Per wave merge:** `pytest robot_follow/tests/ -x`
- **Phase gate:** Full suite green + `python -m robot_follow.robot_follow_app --robot drone --help` and `python -m robot_follow.robot_follow_app --robot rover --help` both exit 0 with expected flag visibility, before `/gsd:verify-work`.

### Wave 0 Gaps
- [ ] `robot_follow/tests/test_ros2_rover_adapter.py` — covers ROVER-01..04, ROVER-06..08 (does not yet exist; planner creates with xfail markers)
- [ ] `robot_follow/tests/test_cli_help_dispatch.py` — covers ROVER-05 (file exists; append 6 xfail-marked tests)
- [ ] Framework install: existing pytest setup in hailo-apps venv is sufficient — no new install needed

---

## Project Constraints (from CLAUDE.md)

| Directive | How Phase 4 honors it |
|-----------|-----------------------|
| Python package is `robot_follow`; primary console script is `robot-follow`; `drone-follow` is preserved alias | Phase 4 introduces nothing under `drone_follow`; new file is `robot_follow/robot_api/adapters/ros2_rover.py`. CLI dispatch already runs through `robot-follow` and `drone-follow` aliases (see Phase 3 plan 03-08 SUMMARY). Both names continue to invoke the same `main()`. |
| `robot_follow/robot_api/` is the actuator boundary — adapters live here | New adapter goes in `robot_follow/robot_api/adapters/ros2_rover.py` (sibling of `mavsdk_drone.py`). |
| `robot_follow/follow_api/` is pure domain — no third-party imports | Rover adapter imports FROM `follow_api.types` + `follow_api.config`, never reaches back the other way. Verified pattern: `mavsdk_drone.py:26-33`. |
| ReID + tracking + branching code is OUT of scope for adapter work | Phase 4 touches none of `pipeline_adapter/`, `reid_analysis/`, `servers/`. |
| `.claude/memory/MEMORY.md` documents `openhd_pairing.md` — never `--clean-start` | Not relevant to Phase 4 (no OpenHD code touched). |
| Boot service config `~/Desktop/drone-follow.conf` preserved | Phase 4 does not touch `scripts/boot/` or `start_air.sh`. The boot service would not currently launch with `--robot rover` (no flag in the conf); this is intentional — boot service is the air-unit drone path. |

---

## Sources

### Primary (HIGH confidence)
- `robot_follow/robot_api/robot.py:38-111` — the protocol the rover must implement
- `robot_follow/robot_api/adapters/mavsdk_drone.py:42-48, 269-290, 562-755` — reference implementation pattern for capability constant, `add_*_args`, and full adapter shape
- `robot_follow/robot_api/orchestrator.py:35-134` — generic loop (consumes Robot, needs zero changes for rover)
- `robot_follow/robot_follow_app.py:200-213, 484-561` — current placeholder + dispatch stub
- `robot_follow/follow_api/types.py:16-46` — `Axis`, `Capabilities`, `RobotCommand`, `SafetyContext` types
- `robot_follow/follow_api/controller.py:39-50, 77-120` — controller emits `yaw_rate` in `caps.yaw_unit` (Q5 lock)
- `setup_env.sh:39-55` — conditional ROS source block (Phase 3 plan 03-09; ROVER-04 prerequisite met)
- `robot_follow/tests/test_mavsdk_drone_adapter.py:1-120` — reference test pattern
- `robot_follow/tests/test_cli_help_dispatch.py:1-59` — reference CLI dispatch test pattern
- `robot_follow/tests/test_robot_protocol_shape.py:39-48` — reference protocol-conformance test
- `.planning/research/PITFALLS.md` — all 4 critical rclpy pitfalls (already-locked research)
- `.planning/phases/03-abstraction/03-RESEARCH.md:432-572` — Phase 3's paper sketch of `Ros2RoverAdapter`
- `.planning/REQUIREMENTS.md:75-83` — ROVER-01..08 specifications
- `.planning/ROADMAP.md:92-102` — Phase 4 success criteria

### Secondary (MEDIUM confidence — used to corroborate, no new claims)
- ROS Answers #342625 — rclpy publishers are thread-safe ([cited via PITFALLS source list])
- gazebosim/ros_gz README — Twist QoS defaults match `ros_gz_bridge` (only relevant for Phase 5, but informs the "depth=10, default reliability" choice in `start_session`)

### Tertiary (none)
No tertiary sources — every claim above is grounded in current-tree code or already-locked research.

---

## Metadata

**Confidence breakdown:**
- File structure + edit list: HIGH — verified on tree at HEAD `ac0edcc`
- rclpy lifecycle ordering: HIGH — PITFALLS Pitfalls 1-3 are already-locked research; Phase 3 paper-sketched the same shape and protocol-validated it
- send_command → Twist mapping: HIGH — Q5 lock in `robot.py:75-78` is unambiguous
- Defensive import: HIGH — PITFALLS Pitfall 2 covers both failure modes (module missing vs C-extension missing)
- Argparse split: HIGH — `add_drone_args` is the verbatim pattern at `mavsdk_drone.py:269-290`
- Test strategy: HIGH — `test_mavsdk_drone_adapter.py` and `test_cli_help_dispatch.py` are working references
- Risk register: MEDIUM — items are well-understood; mitigations are tested patterns

**Research date:** 2026-05-20
**Valid until:** 2026-06-20 (stable code-side scope; rclpy Humble API is in long-term support until May 2027 per REQUIREMENTS Out of Scope)
