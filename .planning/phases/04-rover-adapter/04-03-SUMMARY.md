---
phase: 04-rover-adapter
plan: 03
subsystem: robot_api
tags: [phase-4, rover, ros2, rclpy, adapter, twist, ROVER-01, ROVER-02, ROVER-03, ROVER-04, ROVER-06, ROVER-07, ROVER-08]
requires:
  - 04-01  # test scaffold + rclpy_mock fixture
  - 03-*   # Phase 3 Robot protocol + axes-only Capabilities + RobotCommand
provides:
  - Ros2RoverAdapter class implementing Robot protocol via rclpy + geometry_msgs/Twist
  - ROVER_CAPS = Capabilities(axes=frozenset({FORWARD, YAW}), yaw_unit="rad/s") constant
  - Lazy rclpy import pattern → module imports cleanly on no-ROS boxes
  - ROVER-04 friendly RuntimeError for missing rclpy (used as 04-05 operator gate)
  - Wire path for Plan 04-04 to wire into run_robot()
affects:
  - robot_follow/robot_api/adapters/ros2_rover.py (NEW, 341 lines)
  - robot_follow/robot_api/adapters/__init__.py (docstring update)
  - robot_follow/tests/test_ros2_rover_adapter.py (20 xfail markers stripped → 20 PASS)
tech-stack:
  added: []        # zero new pip / apt deps (rclpy + geometry_msgs are runtime-supplied via apt + source /opt/ros/humble/setup.bash)
  patterns:
    - Defensive lazy import pattern (try-import-inside-__init__ → friendly RuntimeError chain)
    - SignalHandlerOptions.NO + post-init handler assertion (preserves drone-follow's SIGINT handler)
    - Bounded spin_once(timeout_sec=0.05) + threading.Event shutdown signal (no rclpy.spin / executor.spin)
    - Idempotent shutdown ordering: shutdown_event.set() → thread.join(2.0) → node.destroy_node() → rclpy.try_shutdown()
key-files:
  created:
    - robot_follow/robot_api/adapters/ros2_rover.py
  modified:
    - robot_follow/robot_api/adapters/__init__.py
    - robot_follow/tests/test_ros2_rover_adapter.py
key-decisions:
  - "Axes-only Capabilities (no ALTITUDE for rover, no behavioral flags) per feedback_robot_abstraction_axes_only.md"
  - "Twist mapping has NO unit conversion — yaw_rate (rad/s per ROVER_CAPS.yaw_unit) → angular.z verbatim (Q5 lock)"
  - "on_target_lost publishes Twist() (all zeros) — rover does NOT yaw-spin (unlike drone); RINT-02 bottom-edge slow-near-edge deferred to Phase 6"
  - "Lazy import of rclpy + geometry_msgs INSIDE __init__ (not module top) — keeps module importable on no-ROS boxes for the drone path + unit tests"
  - "ControllerConfig is type-checked only (TYPE_CHECKING import) — adapter accepts the arg for signature symmetry with MavsdkDroneAdapter but does not use any field today"
requirements-completed: [ROVER-01, ROVER-02, ROVER-03, ROVER-04, ROVER-06, ROVER-07, ROVER-08]
duration: 15 min
completed: 2026-05-20
---

# Phase 4 Plan 03: Rover Adapter Implementation Summary

`Ros2RoverAdapter` + `ROVER_CAPS` land in a new module `robot_follow/robot_api/adapters/ros2_rover.py`, mounted as a SIBLING to `MavsdkDroneAdapter` under the same `robot_api/adapters/` package. Both adapters implement the same Robot protocol with NO shared code. The 20 Wave-0 xfail markers in `test_ros2_rover_adapter.py` are stripped en bloc and every test PASSES against the real adapter (rclpy mocked via the existing `rclpy_mock` fixture). Full suite went from 304 passed + 20 xfailed → **324 passed + 0 xfailed** — exactly +20 net new PASS, matching the plan's projection.

## Tasks executed (2)

| # | Type     | Commit  | Files                                                                                              |
| - | -------- | ------- | -------------------------------------------------------------------------------------------------- |
| 1 | feat     | `e863dea` | `robot_follow/robot_api/adapters/ros2_rover.py` (NEW, 341 lines); `robot_follow/robot_api/adapters/__init__.py` |
| 2 | test     | `56df101` | `robot_follow/tests/test_ros2_rover_adapter.py` (20 xfail decorators + XFAIL_REASON_ADAPTER constant + stale scaffold docstrings removed) |

## Confirmation: ros2_rover.py landed with the 6-method shape

```python
class Ros2RoverAdapter:
    caps: Capabilities = ROVER_CAPS  # class-level for type-checker

    def __init__(self, args, config: "ControllerConfig"):
    async def connect(self) -> None:
    async def start_session(self) -> None:
    def _spin_loop(self) -> None:                       # private; runs in self._executor_thread
    async def send_command(self, cmd: RobotCommand, safety_ctx: SafetyContext) -> None:
    async def send_zero(self) -> None:
    async def on_target_lost(self, last_detection: Optional[Detection]) -> None:
    async def shutdown(self) -> None:
```

## ROVER_CAPS contract verification

```python
# Module-level constant (axes-only — no ALTITUDE, no behavioral flags per
# feedback_robot_abstraction_axes_only.md)
ROVER_CAPS: Capabilities = Capabilities(
    axes=frozenset({Axis.FORWARD, Axis.YAW}),
    yaw_unit="rad/s",
)

# Asserted in unit test test_caps_is_rover_caps + verification CLI:
assert ROVER_CAPS.axes == frozenset({Axis.FORWARD, Axis.YAW})  # PASS
assert ROVER_CAPS.yaw_unit == "rad/s"                          # PASS
assert Axis.ALTITUDE not in ROVER_CAPS.axes                    # PASS (ROVER-07)
```

CLI verification output (re-run for the SUMMARY):
```
module imports cleanly
ROVER_CAPS contract OK
class attr caps is ROVER_CAPS: True
```

## Lazy-import contract (lazy-only) — verified empty

```
$ grep -nE '^(import|from) (rclpy|geometry_msgs)' robot_follow/robot_api/adapters/ros2_rover.py
(no output — zero matches)
```

The only `import rclpy` / `from geometry_msgs.msg import Twist` lines in the module live INSIDE `Ros2RoverAdapter.__init__`, inside the `try/except ImportError` block. Module-level imports stay at stdlib + `robot_follow.follow_api.types` only. This is what lets the test suite construct the adapter with `rclpy_mock` (via `monkeypatch.setitem(sys.modules, "rclpy", MagicMock())`) on this no-rclpy dev box.

## ROVER-04 friendly RuntimeError message (verbatim — for 04-05 operator gate grep)

The defensive `except ImportError` block in `Ros2RoverAdapter.__init__` raises:

```
ROS 2 not available — the rover adapter requires rclpy + geometry_msgs.
Fix:
  1) sudo apt install ros-humble-ros-base ros-humble-geometry-msgs
  2) source /opt/ros/humble/setup.bash
  3) re-source setup_env.sh
Underlying error: {e}
```

The 04-05 operator gate test will grep this string for the two locked substrings: `"ROS 2 not"` and `"source /opt/ros/humble/setup.bash"`. Both are present, and the unit test `test_friendly_error_when_rclpy_missing` (in `TestImportSafety`) PASSES.

## Test results

### test_ros2_rover_adapter.py (the 6-class scaffold from 04-01)

```
============================= test session starts ==============================
collected 20 items

TestImportSafety::test_module_imports_cleanly_without_rclpy            PASSED
TestImportSafety::test_friendly_error_when_rclpy_missing                PASSED
TestProtocolShape::test_implements_robot_protocol                       PASSED
TestProtocolShape::test_caps_is_rover_caps                              PASSED
TestProtocolShape::test_has_six_methods                                 PASSED
TestSignalHandlerPreservation::test_connect_calls_init_with_signal_handler_options_no  PASSED
TestSignalHandlerPreservation::test_sigint_handler_survives_connect     PASSED
TestTwistPublish::test_publishes_twist_with_forward_and_yaw_rate        PASSED
TestTwistPublish::test_send_command_no_conversion_yaw_rate              PASSED
TestTwistPublish::test_send_command_short_circuits_on_target_lost       PASSED
TestTwistPublish::test_send_zero_publishes_all_zeros                    PASSED
TestTwistPublish::test_on_target_lost_publishes_zero_twist              PASSED
TestLifecycle::test_start_session_creates_node_and_publisher            PASSED
TestLifecycle::test_start_session_starts_executor_thread                PASSED
TestLifecycle::test_shutdown_idempotent                                 PASSED
TestLifecycle::test_shutdown_orders_node_destroy_before_try_shutdown    PASSED
TestLifecycle::test_shutdown_sets_shutdown_event                        PASSED
TestCustomCliArgs::test_custom_cmd_vel_topic                            PASSED
TestCustomCliArgs::test_custom_namespace                                PASSED
TestCustomCliArgs::test_domain_id_sets_env                              PASSED

============================== 20 passed in 0.32s ==============================
```

**~20 PASS / 0 xfail / 0 FAIL** — matches plan projection.

### Full suite delta

| Phase    | Passed | xfailed | Skipped | Delta                  |
| -------- | ------ | ------- | ------- | ---------------------- |
| Pre-plan | 304    | 20      | 1       | baseline               |
| Task 1   | 304    | 0 (20 xpassed) | 1 | adapter lands; markers still active so xfails turn into xpass |
| Task 2   | **324** | **0** | 1     | **+20 net new PASS**   |

Plus `grep -c "xfail" robot_follow/tests/test_ros2_rover_adapter.py` returns **0** (Phase 2 02-05 "nothing left behind" discipline upheld) and `grep -c "XFAIL_REASON_ADAPTER"` returns **0** (stale constant removed).

## Architectural locks verified (empty diffs)

```
$ git diff --name-only HEAD~2 HEAD -- robot_follow/robot_api/adapters/mavsdk_drone.py
(empty — drone adapter byte-identical across both 04-03 commits)

$ git diff --name-only HEAD~2 HEAD -- robot_follow/robot_follow_app.py
(empty — Plan 04-04 territory, not touched)

$ git diff --name-only HEAD~2 HEAD -- robot_follow/follow_api/
(empty — axes-only Capabilities contract preserved; follow_api stays types-only)
```

All three architectural locks held. The complete file list for this plan is exactly:

```
$ git diff --name-only HEAD~2 HEAD
robot_follow/robot_api/adapters/__init__.py
robot_follow/robot_api/adapters/ros2_rover.py
robot_follow/tests/test_ros2_rover_adapter.py
```

## Behavioral notes worth recording

### send_command — no-conversion Twist mapping (ROVER-06)

```python
twist.linear.x  = cmd.forward_m_s   # m/s direct (both sides m/s)
twist.linear.y  = 0.0               # non-holonomic rover
twist.linear.z  = 0.0               # no ALTITUDE axis (ROVER-07)
twist.angular.x = 0.0               # no ROLL
twist.angular.y = 0.0               # no PITCH
twist.angular.z = cmd.yaw_rate      # rad/s direct — NO conversion (Q5 lock)
self._publisher.publish(twist)
```

`send_command` short-circuits on `safety_ctx.target_lost` (Q6 lock) BEFORE any publish, satisfying `Robot.send_command`'s contract at `robot.py:67-77`. The Twist is also a no-op when `self._publisher is None` (partial-init defense).

### on_target_lost — rover STOPS (does not yaw-spin)

Unlike `MavsdkDroneAdapter.on_target_lost`, which spins toward the last-seen side, `Ros2RoverAdapter.on_target_lost` publishes a zero Twist. The `last_detection` argument is intentionally unused (no scan behavior in v1.1). The method docstring carries the `# TODO(Phase 6 / RINT-02)` marker for the future bottom-edge slow-near-edge tuning.

### Shutdown ordering (idempotent, per PITFALLS "Looks Done But Isn't")

1. `self._shutdown_event.set()`  → spin loop exits at the next iteration (bounded by `spin_once(timeout_sec=0.05)`)
2. `self._executor_thread.join(timeout=2.0)`  → bounded wait; logs a warning if thread is still alive, but never hangs (the daemon=True thread cannot outlive the process)
3. `self._node.destroy_node()`  → wrapped in try/except for partial-init safety
4. `self._rclpy.try_shutdown()` (NOT `shutdown()`)  → idempotent variant; safe to call multiple times
5. NULL-OUT `self._node` / `self._publisher` / `self._executor` / `self._executor_thread` so subsequent `send_command` / `send_zero` / `on_target_lost` early-return cleanly

`test_shutdown_orders_node_destroy_before_try_shutdown` attaches a parent mock and asserts `destroy_node` precedes `try_shutdown` in the recorded call list — PASSED.

### SIGINT handler preservation (ROVER-02 / ROVER-08)

`connect()` captures `signal.getsignal(SIGINT)` before `rclpy.init`, passes `signal_handler_options=SignalHandlerOptions.NO`, then asserts the post-init handler `is` the pre-init handler. If clobbered, raises `ConnectionError` (which `Robot.connect`'s docstring at `robot.py:48` says the orchestrator catches and exits cleanly). `test_sigint_handler_survives_connect` locks this from the test side; `test_connect_calls_init_with_signal_handler_options_no` locks the kwarg shape.

## Next-step readiness for Plan 04-04

The wire path is now ready. Plan 04-04 will replace the `NotImplementedError` stub in `robot_follow_app.run_robot()` with construction:

```python
elif args.robot == "rover":
    from robot_follow.robot_api.adapters.ros2_rover import Ros2RoverAdapter
    robot = Ros2RoverAdapter(args, controller_config)
```

Plan 04-02 already registered the argparse surface (`--cmd-vel-topic`, `--ros-namespace`, `--ros-domain-id`) AND the `args.robot ∈ {drone, rover}` selector. Plan 04-03 has now provided the adapter class itself. Plan 04-04 just needs the dispatch + `caps` propagation into the controller (Phase 3 already wired the controller-side gating).

## Planner note for Phase 6 (RINT-01 + RINT-02)

Per RESEARCH § "Planner note for Phase 6": the rover sim will spin uncontrollably with the default `ControllerConfig` because `max_yawspeed=90` was tuned in deg/s for the drone. Phase 6 RINT-01 ships `configs/rover_simulation.json` with rad/s tuning. Phase 6 RINT-02 ships bottom-edge slow-near-edge for the rover (TODO marker placed in `on_target_lost` docstring). Phase 4 ships the wire; Phase 6 ships the tuning + safety overlay. This is the intentional split — do NOT add Phase-6 tuning to Phase 4.

## Deviations from Plan

None — plan executed exactly as written. Both tasks committed via pathspec; both architectural-lock paths (`mavsdk_drone.py`, `robot_follow_app.py`, `follow_api/`) byte-identical across both commits. Net suite delta matches projection (+20 PASS). The test docstring tidy-up (removing stale "xfail" / "scaffold" phrasing) was already inside the plan's task-2 scope ("Update the module docstring … remove the 'Phase 4 Wave 0 scaffold' + 'xfail markers will be stripped in Plan 04-03' language").

Authentication gates: none — adapter requires no auth surface.

## Self-Check: PASSED

- [x] `robot_follow/robot_api/adapters/ros2_rover.py` exists on disk (`test -f` returns 0)
- [x] `git log --oneline --grep="04-03"` lists both `e863dea` (feat) and `56df101` (test) — 2 commits, both via pathspec, no protected-path leakage
- [x] All 20 acceptance criteria from `<success_criteria>` re-verified post-Task-2:
  - ros2_rover.py exists; ROVER_CAPS = {FORWARD, YAW} + rad/s + no ALTITUDE; 0 module-top rclpy imports; defensive RuntimeError contains both locked substrings; 6 Robot methods + caps; `isinstance(adapter, Robot)` true; connect uses SignalHandlerOptions.NO + post-init assertion; _spin_loop uses spin_once(timeout_sec=0.05); send_command short-circuits on target_lost + no conversion; on_target_lost publishes Twist() with TODO marker; shutdown idempotent + correct order; threading.Event (NOT asyncio.Event); daemon Thread; 20 PASS / 0 xfail / 0 FAIL; full suite +20 PASS; mavsdk_drone.py / robot_follow_app.py / follow_api/ byte-identical; adapters/__init__.py docstring updated
- [x] Plan-level `<verification>` block re-run successfully (no exit code != 0)
