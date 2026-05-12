# Architecture Research — Rover Support (v1.1)

**Domain:** rclpy-asyncio bridge + robot abstraction + Gazebo rover sim
**Researched:** 2026-05-12
**Confidence:** HIGH (codebase read directly; ROS 2 patterns verified against official docs and community sources)

---

## 1. rclpy-asyncio Bridge: Which Pattern

**Decision: Option A — background thread running `executor.spin()`, sync `publisher.publish()` called from async code.**

Why:

- `publisher.publish()` IS thread-safe in rclpy (the underlying rcl publisher is protected by a lock). Calling it from any thread — including an asyncio coroutine running on the asyncio event loop thread — is safe. The [rclpy source](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/publisher.py) confirms this; `rcl_publish` itself acquires a lock.
- Option B (`spin_once(timeout_sec=0)` polled from asyncio task) is CPU-wasteful: you must call it fast enough to service inbound callbacks but any sleep wastes latency. ROS 2 docs warn against polling spin_once in a loop.
- Option C (rclpy.task.Future bridging) is only needed when you need to `await` ROS service responses or action results. For a publish-only adapter it adds zero value and has [known limitations on Humble](https://github.com/ros2/rclpy/issues/1461): the asyncio executor feature was experimental in Humble and not backported.

**For the publish-only rover adapter (v1.1):**

```python
# robot_api/adapters/ros2_rover.py

import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist

class Ros2RoverAdapter:
    def __init__(self, node_name: str = "robot_follow_rover"):
        rclpy.init()
        self._node = Node(node_name)
        self._pub = self._node.create_publisher(Twist, "/cmd_vel", 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="rclpy-spin"
        )

    def start(self):
        self._thread.start()

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        # Safe to call from asyncio loop thread — rcl_publish is lock-protected
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._pub.publish(msg)  # thread-safe

    def stop(self):
        self._executor.shutdown()
        rclpy.shutdown()
```

**Why `SingleThreadedExecutor` not `MultiThreadedExecutor`:** For a pure-publisher node with no subscriptions or service handlers, `SingleThreadedExecutor` is sufficient and avoids the [known CPU spike issue](https://github.com/ros2/rclpy/issues/1223) with `MultiThreadedExecutor` even under idle.

**When you add `/odom` subscription (v1.2 or later):** Still keep Option A. Add the subscription to the same node; the background executor thread fires the subscription callback. To get odom data into the asyncio loop, use `asyncio.get_event_loop().call_soon_threadsafe(loop.create_task, coro)` or write into a `threading.Event` + shared dict that the asyncio control loop reads (same pattern as `altitude_cache` dict in `mavsdk_drone.py:_telemetry_altitude_task` — see lines 346-353). Do NOT use `asyncio.run_coroutine_threadsafe` unless you keep a reference to the asyncio loop, which is architecturally fragile.

**Reference:** `m2-farzan/ros2-asyncio` (GitHub) is the most-cited FOSS example of mixing rclpy + asyncio; it wraps exactly this pattern — rclpy node on a background thread, asyncio on the main thread, communicating via thread-safe queues.

---

## 2. Robot Protocol Surface

**New file:** `robot_follow/robot_api/robot.py`

```python
from typing import Protocol, runtime_checkable
from dataclasses import dataclass
from enum import Enum

class YawUnits(Enum):
    DEG_S = "deg_s"   # drone: PX4 yawspeed_deg_s
    RAD_S = "rad_s"   # rover: ROS Twist angular.z

@dataclass(frozen=True)
class Capabilities:
    has_altitude: bool          # drone=True, rover=False
    needs_offboard_handshake: bool  # drone=True, rover=False
    needs_takeoff_landing: bool     # drone=True, rover=False
    yaw_units: YawUnits

@dataclass
class RobotCommand:
    forward_m_s: float
    yawspeed: float             # units given by Capabilities.yaw_units
    altitude_m_s: float = 0.0  # ignored when not Capabilities.has_altitude

@runtime_checkable
class Robot(Protocol):
    @property
    def capabilities(self) -> Capabilities: ...
    async def start_session(self, shutdown: asyncio.Event) -> None: ...
    async def send_command(self, cmd: RobotCommand) -> RobotCommand: ...
    async def send_zero(self) -> None: ...
    def reset_filters(self) -> None: ...
```

**Design rationale:**

- `capabilities` is a property on the robot object (not constructor injection) because the composition root (`drone_follow_app.py`) doesn't know capabilities at parse-time — it depends on which adapter was instantiated. The controller reads `robot.capabilities` once after `start_session` to gate altitude-hold and offboard logic.
- `send_command` is `async` because the drone adapter calls `await drone.offboard.set_velocity_body(...)` and must stay async. The rover adapter calls the sync `publisher.publish()` inside an `async def`, which is fine — sync-safe publish in an async frame does not block the event loop.
- `send_zero` is separate from `send_command(zero)` because it also resets filter state (mirrors current `VelocityCommandAPI.send_zero()` at `mavsdk_drone.py:151`).
- `RobotCommand` replaces `VelocityCommand`. Keep `down_m_s` renamed to `altitude_m_s` and default 0 — this makes rover code read naturally (`altitude_m_s=0.0` is not a lie, it is just unused), and the existing controller's `VelocityCommand(forward, 0.0, yawspeed)` maps directly. Three-field shape is cleanest for the web UI telemetry path (see §4 below).

---

## 3. Config Split

**Decision: Option B — keep one `ControllerConfig`, mark flight-only fields `Optional[float]`, gate by `capabilities.has_altitude`.**

Why Option A loses:

- `ControllerConfig` is already used as a live-mutable shared object across the web UI, follow server, OpenHD bridge, and control loop (see `drone_follow_app.py:334`). Splitting it into two classes makes `load_from_file` / `save_json` / `from_args` non-trivial: you now need composite JSON schemas and the web UI slider path (`ui_state.update_velocity`) becomes aware of which sub-config to mutate.
- The `from_args` method (config.py:256) calls `ControllerConfig.add_args(parser)` which registers all flags. If altitude/offboard fields are on a separate `FlightConfig`, that class needs its own `add_args` and the composition root must call both — adding lines to `drone_follow_app.py:_build_parser()` and an import-time `FlightConfig` reference that leaks flight knowledge into the root.

**Concrete change for Option B:**

Mark these fields `Optional[float]` with `None` default in the dataclass:
- `target_altitude`, `kp_alt_hold`, `min_altitude`, `max_altitude`, `max_climb_speed`, `max_down_speed`

In `__post_init__` validate only when not None. In `from_args`, keep them as is — they fall back to dataclass defaults unless the user passes the flag, so rover configs simply omit them. In `live_control_loop` (mavsdk_drone.py:413), the altitude-hold block already guards on `if current_alt is not None` — add a second guard on `capabilities.has_altitude` (passed as a constructor arg to `VelocityCommandAPI` or threaded via `ControllerConfig`).

This avoids a two-class refactor while keeping the controller's JSON save/load format stable for existing drone users.

---

## 4. RobotCommand Shape

**Decision: Option B — `RobotCommand(forward, yawspeed, altitude_m_s=0.0)`, drop `down_m_s` name, no separate types.**

Why:

- Option A (`down=0` as rover "ignored") is fine semantically but `down_m_s` is a confusing name on a ground vehicle.
- Option C (two separate types) breaks the single boundary the existing code already has. `ui_state.update_velocity(cmd.forward_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, mode)` at mavsdk_drone.py:521 — this web UI telemetry call already passes three floats positionally. Keeping three fields means that line changes from attribute names only (`.down_m_s` → `.altitude_m_s`), not shape.
- The OpenHD bridge reads velocity from `ui_state`, not directly from the command type, so it is not affected.
- `yawspeed` is deliberately unitless in the struct — the rover adapter converts from deg/s (what the controller produces) to rad/s (`geometry_msgs/Twist` angular.z) inside `send_command`, not in `compute_velocity_command`. The controller stays in deg/s because that is how it was calibrated; unit conversion is an adapter concern.

**Migration path:** `VelocityCommand` stays in `follow_api/types.py` as a type alias or renamed to `RobotCommand`. No controller logic changes needed since `compute_velocity_command` already returns `VelocityCommand(forward, 0.0, yawspeed)` — just rename the class and the `down_m_s` field.

---

## 5. Gazebo Rover Sim Layout

**Decision: Option A — new `sim/rover/` directory, parallel to `sim/`.**

Why:

- Option B (`--vehicle drone|rover` flag in `sim/`) sounds clean until you look at what they dispatch to: drone needs `make px4_sitl gz_x500_vision` (a C++ build invocation), rover needs `ros2 launch rover_sim rover_world.launch.py` (a ROS 2 launch system). These are fundamentally different process trees; a single script dispatching to both bloats `start_sim.sh` with branches that have nothing in common.
- `sim/rover/` mirrors exactly how the existing `sim/` is structured (setup, start, bridge, worlds, configs) so the mental model transfers. A user who knows `sim/start_sim.sh --bridge --world X` can immediately read `sim/rover/start_rover_sim.sh --world X` without a new flag grammar.
- Stale-process reaping, bridge PID tracking, and the cleanup trap (start_sim.sh:172-179) are identical patterns that should be copy-edited into `sim/rover/start_rover_sim.sh` — no shared entrypoint needed.

**Proposed layout:**

```
sim/rover/
├── setup_rover_sim.sh          # one-time: verify ros-humble, ros-gz-garden, build check
├── start_rover_sim.sh          # launch gz sim + ros_gz_bridge + camera-udp-shim
├── rover.sdf                   # diff-drive rover body + camera + diff_drive plugin
├── worlds/
│   ├── person_in_front.sdf     # reuse/symlink actor SDF from sim/worlds/
│   └── 2_person_world.sdf
├── bridge/
│   └── camera_udp_shim.py      # ros sensor_msgs/Image → GStreamer UDP H.264 (see §6)
└── configs/
    └── rover_simulation.json   # ControllerConfig defaults for rover
```

---

## 6. Camera-to-Video Flow in Rover Sim

**Decision: Option B — `ros_gz_image_bridge` → `sensor_msgs/Image` ROS topic → thin Python shim that feeds GStreamer appsrc → UDP H.264 RTP.**

Why:

- Option A (gz topic raw JSON) is not viable: `gz topic --json-output` for a camera produces base64-encoded image data in a JSON wrapper; the parse+decode overhead at 30fps is prohibitive and there is no maintained tool that does this reliably.
- Option C (native gz-streaming RTP plugin) does not exist in Gazebo Garden. Gazebo's camera sensor publishes to gz-transport only; there is no built-in RTP sink.
- Option D (gst plugin in gz sim) does not exist.
- Option B reuses the existing `sim/bridge/video_bridge.py` architecture almost verbatim. The only difference is the source: instead of subscribing directly to a gz-transport `Image` topic via `gz.transport13.Node`, the shim subscribes to the bridged `sensor_msgs/Image` ROS topic via rclpy. The GStreamer encode-to-UDP tail is identical.

**Concrete shim (`sim/rover/bridge/camera_udp_shim.py`):**

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
import numpy as np, cv2, gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class CameraUdpShim(Node):
    def __init__(self, host, port, fps, bitrate):
        super().__init__("camera_udp_shim")
        self._host, self._port, self._fps, self._bitrate = host, port, fps, bitrate
        self._pipeline = None
        self._appsrc = None
        self._fc = 0
        self.create_subscription(RosImage, "/camera/image_raw", self._on_image, 1)

    def _on_image(self, msg: RosImage):
        w, h = msg.width, msg.height
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        if self._pipeline is None:
            self._pipeline, self._appsrc = _create_pipeline(
                w, h, self._fps, self._bitrate, self._host, self._port)
        buf = Gst.Buffer.new_wrapped(frame.tobytes())
        buf.pts = self._fc * Gst.SECOND // self._fps
        self._appsrc.emit("push-buffer", buf)
        self._fc += 1
```

The bridge config in `start_rover_sim.sh` runs:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image
```

This keeps `drone-follow --input udp://0.0.0.0:5600` working with zero changes to `pipeline_adapter/`.

**Alternative if ros_gz_bridge has issues:** the gz-transport direct subscription path (as used by the existing `video_bridge.py`) also works for the rover sim since Gazebo Garden is still the simulator. The ros_gz_bridge hop is only needed if ros_gz is the natural starting point for the rover's cmd_vel bridge anyway.

---

## 7. Composition Root Impact

**Recommendation: keep one `run_robot()` function (rename from `run_drone()`), dispatch via the `Robot` protocol.**

Current pattern in `drone_follow_app.py:384-399`:
```python
def run_drone():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(
            run_live_drone(args, shared_state, shutdown, config=controller_config, ...))
    except Exception:
        LOGGER.warning("[drone] Drone connection failed ...")
    finally:
        loop.close()

drone_thread = threading.Thread(target=run_drone, daemon=True)
drone_thread.start()
```

The rover adapter's `start_session(shutdown)` coroutine fits this pattern exactly: it connects (rclpy init, spin thread start), runs the control loop (same `live_control_loop` reused or a thin wrapper), and exits cleanly when `shutdown` is set. The composition root becomes:

```python
def run_robot():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(
            robot.start_session(shared_state, shutdown, config=controller_config, ...))
    except Exception:
        LOGGER.warning("[robot] Robot session failed ...", exc_info=True)
    finally:
        loop.close()

robot_thread = threading.Thread(target=run_robot, daemon=True)
robot_thread.start()
```

`robot` is instantiated before this block based on `args.robot` (`drone` → `MavsdkDroneAdapter`, `rover` → `Ros2RoverAdapter`). The MAVSDK adapter wraps `run_live_drone` behind `start_session`; the rover adapter wraps its rclpy spin + cmd_vel publish loop the same way.

**Shutdown lifecycle is identical:** both adapters respect `shutdown: asyncio.Event`. The rover adapter's `stop()` method calls `executor.shutdown()` + `rclpy.shutdown()` in its `finally` block (same role as `_land_safely` + `DetachedMavsdkServer.__exit__`). The existing `drone_thread.join(timeout=5.0)` + pkill pattern in `drone_follow_app.py:433-445` becomes `robot_thread.join(timeout=5.0)` — the pkill block is gated on `getattr(args, 'robot', 'drone') == 'drone'` or moved into the adapter's cleanup.

---

## 8. CLI Args: Where to Put Adapter-Specific Flags

**Recommendation: each adapter registers its own `add_*_args(parser)` only when that adapter is selected; use a two-pass parse with a sentinel pre-parse.**

Current `_build_parser()` in `drone_follow_app.py:159` calls `add_drone_args(parser)` unconditionally. The pattern already used for pre-parsing UI flags (the `ui_pre` ArgumentParser at line 206) can be reused:

```python
# Step 1: pre-parse --robot to know which adapter to load
robot_pre = argparse.ArgumentParser(add_help=False)
robot_pre.add_argument("--robot", default="drone", choices=["drone", "rover"])
robot_pre_args, _ = robot_pre.parse_known_args()

# Step 2: in _build_parser(), dispatch:
if robot_pre_args.robot == "drone":
    add_drone_args(parser)
elif robot_pre_args.robot == "rover":
    from robot_follow.robot_api.adapters.ros2_rover import add_rover_args
    add_rover_args(parser)
```

**Why not mutually-exclusive groups:** `argparse` mutual-exclusion groups enforce that at most one flag from a group is used in a single invocation. They do not prevent both groups' flags from being registered, so `--serial` still appears in `--help` even when `--robot rover`. The pre-parse dispatch avoids this — rover users never see drone connection flags in `--help`.

**Rover-specific flags to register:**

```python
def add_rover_args(parser) -> None:
    group = parser.add_argument_group("rover")
    group.add_argument("--ros-domain-id", type=int, default=0,
                       help="ROS_DOMAIN_ID for the rover node (default: 0)")
    group.add_argument("--cmd-vel-topic", default="/cmd_vel",
                       help="ROS topic to publish Twist commands (default: /cmd_vel)")
    group.add_argument("--max-angular-z", type=float, default=1.5,
                       help="Clamp on angular.z in rad/s (default: 1.5)")
```

---

## Component Boundaries Summary

```
drone_follow_app.py (composition root)
    │
    ├── pipeline_adapter/          UNCHANGED — emits SharedDetectionState
    ├── follow_api/controller.py   UNCHANGED — returns RobotCommand (renamed VelocityCommand)
    ├── follow_api/config.py       LIGHT CHANGE — altitude fields Optional, validate gated
    ├── servers/                   UNCHANGED — reads ui_state, not RobotCommand directly
    │
    ├── robot_api/robot.py         NEW — Robot protocol + Capabilities + RobotCommand
    ├── robot_api/adapters/
    │   ├── mavsdk_drone.py        MOVED from drone_api/ + wrapped behind Robot.start_session
    │   └── ros2_rover.py          NEW — rclpy Node, background spin thread, sync publish
    │
    └── sim/rover/                 NEW — parallel to sim/, no shared entrypoint
        ├── start_rover_sim.sh
        ├── rover.sdf
        └── bridge/camera_udp_shim.py
```

**Thread / loop ownership (unchanged from existing):**

| Thread | What runs there | Cross-boundary comms |
|--------|----------------|----------------------|
| Main | GStreamer app.run() | writes SharedDetectionState (via GIL-safe dict) |
| robot_thread | asyncio event loop → Robot.start_session | reads SharedDetectionState; writes ui_state |
| rclpy-spin | SingleThreadedExecutor.spin() | called by Ros2RoverAdapter.start(); publisher.publish() called from robot_thread |
| HTTP threads | FollowServer / WebServer | read/write ControllerConfig, ui_state (thread-safe by design) |

---

## Build Order for Phases

1. **Phase 1 — Protocol + rename:** `robot_api/robot.py` (Robot, Capabilities, RobotCommand), rename `VelocityCommand` → `RobotCommand` in types.py, update controller.py + mavsdk_drone.py. No behaviour change.
2. **Phase 2 — Config cleanup:** mark altitude fields Optional, add `capabilities.has_altitude` gate in live_control_loop, rover-safe validate.
3. **Phase 3 — Drone adapter behind protocol:** wrap `run_live_drone` behind `MavsdkDroneAdapter.start_session`; wire `run_robot()` in app.py. Drone path still works identically.
4. **Phase 4 — Rover adapter:** `ros2_rover.py` + `add_rover_args`; two-pass CLI dispatch in `_build_parser`.
5. **Phase 5 — Rover sim:** `sim/rover/rover.sdf`, `start_rover_sim.sh`, `camera_udp_shim.py`, rover world SDF files.
6. **Phase 6 — End-to-end test:** rover sim + drone-follow `--robot rover` + webui, person-follow in Gazebo.

Phase 3 is the critical dependency gate: the Robot protocol must be stable before either adapter is written, and the composition root change must be tested with the drone before the rover adapter is added.

---

## Sources

- [rclpy Publisher thread safety — rcl_publish lock](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/publisher.py)
- [rclpy asyncio executor feature request / limitations (Humble)](https://github.com/ros2/rclpy/issues/1461)
- [MultiThreadedExecutor CPU spike issue](https://github.com/ros2/rclpy/issues/1223)
- [m2-farzan/ros2-asyncio — background thread + asyncio pattern](https://github.com/m2-farzan/ros2-asyncio)
- [rclpy.executors module — official docs](https://docs.ros.org/en/iron/p/rclpy/rclpy.executors.html)
- [ros_gz_bridge integration — Gazebo official docs](https://gazebosim.org/docs/latest/ros2_integration/)
- [ros_gz camera bridge example](https://github.com/arashsm79/ros-ign-gazebo-camera)

---
*Architecture research for: hailo-drone-follow rover support (v1.1)*
*Researched: 2026-05-12*
