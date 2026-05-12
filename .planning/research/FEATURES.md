# Feature Research

**Domain:** ROS 2 cmd_vel rover adapter — sim-only, TurtleBot3-style differential drive
**Researched:** 2026-05-12
**Confidence:** HIGH (core Twist/cmd_vel conventions), MEDIUM (sim camera path, rclpy/asyncio bridging)

---

## Twist Message Conventions (HIGH confidence)

`geometry_msgs/msg/Twist` — standard across every differential-drive rover (TurtleBot3, Husky, ROSbot, NAV2).

| Field | Meaning | Units | Sign |
|-------|---------|-------|------|
| `linear.x` | Forward / backward | m/s | + = forward |
| `linear.y` | Lateral (unused for diff-drive) | m/s | — |
| `linear.z` | Vertical (unused for ground robots) | m/s | — |
| `angular.x` | Unused | rad/s | — |
| `angular.y` | Unused | rad/s | — |
| `angular.z` | Yaw rate (CCW positive — right-hand rule) | rad/s | + = CCW = turn left |

**Mapping from `VelocityCommand`:**
- `forward_m_s` → `linear.x` (direct, same units)
- `yawspeed_deg_s` → `angular.z` after `* math.pi / 180` (controller emits deg/s; ROS expects rad/s)
- `down_m_s` → ignored (ground robot has no vertical axis)

No unit ambiguity: deg/s conversion is the only transform needed at the adapter boundary.

---

## Topic Name Conventions (HIGH confidence)

| Topic | When used |
|-------|-----------|
| `/cmd_vel` | Default for all TurtleBot3, Husky, ROSbot, `diff_drive_controller`. Use this. |
| `/cmd_vel_nav` | Nav2 output before twist_mux. Not relevant (we skip Nav2). |
| `/cmd_vel_smoothed` | Nav2 internal smoothed output. Not relevant. |
| `/cmd_vel_joy`, `/cmd_vel_key` | twist_mux input channels for teleop sources. Not needed for v1.1. |

**Default for v1.1 sim:** publish to `/cmd_vel` directly. No namespace, no remapping, no mux. The Gazebo `DiffDrive` plugin subscribes to `cmd_vel` by convention; `ros_gz_bridge` bridges `geometry_msgs/msg/Twist@gz.msgs.Twist` on that topic.

**Note on TwistStamped:** `ros2_control` `diff_drive_controller` (rolling) now prefers `TwistStamped`. Gazebo's built-in `DiffDrive` plugin still accepts plain `Twist`. For sim-only using the Gazebo plugin (not ros2_control), use plain `Twist`.

---

## Frame Convention (HIGH confidence)

`base_link` is the standard body frame for diff-drive robots. `cmd_vel` is interpreted in `base_link`: +x = forward along robot heading, +z-angular = CCW yaw. `base_footprint` is a projection of `base_link` onto the ground plane; only matters for TF/URDF. Our adapter never needs to know the frame name — it just publishes the Twist and the sim plugin interprets it.

The `forward + yaw` command model from `compute_velocity_command` maps cleanly to `linear.x + angular.z`. No lateral or vertical components are used.

---

## Feature Landscape

### Table Stakes (Users Expect These)

| Feature | Why Expected | Complexity | Notes |
|---------|--------------|------------|-------|
| Publish `geometry_msgs/Twist` to `/cmd_vel` | Every diff-drive rover expects this. Missing = rover does not move. | LOW | `VelocityCommand.forward_m_s` → `linear.x`; `yawspeed_deg_s * π/180` → `angular.z` |
| Background-thread rclpy executor | rclpy blocks if not spun; asyncio owns the main loop in drone-follow. | LOW | `threading.Thread` + `rclpy.spin(node)` pattern; thread-safe publish via `node.create_publisher` (rclpy publish is thread-safe) |
| `--robot rover` CLI flag wires ros2_rover adapter | Without this, no way to run rover mode. | LOW | Mirrors `--robot drone` path; delegates to `ros2_rover.py` adapter |
| Gazebo Garden rover SDF with DiffDrive plugin | Sim needs a robot that responds to `/cmd_vel`. | LOW | Standard `gz::sim::systems::DiffDrive` plugin in SDF |
| Camera → UDP H.264 shim for Gazebo rover sim | drone-follow pipeline expects `--input udp://0.0.0.0:5600`. Reuse existing `sim/bridge/video_bridge.py` pattern. | LOW | Rover SDF adds a camera sensor; `video_bridge.py` topics arg points to rover camera topic |
| `ros_gz_bridge` for cmd_vel (ROS → Gazebo) | Without it, Gazebo DiffDrive plugin never receives the Twist. | LOW | `ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist` |
| `sim/rover/start_rover_sim.sh` launch script | Operators need one-command sim startup (matches `start_sim.sh` pattern). | LOW | Launches: `gz sim`, `ros_gz_bridge`, `video_bridge.py`, optional MAVLink relay |
| Rover safety defaults config (`configs/rover_simulation.json`) | Frame-edge safety semantics differ: bottom-edge means "person is close, slow" not "tilt warning". | LOW | `bottom_margin_safety` tuned lower; `yaw_only=false`; `max_forward` / `max_backward` reduced for ground |

### Differentiators (Add in v1.2+)

| Feature | Value Proposition | Complexity | Notes |
|---------|-------------------|------------|-------|
| twist_mux integration | Allows teleop override of follow controller; matches production robot pattern (Husky, ROSbot). | MEDIUM | twist_mux sits between follow adapter output and `/cmd_vel`; priorities: e-stop > teleop > follow. Not needed for sim-only. |
| Odometry consumption (`/odom`) | Enables closed-loop distance estimation; improves safety when bbox is unreliable. | MEDIUM | Requires subscribing to `nav_msgs/Odometry`; needs integration with controller. |
| Multi-rover namespace support | Run follow on `/rover1/cmd_vel`, `/rover2/cmd_vel` simultaneously. | MEDIUM | Parameterise topic as `--rover-cmd-vel-topic`. |
| Rover-specific search behaviour | On target loss, rover spins in place (yaw search) rather than drifting forward. | LOW | Already supported by `search_yawspeed_slow` in controller; just needs correct config value. |
| Hardware rover path (v1.2) | Connect to a real differential-drive robot (e.g. ROSbot, TurtleBot3 Burger/Waffle). | LOW | Same adapter; just point at real robot's `/cmd_vel`. No code change expected. |

### Anti-Features (Explicitly Exclude)

| Feature | Why Requested | Why Problematic | What to Do Instead |
|---------|---------------|-----------------|-------------------|
| Nav2 integration | "Proper ROS 2 navigation" — sends Nav2 goals instead of raw cmd_vel | Massive dependency footprint (Nav2, costmaps, AMCL/SLAM); completely changes the control loop abstraction; incompatible with real-time bbox-driven follow | Publish raw `/cmd_vel` directly. Nav2 is for waypoint navigation, not reactive follow. |
| SLAM / mapping | "The rover should build a map" | Orthogonal concern; requires LiDAR or depth camera; heavy compute; blocks the simple follow use-case | Out of scope for follow-mode entirely. Add if a separate mapping milestone is created. |
| ros2_control + hardware interface | "Production ROS 2 pattern uses ros2_control" | Overkill for sim-only; adds URDF/XACRO, controller manager, hardware interface layers; `diff_drive_controller` now expects `TwistStamped` which conflicts with plain Twist used elsewhere | Use Gazebo's built-in `DiffDrive` SDF plugin for sim. Revisit for real hardware in v1.2. |
| E-stop topic subscription in v1.1 | "Safety requires e-stop" | Correct for production, but adds state machine complexity before the basic loop is validated | Defer to v1.2 with real hardware; for sim, Ctrl+C sends zero velocity on shutdown via `on_shutdown` hook. |
| Ackermann steering support | "What about car-like robots?" | Different kinematics — needs `linear.x + angular.z` → steering angle + throttle transform; separate adapter | Separate `ackermann_rover.py` adapter if ever needed. Not TurtleBot3-style. |
| ROS 2 action / service interface for follow_id | "Clean ROS 2 API" | Adds interface complexity; the existing follow_id semantics (−1/0/N) are already surfaced via web UI and OpenHD; no ROS consumer exists | Keep follow_id internal. Expose via web UI only. |

---

## Feature Dependencies

```
[VelocityCommand from follow_api/controller.py]
    └──translated-by──> [ros2_rover.py adapter (linear.x + angular.z)]
                            └──publishes-to──> [/cmd_vel ROS topic]
                                                   └──bridged-by──> [ros_gz_bridge (Twist → gz.msgs.Twist)]
                                                                        └──consumed-by──> [Gazebo DiffDrive plugin]

[Gazebo DiffDrive plugin]
    └──drives──> [Rover SDF model in Gazebo Garden]

[Gazebo camera sensor in Rover SDF]
    └──exposed-via──> [gz-transport topic]
                          └──encoded-by──> [video_bridge.py (reused from sim/bridge/)]
                                               └──→ UDP H.264 RTP :5600
                                                        └──consumed-by──> [drone-follow --input udp://0.0.0.0:5600]

[rclpy background thread executor]
    └──required-by──> [ros2_rover.py] (rclpy.spin blocks; asyncio owns main loop)

[twist_mux] ──enhances──> [/cmd_vel] (v1.2 differentiator, not v1.1)
[Nav2] ──conflicts-with──> [direct /cmd_vel publish] (anti-feature)
```

### Dependency Notes

- **rclpy background thread required by ros2_rover adapter:** rclpy's `spin()` is blocking. The existing drone-follow asyncio loop cannot yield to it. Pattern: `threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()`. Publisher calls are thread-safe in rclpy (Humble). The asyncio loop calls `node.publish()` directly from its thread.
- **video_bridge.py reuse:** The existing `sim/bridge/video_bridge.py` already implements gz-transport → UDP H.264 RTP. The rover only needs to point it at the rover camera's gz topic (`--topic /rover/camera` or similar). No new code needed for the camera path.
- **ros_gz_bridge required before Gazebo DiffDrive sees cmd_vel:** The DiffDrive plugin consumes `gz.msgs.Twist` on a gz-transport topic; ros_gz_bridge creates the ROS↔gz translation. Without it, `/cmd_vel` messages never reach the Gazebo simulation.

---

## Sim Camera Plumbing Recommendation

**Use: gz-transport → `video_bridge.py` → UDP H.264 → `--input udp://0.0.0.0:5600`**

Rationale:
- Already implemented and validated for the drone sim. Zero new infrastructure.
- `ros_gz_bridge` image path introduces an extra hop (gz → ROS Image topic → consumer) and has a known 15 Hz ceiling on camera publish rate in Gazebo Garden with ros_gz_bridge. The UDP path bypasses this.
- `ros_gz_image` with `image_transport` is cleaner from a ROS perspective but requires the Hailo pipeline to subscribe to a ROS Image topic — incompatible with the current GStreamer UDP input model.
- Direct gz-transport subscription (what `video_bridge.py` does) has lowest latency and no ROS dependency on the camera path.

**Do not use `ros_gz_bridge` for camera images.** Use it only for cmd_vel (ROS → Gazebo direction).

---

## MVP Definition

### Launch With (v1.1)

- [x] `ros2_rover.py` adapter: background-thread rclpy node, publishes `geometry_msgs/Twist` to `/cmd_vel`
- [x] `--robot rover` CLI flag wires the adapter; `--robot drone` path unchanged
- [x] Gazebo Garden rover SDF with `DiffDrive` plugin + camera sensor (`sim/rover/`)
- [x] `ros_gz_bridge` launch fragment bridging `/cmd_vel` ROS → gz
- [x] `video_bridge.py` reused as-is; rover start script points `--topic` at rover camera gz topic
- [x] `sim/rover/start_rover_sim.sh` (or `start_sim.sh --robot rover`) launches gz sim + bridge + video bridge
- [x] `configs/rover_simulation.json` with rover-safe defaults (reduced speeds, tuned frame margins)
- [x] End-to-end test: rover follows walking actor in Gazebo Garden

### Add After Validation (v1.2)

- [ ] Real rover hardware support — same adapter, point at physical robot's `/cmd_vel`
- [ ] E-stop subscription for hardware safety
- [ ] twist_mux integration for teleop override

### Future Consideration (v2+)

- [ ] Odometry-aware distance estimation
- [ ] Multi-rover namespace support
- [ ] Ackermann steering adapter

---

## Feature Prioritization Matrix

| Feature | User Value | Implementation Cost | Priority |
|---------|------------|---------------------|----------|
| `ros2_rover.py` adapter (Twist publish) | HIGH | LOW | P1 |
| `--robot rover` CLI flag | HIGH | LOW | P1 |
| Gazebo DiffDrive rover SDF | HIGH | LOW | P1 |
| ros_gz_bridge for cmd_vel | HIGH | LOW | P1 |
| video_bridge.py reuse for rover camera | HIGH | LOW | P1 |
| `configs/rover_simulation.json` | MEDIUM | LOW | P1 |
| twist_mux integration | MEDIUM | MEDIUM | P2 |
| Odometry consumption | MEDIUM | MEDIUM | P2 |
| Real rover hardware path | HIGH | LOW | P2 (v1.2) |
| Nav2 integration | LOW | HIGH | P3 (anti-feature) |
| SLAM / mapping | LOW | HIGH | P3 (anti-feature) |

---

## Sources

- [diff_drive_controller — ros2_controllers docs](https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) — topic names, frame IDs, velocity units (HIGH confidence)
- [TurtleBot3 Basic Operation — ROBOTIS](https://emanual.robotis.com/docs/en/platform/turtlebot3/basic_operation/) — `/cmd_vel` as standard topic name (HIGH confidence)
- [twist_mux — Controlling a Robot with Multiple Inputs](https://robofoundry.medium.com/controlling-a-robot-with-multiple-inputs-using-twist-mux-4535b8ed9559) — mux pattern, priority config, output topic flexibility (MEDIUM confidence)
- [ros_gz_bridge ROS Package](https://index.ros.org/p/ros_gz_bridge/) — topic mapping syntax, image bridge limitations (HIGH confidence)
- [Gazebo Garden ROS 2 Integration](https://gazebosim.org/docs/garden/ros2_integration/) — bridge syntax `@`, `[`, `]` directionality (HIGH confidence)
- [gz-sensors camera rate issue](https://github.com/gazebosim/gz-sensors/issues/332) — 15 Hz ceiling with ros_gz_bridge in Garden (MEDIUM confidence — issue thread)
- [rclpy asyncio integration patterns](https://github.com/m2-farzan/ros2-asyncio) — background thread executor pattern (MEDIUM confidence)

---
*Feature research for: ROS 2 cmd_vel rover adapter (sim-only, TurtleBot3-style)*
*Researched: 2026-05-12*
