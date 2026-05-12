# State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** Bootstrap to v1.0-shipped baseline; v1.1 milestone defining.

## Current Position

Phase: Not started (defining requirements for milestone v1.1)
Plan: —
Status: Bootstrapping `.planning/` to v1.0-shipped baseline, then defining v1.1 milestone (robot abstraction + rover support).
Last activity: 2026-05-12 — `.planning/` initialized from CLAUDE.md.

## Accumulated Context

### Pre-GSD planning (historical)

- `docs/.planning/plans/2026-04-16-repo-review-bugfixes.md` — repo review bugfix plan
- `docs/.planning/plans/2026-04-29-post-merge-stabilization.md` — post-merge stabilization (1784 lines, large stabilization effort after `feature/sync-from-hailo-apps-infra` merge)

### Code-audit findings (2026-05-12, pre-v1.1)

Pre-milestone read-only audit identified items to address in the v1.1 cleanup phase (full details in conversation history; key items captured here so they survive context resets):

**Confirmed dead code:**
- `sim/world_loader.py` (140 LOC orphan; `start_sim.sh` uses `PX4_GZ_WORLD`)
- `scripts/bench_reid_callback.py` (imports non-existent `reid_worker`)
- `follow_api/config.py` `vfov` field + `--vfov` flag (plumbed but never read)
- `mavsdk_drone.py:47` `--mission-duration` (undocumented, silently lands after 5 min)
- `drone_follow_app.py:54` `getattr(args, "serial_baud", 115200)` fallback (unreachable; lies if it fires)
- `vision_branches.py:397` `strip_tiles_and_highlight_target` alias (no callers)
- `state.py:23` `available_ids=None` default + branch (every caller passes a set)
- `mavsdk_drone.py:642` `shutdown_read_fd` param + pipe-reader block (unreachable)
- `_NULLABLE_FIELDS = set()` (empty) in both `web_server.py:325` and `openhd_bridge.py:68`
- `create_app(controller_config=...)` kwarg in `hailo_drone_detection_manager.py` (no caller passes it)

**Drone-leakage in supposedly generic modules** (key list for the abstraction):
- `follow_api/config.py` — 9 altitude/flight fields + `validate()` enforcing altitude relationships
- `follow_api/controller.py` — emits `down` axis, altitude-hold logic
- `follow_api/types.py:VelocityCommand` — MAVSDK-shaped tuple `(forward, down, yawspeed)`
- `web_server._CONFIG_FIELDS` + `openhd_bridge._CONFIG_PARAMS` — parallel altitude knob lists
- `hailo_drone_detection_manager.py:1271` — tracker `track_thresh=0.4, track_buffer=90 (3s)` tuned for aerial; wrong for ground rover; not config-driven
- Search-after-loss state machine in controller + `live_control_loop` assumes yaw-spin recovery (drone semantics)

**Duplication to merge before abstracting:**
- MAVSDK-server `pkill` reaper in two places (`mavsdk_drone.py` + `drone_follow_app.py:441`)
- CLI pre-parsing 3× in `drone_follow_app.py` (`ui_pre`, `reid_pre`, `tracker_pre`)
- `_telemetry_position_task` + `_telemetry_altitude_task` (both stream `telemetry.position()`)
- Branch-decision split between `get_pipeline_string()` and `vision_branches.assemble_output_stage()`
- Implicit-display rule duplicated in 3 places

**Hot-path wins:**
- `web_server.py:84` — `_frame_event.set()` then immediate `clear()` races against non-blocked readers → 2 s timeout fallthrough
- `openhd_bridge.py:332` — fresh socket per `_send_immediate_report`
- `hailo_drone_detection_manager.py:88` — `next(q for q in persons if id(q) == prev)` linear scan

### Architectural decisions for v1.1 (locked in conversation)

- **Rover stack:** ROS 2 Humble + `cmd_vel` (Twist on `/cmd_vel`). NOT PX4 Rover via MAVSDK.
- **Sim approach:** Separate Gazebo Garden rover sim with `ros_gz_bridge`. NOT PX4 rover airframe.
- **Package rename:** `drone_follow` → `robot_follow` as own atomic phase (Phase 0).
- **Scope:** Sim-only milestone — real rover hardware deferred to v1.2.
- **Abstraction shape:**
  - `robot_api/robot.py` with `Robot` protocol + `Capabilities` named tuple (`has_altitude`, `needs_offboard_handshake`, `needs_takeoff_landing`, `yaw_units`)
  - `robot_api/adapters/mavsdk_drone.py` (move existing) + `robot_api/adapters/ros2_rover.py` (new)
  - `follow_api/config/` split into `base.py` (generic) + `flight.py` (drone-only)
  - `RobotCommand(forward, yaw, altitude_axis: Optional[float])` replaces `VelocityCommand`
- **Risks flagged:**
  - rclpy ↔ asyncio bridge (background-thread executor pattern)
  - ROS 2 is system package, not pip; `setup_env.sh` needs conditional `source /opt/ros/humble/setup.bash` when `--robot rover`
  - HailoRT venv has `--system-site-packages` so `import rclpy` should resolve once ROS installed
