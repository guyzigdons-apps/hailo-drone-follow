---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: Robot abstraction + rover support (sim-only)
status: planning
stopped_at: Phase 1 ready to plan; axes-only Capabilities decision recorded (ABS-01/05/06/ROVER-07/RINT-02 updated)
last_updated: "2026-05-14T16:30:00.000Z"
last_activity: 2026-05-14 — Design review deck + axes-only Capabilities design decision captured
progress:
  total_phases: 6
  completed_phases: 0
  total_plans: 0
  completed_plans: 0
  percent: 0
---

# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** v1.1 Phase 1 — Rename (ready to plan)

## Current Position

Phase: 1 of 6 (Rename)
Plan: — (not yet planned)
Status: Ready to plan
Last activity: 2026-05-12 — Roadmap created; v1.1 Phases 1-6 defined.

Progress: [░░░░░░░░░░] 0%

## Performance Metrics

**Velocity:**
- Total plans completed: 0
- Average duration: —
- Total execution time: 0 h

**By Phase:**

| Phase | Plans | Total | Avg/Plan |
|-------|-------|-------|----------|
| - | - | - | - |

*Updated after each plan completion*

## Accumulated Context

### Key architectural decisions for v1.1

- **Phase 3 is the hard gate.** Rover adapter (Phase 4) and rover sim (Phase 5) cannot start until drone SITL passes behind `MavsdkDroneAdapter`.
- **Phases 4 and 5 are parallel** once Phase 3 lands.
- **Axes-only `Capabilities`** (decided 2026-05-14 during design review). `Capabilities` is `{axes: frozenset[Axis], yaw_unit}` — mechanical only. `follow_api/controller` must not know what kind of robot it's moving. Behaviors (retreat-from-tilt, slow-near-edge, takeoff/land, yaw-spin-on-loss, offboard handshake) live **inside the adapter**, not as flags in `Capabilities`. ABS-01/ABS-05/ABS-06/ROVER-07/RINT-02 updated to reflect this. Rationale: a `bottom_edge_policy` flag smuggles robot knowledge back into the controller behind a flag, defeating the abstraction. See [[feedback-robot-abstraction-axes-only]] (user memory).
- **Signal handler:** `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is mandatory. Verify with `signal.getsignal(signal.SIGINT)` post-init.
- **Camera path:** `video_bridge.py` reuse (gz-transport → UDP H.264), NOT `ros_gz_image_bridge` (15 Hz ceiling).
- **Garden-era package names:** `ros-humble-ros-gzgarden-bridge`, `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system`. `ignition::` prefix = silent load failure.

### Blockers/Concerns

- SIGINT behavior under Humble specifically: smoke-test `SignalHandlerOptions.NO` early in Phase 4 before full adapter build.
- Rover camera gz topic name: confirm actual topic (`/model/rover/camera` vs `/camera`) with `gz topic -l` when `rover.sdf` first loads before hardcoding in `start_rover_sim.sh`.

### Pending Todos

None yet.

## Session Continuity

Last session: 2026-05-14T16:30:00.000Z
Stopped at: Phase 1 ready to plan; axes-only Capabilities decision recorded
Resume file: .planning/phases/01-rename/01-CONTEXT.md
