---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: Robot abstraction + rover support (sim-only)
current_plan: 2
status: executing
stopped_at: Completed 01-01-PLAN.md
last_updated: "2026-05-14T14:07:25.008Z"
last_activity: "2026-05-14 — 01-01 executed: forward-compatible smoke test landed (commit 0869454)."
progress:
  total_phases: 6
  completed_phases: 0
  total_plans: 3
  completed_plans: 1
  percent: 33
---

# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** v1.1 Phase 1 — Rename (ready to plan)

## Current Position

Phase: 1 of 6 (Rename)
Plan: 1 of 3 complete (Wave 0 — smoke test gate landed)
Current Plan: 2
Total Plans in Phase: 3
Status: In progress
Last activity: 2026-05-14 — 01-01 executed: forward-compatible smoke test landed (commit 0869454).

Progress: [███░░░░░░░] 33%

## Performance Metrics

**Velocity:**
- Total plans completed: 1
- Average duration: 1 min
- Total execution time: 0 h

**By Phase:**

| Phase     | Plans | Total | Avg/Plan |
|-----------|-------|-------|----------|
| 01-rename | 1     | 1 min | 1 min    |

*Updated after each plan completion*

## Accumulated Context

### Key architectural decisions for v1.1

- **Phase 3 is the hard gate.** Rover adapter (Phase 4) and rover sim (Phase 5) cannot start until drone SITL passes behind `MavsdkDroneAdapter`.
- **Phases 4 and 5 are parallel** once Phase 3 lands.
- **Axes-only `Capabilities`** (decided 2026-05-14 during design review). `Capabilities` is `{axes: frozenset[Axis], yaw_unit}` — mechanical only. `follow_api/controller` must not know what kind of robot it's moving. Behaviors (retreat-from-tilt, slow-near-edge, takeoff/land, yaw-spin-on-loss, offboard handshake) live **inside the adapter**, not as flags in `Capabilities`. ABS-01/ABS-05/ABS-06/ROVER-07/RINT-02 updated to reflect this. Rationale: a `bottom_edge_policy` flag smuggles robot knowledge back into the controller behind a flag, defeating the abstraction. See [[feedback-robot-abstraction-axes-only]] (user memory).
- **Signal handler:** `rclpy.init(signal_handler_options=SignalHandlerOptions.NO)` is mandatory. Verify with `signal.getsignal(signal.SIGINT)` post-init.
- **Camera path:** `video_bridge.py` reuse (gz-transport → UDP H.264), NOT `ros_gz_image_bridge` (15 Hz ceiling).
- **Garden-era package names:** `ros-humble-ros-gzgarden-bridge`, `gz::sim::systems::DiffDrive` with filename `gz-sim-diff-drive-system`. `ignition::` prefix = silent load failure.

### Phase 1 decisions (2026-05-14, 01-01 execute)

- **Skip-guards land in 01-01 and get stripped in 01-02.** A single forward-compatible test file (`drone_follow/tests/test_install_smoke.py`) is mutated across two commits — Wave 0 adds `_skip_if_pre_rename()` guards, Wave 1 strips them in the same commit as the dir rename. Avoids the duplicate-then-delete sprawl of a parallel "renamed" test file.
- **`drone-follow` console-script alias is always-on (no skip-guard).** Same `drone-follow --help` contract pre- and post-rename — pre-rename it's the real script, post-rename it's a pyproject.toml alias entry point. `scripts/start_air.sh`, the boot service, and user muscle memory all invoke `drone-follow`, so a skip-guard window would be a regression risk.

### Blockers/Concerns

- SIGINT behavior under Humble specifically: smoke-test `SignalHandlerOptions.NO` early in Phase 4 before full adapter build.
- Rover camera gz topic name: confirm actual topic (`/model/rover/camera` vs `/camera`) with `gz topic -l` when `rover.sdf` first loads before hardcoding in `start_rover_sim.sh`.

### Pending Todos

None yet.

## Session Continuity

Last session: 2026-05-14T14:07:25.006Z
Stopped at: Completed 01-01-PLAN.md
Resume file: None
