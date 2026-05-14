---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: Robot abstraction + rover support (sim-only)
current_plan: 3
status: executing
stopped_at: Completed 01-02-PLAN.md
last_updated: "2026-05-14T15:26:07.021Z"
last_activity: "2026-05-14 — 01-02 executed: atomic rename landed (commit 5850558); 77 files renamed/modified."
progress:
  total_phases: 6
  completed_phases: 0
  total_plans: 3
  completed_plans: 2
  percent: 67
---

# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** v1.1 Phase 1 — Rename (ready to plan)

## Current Position

Phase: 1 of 6 (Rename)
Plan: 2 of 3 complete (Wave 1 — atomic rename landed)
Current Plan: 3
Total Plans in Phase: 3
Status: In progress
Last activity: 2026-05-14 — 01-02 executed: atomic rename landed (commit 5850558); 77 files renamed/modified.

Progress: [███████░░░] 67%

## Performance Metrics

**Velocity:**
- Total plans completed: 2
- Average duration: 37 min
- Total execution time: 74 min (1.2 h)

**By Phase:**

| Phase     | Plans | Total  | Avg/Plan |
|-----------|-------|--------|----------|
| 01-rename | 2     | 74 min | 37 min   |

*Updated after each plan completion*

| Plan record         | Duration | Tasks    | Files     |
|---------------------|----------|----------|-----------|
| Phase 01-rename P01 | 1 min    | 1 task   | 1 file    |
| Phase 01-rename P02 | 73 min   | 3 tasks  | 77 files  |

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

### Phase 1 decisions (2026-05-14, 01-02 execute)

- **Atomic single-commit rename landed as `5850558`.** Per CONTEXT-locked decision: dir rename + 48 imports + pyproject + install.sh + docs + skip-guard strip + parser.prog pin all in ONE commit (`refactor(01-02): rename drone_follow -> robot_follow`). Every commit on `feature/rover-support` stays buildable; `git bisect` survives every intermediate state. 77 files renamed/modified, +232/-241 lines.
- **Pin `parser.prog="robot-follow"` to make --help byte-identical across aliases.** Auto-fix during Task 3: argparse derives `prog` from `sys.argv[0]`, so `robot-follow --help` and `drone-follow --help` differed by their `usage:` line. Pinning `prog` in `_build_parser()` immediately after `get_pipeline_parser()` returns guarantees byte-identical output regardless of invocation name. Required for Phase 1 success criterion 2.
- **Console-script alias semantics confirmed.** Both `robot-follow` and `drone-follow` declared in `pyproject.toml [project.scripts]` mapping to `robot_follow.robot_follow_app:main`. After `pip install -e .`, both shims embed identical `from robot_follow.robot_follow_app import main`. With parser.prog pinned, runtime behavior is indistinguishable — the alias is a pure entry-point rename, not a separate code path.

### Blockers/Concerns

- SIGINT behavior under Humble specifically: smoke-test `SignalHandlerOptions.NO` early in Phase 4 before full adapter build.
- Rover camera gz topic name: confirm actual topic (`/model/rover/camera` vs `/camera`) with `gz topic -l` when `rover.sdf` first loads before hardcoding in `start_rover_sim.sh`.

### Pending Todos

None yet.

## Session Continuity

Last session: 2026-05-14T15:25:23.932Z
Stopped at: Completed 01-02-PLAN.md
Resume file: None
