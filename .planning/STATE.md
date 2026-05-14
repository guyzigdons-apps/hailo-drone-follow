---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: Robot abstraction + rover support (sim-only)
current_plan: 4
status: ready_for_verification
stopped_at: "Completed 01-03-PLAN.md — Phase 1 ready for /gsd:verify-work"
last_updated: "2026-05-14T15:53:45.164Z"
last_activity: "2026-05-14 — 01-03 completed (A passed, B+C deferred with mitigations). Phase 1 ready for goal-verification."
progress:
  total_phases: 6
  completed_phases: 1
  total_plans: 3
  completed_plans: 3
  percent: 100
---

# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** v1.1 Phase 1 — Rename (ready to plan)

## Current Position

Phase: 1 of 6 (Rename) — **complete, ready for /gsd:verify-work**
Plan: 3 of 3 complete (Wave 2 — manual verifications resolved)
Current Plan: 4 (Phase 1 done; Phase 2 unblocked)
Total Plans in Phase: 3
Status: Ready for verification
Last activity: 2026-05-14 — 01-03 completed (A passed, B+C deferred with mitigations). Phase 1 ready for goal-verification.

Progress: [██████████] 100%

## Performance Metrics

**Velocity:**
- Total plans completed: 3
- Average duration: 32 min
- Total execution time: 96 min (1.6 h)

**By Phase:**

| Phase     | Plans | Total  | Avg/Plan |
|-----------|-------|--------|----------|
| 01-rename | 3     | 96 min | 32 min   |

*Updated after each plan completion*

| Plan record         | Duration | Tasks    | Files     |
|---------------------|----------|----------|-----------|
| Phase 01-rename P01 | 1 min    | 1 task   | 1 file    |
| Phase 01-rename P02 | 73 min   | 3 tasks  | 77 files  |
| Phase 01-rename P03 | 22 min | 2 tasks | 1 files |

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

### Phase 1 decisions (2026-05-14, 01-03 execute)

- **Verification A executed by operator in their own venv shell on the x86_64 dev box.** `./install.sh --skip-apps --skip-hefs --skip-ui` re-runs cleanly post-rename: legacy-uninstall banner present at `install.sh:98`, final `pip install -e .` reports `Successfully installed robot-follow-1.1.0.dev0`, post-run pip metadata is exclusive (`pip show robot-follow` exits 0, `pip show drone-follow` exits non-zero), `diff <(robot-follow --help) <(drone-follow --help)` is empty, and `head -5 "$(which drone-follow)"` shows `from robot_follow.robot_follow_app import main`. Idempotency confirmed (the run was the second post-rename install on this dev box; first uninstalled the prior `robot-follow-1.1.0.dev0`, then reinstalled cleanly).
- **Verifications B and C deferred per operator scope; mitigations cited on disk.** Verification B (deployed-RPi boot-service ExecStart path check) deferred because no air-unit RPi was reachable from this dev box at execution time. Mitigation: Plan 01-02 Task 1 Step 4 ran `git diff 5850558 scripts/boot/` and found the diff empty — `scripts/boot/drone-follow-boot.service` and `scripts/boot/drone-follow-boot.sh` were not touched by the rename commit, so the deployed `ExecStart` path is mechanically unchanged. Verification C (full pytest suite on a Hailo-capable host) deferred because the dev box has no Hailo HW. Mitigation: Plan 01-02 ran `pytest robot_follow/tests/test_install_smoke.py -v` to 10/10 PASSED, covering the RENAME-01..05 surface that's testable without Hailo HW. Both deferrals become regression smoke steps at the next field deployment / Hailo-host sync, not open Phase-1 blockers.

### Blockers/Concerns

- SIGINT behavior under Humble specifically: smoke-test `SignalHandlerOptions.NO` early in Phase 4 before full adapter build.
- Rover camera gz topic name: confirm actual topic (`/model/rover/camera` vs `/camera`) with `gz topic -l` when `rover.sdf` first loads before hardcoding in `start_rover_sim.sh`.

### Pending Todos

None yet.

## Session Continuity

Last session: 2026-05-14T15:53:45.163Z
Stopped at: Completed 01-03-PLAN.md — Phase 1 ready for /gsd:verify-work
Resume file: None
