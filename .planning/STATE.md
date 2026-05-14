---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: Robot abstraction + rover support (sim-only)
current_plan: 5
status: executing
stopped_at: Completed 02-01-PLAN.md — CLEAN-01/02/06/10 landed (Tasks 1 and 2 swept into parallel 02-02 commits; Task 3 clean as 0b40abd)
last_updated: "2026-05-14T17:12:35.625Z"
last_activity: 2026-05-14
progress:
  total_phases: 6
  completed_phases: 1
  total_plans: 11
  completed_plans: 7
  percent: 64
---

# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-05-12)

**Core value:** The pipeline keeps a target person in frame and computes safe velocity commands for the robot, even when the target is briefly occluded — without operator input.
**Current focus:** v1.1 Phase 2 — Cleanup (Wave 0 done; Waves 1-3 ahead)

## Current Position

Phase: 2 of 6 (Cleanup) — **in progress, Wave 1 complete (02-01, 02-02, 02-03 all landed)**
Plan: 4 of 8 complete (02-00 + 02-01 + 02-02 + 02-03)
Current Plan: 5
Total Plans in Phase: 8
Status: In progress
Last activity: 2026-05-14

Progress: [██████░░░░] 64%

## Performance Metrics

**Velocity:**
- Total plans completed: 4
- Average duration: 24.5 min
- Total execution time: 98 min (1.6 h)

**By Phase:**

| Phase      | Plans | Total  | Avg/Plan |
|------------|-------|--------|----------|
| 01-rename  | 3     | 96 min | 32 min   |
| 02-cleanup | 1     | 2 min  | 2 min    |

*Updated after each plan completion*

| Plan record         | Duration | Tasks    | Files     |
|---------------------|----------|----------|-----------|
| Phase 01-rename P01 | 1 min    | 1 task   | 1 file    |
| Phase 01-rename P02 | 73 min   | 3 tasks  | 77 files  |
| Phase 01-rename P03 | 22 min | 2 tasks | 1 files |
| Phase 02-cleanup P00 | 2 min | 2 tasks | 2 files |
| Phase 02-cleanup P03 | 3 min | 3 tasks tasks | 3 files files |
| Phase 02-cleanup P02 | 4 min | 3 tasks | 6 files |
| Phase 02-cleanup P01 | 5 min | 3 tasks | 4 files |

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

### Phase 2 decisions (2026-05-14, 02-00 execute)

- **Wave-0 xfail scaffolds use `strict=False`.** A coincidental pre-fix pass (e.g., SharedUIState being benign under low test load) reports as `xpass` instead of breaking the suite. Strict semantics belong in plans 02-05 / 02-07 when the markers come off and the tests carry the load of regression detection. Wave 0's job is "tests exist, suite stays green", not "lock in correctness".
- **Lazy-import for `decide_branches` via `_decide()` helper.** Module-level `from robot_follow.pipeline_adapter.vision_branches import decide_branches` would `ImportError` at collection time (the symbol doesn't land until 02-05). The lazy helper keeps collection green and makes intent obvious in the source. `git grep decide_branches robot_follow/tests/` finds exactly the call sites, not import noise.
- **Pre-existing controller failures deferred (DEFER-02-00-A).** `test_controller.py::TestDistanceForward::{test_center_y_is_ignored, test_clamped_to_max_forward}` fail on the clean tree (HEAD = 5f15982) — `cmd_bot.forward_m_s` is `-0.75` but the tests expect `0.0`. Verified by stash-pre/stash-post comparison; not caused by Wave 0. Logged at `.planning/phases/02-cleanup/deferred-items.md`. Out of scope for Phase 2 (CLEAN-01..18 doesn't touch the controller); recommended action: fold into a follow-up plan or extra CLEAN-19 before Phase 3 starts (Phase 3 touches the controller via the adapter boundary; this should be green before then).

### Phase 2 decisions (2026-05-14, 02-03 execute)

- **CLEAN-04 documents `--mission-duration` instead of removing it.** RESEARCH § CLEAN-04 confirmed the flag is read at `mavsdk_drone.py:767` (auto-land timeout in the `--takeoff-landing` branch) and `:799` (control-loop iteration timeout in the no-takeoff-landing branch). Removing it would silently turn the 300 s default into an infinite mission. Fix is `help=` string + a CLAUDE.md bullet noting the surprise hazard and how to disable the watchdog (`--mission-duration 86400`). Reinforces the wave-1 pattern: distrust the planner's "remove unused flag" instinct until the read sites are grepped.
- **CLEAN-05's outer `getattr(args, "serial", None) is not None` guard is real and kept.** Only the inner `getattr(args, "serial_baud", 115200)` was unreachable (the `--serial-baud` registration at `mavsdk_drone.py:41` has `default=57600`, and the parser builds before `_resolve_serial_connection` runs). The `--serial` field genuinely has a `None` default, so the outer guard is testing whether the operator passed the flag — keep it.
- **CLEAN-08 keeps `import os` at the top of `mavsdk_drone.py`.** Other sites use `os.getuid()` (line 230), `os.path.join` / `os.path.exists` (lines 212, 216), and `os.environ`. Removing the import together with the pipe-reader block would have introduced a `NameError`. Lesson: when stripping the only visible user of a stdlib import, grep the whole file for other call sites before pulling the import too.

### Phase 2 decisions (2026-05-14, 02-01 execute)

- **`DroneFollowUserData.controller_config` attribute kept (init to `None`) even after removing the constructor kwarg.** The callback at `hailo_drone_detection_manager.py:278` reads `user_data.controller_config`; the real value is attached post-construction at `robot_follow_app.py:340`. Removing the attribute outright would `AttributeError` before the attach landed. Only the dead constructor kwarg path got stripped.
- **Parallel-plan working-tree race accepted as Rule-3 deviation.** Plans 02-01, 02-02, and 02-03 ran concurrently in the same working tree. 02-02's commit step picked up 02-01's unstaged Task-1 deletions (`sim/world_loader.py`, `scripts/bench_reid_callback.py`) into commit `cd26780`, and Task-2's `vision_branches.py` alias edit into commit `f923870`. Task 3 (CLEAN-10) was committed cleanly as `0b40abd` via targeted `git add <file>`. Success criteria all met; attribution drift recorded in `02-01-SUMMARY.md` deviations. Lesson for future parallel waves: spawn each plan agent into its own `git worktree add`-managed worktree, or have agents use `git add <file>` (never `git add .`) and stagger commits tightly enough that sibling unstaged work isn't visible.

### Blockers/Concerns

- SIGINT behavior under Humble specifically: smoke-test `SignalHandlerOptions.NO` early in Phase 4 before full adapter build.
- Rover camera gz topic name: confirm actual topic (`/model/rover/camera` vs `/camera`) with `gz topic -l` when `rover.sdf` first loads before hardcoding in `start_rover_sim.sh`.
- Pre-existing `test_controller.py` failures (DEFER-02-00-A) — 2 tests fail in `TestDistanceForward`; not blocking Phase 2 but should be green before Phase 3.

### Pending Todos

None yet.

## Session Continuity

Last session: 2026-05-14T17:11:54.358Z
Stopped at: Completed 02-01-PLAN.md — CLEAN-01/02/06/10 landed (Tasks 1 and 2 swept into parallel 02-02 commits; Task 3 clean as 0b40abd)
Resume file: None
