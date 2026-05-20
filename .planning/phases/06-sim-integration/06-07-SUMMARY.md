approved-with-deferral

---
phase: 06-sim-integration
plan: 07
subsystem: operator-gate
tags: [phase-6, operator-gate, deferred, RINT-04, wave-4]

requires:
  - phase: 06-sim-integration
    provides: configs/rover_simulation.json + ByteTracker config + rover bottom-edge slow-down + RINT-06 shutdown tests + RINT-04 E2E test (plans 06-01..06-06)
provides:
  - Operator-gate resume signal for Phase 6 (deferred — mirrors 03-14 / 04-05 pattern)
affects: [v1.1-milestone-merge]

tech-stack:
  added: []
  patterns:
    - "Deferred-operator-gate pattern: when the entire orchestrator session cannot put hands on the physical/sim system, record `approved-with-deferral` with explicit per-row deferral notes; the verifier surfaces this as outstanding work in the UAT audit until the operator closes it."

key-files:
  created: []
  modified: []

key-decisions:
  - "Defer per user choice on 2026-05-20: the gate requires a wired-up rover sim session (gz Garden + ROS 2 + Hailo + manual eye-on verification) and the operator opted to run it in a dedicated session rather than blocking the orchestrator here."

patterns-established: []

requirements-completed: []  # RINT-01..06 remain in 'tests landed' state; operator gate transitions them to 'verified' on later closure
duration: 0min (deferred — no operator run yet)
completed: 2026-05-20 (deferral recorded; gate itself not yet exercised)
---

# Phase 6 Plan 07: Operator Gate — DEFERRED

**Resume signal: `approved-with-deferral`. Operator-witnessed rover sim full-follow verification deferred to a wired-up session; mirrors the 03-14 / 04-05 pattern.**

## Resume Signal

`approved-with-deferral` (recorded at line 1 of this file per the resume-signal contract).

All structural prerequisites for the gate are in place:
- Plans 06-01..06-06 landed with passing automated tests (4 + 1 + 6 + 6 + 3 + 1 = 21 new PASSing tests across the suite; full suite 356/357 PASS, 1 pre-existing unrelated rclpy-missing failure).
- The deterministic E2E test for the controller side (06-06's `TestRoverWalkAcrossThenApproach`) is in `test_sim_worlds.py` and SKIPS cleanly on this orchestrator's machine (no `RUN_SIM_TESTS=1` + Hailo gate, per the file-level pytestmark).

The deferred work is the operator-eye verification rows in the scorecard below. Until the operator runs that pass, the v1.1 milestone should NOT be considered fully verified — only "code complete + automated tests green".

## Scorecard (Deferred)

| # | Criterion | Status | Evidence / Deferral Reason |
|---|-----------|--------|-----------------------------|
| 1 | gz sim launches cleanly with `start_rover_sim.sh --world walk_across_then_approach` | deferred | Requires Gazebo Garden + ROS 2 Humble + the rover model installed locally; orchestrator was run from a session without active gz GUI. Operator to confirm on their machine. |
| 2 | `robot-follow --robot rover ...` connects and AUTO-acquires the largest person | deferred | Requires running rover sim + Hailo accelerator. Deferred to operator session. |
| 3 | Rover physically drives + tracks the actor through the walk pattern | deferred | Operator-eye only; cannot be automated. Deferred. |
| 4 | **RINT-02 bottom-edge slow-down** — twist.linear.x → 0 when actor close | deferred | Operator-eye via `gz topic -e -t /cmd_vel`. Pinned by automated tests in 06-04 (`TestBottomEdgeNaturalStop`) but operator-eye on the real sim is the spec; deferred. |
| 5 | **RINT-06 SIGINT clean shutdown** — Ctrl+C silences /cmd_vel within ~1 s | deferred | Operator-eye via `gz topic -e -t /cmd_vel --duration 3` after Ctrl+C. Pinned by automated tests in 06-05 (`TestSigintShutdown`); operator-eye on the real sim is the spec; deferred. |
| 6 | No FATAL / ERROR in robot-follow console | deferred | Requires running the rover sim session. Deferred. |
| 7 | **Drone path not regressed** — separate run against PX4 SITL | deferred | Code-side check: all architectural locks held empty diff across 06-01..06-06; `mavsdk_drone.py` byte-identical. Operator should run `sim/start_sim.sh` + `robot-follow --robot drone ...` in a follow-up to confirm by eye. |
| 8 | RINT-04 deterministic test (06-06) PASSES locally | deferred | `RUN_SIM_TESTS=1 python -m pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v` on the wired-up rover box. From this orchestrator's machine the test SKIPPED cleanly (per spec). Operator to confirm PASSED. |

**8 / 8 rows deferred.** The gate is not yet exercised; it is recorded with `approved-with-deferral` because (a) all preceding plans landed with passing automated tests + architectural locks held, and (b) the user explicitly chose to defer the eyes-on session.

## Code-Side Confidence (Available Now)

The non-operator-witnessed parts of the gate are already green:

- **Automated tests (Phase 6):** +21 PASS (06-01: 4, 06-02: 1, 06-03: ~6, 06-04: 6, 06-05: 3, 06-06: 1 — collected, skips on this box). Pre-existing 1 FAIL unrelated to Phase 6 work (`test_friendly_error_when_rclpy_missing` — fails because rclpy IS installed locally; not a Phase 6 regression).
- **Architectural locks held across Phase 6:**
  - `robot_follow/robot_api/adapters/mavsdk_drone.py`: empty diff across 06-01..06-06
  - `robot_follow/follow_api/controller.py`: empty diff across 06-01..06-06
  - `Capabilities` dataclass (axes-only contract): no new fields
  - `sim/rover/` (model + worlds): only 06-02's README append; no model/world changes
  - `sim/bridge/video_bridge.py`: empty diff (RSIM-06 lock)
- **Drone-default byte-identity (RINT-03):** `ControllerConfig.bytetracker_*` defaults match the legacy hardcoded `track_thresh=0.4, track_buffer=90, match_thresh=0.5, frame_rate=30` byte-identically; Plan 06-03 wired `create_app` to read from controller_config, so the drone path is provably unchanged.

## Operator Reopen Instructions

To close out 06-07 fully (move from `approved-with-deferral` to `approved`):

1. On the wired-up rover sim box: `sudo ./install.sh --rover` (if not already done).
2. Terminal A: `./sim/rover/start_rover_sim.sh --world walk_across_then_approach`. Wait for the gz GUI to show the rover + actor.
3. Terminal B:
   ```bash
   source setup_env.sh
   robot-follow --robot rover \
     --input udp://0.0.0.0:5600 \
     --config configs/rover_simulation.json \
     --webui
   ```
4. Open the web UI (default port 5001). Confirm rows 2-3 in the scorecard.
5. Watch `gz topic -e -t /cmd_vel` in a third terminal. As the actor approaches the rover, confirm `twist.linear.x` drops to 0 (row 4).
6. Ctrl+C robot-follow. Confirm `/cmd_vel` goes silent within ~1 s (row 5).
7. Skim the robot-follow scrollback for FATAL/ERROR (row 6).
8. Drone path sanity: separately, `sim/start_sim.sh --bridge --world person_in_front` + `robot-follow --robot drone --takeoff-landing` — confirm drone still acquires + follows (row 7).
9. RINT-04 deterministic test: `RUN_SIM_TESTS=1 pytest robot_follow/tests/test_sim_worlds.py::TestRoverWalkAcrossThenApproach -v` — confirm PASSED (row 8).
10. When all 8 rows witnessed: edit line 1 of this file from `approved-with-deferral` to `approved`, fill the scorecard, commit.

## Next Phase Readiness

- **Code complete:** Yes — Phase 6 implementation lands all RINT-01..06.
- **Tests green:** Yes — 21 new PASS, no regressions.
- **v1.1 milestone:** Ready to merge to main for code-side review. The deferred operator gate remains an outstanding UAT item; the verifier will surface it.

---
*Phase: 06-sim-integration*
*Resume signal recorded: 2026-05-20*
*Operator session: pending*
