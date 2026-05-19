failed: Web UI velocity status bar stale (Phase 3 regression) + pre-existing INFO log spam surfaced during SITL gate

---
phase: 03-abstraction
plan: 10
status: failed
gate: operator-witnessed SITL
world: walk_across_then_approach
operator: guyz
date: 2026-05-19
---

## Resume signal

**failed** — observed two issues during the witnessed SITL run. One is a Phase 3 regression that breaks operator situational awareness in flight; the other is pre-existing INFO-level log spam that became more noticeable during the gate.

Phase 3 does NOT close. Run `/gsd:plan-phase 3 --gaps` to generate a fix plan; re-verify after the fix lands.

## Findings

### F1 — Web UI velocity status bar stuck at `IDLE | Fwd 0.00 m/s | Down 0.00 m/s | Yaw 0.0 deg/s` (regression)

**Severity:** high (operator situational awareness)
**Origin:** Phase 3 — commit `7f602d2` (03-07 atomic controller→Robot signature migration)
**File evidence:**
- `robot_follow/servers/web_server.py:114` — `WebServerStore.update_velocity(forward, down, yawspeed, mode)` exists, populates the SSE payload at line 92 / 105.
- `robot_follow/robot_api/adapters/mavsdk_drone.py:683` — `MavsdkDroneAdapter.send_command()` computes the smoothed `VelocityBodyYawspeed` and pushes it to MAVSDK, but never reports back to `ui_state`. Same omission in `send_zero` (line 712) and `on_target_lost` (line 724).
- `robot_follow/robot_api/orchestrator.py` — `run_robot_loop` has no `ui_state` parameter; the state machine never publishes mode + velocity.
- Pre-Phase-3 call site (in deleted `live_control_loop`): `ui_state.update_velocity(cmd.forward_m_s, cmd.down_m_s, cmd.yawspeed_deg_s, mode)` was emitted on every tick.

**Symptom:** during the SITL run, the drone visibly yaws + commands forward velocity, but the web UI status bar above the video feed permanently shows the `WebServerStore._velocity` constructor default (`mode="IDLE"`, all speeds 0.0). The pipeline-side state pill (`Auto (largest person)` / `Following: ID N`) updates correctly — only the velocity readout is stale.

**Suggested fix shape:**
- Thread `ui_state` through to `run_robot_loop` (orchestrator already has `cmd` + state-machine mode in scope).
- After each branch (`send_command` / `on_target_lost` / hold), call `ui_state.update_velocity(forward, down, yawspeed, mode)` with the mode string the orchestrator state machine produced.
- Composition root in `robot_follow_app.py` already has `ui_state` available at the `run_robot_loop(...)` call site — small wire-through.
- Decide whether the values published reflect the controller's pre-smoothing `cmd` or the adapter's post-smoothing `smoothed` (pre-Phase-3 used pre-smoothing). Pick one and document; the gap plan should call this out.
- Rover symmetry: the gap plan should land this in a way that the eventual rover adapter also produces a coherent velocity readout (mode strings TBD for rover in Phase 4 — at minimum `IDLE` / `MOVING` / `LOST`).

**Acceptance for the gap plan:**
- Status bar updates at ≥ 5 Hz during follow.
- `mode` reflects orchestrator state (e.g. `AUTO`, `LOCKED`, `LOST`, `IDLE`).
- New unit test (e.g. `test_orchestrator_ui_update.py`) that drives `run_robot_loop` for a few ticks against a FakeRobot + spy ui_state and asserts the spy received non-zero velocity + correct mode.

### F2 — `ctrl: bh=... factor=... filtered=...` INFO log spam (pre-existing, polish)

**Severity:** low (noise, not a regression — git history shows the line has been at INFO since the v1.1 rename `5850558`)
**Origin:** pre-Phase-3 (predates this milestone). Not caused by the abstraction work.
**File evidence:** `robot_follow/pipeline_adapter/hailo_drone_detection_manager.py:533` emits at INFO every tick a target is in frame (~10 Hz).
**Suggested fix:** demote to `LOGGER.debug(...)`. Single-line change; trivial.
**Folding into the gap plan:** include alongside F1 since both surfaced together and both are touched by an "is the operator getting good telemetry?" cluster.

## Acceptance criteria scorecard

| # | Criterion | Result |
|---|-----------|--------|
| 1 | Drone arms + takes off cleanly | ✅ |
| 2 | Drone faces + follows actor for the full walk pattern | ✅ (drone visibly tracks; only the UI readout is stale) |
| 3 | Altitude held within ±0.5 m of target | ✅ |
| 4 | Smooth motion (no oscillation/jerk) | ✅ |
| 5 | Clean Ctrl+C shutdown with land | (not verified yet — gate paused on F1 finding) |
| 6 | No new FATAL / ERROR log lines vs pre-Phase-3 | ⚠ pre-existing INFO spam louder than expected (F2) |
| 7 | (Optional) ROS env-leak step | deferred — no ROS on this box |
| **GATE** | **Operator can observe commanded velocities in flight** | **❌ FAILED — F1** |

## Next action

```bash
/clear
/gsd-plan-phase 3 --gaps
```

The gaps planner will read this SUMMARY + `03-VERIFICATION.md` (not yet created — verifier didn't run because the gate failed) and produce a 3.1 gap-closure phase covering F1 + F2.

After the gap phase lands and tests pass, re-run this SITL gate. On second-pass approval, replace this SUMMARY's first line with `approved` (preserving the rest of the artifact as history of what was caught).
