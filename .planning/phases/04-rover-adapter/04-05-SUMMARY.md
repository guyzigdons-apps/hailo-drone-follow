---
phase: 04-rover-adapter
plan: 05
type: summary
status: deferred-pending-operator
wave: 4
gap_closure: false
requirements: [ROVER-01, ROVER-02, ROVER-03, ROVER-04, ROVER-05, ROVER-06, ROVER-07, ROVER-08]
resume_signal: deferred
---

deferred — Phase 4 code-complete; real-rclpy operator smoke requires a Humble machine the dev box doesn't have. Operator runs at convenience and overwrites this file.

## Status

All Phase 4 code-level requirements landed across 04-01..04-04:

| Commit | What |
|--------|------|
| `c538bc1`, `440204d`, `b233129` | 04-01 test scaffolds (~20 adapter xfails + 6 CLI xfails) |
| `2f925a1`, `37bfcaa`, `84b5fda` | 04-02 `add_rover_args` body (ROVER-05) |
| `e863dea`, `56df101`, `9230e08` | 04-03 `Ros2RoverAdapter` + `ROVER_CAPS` (ROVER-01..04, 06, 07) |
| `f541440`, `5d0b439`, `6705764` | 04-04 composition-root wiring + integration smoke (ROVER-08) |

**Test state:** 326 passed / 0 xfailed / 1 skipped. All Phase 4 unit tests
pass with `sys.modules["rclpy"] = MagicMock()` mocking — the adapter's
contracts (Twist mapping, SignalHandlerOptions.NO, friendly RuntimeError,
spin_once lifecycle, caps shape) are exercised against the mock.

**Architectural locks held across the whole phase:**
- `robot_api/adapters/mavsdk_drone.py` byte-identical (drone path not regressed)
- `robot_follow_app.py` has zero top-level `rclpy` / `geometry_msgs` imports
- `ros2_rover.py` has zero top-level `rclpy` / `geometry_msgs` imports
- `ROVER_CAPS` = `Capabilities(axes=frozenset({Axis.FORWARD, Axis.YAW}), yaw_unit="rad/s")` — no ALTITUDE
- `send_command`: `yaw_rate` → `Twist.angular.z` direct, no conversion
- `add_rover_args` lives in `robot_follow_app.py` (NOT in the adapter)
- `--help` works on no-rclpy boxes for both `--robot drone` and `--robot rover`

## What's pending operator verification

Per `04-05-PLAN.md` scorecard, on a machine with `/opt/ros/humble` installed:

1. Happy path — `--robot rover --cmd-vel-topic /cmd_vel` runs without error
2. Sad path — friendly RuntimeError on missing ROS; substrings `"ROS 2 not"` and `"source /opt/ros/humble/setup.bash"` present (verbatim ROVER-04 message string is in `04-03-SUMMARY.md`)
3. `--robot rover --help` lists only rover flags; drone-only flags absent
4. `ros2 topic echo /cmd_vel` shows Twist messages with correct linear.x / angular.z mapping
5. `signal.getsignal(signal.SIGINT)` is drone-follow's handler post-connect
6. `time kill -INT <pid>` results in graceful shutdown within ~5s
7. Drone path not regressed — prefer 03-14 SITL re-run for this row, per the
   plan-checker watch-list item #4 (USB-smoke is acceptable but weaker)
8. `git status --porcelain` clean — operator did not accidentally commit
   anything through this gate

## Resume

1. On a Humble machine: `pip install -e .` (or set up venv per CLAUDE.md), `source /opt/ros/humble/setup.bash`, then run the scorecard rows.
2. Fill the scorecard in this file.
3. Overwrite this file with `approved` / `approved-with-deferral` / `failed` per the resume-signal contract.
4. Update REQUIREMENTS.md to flip ROVER-01..08 → Complete on approval.

If the gate surfaces new defects, follow-up gap plans land as 04-06+ per the
03-13/15/16 pattern. Phase 5 (rover sim) and Phase 6 (sim integration) can
proceed in parallel — the only dependency they have on Phase 4 is the
code-level contract (already met).
