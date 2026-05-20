---
phase: 05-rover-sim
type: verification
status: operator-confirmed
date: 2026-05-20
requirements_verified: [RSIM-01, RSIM-02, RSIM-04, RSIM-05, RSIM-06]
requirements_static: [RSIM-03, RSIM-07]
---

## Outcome

Operator confirmed Phase 5 end-to-end on `feature/rover-support` HEAD `a46bc56`.

## Evidence

The operator ran `sudo ./install.sh --rover` (hit and resolved one gap — missing
ROS 2 apt repo; surfaced by the new exit-7 preflight added in commit `b89c1a1`)
and then `./sim/rover/start_rover_sim.sh` on a clean Ubuntu 22.04 jammy box.

Observed at the operator's terminal:

```
gz topic -l   # /camera, /cmd_vel, /model/rover/odometry all visible
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}, angular: {z: 0.2}}' -r 10
  publishing #1: ... (rover responded in gz GUI)
gst-launch-1.0 udpsrc port=5600 ! application/x-rtp,... ! rtph264depay ! avdec_h264 ! autovideosink
  # H.264 stream from rover camera displayed
```

## RSIM coverage verified live

| Req | What was verified |
|-----|-------------------|
| RSIM-01 | gz sim loaded `gz::sim::systems::DiffDrive` from `sim/rover/rover/model.sdf` (the model package created in `a46bc56`); no `ignition::` silent-load failure. |
| RSIM-02 | `<topic>cmd_vel</topic>` override resolved: `/cmd_vel` (not `/model/rover/cmd_vel`) was the gz-side topic the bridge mapped. |
| RSIM-04 | `start_rover_sim.sh` brought up gz + parameter_bridge + video_bridge with the corrected `geometry_msgs/msg/Twist@gz.msgs.Twist` syntax. |
| RSIM-05 | `install.sh --rover` installed `ros-humble-ros-base`, `ros-humble-geometry-msgs`, `ros-humble-ros-gzgarden-bridge`, and `gz-garden` (preceded by the operator manually adding the ROS 2 apt repo per the exit-7 friendly error). |
| RSIM-06 | `sim/bridge/video_bridge.py` reused verbatim; H.264 RTP from rover camera consumed by GStreamer on UDP 5600 — same wire as the drone path. |

## RSIM coverage static (smoke test alone)

| Req | Static check that holds |
|-----|--------------------------|
| RSIM-03 | 3 rover-adapted world SDFs under `sim/rover/worlds/` (`walk_across_then_approach`, `random_walk`, `circle_around`) — verified by `test_world_sdfs_include_rover_model`. The live run used `walk_across_then_approach`. |
| RSIM-07 | `sim/rover/README.md` documents Garden EOL Nov 2024 + Harmonic migration recipe + `gz topic -l` smoke step — verified by `test_readme_documents_eol_and_smoke`. |

## Gap closures landed during operator verification

- `b89c1a1` — fix(05-04): install.sh --rover preflight catches missing ROS 2 apt repo (exits 7 with copy-paste commands instead of opaque "Unable to locate package").
- `a46bc56` — fix(05-01): convert `rover.sdf` into a proper Gazebo model package (`rover/model.sdf` + `rover/model.config`) so `model://rover` resolves under `GZ_SIM_RESOURCE_PATH`.

## Phase 5 status

Code-complete and operator-verified. Phase 6 (Sim integration) unblocked.

The "full E2E rover follows walking actor" gate is **Phase 6 RINT-04**, not Phase 5.
This document records that the sim wire stack (DiffDrive plugin, cmd_vel bridge,
camera UDP) works on the operator's machine — Phase 6 wires drone-follow into
the same UDP stream and exercises the controller path.
