# Rover sim (Phase 5, v1.1)

Phase 5 of the drone-follow v1.1 milestone ships the Gazebo rover sim
infrastructure: a diff-drive SDF model with a forward camera, three
adapted actor worlds, a one-command launcher, and the apt-package
install path.  This README is the operator-facing entry point.

## What's here

- `rover/model.sdf` + `rover/model.config` — diff-drive rover model
  package with a forward camera (Garden plugin:
  `gz::sim::systems::DiffDrive`). Resolved by the worlds via
  `<include><uri>model://rover</uri></include>`; the launcher exports
  `GZ_SIM_RESOURCE_PATH=sim/rover/:…` so `gz sim` finds it.
- `worlds/walk_across_then_approach.sdf` — primary actor world; the
  rover spawns at origin, actor walks across the frame at X=4 then
  approaches.  Phase 6 RINT-04 uses this as the E2E test target.
- `worlds/random_walk.sdf` — actor random-walk pattern.
- `worlds/circle_around.sdf` — actor circles around origin at r=2.5 m.
- `start_rover_sim.sh` — one-command launcher (Plan 05-05).

## Prerequisites

- Ubuntu 22.04 (Jammy)
- ROS 2 Humble installed at `/opt/ros/humble`
- Gazebo Garden installed (`gz` CLI available; `gz sim --version` should
  report 7.x)
- This repo's existing drone-sim deps (run `./install.sh` for the base
  hailo-apps venv).

> Note: this dev environment ships **Gazebo Harmonic** (`gz-sim8`) by
> default.  See "Garden vs Harmonic" below for the migration recipe.

## Quick start

Three commands from a clean checkout:

```bash
# 1. Install rover deps (apt: ros-humble-ros-base, ros-humble-geometry-msgs,
#    ros-humble-ros-gzgarden-bridge, gz-garden).  Idempotent.
sudo ./install.sh --rover

# 2. Launch sim (Gazebo + ros_gz_bridge cmd_vel + video_bridge.py).
sim/rover/start_rover_sim.sh                                    # default world
sim/rover/start_rover_sim.sh --world random_walk                # alternate world

# 3. In a second terminal, launch drone-follow against the sim:
robot-follow --robot rover --input udp://0.0.0.0:5600
```

The rover should follow the walking actor.  drone-follow opens an X11
display window by default (implicit-display rule: when neither `--openhd`
nor `--webui` is set, the display branch is on).  Pass `--webui` for the
browser UI on port 5001, or `--openhd` for the OpenHD RTP stream —
mutually exclusive.

## Smoke test

Run from a second terminal after `start_rover_sim.sh` is up:

```bash
# 1. Topic discovery — should list /cmd_vel, /camera, /world/<name>/clock.
#    Proves both plugins (DiffDrive, Camera) loaded.
gz topic -l | grep -E '(cmd_vel|camera|clock)'

# 2. Verify a Twist message reaches DiffDrive on /cmd_vel.  Proves the
#    <topic>cmd_vel</topic> override in rover.sdf (Pitfall 6 / RSIM-02)
#    worked — without it, DiffDrive subscribes on /model/rover/cmd_vel
#    and the ros_gz_bridge mapping silently fails.
gz topic -e -t /cmd_vel -n 1

# 3. Drive the rover from ROS via the bridge.  Should roll forward
#    at 0.3 m/s for a moment.
ros2 topic pub --once /cmd_vel geometry_msgs/Twist \
    '{linear: {x: 0.3}, angular: {z: 0.0}}'

# 4. Verify camera frames reach UDP 5600 (independent of drone-follow).
#    Proves the video_bridge.py reuse works.
ffplay -fflags nobuffer udp://0.0.0.0:5600
```

If `gz topic -l` does NOT show `/cmd_vel`, the DiffDrive plugin failed
to load — the most common cause is an `ignition::` prefix sneaking back
into `rover.sdf` (silent load failure on Garden, see PITFALLS Pitfall 5).
Check `gz sim` stdout for `Loaded system [gz::sim::systems::DiffDrive]`.

## Gazebo Garden EOL notice

**Gazebo Garden reached end-of-life in November 2024.**  Binary
packages remain available from `packages.osrfoundation.org` but no
longer receive security updates.

We use Garden for v1.1 because:

- The bundled PX4 SITL drone sim (`sim/PX4-Autopilot/`) already
  uses Garden.  Pulling in a second Gazebo would conflict on shared
  libs.
- Migration cost is low (see next section).
- v1.1 is sim-only (no real rover hardware), so the security debt
  is acceptable.

### Migration to Harmonic (post-v1.1 cleanup)

The migration is mechanical:

```diff
# install.sh
- ros-humble-ros-gzgarden-bridge
+ ros-humble-ros-gzharmonic-bridge

- gz-garden
+ gz-harmonic
```

**SDF files require ZERO changes** — `gz::sim::systems::DiffDrive`,
`gz-sim-diff-drive-system`, `<topic>cmd_vel</topic>`, and the camera
sensor stanza are all forward-compatible with Harmonic.

The DiffDrive plugin's `gz-sim-diff-drive-system` filename resolves
against the runtime gz-sim version: gz-sim7 on Garden, gz-sim8 on
Harmonic.  Both ship the same plugin under the same logical name.

Trigger for migration: a CVE in Garden's apt graph, or a new Hailo
runtime that breaks on Garden's transitive deps.  Until then, hold.

## Port 5600 conflict with PX4 SITL

Both PX4 SITL (`sim/start_sim.sh --bridge`) and rover sim
(`sim/rover/start_rover_sim.sh`) bind `udp://0.0.0.0:5600` for the
camera feed.  **They cannot run simultaneously on the same machine.**

This is a documented constraint, not a bug.  If you need both sims
running for cross-validation, pick one host for the drone sim and
another for the rover sim, and point drone-follow's `--input` at the
appropriate UDP source on each.  See `.planning/research/PITFALLS.md`
"Integration Gotchas" for the table entry that owns this constraint.

Phase 6 RINT-05 covers this trade-off acceptance.  A future
`--openhd-port`-style flag for the rover sim camera could remap the
port if a future milestone requires it.

## Known gotchas

These are documented in detail in `.planning/research/PITFALLS.md` —
do not re-quote here; that file is the single source of truth.  Quick
links:

- **Pitfall 5 — `ignition::` prefix in SDF** is a silent load failure
  on Garden.  Use `gz::sim::systems::DiffDrive` and
  `gz-sim-diff-drive-system`.
- **Pitfall 6 — DiffDrive topic namespace.**  Without
  `<topic>cmd_vel</topic>` in the plugin block, DiffDrive subscribes
  on `/model/rover/cmd_vel` and the bridge mapping silently fails.
- **Pitfall 7 — Garden EOL.**  See "Gazebo Garden EOL notice" above.

## Bridge syntax reference

The ros_gz_bridge `parameter_bridge` topic spec uses these directional
symbols:

| Symbol | Direction | Example |
|--------|-----------|---------|
| `@` (between types) | bidirectional | `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist` |
| `[` (replaces 2nd `@`) | ROS → GZ only | `/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist` |
| `]` (replaces 2nd `@`) | GZ → ROS only | `/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry` |

We use the bidirectional form (`@`) for `/cmd_vel`.  `REQUIREMENTS.md`
RSIM-04 wording with a trailing closing-bracket symbol is a typo — the
GZ→ROS directional symbol replaces the second `@` (it sits *between*
the ROS and GZ type strings), it does not trail the type.

The Gazebo message prefix is `gz.msgs.*` (Garden / Harmonic), NOT
`ignition.msgs.*` (Fortress).

## See also

- `.planning/REQUIREMENTS.md` § RSIM-01..07 — Phase 5 requirements.
- `.planning/research/PITFALLS.md` — Garden / ROS / venv pitfall catalog.
- `.planning/phases/05-rover-sim/RESEARCH.md` — Phase 5 research notes.
- `sim/start_sim.sh` — drone-sim launcher, shares cleanup patterns.
- `sim/bridge/video_bridge.py` — shared camera→UDP bridge (UNCHANGED
  between drone and rover sims per RSIM-06 / SUMMARY.md camera
  plumbing decision).
