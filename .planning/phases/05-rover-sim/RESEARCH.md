# Phase 5: Rover sim — Research

**Researched:** 2026-05-20
**HEAD verified:** `ac0edcc` (`feature/rover-support`)
**Domain:** Gazebo Garden differential-drive rover SDF + cmd_vel bridge + camera UDP plumbing for drone-follow. Pure sim infrastructure (no Python edits in `robot_follow/`). Phase 4 (rclpy adapter) is the OTHER side of the wire — Phase 5 stands up the Gazebo world + bridges that adapter publishes to.
**Confidence:** HIGH — every claim grounded in a verified file on the current tree, an apt-cache verified package, an installed `gz` CLI, OR cited from `.planning/research/PITFALLS.md` (already-locked research). The hard library questions are answered in PITFALLS; Phase 5 just assembles them into files.

---

## Summary

Phase 5 is mostly **not Python**. It ships:
1. An SDF rover model with a forward-facing camera and a Garden-era `DiffDrive` plugin.
2. SDF worlds adapting the existing actor walk patterns (`walk_across_then_approach`, `random_walk`, `circle_around`) for ground-vehicle perspective.
3. A bash launcher `sim/rover/start_rover_sim.sh` that starts gz sim + `ros_gz_bridge parameter_bridge` + the existing `sim/bridge/video_bridge.py`.
4. Apt-package install integration (`install.sh --rover`) for `ros-humble-ros-gzgarden-bridge` and friends.
5. A README documenting Garden EOL + `gz topic -l` smoke-test step.

Every Gazebo / ROS / bridge pitfall has already been researched and locked in `.planning/research/PITFALLS.md` Pitfalls 5–7 + Integration Gotchas table. This research **does not redo** that work; it instantiates the locked decisions into concrete files.

**Verified on this dev box:** `apt-cache search ros-humble-ros-gzgarden` returns `ros-humble-ros-gzgarden-bridge` (and `-image`, `-sim`, `-interfaces`, `-sim-demos`, the meta `ros-humble-ros-gzgarden`). The package names from REQUIREMENTS.md RSIM-05 are correct.

**`gz` CLI is installed locally**; `gz sdf -k <file>` is a real SDFormat validator that can run in CI for SDF lint (skip-on-no-gz pattern). `gz topic -l` is the canonical "did the plugin load?" smoke check.

**The biggest land-mine** is one the requirements text gets wrong: the `[`/`]` direction symbol in the `parameter_bridge` topic spec. The current README at gazebosim/ros_gz says `topic@ROS_type[GZ_type` is ROS→GZ and `topic@ROS_type]GZ_type` is GZ→ROS — the symbols replace the second `@`, they don't trail it. REQUIREMENTS.md RSIM-04 and PITFALLS.md Integration Gotchas both have ambiguous wording. The plan should use bidirectional (`@`) for the cmd_vel bridge — it's the simplest, costs nothing, and matches every working example shipped on the gz-sim8 README. See § "ros_gz_bridge syntax — current truth" below.

**Primary recommendation:** Plan as ~5–6 bisectable commits — (1) SDF lint test scaffold + `gz sdf -k` skip-on-no-gz harness; (2) `sim/rover/rover.sdf` + base `empty_rover_world.sdf`; (3) three actor worlds adapted from `sim/worlds/`; (4) `start_rover_sim.sh` + SIGINT process-group cleanup; (5) `install.sh --rover` extension; (6) `README.md` + Garden-EOL + `gz topic -l` smoke checklist. **No Python module edits** in `robot_follow/`. A single optional Python smoke test in `robot_follow/tests/test_rover_sim_smoke.py` can assert (a) SDF files parse with `gz sdf -k`, (b) `start_rover_sim.sh` passes `shellcheck`, (c) `install.sh --rover` lists the right apt packages — all skip when `gz` / `shellcheck` aren't installed.

---

## User Constraints (from REQUIREMENTS.md + PITFALLS.md + SUMMARY.md)

> Phase 5 has no `05-CONTEXT.md` yet (this research runs ahead of `/gsd:discuss-phase`). Constraints below are inherited from the milestone-level requirements and from already-locked research.

### Locked Decisions (RSIM-01..07 + PITFALLS Pitfalls 5–7 + SUMMARY.md Camera Plumbing Decision)

1. **Garden-era plugin names mandatory.** `<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">`. The `ignition::` prefix produces a **silent load failure** — Gazebo starts, no error, `cmd_vel` ignored. Source: PITFALLS Pitfall 5; RSIM-01.

2. **DiffDrive `<topic>cmd_vel</topic>` override.** Without it the plugin subscribes to `/model/<model_name>/cmd_vel` and the bridge config has to know the model name. With it, both sides are `/cmd_vel`. Source: PITFALLS Pitfall 6; RSIM-02.

3. **Camera plumbing is `sim/bridge/video_bridge.py` reuse, NOT `ros_gz_image_bridge`.** SUMMARY.md § "Camera Plumbing Decision" locked this — `ros_gz_image_bridge` has a documented ~15 Hz publish ceiling in Garden (`gazebosim/ros_gz#368`, also in PITFALLS Integration Gotchas table). `video_bridge.py` does direct gz-transport → H.264 UDP today for the drone sim; the rover sim just passes a different `--topic`. Source: SUMMARY.md lines 78–99; RSIM-06.

4. **UDP port stays 5600** (matches `drone-follow --input udp://0.0.0.0:5600`). No new `--openhd-port`-style flag in Phase 5. Source: RSIM-06.

5. **Apt packages for `install.sh --rover`:** `ros-humble-ros-base`, `ros-humble-geometry-msgs`, **`ros-humble-ros-gzgarden-bridge`** — NOT `ros-humble-ros-gz-bridge` (Fortress). Hard-fail with friendly error if `/opt/ros/humble` or `gz` is missing. Source: RSIM-05; **VERIFIED** on this dev box via `apt-cache search`.

6. **README must document** Garden EOL (Nov 2024), Harmonic migration path (SDF `gz::` prefixes already compatible — just change apt package names), and the `gz topic -l` smoke-test step. Source: RSIM-07.

7. **Three actor worlds adapted for rover:** `walk_across_then_approach`, `random_walk`, `circle_around`. The other three drone worlds (`person_in_front`, `2_person_world`, `2_persons_diagonal`) are out of scope for Phase 5. Source: RSIM-03 + roadmap Success Criterion 4 (only `walk_across_then_approach` is required to render; the other two are bonus).

8. **`video_bridge.py` is reused verbatim** — no edits. The rover sim invokes it with `--topic <rover_camera_topic>`. Source: RSIM-06; current `sim/bridge/video_bridge.py:36–39` already has `--topic` flag with default `/camera`.

### Claude's Discretion

- **`setup_rover_sim.sh` vs `install.sh --rover`:** RSIM-05 lists both ("`install.sh --rover` OR `sim/rover/setup_rover_sim.sh`"). **Recommendation: ship `install.sh --rover` only**, mirror the existing `--skip-*` flag pattern. Adding a separate `setup_rover_sim.sh` doubles maintenance burden for no win; if a future user wants rover-only install they pass `--skip-apps --skip-hefs --skip-ui --rover`. See § Open Questions Q1 — this is the only real ambiguity.

- **Rover wheel parameters:** RSIM-01 doesn't pin numeric values. Pick defaults from the upstream gz-sim diff_drive example at `/usr/share/gz/gz-sim8/worlds/diff_drive.sdf` (verified on disk; `wheel_separation=1.25`, `wheel_radius=0.3`, `max_linear_velocity=0.5`, `max_angular_velocity=1.0`). **Recommendation:** scale down to `wheel_separation=0.5`, `wheel_radius=0.15`, `max_linear_velocity=1.0`, `max_angular_velocity=2.0` — a smaller, faster rover suits follow-the-person tighter than a 2 m van.

- **Camera resolution:** The Hailo pipeline default is 1280×720 (verified at `sim/PX4-Autopilot/.../x500_vision/model.sdf` lines 25–37). **Recommendation:** match it (1280×720, 30 Hz) for zero pipeline-side changes. Camera pose: 0.15 m forward of chassis center, 0.25 m up, level pitch — standard "front bumper" mount.

- **Smoke-test location:** RSIM-07's mandate is "`gz topic -l` step documented in README". **Recommendation:** add an OPTIONAL Python smoke test at `robot_follow/tests/test_rover_sim_smoke.py` that (a) shellchecks the bash, (b) `gz sdf -k`'s every SDF, (c) parses `install.sh` for the rover apt list. **All skip-on-missing-tool** so it never blocks dev-box CI. The E2E "rover follows actor in Gazebo" test belongs in **Phase 6 (RINT-04)**, not Phase 5.

- **Process-group cleanup in start_rover_sim.sh:** New land-mine (not in PITFALLS — it's bash, not ROS). **Recommendation:** use `trap cleanup EXIT INT TERM` + `setsid`-style process-group kill, OR collect PIDs and `kill 0` on the group. `start_sim.sh:173-179` has the prior-art pattern (`cleanup() { kill "$pid"; wait "$pid"; }`) — copy it.

- **Whether `start_rover_sim.sh` should accept `--world NAME` like `start_sim.sh`:** **Recommendation: yes.** Mirror the drone-sim flag for consistency; default to `walk_across_then_approach`. Out of scope: `--remote IP` and `--bridge` toggle (rover sim always bridges; nothing to switch off).

### Deferred Ideas (OUT OF SCOPE for Phase 5)

- `configs/rover_simulation.json` and any controller knobs — **Phase 6 (RINT-01)**.
- Bottom-edge safety repurpose — **Phase 6 (RINT-02)**.
- ByteTracker retuning — **Phase 6 (RINT-03)**.
- End-to-end "rover follows actor" deterministic test — **Phase 6 (RINT-04)**.
- SIGINT integration test against the live rclpy adapter — **Phase 6 (RINT-06)**; Phase 5 only validates that `start_rover_sim.sh` cleans up its own subprocesses.
- `/odom` subscription — **v1.2 (HW-03)**.
- twist_mux / e-stop / bumper / diagnostics — **v1.2 (HW-04..HW-07)**.
- Harmonic migration — REQUIREMENTS § Out of Scope ("Garden is EOL but acceptable for v1.1 sim-only").

---

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| RSIM-01 | `sim/rover/rover.sdf` with Garden-era DiffDrive plugin + forward camera | § File structure → `rover.sdf` template; § ros_gz_bridge syntax; PITFALLS Pitfall 5 |
| RSIM-02 | `<topic>cmd_vel</topic>` override in SDF | § rover.sdf template (line marked `<topic>cmd_vel</topic>`); PITFALLS Pitfall 6 |
| RSIM-03 | Three rover-adapted actor worlds (`walk_across_then_approach`, `random_walk`, `circle_around`) | § Adapting drone worlds for rover |
| RSIM-04 | `sim/rover/start_rover_sim.sh` one-command launcher | § `start_rover_sim.sh` flow pseudocode |
| RSIM-05 | `install.sh --rover` installs apt packages, hard-fails friendly | § `install.sh --rover` extension; verified apt names |
| RSIM-06 | Reuse `sim/bridge/video_bridge.py` for camera→UDP 5600 | § video_bridge.py reuse contract; SUMMARY.md camera plumbing decision |
| RSIM-07 | `README.md` with Garden EOL + `gz topic -l` smoke step | § README structure |

---

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Rover physical model + diff-drive dynamics | Gazebo (gz-sim) | — | DiffDrive plugin lives in `gz-sim-diff-drive-system`. ROS never sees joints. |
| Actor walking trajectory | Gazebo (gz-sim) | — | `<actor>` + `<script>` waypoints; identical to drone worlds. |
| `/cmd_vel` ROS → GZ transport | `ros_gz_bridge` (parameter_bridge) | — | Single-purpose bridge process. No business logic. |
| Camera frame → UDP H.264 RTP | `sim/bridge/video_bridge.py` (already exists) | gz-transport13 | Direct gz-transport subscriber, GStreamer x264enc, no ROS hop. |
| UDP frame ingest | `robot_follow.pipeline_adapter` (already exists) | GStreamer udpsrc | Same `--input udp://0.0.0.0:5600` path the drone sim uses today. |
| Process lifecycle (gz sim + bridge + video bridge) | `sim/rover/start_rover_sim.sh` | bash + trap | Process-group cleanup on SIGINT; mirrors `sim/start_sim.sh:173-179`. |
| Apt dependency install | `install.sh --rover` | apt + sudo | Hard-fails with friendly errors on missing `/opt/ros/humble` or `gz`. |
| SDF validity | `gz sdf -k` (sim-only test) | pytest skip-on-no-gz | CI-friendly; never blocks dev boxes without gz. |

**Key insight:** Phase 5 touches **zero** Python production code under `robot_follow/`. The only Python it may add is `robot_follow/tests/test_rover_sim_smoke.py`, which is itself optional and skip-on-no-tool.

---

## File Structure to Create

```
sim/rover/                              # NEW directory
├── rover.sdf                           # Diff-drive rover model w/ camera
├── worlds/                             # Rover-adapted actor worlds
│   ├── walk_across_then_approach.sdf   # primary test world (RINT-04 in Phase 6)
│   ├── random_walk.sdf
│   └── circle_around.sdf
├── start_rover_sim.sh                  # one-command launcher
└── README.md                           # Garden EOL + smoke checklist (RSIM-07)

install.sh                              # EDIT — add --rover flag + apt install branch
robot_follow/tests/test_rover_sim_smoke.py   # OPTIONAL: SDF lint + shellcheck + apt-list parse (skip-on-no-tool)
```

**Not created (defer to Phase 6):**
- `sim/rover/setup_rover_sim.sh` — fold into `install.sh --rover`. See § Open Questions Q1.
- `configs/rover_simulation.json` — RINT-01, Phase 6.
- `robot_follow/tests/test_sim_worlds.py` rover variant — RINT-04, Phase 6.

---

## Concrete `rover.sdf` Template

Source: synthesized from `/usr/share/gz/gz-sim8/worlds/diff_drive.sdf` (verified on disk) + camera stanza from `sim/PX4-Autopilot/Tools/simulation/gz/models/x500_vision/model.sdf:20-46` (verified on disk) + RSIM-01/02 locks.

```xml
<?xml version="1.0" ?>
<!--
  sim/rover/rover.sdf — differential-drive rover model with forward-facing camera.

  - DiffDrive plugin uses gz::sim::systems::DiffDrive (Garden+; `ignition::`
    prefix is a silent-load-failure footgun per PITFALLS.md Pitfall 5).
  - <topic>cmd_vel</topic> overrides the plugin's default
    /model/rover/cmd_vel so ros_gz_bridge maps /cmd_vel ↔ /cmd_vel cleanly
    (PITFALLS Pitfall 6 / RSIM-02).
  - Camera matches drone-follow pipeline default (1280×720, 30 Hz, ~66° hfov)
    so SITL-tuned gains transfer cleanly between sim and real (x500_vision
    convention).
-->
<sdf version="1.9">
  <model name="rover">
    <pose>0 0 0.15 0 0 0</pose>
    <self_collide>false</self_collide>

    <link name="chassis">
      <pose>0 0 0.10 0 0 0</pose>
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.05</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.10</iyy><iyz>0</iyz>
          <izz>0.12</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><box><size>0.40 0.30 0.15</size></box></geometry>
        <material>
          <ambient>0.2 0.4 0.8 1</ambient>
          <diffuse>0.2 0.4 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.40 0.30 0.15</size></box></geometry>
      </collision>
    </link>

    <!-- Forward-facing camera. Topic, FOV, resolution, update rate all
         match drone x500_vision (sim/PX4-Autopilot/.../x500_vision/model.sdf). -->
    <link name="camera_link">
      <pose>0.20 0 0.20 0 0 0</pose>
      <inertial><mass>0.01</mass>
        <inertia><ixx>1e-5</ixx><iyy>1e-5</iyy><izz>1e-5</izz></inertia>
      </inertial>
      <sensor name="camera" type="camera">
        <topic>/camera</topic>
        <gz_frame_id>camera_link</gz_frame_id>
        <pose>0 0 0 0 0 0</pose>
        <camera>
          <horizontal_fov>1.152</horizontal_fov>  <!-- 66° -->
          <image><width>1280</width><height>720</height></image>
          <clip><near>0.1</near><far>3000</far></clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
        <visualize>true</visualize>
      </sensor>
    </link>
    <joint name="camera_joint" type="fixed">
      <parent>chassis</parent>
      <child>camera_link</child>
    </joint>

    <!-- Left/right drive wheels (cylinders, axis along chassis Y). -->
    <link name="left_wheel">
      <pose>0 0.20 0 -1.5708 0 0</pose>
      <inertial><mass>0.5</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>
    <link name="right_wheel">
      <pose>0 -0.20 0 -1.5708 0 0</pose>
      <inertial><mass>0.5</mass>
        <inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><cylinder><radius>0.15</radius><length>0.05</length></cylinder></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
    </link>

    <!-- Rear caster (passive ball) for stability. -->
    <link name="caster">
      <pose>-0.18 0 -0.05 0 0 0</pose>
      <inertial><mass>0.1</mass>
        <inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz></inertia>
      </inertial>
      <visual name="visual">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </visual>
      <collision name="collision">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
      </collision>
    </link>

    <joint name="left_wheel_joint" type="revolute">
      <parent>chassis</parent><child>left_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>
    <joint name="right_wheel_joint" type="revolute">
      <parent>chassis</parent><child>right_wheel</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>-1.79769e+308</lower><upper>1.79769e+308</upper></limit>
      </axis>
    </joint>
    <joint name="caster_joint" type="ball">
      <parent>chassis</parent><child>caster</child>
    </joint>

    <!-- Garden-era diff-drive plugin. ignition:: prefix = silent load failure. -->
    <plugin filename="gz-sim-diff-drive-system"
            name="gz::sim::systems::DiffDrive">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.40</wheel_separation>   <!-- 2 × 0.20 m -->
      <wheel_radius>0.15</wheel_radius>
      <!-- RSIM-02: override default topic /model/rover/cmd_vel → /cmd_vel -->
      <topic>cmd_vel</topic>
      <odom_publish_frequency>10</odom_publish_frequency>
      <!-- Conservative caps; rover-safe values (matches rover_simulation.json
           defaults pending in Phase 6 RINT-01). -->
      <max_linear_velocity>1.0</max_linear_velocity>
      <min_linear_velocity>-0.5</min_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
      <min_angular_velocity>-2.0</min_angular_velocity>
      <max_linear_acceleration>2.0</max_linear_acceleration>
      <min_linear_acceleration>-2.0</min_linear_acceleration>
      <max_angular_acceleration>4.0</max_angular_acceleration>
      <min_angular_acceleration>-4.0</min_angular_acceleration>
    </plugin>
  </model>
</sdf>
```

**Verification command (planner adds to plan):**
```bash
gz sdf -k sim/rover/rover.sdf   # exit 0 if SDF parses; non-zero otherwise
```

---

## Adapting Drone Worlds for Rover

The three required worlds (`walk_across_then_approach`, `random_walk`, `circle_around`) already exist under `sim/worlds/` for the drone. The adaptation for rover is:

| Element | Drone world | Rover world |
|---------|-------------|-------------|
| Actor `<pose>` Z | 1.0 (eye level with ~3 m drone) | 1.0 (eye level for ground rover camera at 0.20 m height; trajectory unchanged) |
| Actor X start | 10 (matches drone forward distance) | 4 (closer — rover has less forward range; actor must enter frame within ~5 s of bring-up) |
| Drone model | spawned by PX4 SITL via `PX4_GZ_WORLD` | **`<include><uri>model://rover</uri></include>`** at `<pose>0 0 0.15 0 0 0</pose>` |
| World plugins | Same set as drone (Physics, UserCommands, SceneBroadcaster, Contact, Imu, AirPressure, Sensors) | **Drop Imu + AirPressure + Contact** — rover doesn't need them. Keep Physics, UserCommands, SceneBroadcaster, Sensors. |
| `GZ_SIM_RESOURCE_PATH` | `sim/worlds:` (for Walking actor mesh) | Add `sim/rover:sim/worlds:` so both `model://rover` AND `model://Walking actor` resolve. |
| `<scene>`, `<light>`, `<ground_plane>` | Identical |Identical — copy verbatim |

**Recommendation:** Copy each source world verbatim, then apply a diff that (a) inserts `<include><uri>model://rover</uri></include>` after the ground_plane block, (b) scales actor X by 0.4 to compress the trajectory, (c) removes the unused Imu/AirPressure/Contact plugins. **Do not** edit the actor trajectory mid-pose-segment — copy the waypoint table whole then scale.

**Concrete example** (rover variant of `walk_across_then_approach.sdf`, abbreviated — full file is ~80 lines):

```xml
<sdf version="1.9">
  <world name="walk_across_then_approach">
    <!-- Rover model — spawned by world, not by PX4 -->
    <include>
      <uri>model://rover</uri>
      <pose>0 0 0.15 0 0 0</pose>
    </include>

    <!-- Actor — trajectory waypoints scaled to rover range -->
    <actor name="person">
      <skin><filename>model://Walking actor/meshes/walk.dae</filename></skin>
      <animation name="walking">
        <filename>model://Walking actor/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <!-- X=4 (was 10), Y range ±2.5 (was ±6). Walk speed ~1 m/s preserved. -->
          <waypoint><time>0</time><pose>4 -2.5 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>5</time><pose>4 -2.5 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>10</time><pose>4 2.5 0 0 0 1.5708</pose></waypoint>
          <!-- ... remaining phases scaled similarly ... -->
        </trajectory>
      </script>
    </actor>

    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <plugin name='gz::sim::systems::Physics' filename='gz-sim-physics-system'/>
    <plugin name='gz::sim::systems::UserCommands' filename='gz-sim-user-commands-system'/>
    <plugin name='gz::sim::systems::SceneBroadcaster' filename='gz-sim-scene-broadcaster-system'/>
    <plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'>
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.8</gravity>
    <!-- ground_plane, sun, scene — copy from sim/worlds/walk_across_then_approach.sdf -->
  </world>
</sdf>
```

**Important:** The actor `<pose>` Z value (1.0 in drone worlds — "head height with drone at 3 m") may need tweaking for the ground rover camera at 0.20 m height. Test the first world with `gz sim -r sim/rover/worlds/walk_across_then_approach.sdf` headed once; if the actor is below frame, drop Z to 0.0. **Defer to Phase 6 (RINT-04) E2E test for tuning.** Phase 5's bar is "actor renders + rover model appears on ground plane" (Success Criterion 4).

---

## `start_rover_sim.sh` Flow

Pseudocode — mirrors `sim/start_sim.sh:18-196` patterns (REAP_STALE preflight, cleanup trap, BG_PIDS array). 

```bash
#!/usr/bin/env bash
# sim/rover/start_rover_sim.sh — one-command rover sim launcher.
#
# Starts:
#   1. gz sim    -- Gazebo Garden with rover world
#   2. ros2 run ros_gz_bridge parameter_bridge   -- /cmd_vel ROS <-> GZ
#   3. sim/bridge/video_bridge.py   -- /camera gz topic -> UDP 5600
#
# Usage: sim/rover/start_rover_sim.sh [--world NAME] [--no-reap-stale]
#
# Cleanup: SIGINT (Ctrl+C) traps and kills all 3 children + their groups.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_DIR="$(dirname "$SCRIPT_DIR")"                  # sim/
PROJECT_ROOT="$(dirname "$SIM_DIR")"
ROVER_WORLDS="$SCRIPT_DIR/worlds"
BRIDGE_SCRIPT="$SIM_DIR/bridge/video_bridge.py"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

# --- Parse args ---
WORLD="walk_across_then_approach"
REAP_STALE=true
while [[ $# -gt 0 ]]; do
  case $1 in
    --world)
      WORLD="${2:?--world requires a name}"; shift 2 ;;
    --no-reap-stale)
      REAP_STALE=false; shift ;;
    -h|--help)
      sed -n '2,12p' "$0"; exit 0 ;;
    *)
      echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

WORLD_FILE="$ROVER_WORLDS/$WORLD.sdf"
[ -f "$WORLD_FILE" ] || {
  echo -e "${RED}World not found: $WORLD_FILE${NC}" >&2
  ls "$ROVER_WORLDS"/*.sdf 2>/dev/null | xargs -n1 basename
  exit 2
}

# --- Preflight ---
command -v gz >/dev/null || {
  echo -e "${RED}'gz' CLI missing. Install Gazebo Garden:${NC}" >&2
  echo "  sudo apt install gz-garden  (after adding the osrfoundation apt repo)" >&2
  exit 3
}
[ -f /opt/ros/humble/setup.bash ] || {
  echo -e "${RED}/opt/ros/humble/setup.bash missing. Install ROS 2 Humble:${NC}" >&2
  echo "  sudo apt install ros-humble-ros-base ros-humble-ros-gzgarden-bridge" >&2
  exit 4
}
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# --- Preflight: reap stale processes from a prior run ---
# Mirrors start_sim.sh:80-117 — patterns scoped to this repo so other gz/ros
# instances on the machine aren't touched.
preflight_reap_stale() {
  local patterns=(
    "gz sim.*$ROVER_WORLDS"
    "ros_gz_bridge parameter_bridge"
    "$BRIDGE_SCRIPT"
  )
  local pids
  pids=$(for pat in "${patterns[@]}"; do pgrep -f -- "$pat" 2>/dev/null || true
         done | sort -u | grep -vE "^($$|$PPID)$" || true)
  [ -z "$pids" ] && return 0

  echo -e "${YELLOW}Stale processes detected:${NC}" >&2
  while read -r pid; do [ -n "$pid" ] && ps -p "$pid" -o pid=,args= 2>/dev/null
                       done <<<"$pids" >&2
  if ! $REAP_STALE; then
    echo -e "${RED}Refusing to start. Kill them or rerun without --no-reap-stale.${NC}" >&2
    exit 5
  fi
  while read -r pid; do [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
                       done <<<"$pids"
  sleep 1
  while read -r pid; do [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
                       done <<<"$pids"
}
preflight_reap_stale

# --- Resource paths so model:// includes resolve ---
# sim/rover/ for model://rover; sim/worlds/ for model://Walking actor.
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR:$SIM_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# --- Track children for cleanup ---
BG_PIDS=()
cleanup() {
  echo -e "${YELLOW}Shutting down rover sim...${NC}" >&2
  for pid in "${BG_PIDS[@]}"; do
    # Kill the whole process group (negative pid). gz sim and ros2 spawn
    # children; without this they survive Ctrl+C and the next launch fails.
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  for pid in "${BG_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo -e "${GREEN}Starting rover sim ($WORLD)${NC}"
echo "  World:  $WORLD_FILE"
echo "  cmd_vel: ROS /cmd_vel <--> GZ /cmd_vel (bidirectional)"
echo "  camera:  GZ /camera --> udp://127.0.0.1:5600 (H.264 RTP)"
echo ""

# --- Launch Gazebo Garden ---
# setsid so children form their own process group (cleanup uses `kill -- -PID`).
setsid gz sim -r "$WORLD_FILE" &
BG_PIDS+=($!)
sleep 3   # give gz sim time to register topics before bridge tries to attach

# --- Launch ros_gz_bridge ---
# Bidirectional cmd_vel bridge. Garden uses gz.msgs prefix (verified in
# /usr/share/gz/gz-sim8 README; PITFALLS.md Integration Gotchas).
setsid ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
BG_PIDS+=($!)

# --- Launch video bridge ---
# Reuses sim/bridge/video_bridge.py verbatim. The PROTOCOL_BUFFERS env var
# avoids a protobuf C++/python version mismatch (see video_bridge.py:14-16).
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    python3 "$BRIDGE_SCRIPT" --topic /camera --host 127.0.0.1 --port 5600 &
BG_PIDS+=($!)

echo -e "${GREEN}Smoke-test from another terminal:${NC}"
echo "  gz topic -l                          # /cmd_vel, /camera, /world/.../clock should appear"
echo "  gz topic -e -t /cmd_vel -n 1          # echo one Twist (proves DiffDrive subscribed)"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'   # rover rolls forward"
echo "  ffplay -fflags nobuffer udp://0.0.0.0:5600    # camera feed (or drone-follow --input udp://...)"
echo ""
echo "Press Ctrl+C to stop."
wait
```

**Critical bash details (planner must enforce):**

1. **`setsid`** before each background launcher so each child becomes a process-group leader. `kill -- -$pid` then nukes the group. Without this, `gz sim`'s children outlive Ctrl+C and block the next launch.
2. **`trap cleanup EXIT INT TERM`** — three signals, one cleanup. `EXIT` catches normal returns; `INT` catches Ctrl+C; `TERM` catches `kill <pid>`.
3. **`sleep 3` between gz sim and bridge** — `parameter_bridge` exits with `unknown topic` if /cmd_vel doesn't exist yet in gz-transport. Three seconds is the empirical value used in `start_sim.sh`-style launchers across the gz ecosystem. Not a hard rule; the bridge can be retried on failure if a future plan wants a polling loop instead.
4. **Source ROS BEFORE checking `ros2 run`** — `command -v ros2` lies before sourcing.

**Skip-if-no-tool exit codes (testable):** 3 = missing `gz`, 4 = missing `/opt/ros/humble`, 5 = stale processes refused. Phase 5 smoke test can `start_rover_sim.sh && rc=$? ; [[ $rc -eq 3 || $rc -eq 4 ]]` on a clean box to verify the friendly errors fire.

---

## `install.sh --rover` Extension

Patch points in `install.sh` (verified at `install.sh:27-51`):

```bash
# Add to the existing while loop at install.sh:38-51:
ROVER_DEPS=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-submodule) SKIP_SUBMODULE=true; shift ;;
    --skip-apps)      SKIP_APPS=true; shift ;;
    --skip-python)    SKIP_PYTHON=true; shift ;;
    --skip-hefs)      SKIP_HEFS=true; shift ;;
    --skip-ui)        SKIP_UI=true; shift ;;
    --rover)          ROVER_DEPS=true; shift ;;        # NEW
    -h|--help)        sed -n '2,16p' "$0"; exit 0 ;;
    *) echo "Unknown flag: $1" >&2; exit 2 ;;
  esac
done

# Add a new step (renumber existing [N/5] -> [N/6]):
if $ROVER_DEPS; then
  echo "==> [6/6] Installing ROS 2 Humble + Gazebo Garden bridge"

  # Hard-fail preflight: /opt/ros/humble must exist OR be installable now.
  if ! command -v apt-get >/dev/null; then
    echo "ERROR: --rover requires apt-get (Ubuntu/Debian only)." >&2
    exit 6
  fi

  # Check that the user has the osrfoundation apt repo configured (Garden
  # ships from there, not from packages.ros.org). Friendly error if missing.
  if ! apt-cache search ros-humble-ros-gzgarden-bridge 2>/dev/null \
       | grep -q ros-humble-ros-gzgarden-bridge; then
    echo "ERROR: ros-humble-ros-gzgarden-bridge not visible to apt-cache." >&2
    echo "       Add the osrfoundation repo first:" >&2
    echo "         sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \\" >&2
    echo "              -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg" >&2
    echo "         echo 'deb [signed-by=...] http://packages.osrfoundation.org/gazebo/ubuntu-stable \$(lsb_release -cs) main' \\" >&2
    echo "              | sudo tee /etc/apt/sources.list.d/gazebo-stable.list" >&2
    echo "         sudo apt update" >&2
    exit 7
  fi

  sudo apt-get install -y \
       ros-humble-ros-base \
       ros-humble-geometry-msgs \
       ros-humble-ros-gzgarden-bridge \
       gz-garden

  # Verify gz CLI is now on PATH.
  command -v gz >/dev/null || {
    echo "ERROR: 'gz' still not found after install — check apt logs." >&2
    exit 8
  }
fi
```

**Verified package names** (from `apt-cache search ros-humble-ros-gzgarden` on this dev box, 2026-05-20):
- `ros-humble-ros-gzgarden` — meta-package
- `ros-humble-ros-gzgarden-bridge` — **THE ONE WE NEED**
- `ros-humble-ros-gzgarden-image` — image bridge (we don't use this; SUMMARY.md decision)
- `ros-humble-ros-gzgarden-interfaces` — pulled in transitively
- `ros-humble-ros-gzgarden-sim` — Gazebo launch wrappers (unused)
- `ros-humble-ros-gzgarden-sim-demos` — demos (unused)

**Apt package — do NOT confuse with:**
- `ros-humble-ros-gz-bridge` — this is the **Fortress** binding (per PITFALLS Pitfall 5 and the apt-cache listing above where it doesn't even exist as a flat package name without `garden`/`harmonic`).
- `ros-humble-ros-gzharmonic-bridge` — Harmonic binding (post-Garden EOL migration target).

---

## `video_bridge.py` Reuse Contract

Verified at `sim/bridge/video_bridge.py` (lines 33-45):

| CLI flag | Default | What Phase 5 passes |
|----------|---------|---------------------|
| `--topic` | `/camera` | `/camera` (matches `<topic>/camera</topic>` in rover.sdf) |
| `--host`  | `127.0.0.1` | `127.0.0.1` (loopback; multi-machine deferred to v1.2) |
| `--port`  | `5600` | `5600` (RSIM-06 lock; matches `drone-follow --input udp://0.0.0.0:5600`) |
| `--bitrate` | `2000` (kbps) | `2000` (default) |
| `--fps` | `30` | `30` (matches camera sensor `<update_rate>30</update_rate>`) |
| `--discover` | flag | not used in `start_rover_sim.sh` — manual debugging only |

The bridge **subscribes to gz-transport13** via the `gz.transport13.Node` Python API (`video_bridge.py:26`). This works on Garden (`gz-sim7` uses gz-transport12) **only because PX4's gz_transport13_compat.patch already converted the venv to gz-transport13** (verified at `sim/setup_sim.sh:75`). The same patch path is shared between drone sim and rover sim — no new patch needed.

**No edits to `video_bridge.py`.** The launcher just passes flags.

---

## ros_gz_bridge syntax — current truth

The PITFALLS.md and REQUIREMENTS.md text both have ambiguous wording on the directional symbols. The verified syntax (from the gazebosim/ros_gz `ros2` branch README) is:

| Symbol | Direction | Example |
|--------|-----------|---------|
| `@` (between ROS type and GZ type) | Bidirectional | `/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist` |
| `[` (replaces the second `@`) | ROS → GZ only | `/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist` |
| `]` (replaces the second `@`) | GZ → ROS only | `/odom@nav_msgs/msg/Odometry]gz.msgs.Odometry` |

**The `]` does NOT trail the GZ type** as REQUIREMENTS.md RSIM-04 implies (`Twist@gz.msgs.Twist]` is malformed). It replaces the second `@`.

**Recommended for `/cmd_vel`: bidirectional (`@`).** The ROS adapter only publishes (never subscribes to /cmd_vel), so technically `[` would be fine. But `@` is the default in every working example shipped with `/usr/share/gz/gz-sim8` and costs nothing extra. Keep it simple.

**Gazebo message prefix on Garden: `gz.msgs.*`** (NOT `ignition.msgs.*`). The `gz`-prefixed names are used in every current branch of the ros_gz repo and verified against the on-disk gz-sim8 diff_drive example.

**Concrete bridge invocation Phase 5 emits:**
```bash
ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

**The planner should also alert reviewers** that the REQUIREMENTS.md RSIM-04 wording `geometry_msgs/Twist@gz.msgs.Twist]` is a typo — clarify in the plan rationale that bidirectional `@` is the chosen form. Tag this as a soft-locked deviation justified by the verified upstream README.

---

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Camera frames → UDP H.264 | A new `rclpy.Image` subscriber + custom GStreamer writer | `sim/bridge/video_bridge.py` (already exists, gz-transport direct) | SUMMARY.md camera-plumbing decision; ROS image bridge caps at ~15 Hz on Garden (`ros_gz#368`). |
| cmd_vel ROS → GZ transport | Custom Python translator (`rclpy.Twist` → `gz.msgs.Twist`) | `ros2 run ros_gz_bridge parameter_bridge` | Maintained upstream; one CLI invocation; covers all message types in `ros_gz_bridge/conversions/`. |
| Diff-drive physics | Custom `<plugin>` shim | `gz-sim-diff-drive-system` (built-in) | Handles slip, accel limits, odom publish, all in C++; pure-Python re-impl would be a tarpit. |
| Process cleanup | `pkill -f gz_sim` global murder | `setsid` + per-PID group kill via `trap` | Global pkill nukes unrelated gz instances (the dev's other drone sim). Per-PID is correct. |
| SDF validation | A regex-based XML linter | `gz sdf -k <file>` | Built into the `gz` CLI; understands gz-sim semantic rules (joint existence, plugin loadability) that regex can't. |
| Stale-process detection | Hope | `pgrep -f` scoped to repo paths (mirrors `start_sim.sh:80-117`) | Existing prior art; respects PID 1/PPID excludes. |

---

## Architecture Patterns

### Process Topology (System Architecture Diagram)

```
                          ┌──────────────────────────────────┐
                          │ sim/rover/start_rover_sim.sh     │
                          │ (trap cleanup EXIT INT TERM)     │
                          │                                  │
                          │  Spawns 3 process groups via     │
                          │  setsid. Cleans up via kill -G.  │
                          └────────────┬─────────────────────┘
                                       │
                ┌──────────────────────┼──────────────────────────┐
                ▼                      ▼                          ▼
   ┌───────────────────┐   ┌──────────────────────────┐   ┌──────────────────────┐
   │   gz sim -r       │   │ ros2 run ros_gz_bridge   │   │ video_bridge.py      │
   │   <world>.sdf     │   │ parameter_bridge         │   │ --topic /camera      │
   │                   │   │ /cmd_vel@.../Twist@.Twist│   │ --port 5600          │
   │ (Garden, ogre2)   │   │                          │   │                      │
   └────────┬──────────┘   └────────┬─────────────────┘   └──────────┬───────────┘
            │                       │                                │
            │ gz-transport13        │ gz <--> ROS                    │ gz-transport13
            │                       │                                │ Image message
            ▼                       ▼                                ▼
   ┌─────────────────────────────────────────────────────────────────────────────┐
   │                       gz-transport13 broker                                 │
   │                                                                             │
   │   /world/<w>/clock   /camera (raw Image)   /cmd_vel (Twist)                 │
   │   /model/rover/odometry (DiffDrive output, unused in v1.1)                  │
   └───────────────────────┬───────────────────────────────────────┬─────────────┘
                           │ Twist subscribed by                   │ Image consumed by
                           │ DiffDrive plugin                      │ video_bridge.py
                           ▼                                       ▼
                   ┌──────────────────┐                  ┌──────────────────────┐
                   │ Rover physics    │                  │ x264enc | rtph264pay │
                   │ (gz-sim)         │                  │ → udp://...:5600     │
                   └──────────────────┘                  └──────────┬───────────┘
                                                                    │
                                                                    │ H.264 RTP
                                                                    ▼
                                                       ┌────────────────────────────┐
                                                       │ drone-follow --input       │
                                                       │  udp://0.0.0.0:5600        │
                                                       │ (Phase 4 Ros2RoverAdapter  │
                                                       │  publishes /cmd_vel)       │
                                                       └────────────────────────────┘
```

### Component Responsibilities

| Component | Responsibility | File(s) |
|-----------|---------------|---------|
| `start_rover_sim.sh` | Process orchestration, SIGINT cleanup, env preflight | `sim/rover/start_rover_sim.sh` |
| `rover.sdf` | Rover model: links, joints, camera sensor, DiffDrive plugin | `sim/rover/rover.sdf` |
| World SDFs | Actor trajectory + rover spawn pose + ground plane + lights | `sim/rover/worlds/*.sdf` |
| `parameter_bridge` | `/cmd_vel` ROS↔GZ wire conversion | (binary; not in this repo) |
| `video_bridge.py` | gz Image → H.264 UDP RTP | `sim/bridge/video_bridge.py` (UNCHANGED) |
| `install.sh --rover` | Apt install of ROS + ros_gz_bridge + gz-garden | `install.sh` (EDIT) |
| `README.md` | Garden EOL note + smoke-test step | `sim/rover/README.md` |

---

## README Structure (RSIM-07)

Outline only — planner writes the file. Should include:

1. **Purpose & scope** — Phase 5 sim infrastructure; one-liner.
2. **Quick start** — three commands: `sudo ./install.sh --rover`, then in two terminals `sim/rover/start_rover_sim.sh` + `robot-follow --robot rover --input udp://0.0.0.0:5600`.
3. **Smoke test** — `gz topic -l` shows `/cmd_vel`, `/camera`, `/world/<name>/clock`; `gz topic -e -t /cmd_vel -n 1` shows Twist; `ros2 topic pub --once /cmd_vel ...` rolls the rover; `ffplay udp://0.0.0.0:5600` shows camera. (Per RSIM-07.)
4. **Gazebo Garden EOL notice** — November 2024; binaries still on osrfoundation but unmaintained; security debt acknowledged for v1.1.
5. **Harmonic migration path** — Phase 5 SDF uses `gz::sim::systems::*` and `gz.msgs.*` exclusively, which are Harmonic-compatible. Migration is `s/gzgarden/gzharmonic/` in `install.sh` and `s/gz-sim7/gz-sim8/` in any plugin path. No SDF edits required.
6. **Port conflict with PX4 SITL** — both use UDP 5600 for video. Cannot run both sims simultaneously on the same machine. Documented but not blocked.
7. **Known gotchas** — link to PITFALLS.md Pitfalls 5–7 by section name, not full re-quote.
8. **Architecture diagram** — copy the ASCII diagram from § Architecture Patterns above (or summarize).

---

## Common Pitfalls

### Pitfall A: `ignition::` prefix in SDF
**Source:** PITFALLS Pitfall 5. **Phase 5 instantiation:** `rover.sdf` must use `gz::sim::systems::DiffDrive` exactly. **Detection:** Gazebo startup log must contain `Loaded system [gz::sim::systems::DiffDrive]`. Add this grep to the README smoke checklist.

### Pitfall B: `/model/rover/cmd_vel` namespace mismatch
**Source:** PITFALLS Pitfall 6. **Phase 5 instantiation:** `<topic>cmd_vel</topic>` inside the DiffDrive plugin block. **Detection:** `gz topic -l` shows `/cmd_vel`, NOT `/model/rover/cmd_vel`.

### Pitfall C: `ros-humble-ros-gz-bridge` (Fortress) installed instead of `ros-humble-ros-gzgarden-bridge`
**Source:** PITFALLS Pitfall 7. **Phase 5 instantiation:** `install.sh --rover` lists the explicit Garden package name; apt-cache preflight check above. **Detection:** `dpkg -l | grep ros-humble-ros-gzgarden-bridge`.

### Pitfall D: Wrong directional symbol in `parameter_bridge` topic spec
**Source:** New (caught during this research). **Phase 5 instantiation:** use bidirectional `@geometry_msgs/msg/Twist@gz.msgs.Twist` — the `[` and `]` symbols REPLACE the second `@`, they do not trail the GZ type. REQUIREMENTS.md RSIM-04 wording is ambiguous; plan should follow the verified gazebosim/ros_gz README form.

### Pitfall E: bash launcher leaks child processes on Ctrl+C
**Source:** New (bash, not in PITFALLS). **Phase 5 instantiation:** `setsid` before each background launcher + `trap cleanup EXIT INT TERM` + `kill -- -$pid` on the negative-PID process group. Without this, `gz sim` survives Ctrl+C and the next launch fails with "World already loaded".

### Pitfall F: `parameter_bridge` started before `gz sim` registers `/cmd_vel`
**Source:** New (race condition observed in gz-sim ecosystem). **Phase 5 instantiation:** `sleep 3` between `gz sim` launch and `parameter_bridge` launch. Empirical value matching the same delay used in upstream gz-ros2-control tutorials.

### Pitfall G: `GZ_SIM_RESOURCE_PATH` doesn't include `sim/rover/`
**Source:** Inferred from `sim/start_sim.sh:121` pattern. **Phase 5 instantiation:** `export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR:$SIM_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"` so both `model://rover` AND `model://Walking actor` resolve.

### Pitfall H: Camera resolution mismatch with Hailo pipeline tiling
**Source:** Inferred from `hailo-apps gstreamer_app.py` defaults. **Phase 5 instantiation:** match `x500_vision/model.sdf:25-37` — 1280×720, 30 Hz, ~66° hfov. Deviating triggers tile-cropper resampling and ReID embedding drift between sim and real-hardware deployments.

### Pitfall I: Port 5600 conflict with PX4 SITL
**Source:** PITFALLS Integration Gotchas table (last row). **Phase 5 instantiation:** README documents that both sims use UDP 5600 and cannot run simultaneously on the same machine. **Do not block** — this is a documented constraint, not a Phase 5 bug to fix.

---

## Test Strategy (Phase 5 only — E2E is Phase 6)

What's actually testable for Phase 5 in CI / dev-box loops:

| Test | Method | Skip Condition |
|------|--------|----------------|
| SDF parses & loads cleanly | `gz sdf -k sim/rover/rover.sdf` + per world | `command -v gz` returns nothing |
| Bash launcher passes shellcheck | `shellcheck sim/rover/start_rover_sim.sh` | `command -v shellcheck` returns nothing |
| `install.sh --rover` lists correct apt packages | grep `install.sh` source for `ros-humble-ros-gzgarden-bridge` (parse-only, do not run apt) | always runs |
| `start_rover_sim.sh --help` exits 0 | `sim/rover/start_rover_sim.sh --help` | always runs |
| `start_rover_sim.sh` friendly-errors when ROS missing | invoke under `PATH=/usr/bin`; assert exit code 4 | always runs |
| `start_rover_sim.sh` friendly-errors when gz missing | invoke under stripped PATH; assert exit code 3 | always runs |
| `rover.sdf` references no `ignition::` prefix | grep `sim/rover/rover.sdf` for `ignition::` — must return nothing | always runs |

**Single Python smoke file:** `robot_follow/tests/test_rover_sim_smoke.py` (optional, planner's call):

```python
# All asserts skip when their respective tool is unavailable.
def test_sdf_parses_with_gz_sdf():
    if not shutil.which("gz"): pytest.skip("gz not installed")
    for sdf in Path("sim/rover").rglob("*.sdf"):
        r = subprocess.run(["gz", "sdf", "-k", str(sdf)], capture_output=True)
        assert r.returncode == 0, f"{sdf}: {r.stderr.decode()}"

def test_launcher_passes_shellcheck():
    if not shutil.which("shellcheck"): pytest.skip("shellcheck not installed")
    r = subprocess.run(["shellcheck", "sim/rover/start_rover_sim.sh"])
    assert r.returncode == 0

def test_install_sh_rover_lists_garden_bridge():
    src = Path("install.sh").read_text()
    assert "ros-humble-ros-gzgarden-bridge" in src
    assert "ros-humble-ros-gz-bridge" not in src.replace("ros-humble-ros-gzgarden-bridge", "")

def test_rover_sdf_uses_gz_not_ignition():
    src = Path("sim/rover/rover.sdf").read_text()
    assert "ignition::" not in src, "Use gz::sim::systems::DiffDrive on Garden"
    assert "gz::sim::systems::DiffDrive" in src
    assert "<topic>cmd_vel</topic>" in src  # RSIM-02
```

**Out of scope for Phase 5:** running `gz sim` headless in CI (no GPU). All E2E "the rover actually rolls" verification is **Phase 6 RINT-04**, requiring an operator gate per the Phase 3 pattern.

---

## Runtime State Inventory

> Phase 5 creates new files only; no rename / migration. Runtime State Inventory is N/A.

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | None — verified by file structure scope (`sim/rover/` is new). | None |
| Live service config | None — no daemons or persistent services touched. | None |
| OS-registered state | None — `drone-follow-boot.service` unchanged (no rover boot variant in v1.1). | None |
| Secrets / env vars | None — `GZ_SIM_RESOURCE_PATH` is exported in-script, not persisted. | None |
| Build artifacts | None — Phase 5 ships SDF + bash + README only. No Python re-install needed. | None |

---

## Environment Availability

Audited on this dev box (2026-05-20). Confirms RSIM-05's hard-fail logic works.

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| `gz` CLI (gz-tools2) | `gz sim`, `gz topic`, `gz sdf -k` | ✓ | gz-tools2 2.0.3-1~jammy | None — `install.sh --rover` installs |
| `gz-sim*-cli` (Garden = gz-sim7, Harmonic = gz-sim8) | Sim runtime | gz-sim8 only (Harmonic) | 8.11.0-1~jammy | **Garden NOT installed locally** — see § Open Questions Q3 |
| `gz-transport13` | `video_bridge.py` | ✓ | 13.5.0-1~jammy | None |
| `/opt/ros/humble/setup.bash` | ROS sourcing | (not checked — depends on box) | — | Hard-fail with friendly install command |
| `ros-humble-ros-gzgarden-bridge` (apt) | `parameter_bridge` | apt-cache visible | n/a (not installed here) | None — `install.sh --rover` installs |
| `python3` + `gz.msgs10`, `gz.transport13` pip | `video_bridge.py` | (assumed present from drone sim) | — | `sim/setup_sim.sh` covers this for drone path |
| `shellcheck` | Phase 5 smoke test | — | — | skip-on-missing |

**Notable:** This dev box has **Harmonic (gz-sim8)**, not Garden (gz-sim7). The `sim/gz_garden_env.sh` shim is designed for the case where both are installed and Garden must be forced. Phase 5 should run on Harmonic transparently if `gz-garden` apt packages aren't available — § Open Questions Q3.

---

## Validation Architecture

### Test Framework
| Property | Value |
|----------|-------|
| Framework | pytest (existing; `pyproject.toml` + `robot_follow/tests/`) |
| Config file | `pyproject.toml` § `[tool.pytest.ini_options]` |
| Quick run command | `pytest robot_follow/tests/test_rover_sim_smoke.py -x` |
| Full suite command | `pytest robot_follow/tests/ -x` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|--------------|
| RSIM-01 | rover.sdf uses gz:: prefix + has DiffDrive plugin | unit (text parse) | `pytest -k test_rover_sdf_uses_gz_not_ignition` | ❌ Wave 0 |
| RSIM-02 | rover.sdf overrides `<topic>cmd_vel</topic>` | unit (text parse) | `pytest -k test_rover_sdf_uses_gz_not_ignition` (combined) | ❌ Wave 0 |
| RSIM-03 | 3 actor worlds present + parse | integration (gz sdf -k) | `pytest -k test_sdf_parses_with_gz_sdf` | ❌ Wave 0 (skip-on-no-gz) |
| RSIM-04 | start_rover_sim.sh runs & cleans up; flags work | shellcheck + invocation | `pytest -k test_launcher_passes_shellcheck` + `--help` smoke | ❌ Wave 0 |
| RSIM-05 | install.sh --rover lists correct apt packages | unit (text parse) | `pytest -k test_install_sh_rover_lists_garden_bridge` | ❌ Wave 0 |
| RSIM-06 | video_bridge.py reused (not edited) | unit (git status / file checksum) | `pytest -k test_video_bridge_unchanged` (optional) | ❌ Wave 0 |
| RSIM-07 | README.md exists + mentions Garden EOL + `gz topic -l` | unit (text parse) | `pytest -k test_readme_documents_eol_and_smoke` | ❌ Wave 0 |

**E2E test (RINT-04)** — the "rover follows a walking actor end-to-end" deterministic test — is **Phase 6's responsibility**, not Phase 5. Phase 5 only validates files exist, parse, and are wired correctly.

### Sampling Rate
- **Per task commit:** `pytest robot_follow/tests/test_rover_sim_smoke.py -x`
- **Per wave merge:** `pytest robot_follow/tests/ -x`
- **Phase gate:** full suite green + manual `sim/rover/start_rover_sim.sh` (requires Garden or Harmonic — § Open Questions Q3) + `gz topic -l` showing `/cmd_vel` and `/camera`.

### Wave 0 Gaps
- [ ] `robot_follow/tests/test_rover_sim_smoke.py` — covers RSIM-01..05, RSIM-07
- [ ] Confirm pytest discovers files under `sim/` (it currently doesn't; tests live in `robot_follow/tests/`)
- [ ] (Optional) `shellcheck` and `gz` install in CI; otherwise tests skip — acceptable per Phase 5 ethos

---

## Security Domain

`security_enforcement` is not explicitly configured (`.planning/config.json` absent from this repo's standard search paths — defaults to enabled per agent spec). Apply ASVS to the Phase 5 scope:

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | No auth surface — sim is local-only. |
| V3 Session Management | no | No sessions. |
| V4 Access Control | no | All processes run as the invoking user. |
| V5 Input Validation | yes | `start_rover_sim.sh` parses `--world NAME`; must reject path-traversal (`../etc/passwd`). |
| V6 Cryptography | no | No keys, no secrets, no crypto. |
| V12 File Upload | no | No upload paths. |
| V14 Configuration | yes | UDP port 5600 binds locally (127.0.0.1); README documents this. No public-facing services. |

### Known Threat Patterns for bash + sim launchers

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Path traversal via `--world` arg | Tampering | Resolve via `realpath` + `[ -f "$WORLD_FILE" ]` against `$ROVER_WORLDS` allowlist; reject if outside. |
| Stale process group survives Ctrl+C | Denial of Service (sim cannot relaunch) | `setsid` + `trap cleanup EXIT INT TERM` + `kill -- -$pid` group kill. |
| Unbound `$1` shifts → arg injection | Tampering | `set -e`; explicit `case` with `*) exit 2` default; quote all args. |
| `sudo apt install` non-interactively pulling unexpected packages | Tampering (supply chain) | Pin exact apt names; fail if `apt-cache search` doesn't visibly resolve the Garden package; document the osrfoundation repo source. |
| UDP 5600 binding open to LAN | Information Disclosure | Default `--host 127.0.0.1` in `video_bridge.py`; document upgrading to LAN only when explicitly remoting. |

---

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| `libgazebo_ros_diff_drive.so` (Gazebo Classic) | `gz-sim-diff-drive-system` plugin filename | Gazebo Classic EOL 2024 | Old SO no longer exists on supported distros. |
| `ignition::gazebo::systems::DiffDrive` (Fortress) | `gz::sim::systems::DiffDrive` (Garden+) | Ignition→Gz rename 2022 | Old name = silent load failure on Garden (PITFALLS Pitfall 5). |
| `ignition.msgs.Twist` (Fortress) | `gz.msgs.Twist` (Garden+) | Same rename | Old prefix = `parameter_bridge` resolves type wrong, runtime error. |
| `ros-humble-ros-gz-bridge` (Fortress binding) | `ros-humble-ros-gzgarden-bridge` (Garden binding) | osrfoundation repo split 2023 | Wrong package = bridge binary present but linked against wrong gz-transport ABI. |
| Custom rclpy Image subscriber for camera | gz-transport13 direct subscription (`video_bridge.py`) | SUMMARY.md camera-plumbing decision 2026-05-12 | ROS bridge caps at ~15 Hz; gz-transport direct has no such cap. |
| `gazebo` CLI (Gazebo Classic) | `gz sim` CLI (Garden/Harmonic) | gz tools 2.0 release | Different CLI surface; `gz topic -l` is the Garden equivalent of `gz topic list` (Classic). |

**Deprecated / outdated:**
- Anything from a pre-2023 Gazebo tutorial using `libgazebo_ros_*` (Classic) or `ignition_*` (Fortress) — do not copy.
- `ros-humble-ros-gz-bridge` (the plain name) — NOT a real apt package; either Garden (`-gzgarden-`) or Harmonic (`-gzharmonic-`) suffix is required.

---

## Code Examples

### Verified Diff-Drive SDF (upstream gz-sim8 example, structural reference)
Source: `/usr/share/gz/gz-sim8/worlds/diff_drive.sdf:279-295` (read in this research session)
```xml
<plugin filename="gz-sim-diff-drive-system"
        name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>1.25</wheel_separation>
  <wheel_radius>0.3</wheel_radius>
  <odom_publish_frequency>1</odom_publish_frequency>
  <max_linear_velocity>0.5</max_linear_velocity>
  <max_angular_velocity>1</max_angular_velocity>
  <!-- ...no <topic> override; uses /model/vehicle_blue/cmd_vel default. -->
</plugin>
```
Phase 5 differs by: (a) smaller wheel separation/radius (rover, not van), (b) adding `<topic>cmd_vel</topic>` override (RSIM-02), (c) higher `max_*` velocity (1 m/s linear, 2 rad/s angular vs upstream's 0.5/1).

### Verified Camera Sensor SDF (drone x500_vision, identical for rover)
Source: `sim/PX4-Autopilot/Tools/simulation/gz/models/x500_vision/model.sdf:20-46` (read in this research session)
```xml
<sensor name="camera" type="camera">
  <topic>/camera</topic>
  <gz_frame_id>camera_link</gz_frame_id>
  <pose>0 0 0 0 0 0</pose>
  <camera>
    <horizontal_fov>1.152</horizontal_fov>
    <image><width>1280</width><height>720</height></image>
    <clip><near>0.1</near><far>3000</far></clip>
  </camera>
  <always_on>1</always_on>
  <update_rate>30</update_rate>
  <visualize>true</visualize>
</sensor>
```

### Verified Process Cleanup Pattern (drone sim, reused)
Source: `sim/start_sim.sh:171-179` (read in this research session)
```bash
BG_PIDS=()
cleanup() {
    for pid in "${BG_PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT
```
Phase 5 extends this with `setsid` (process-group leaders) + `kill -- -$pid` (group-kill) — necessary because `gz sim` spawns children that survive a bare `kill $pid`.

---

## Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
|---|------|-----------|--------|------------|
| R1 | Dev box has Harmonic (gz-sim8), not Garden — `install.sh --rover` would try to install Garden, conflicting | MEDIUM (verified on this box; new contributors likely vary) | MEDIUM — fresh Garden install pulls in libgz-* dups | README documents that Harmonic users should use `ros-humble-ros-gzharmonic-bridge` instead. **Or** make `--rover` detect existing gz-sim version (`dpkg -s gz-sim8-cli`) and pick the matching bridge package. See § Open Questions Q3. |
| R2 | `parameter_bridge` race — bridge starts before `gz sim` registers `/cmd_vel` | LOW (3 s sleep mitigates) | LOW — bridge exits with clear error; restart from same `start_rover_sim.sh` works | `sleep 3` before bridge launch; document the friendly-error path in README. |
| R3 | `setsid` not available on all platforms | VERY LOW (Linux has it since 2.6.x) | LOW (Phase 5 is sim-only, Linux-only) | Document Linux-only support in README; macOS dev boxes are not a v1.1 target. |
| R4 | `gz sdf -k` validator rejects valid SDF in test, blocking CI | LOW | LOW (skip-on-no-gz) | Tests skip cleanly; planner adds `pytest.skip("gz not installed")` to each gz-dependent test. |
| R5 | Garden EOL — Nov 2024; apt packages disappear from osrfoundation repo | LOW (2 yrs of grace history typical) | MEDIUM — install.sh --rover fails; users must migrate to Harmonic | README documents Harmonic migration (`s/gzgarden/gzharmonic/`); SDF `gz::` prefixes already compatible. Defer migration to post-v1.1. |
| R6 | Camera FOV mismatch between rover sim and real-hardware Hailo pipeline | LOW (1280×720 + 66° HFOV is the canonical default) | MEDIUM — controller gains would not transfer to v1.2 hardware | Pin rover camera to x500_vision sensor parameters verbatim. |
| R7 | DiffDrive max accel values too aggressive — rover spins in place / wheelies on hard yaw commands | MEDIUM (DiffDrive plugin defaults are conservative; ours bump them) | LOW (sim-only; no broken hardware) | Conservative defaults in rover.sdf (`max_linear_acceleration=2.0`, `max_angular_acceleration=4.0`); deferred to Phase 6 RINT-01 tuning. |
| R8 | Port 5600 conflict with PX4 SITL | HIGH (both sims use 5600) | LOW (one-at-a-time use, documented) | README documents the constraint; no Phase 5 code change needed. RINT-05 covers this in Phase 6. |

---

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `sleep 3` between gz sim and parameter_bridge is empirically sufficient | start_rover_sim.sh flow | Bridge may fail to attach on slow boxes — friendly retry pattern is a Phase-5 enhancement if seen in practice. |
| A2 | Rover wheel parameters (sep 0.4 m, radius 0.15 m, max_v 1.0 m/s) suit "follow a person walking" | rover.sdf template | Tuning may be needed in Phase 6 RINT-01; rover.sdf becomes the locked starting point. |
| A3 | Garden's `gz topic -l` includes `/cmd_vel` and `/camera` immediately after `gz sim -r` | start_rover_sim.sh smoke step | If gz-transport advertises topics lazily, planner adds a poll loop (`until gz topic -l \| grep -q /camera; do sleep 1; done`). |
| A4 | Three worlds (`walk_across_then_approach`, `random_walk`, `circle_around`) is the right starting set | Adapting drone worlds | If E2E coverage (Phase 6 RINT-04) demands more, add them incrementally. |
| A5 | Apt-cache visibility of `ros-humble-ros-gzgarden-bridge` is a reliable proxy for "osrfoundation repo configured" | install.sh --rover | Belt-and-suspenders: explicit failure message lists the curl/apt commands to add the repo. |
| A6 | The dev-box discovery (Harmonic installed, not Garden) generalizes — many target machines will have Harmonic not Garden | Risk R1 | If false, R1 is over-cautious; if true, Phase 5 must support both gz versions transparently. See Q3. |
| A7 | `setsid` + `kill -- -$pid` is the right cleanup pattern (vs `pkill -P $$` or similar) | start_rover_sim.sh | If killing via pgid misses `gz sim --gui` children, a fallback `pkill -f gz_sim` would be needed — but that's the risky pattern; group-kill is safer. |
| A8 | Pipeline default 1280×720 from `x500_vision.sdf` matches the live `hailo-apps gstreamer_app.py` default | rover.sdf camera stanza | If different, ReID embeddings drift between sim and real. Test by `--input udp://...:5600` echo of resolution. |

---

## Open Questions

> All have a strong recommendation. Planner may proceed with the recommendation OR open a `/gsd:discuss-phase` round.

### Q1: `setup_rover_sim.sh` as a separate script, OR fold into `install.sh --rover`?
**RSIM-05 wording:** "via `install.sh --rover` **OR** `sim/rover/setup_rover_sim.sh`."
**Recommendation:** **install.sh --rover only.** Mirror existing `--skip-*` flag pattern. A separate `setup_rover_sim.sh` duplicates apt-install logic that already lives in `install.sh`. Users who want rover-only install pass `--skip-apps --skip-hefs --skip-ui --rover`.
**Risk if wrong:** Trivial — splitting into two scripts later is a 1-commit refactor.

### Q2: Should `start_rover_sim.sh` accept `--world NAME` (mirror `start_sim.sh`)?
**Recommendation:** **Yes.** Default to `walk_across_then_approach`. Consistent with drone-sim ergonomics.
**Risk if wrong:** None — adding a flag is a 5-line bash diff.

### Q3: Garden vs Harmonic — pick one, or detect and adapt?
**Context:** This dev box has Harmonic (gz-sim8). REQUIREMENTS.md RSIM-05 hard-codes `ros-humble-ros-gzgarden-bridge` (Garden). PX4 SITL already uses Garden per `sim/gz_garden_env.sh`. SUMMARY.md notes Garden is EOL.
**Recommendation:** **Garden as locked v1.1 target** (matches PX4 SITL choice, matches RSIM-05 verbatim). README documents the Harmonic-migration `s/gzgarden/gzharmonic/` path. **Do not** add auto-detection in v1.1 — it doubles the test matrix for a single-machine sim feature.
**Risk if wrong:** Harmonic-only dev boxes (like this one) need a manual Harmonic-bridge install OR a one-time Garden-install via the osrfoundation apt repo. Both are documented; neither blocks the milestone.
**Open:** If `/gsd:discuss-phase` reveals the team has standardized on Harmonic already (post-Nov 2024 EOL), flip the locked target to Harmonic in CONTEXT.md and re-spin Phase 5.

---

## Sources

### Primary (HIGH confidence)

- `/usr/share/gz/gz-sim8/worlds/diff_drive.sdf` — verified diff_drive plugin SDF block (lines 279-295), upstream gz-sim8 example. Read in this research session.
- `sim/PX4-Autopilot/Tools/simulation/gz/models/x500_vision/model.sdf:20-46` — canonical camera sensor stanza already used by the drone sim.
- `sim/bridge/video_bridge.py:33-45` — CLI surface verified: `--topic`, `--host`, `--port`, `--bitrate`, `--fps`, `--discover`.
- `sim/start_sim.sh:80-117, 171-196` — process cleanup + reap-stale prior-art pattern.
- `install.sh:27-51` — flag-parse skeleton to extend with `--rover`.
- `setup_env.sh:39-55` — ROS sourcing already conditional; nothing for Phase 5 to add.
- `.planning/research/PITFALLS.md` Pitfalls 5–7 + Integration Gotchas table — Garden plugin naming, topic namespace, ros_gz package selection.
- `.planning/research/SUMMARY.md` lines 78–99 — Camera Plumbing Decision (video_bridge.py reuse vs ros_gz_image_bridge) locked rationale.
- `.planning/REQUIREMENTS.md` RSIM-01..07 — locked requirements.
- `apt-cache search ros-humble-ros-gzgarden` output on this dev box (2026-05-20) — verified package names.

### Secondary (MEDIUM confidence)

- [ros_gz_bridge ros2 branch README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md) — verified `@`/`[`/`]` directional symbol semantics.
- [Gazebo Garden ROS installation docs](https://gazebosim.org/docs/garden/ros_installation/) — confirms Garden EOL status; references binary packages from osrfoundation.
- [ros_gz_bridge Humble docs](https://docs.ros.org/en/humble/p/ros_gz_bridge/) — bridge parameter syntax overview (note: doc page is for Fortress-flavor Humble binding; Garden flavor diverges as captured above).
- [Gazebo discourse: Issue with ros2-humble/gz-garden ros_gz_bridge installation](https://discourse.openrobotics.org/t/issue-with-ros2-humble-gz-garden-ros-gz-bridge-installation/48945) — confirms the Humble+Garden pairing is supported but requires the osrfoundation repo.

### Tertiary (cited by reference, not re-verified this session)

- [gazebosim/ros_gz#368](https://github.com/gazebosim/ros_gz/issues/368) — ROS image bridge ~15 Hz publish ceiling on Garden (cited by SUMMARY.md camera plumbing decision; provenance LOCKED, not re-checked here).
- [gz-sim8 examples](https://github.com/gazebosim/gz-sim/tree/gz-sim8/examples/worlds) — diff_drive.sdf upstream reference.

---

## Metadata

**Confidence breakdown:**
- File structure & layout: HIGH — every path verified against existing sim/ tree.
- SDF templates: HIGH — diff_drive plugin verified on disk; camera stanza copied from working x500_vision; only the wheel/chassis dimensions are recommended-not-verified.
- ros_gz_bridge syntax: HIGH — verified against current gazebosim/ros_gz ros2 README. Caught the REQUIREMENTS.md typo.
- Apt package names: HIGH — `apt-cache search ros-humble-ros-gzgarden` confirms on this dev box.
- Bash cleanup pattern: HIGH — prior art at `sim/start_sim.sh:171-196` already in repo; `setsid` extension is widely-attested Linux idiom.
- Pitfalls: HIGH — copied from already-locked PITFALLS.md research.
- Test strategy: MEDIUM — `gz sdf -k` validator behavior tested locally (`--help` works); SDF lint output format not exhaustively checked.
- Risk register: MEDIUM — R1 (Harmonic vs Garden on this box) is a genuine ambiguity flagged in Open Questions Q3.

**Research date:** 2026-05-20
**Valid until:** 2026-06-20 (30 days — Gazebo Garden EOL means apt packages could disappear; recheck before Phase 5 plan execution if a month elapses)
