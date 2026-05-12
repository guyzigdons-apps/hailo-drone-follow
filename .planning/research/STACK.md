# Stack Research

**Domain:** ROS 2 cmd_vel rover adapter + Gazebo Garden rover sim (additions to hailo-drone-follow v1.1)
**Researched:** 2026-05-12
**Confidence:** HIGH (all packages verified against apt cache on the dev machine with osrfoundation repo configured)

> This file covers ONLY the stack additions for the v1.1 rover milestone. The validated drone stack
> (GStreamer, MAVSDK, Hailo, Vite/Node) is out of scope here.

---

## 1. ROS 2 Distribution: Humble (only choice for Ubuntu 22.04)

**Recommendation: ROS 2 Humble Hawksbill. No alternatives.**

| Distro | Ubuntu 22.04 binaries | EOL | Notes |
|--------|-----------------------|-----|-------|
| Humble | YES (tier-1 platform) | May 2027 | Only LTS that targets Jammy as its primary platform |
| Iron | YES (but limited) | Dec 2024 (already EOL) | Dead — do not use |
| Jazzy | NO (targets Noble 24.04) | May 2029 | Wrong Ubuntu; no Jammy debs |

Source: https://endoflife.date/ros-2 — verified 2026-05-12.

Humble is the correct choice and the only reasonable one. It ships with Python 3.10, which matches the
existing `venv_hailo_apps` (confirmed: `include-system-site-packages = true`, Python 3.10.13).

---

## 2. Apt Packages — Full Install List

Two apt repositories are required. The osrfoundation repo is already configured on the dev machine
(`/etc/apt/sources.list.d/gazebo-stable.list`). The ROS 2 repo (`packages.ros.org`) must be added
separately for `ros-humble-*` packages.

### ROS 2 Humble core (from packages.ros.org)

```bash
sudo apt install ros-humble-ros-base          # rclpy, rcl, rmw, ament — minimal non-GUI install
sudo apt install ros-humble-geometry-msgs     # geometry_msgs/Twist for /cmd_vel
```

`ros-humble-ros-base` pulls in `ros-humble-rclpy`, `ros-humble-std-msgs`, and the Python client
library stack. Prefer `ros-base` over `ros-desktop` — we have no GUI tools requirement.

### ros_gz_bridge for Gazebo Garden (from packages.osrfoundation.org — already configured)

```bash
sudo apt install ros-humble-ros-gzgarden-bridge
```

- Package: `ros-humble-ros-gzgarden-bridge`
- Version in apt: `0.244.11-1002jammy` (verified in apt cache)
- Source repo: `http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main`
- Conflicts with: `ros-humble-ros-gz-bridge` (the Fortress bridge — do not install both)

The metapackage `ros-humble-ros-gzgarden` (which also pulls in `*-sim` and `*-sim-demos`) is not
needed — the rover sim only needs the bridge.

### Gazebo Garden (from packages.osrfoundation.org)

Already a prerequisite per CLAUDE.md. The DiffDrive plugin ships inside `libgz-sim7-plugins`,
which is a dependency of `gz-sim7-cli` (pulled in by `gz-garden` metapackage). No extra plugin
packages needed.

```bash
sudo apt install gz-garden                   # metapackage: gz-sim7-cli + python3-gz-sim7 + all libs
```

`gz-garden` version `1.0.0-1~jammy` — verified in apt cache. The `libgz-sim7-plugins` package
(`7.9.0-1~jammy`) includes `gz-sim-diff-drive-system` (DiffDrive), `gz-sim-sensors-system`,
`gz-sim-user-commands-system`, and others needed for the rover SDF.

### Summary: complete new-package install

```bash
# Step 1: Add ROS 2 Humble apt repo (if not present)
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list
# osrfoundation repo is assumed present (needed for gz-garden anyway)

# Step 2: Install
sudo apt update
sudo apt install \
  ros-humble-ros-base \
  ros-humble-geometry-msgs \
  ros-humble-ros-gzgarden-bridge \
  gz-garden
```

---

## 3. ros_gz_bridge Package Name for Gazebo Garden

The naming convention follows `ros-{distro}-ros-gz{gazebo_codename}[-component]`:

| Package | Purpose |
|---------|---------|
| `ros-humble-ros-gz` | Fortress bridge (default Humble pairing — do NOT install) |
| `ros-humble-ros-gzgarden-bridge` | Garden bridge — use this |
| `ros-humble-ros-gzharmonic-bridge` | Harmonic bridge — not needed here |

The Garden bridge conflicts with the Fortress bridge. Installing `ros-humble-ros-gz` (Fortress)
would break the Garden integration and conflict with the existing `gz-sim7` install.

For the rover sim, the bridge is invoked as a standalone process (not a launch file node) to keep
the rover sim self-contained and consistent with how the existing drone sim works:

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
```

Note: Gazebo Garden uses the `gz.msgs.Twist` prefix (not `ignition.msgs.Twist` which was the
pre-Garden naming). Using the old prefix silently causes the bridge to fail.
Source: https://github.com/gazebosim/ros_gz/issues/318

---

## 4. Venv + ROS 2 Integration

**Situation:** The existing venv (`hailo-apps/venv_hailo_apps`) was created with
`--system-site-packages` and uses Python 3.10.13, exactly matching what `ros-humble-rclpy` is
compiled for (pybind11 `.so` targeting `cpython-310`).

**How it works:**
1. `source setup_env.sh` activates the venv (Python 3.10.13 becomes `python`).
2. `source /opt/ros/humble/setup.bash` appends `/opt/ros/humble/lib/python3.10/site-packages`
   to `PYTHONPATH` and sets `ROS_DISTRO`, `AMENT_PREFIX_PATH`, `ROS_PACKAGE_PATH`.
3. Because the venv was built with `include-system-site-packages = true`, rclpy's pybind11
   extension (`.so`) is visible without any PYTHONPATH manipulation.
4. `import rclpy` works inside the venv after both sources are applied.

**Ordering is critical:** source the venv FIRST, then setup.bash. Reversing the order causes
setup.bash to override `python` to the system interpreter, bypassing the venv entirely.

**`setup_env.sh` must be extended** to source `/opt/ros/humble/setup.bash` after activating
the venv — or callers must do it themselves. The cleanest approach: add an optional check in
`setup_env.sh`:

```bash
# At the end of setup_env.sh, after venv activation:
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi
```

This is a no-op on machines without ROS installed, and activates on machines that have it.

**Confidence:** MEDIUM — the Python version match (3.10.13 = 3.10.13) is the key guarantee.
The general pattern is confirmed in https://github.com/ros2/ros2/issues/1469 and community usage.

---

## 5. Gazebo Garden DiffDrive SDF

The `gz-sim-diff-drive-system` plugin ships inside `libgz-sim7-plugins` (part of `gz-garden`
install chain). No separate apt package is needed.

Correct SDF snippet for a differential-drive rover:

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_radius>0.1</wheel_radius>
  <odom_publish_frequency>10</odom_publish_frequency>
  <topic>cmd_vel</topic>          <!-- Gazebo-internal topic, NOT the ROS topic -->
</plugin>
```

The `<topic>` field here is the *Gazebo Transport* topic. The `ros_gz_bridge` maps this to the
ROS `/cmd_vel` topic. The full topic path Gazebo sees is `/model/{model_name}/cmd_vel` by default
when the model name is included; configuring `<topic>cmd_vel</topic>` makes it listen on `/cmd_vel`
directly in Gazebo Transport, simplifying the bridge configuration.

Source: https://gazebosim.org/docs/garden/moving_robot/

---

## 6. Distro Pinning Strategy

**Pin to Humble.** Do not accept arbitrary `>= Humble`.

Rationale:
- This project targets Ubuntu 22.04 (both dev x86_64 and RPi 5 Bookworm arm64). Jazzy has no Jammy
  binaries. Rolling has no stability guarantees.
- The `ros-humble-ros-gzgarden-bridge` is a specific Humble+Garden binary from osrfoundation;
  there is no distro-agnostic equivalent.
- `install.sh` should `set -e` on a `[[ "$ROS_DISTRO" == "humble" ]]` check after sourcing
  setup.bash, or emit a hard error if `/opt/ros/humble` is absent.

```bash
# Suggested guard in sim/setup_rover.sh or install.sh --rover:
if [[ ! -d /opt/ros/humble ]]; then
  echo "ERROR: ROS 2 Humble not found at /opt/ros/humble. Install it first." >&2
  exit 1
fi
```

---

## 7. Fresh Ubuntu 22.04 Rover Sim Setup (step-by-step)

```bash
# 1. ROS 2 Humble apt repo
sudo apt install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc \
  | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

# 2. osrfoundation Gazebo repo (needed for gz-garden + ros-gzgarden packages)
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
  | sudo tee /etc/apt/sources.list.d/gazebo-stable.list

# 3. Install packages
sudo apt update
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-geometry-msgs \
  ros-humble-ros-gzgarden-bridge \
  gz-garden \
  python3-gz-transport13 \
  python3-gz-msgs10

# 4. Install hailo-drone-follow (existing flow — creates venv with system-site-packages)
./install.sh

# 5. Activate env — venv first, then ROS
source setup_env.sh
source /opt/ros/humble/setup.bash

# 6. Verify
python -c "import rclpy; print('rclpy OK')"
python -c "import geometry_msgs; print('geometry_msgs OK')"
gz sim --version
```

`setup_env.sh` should be updated to auto-source ROS 2 setup.bash (see §4 above).

---

## 8. What NOT to Install

| Package | Why skip it |
|---------|------------|
| `ros-humble-desktop` | Pulls in RViz, Gazebo Classic, Qt dev tools — 1.5 GB; we don't need GUI ROS tools |
| `ros-humble-nav2-*` | Full navigation stack; we publish raw `/cmd_vel`, not Nav2 action goals |
| `ros-humble-move-base-msgs` | Nav stack messages — not used |
| `ros-humble-gazebo-ros-pkgs` | This is the Gazebo *Classic* bridge — wrong simulator |
| `ros-humble-ros-gz` | Fortress bridge; conflicts with `ros-humble-ros-gzgarden-bridge` |
| `ros-humble-ros-gz-bridge` | Same conflict — Fortress bridge package |
| `ros2-control`, `ros2-controllers` | Hardware abstraction layer — not needed for sim-only cmd_vel |

`nav_msgs` is pulled in as a transitive dependency of `ros-humble-ros-gzgarden-bridge` (it's in
its `Depends:` line) but that's fine — it's a lightweight message package and we don't import it.

---

## Alternatives Considered

| Recommended | Alternative | Why Not |
|-------------|-------------|---------|
| ROS 2 Humble | ROS 2 Jazzy | No Ubuntu 22.04 Jammy binaries; requires upgrade to Noble 24.04 |
| `ros-humble-ros-gzgarden-bridge` | Build ros_gz from source with GZ_VERSION=garden | Source build requires colcon, rosdep, 10+ minutes; binary is identical |
| `ros-humble-ros-gzgarden-bridge` | `ros-humble-ros-gz` (Fortress) | Conflicts; Fortress uses `gz-sim6`, incompatible with existing `gz-sim7` install |
| Pure ROS 2 bridge | PX4 Rover MAVSDK | PX4 Rover support in MAVSDK is immature; ROS 2 cmd_vel is the standard for ground robots |

---

## Version Compatibility Matrix

| Package | Version | Compatible With |
|---------|---------|----------------|
| ros-humble-ros-gzgarden-bridge | 0.244.11-1002jammy | gz-transport12 (≥12.2.0), gz-msgs9 (≥9.4.0) |
| gz-garden (gz-sim7) | 7.9.0 | gz-transport12; NOT compatible with gz-transport13 (Harmonic) |
| python3-gz-transport13 | system | Already installed per CLAUDE.md; NOT the transport used by Garden plugins |
| rclpy (ros-humble) | 3.3.x | Python 3.10 only (pybind11 .so is CPython 3.10-specific) |
| venv_hailo_apps | Python 3.10.13 | Matches rclpy binding exactly |

Note: `python3-gz-transport13` is already on the system for the PX4 SITL video bridge. This is
Gazebo *Harmonic*'s transport layer, used by the Python bridge script. It does NOT conflict with the
C++ `libgz-transport12` used by the Garden sim plugins and ros_gz_bridge binary.

---

## Sources

- https://endoflife.date/ros-2 — Humble EOL May 2027, Iron EOL Dec 2024, Jazzy targets Noble
- https://gazebosim.org/docs/latest/ros_installation/ — ROS/Gazebo compatibility matrix
- `apt-cache show ros-humble-ros-gzgarden-bridge` on this machine — package version 0.244.11-1002jammy confirmed
- `apt-cache show gz-garden` on this machine — version 1.0.0-1~jammy, deps confirmed
- `apt-cache show libgz-sim7-plugins` — version 7.9.0-1~jammy, DiffDrive plugin included
- https://github.com/gazebosim/ros_gz/issues/318 — gz.msgs.Twist (not ignition.msgs.Twist) for Garden
- https://gazebosim.org/docs/garden/moving_robot/ — DiffDrive SDF plugin config
- https://github.com/ros2/ros2/issues/1469 — rclpy venv + Python version must match (3.10)
- `/home/guyz/code/guyz/hailo-drone-follow/hailo-apps/venv_hailo_apps/pyvenv.cfg` — confirmed `include-system-site-packages = true`, Python 3.10.13

---
*Stack research for: ROS 2 cmd_vel rover adapter + Gazebo Garden rover sim (hailo-drone-follow v1.1)*
*Researched: 2026-05-12*
