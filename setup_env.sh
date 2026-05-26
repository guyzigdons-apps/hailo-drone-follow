#!/usr/bin/env bash
# drone-follow env activator. Source from anywhere:
#   source <repo>/setup_env.sh
#
# Resolves the hailo-apps submodule at ./hailo-apps, exports HAILO_APPS_PATH,
# delegates to the submodule's setup_env.sh (kernel check, venv activation,
# PYTHONPATH, /usr/local/hailo/resources/.env load), and restores cwd.

# Resolve this script's directory (bash/zsh).
if [ -n "${BASH_SOURCE:-}" ]; then
    _DF_SCRIPT_PATH="${BASH_SOURCE[0]}"
elif [ -n "${ZSH_VERSION:-}" ]; then
    # shellcheck disable=SC2154
    _DF_SCRIPT_PATH="${(%):-%x}"
else
    _DF_SCRIPT_PATH="$0"
fi
SCRIPT_DIR="$(cd "$(dirname "$_DF_SCRIPT_PATH")" && pwd)"
APPS_DIR="$SCRIPT_DIR/hailo-apps"
unset _DF_SCRIPT_PATH

if [[ ! -d "$APPS_DIR/venv_hailo_apps" ]]; then
    echo "ERROR: $APPS_DIR/venv_hailo_apps not found. Run ./install.sh first." >&2
    return 1 2>/dev/null || exit 1
fi

export HAILO_APPS_PATH="$APPS_DIR"

# hailo-apps' setup_env.sh uses $(pwd) as PROJECT_ROOT for PYTHONPATH and to
# locate venv_hailo_apps, so cd into the submodule before sourcing and back out
# after.
_DF_ORIG_PWD="$(pwd)"
cd "$APPS_DIR" || { echo "ERROR: cannot cd to $APPS_DIR" >&2; return 1 2>/dev/null || exit 1; }
# shellcheck disable=SC1091
source "$APPS_DIR/setup_env.sh"
cd "$_DF_ORIG_PWD" || true
unset _DF_ORIG_PWD

# Conditionally source ROS 2 Humble after the venv is active.
#
# Rationale (CONTEXT 03-abstraction § setup_env.sh ROS sourcing,
# PITFALLS.md Pitfall 2):
# - Venv-first sourcing keeps the hailo-apps venv's site-packages ahead
#   of /opt/ros/humble/lib/python3.10/site-packages on sys.path, so any
#   pure-Python shim in the venv that shadows an rclpy submodule does NOT
#   hide rclpy's _rclpy_pybind11 .so.
# - Idempotent: drone-only users on a non-ROS box see no change; rover
#   users get ROS for free; drone users on a ROS-equipped box source ROS
#   but the drone path is unaffected (ROS env vars don't conflict with
#   the venv's Python or MAVSDK).
# - Single file check (no `command -v ros2` — chicken-and-egg before sourcing).
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
