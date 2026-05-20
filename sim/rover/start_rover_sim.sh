#!/usr/bin/env bash
# sim/rover/start_rover_sim.sh — one-command rover sim launcher.
#
# Starts:
#   1. gz sim    -- Gazebo Garden with the rover world (rover model included)
#   2. ros2 run ros_gz_bridge parameter_bridge -- /cmd_vel ROS <-> GZ bridge
#   3. sim/bridge/video_bridge.py -- /camera gz topic -> UDP 5600 RTP
#
# Usage: sim/rover/start_rover_sim.sh [--world NAME] [--no-reap-stale] [-h]
#
# Exit codes:
#   2 — bad CLI args / unknown world
#   3 — `gz` CLI missing (install Gazebo Garden via sudo ./install.sh --rover)
#   4 — /opt/ros/humble/setup.bash missing (install ROS 2 Humble + bridge)
#   5 — stale processes refused (rerun without --no-reap-stale to auto-reap)
#
# Cleanup: SIGINT (Ctrl+C) traps and group-kills all 3 children via setsid +
# `kill -- -$pid`. EXIT and TERM also trapped (defensive).
#
# References:
#   - RESEARCH.md § start_rover_sim.sh Flow
#   - PITFALLS.md Pitfalls 5, 6, 7 (Gazebo Garden plugin / topic / EOL)
#   - sim/start_sim.sh:80-117, 171-196 (prior-art patterns)
#   - sim/rover/README.md § Smoke test (downstream verification steps)

set -e

# Path computations use bash parameter expansion (NOT external `dirname`) so
# that a stripped-PATH invocation (e.g. PATH=/tmp for the preflight-exit-3
# verify gate) still reaches the gz/ROS preflights below.  ${var%/*} is the
# bash equivalent of dirname.
_SCRIPT_PATH="${BASH_SOURCE[0]}"
# Resolve relative paths so SCRIPT_DIR is always absolute.
if [[ "$_SCRIPT_PATH" != /* ]]; then
  _SCRIPT_PATH="$PWD/$_SCRIPT_PATH"
fi
SCRIPT_DIR="${_SCRIPT_PATH%/*}"
SIM_DIR="${SCRIPT_DIR%/*}"                          # sim/
PROJECT_ROOT="${SIM_DIR%/*}"
ROVER_WORLDS="$SCRIPT_DIR/worlds"
BRIDGE_SCRIPT="$SIM_DIR/bridge/video_bridge.py"

# PROJECT_ROOT is computed for downstream / debug invocations.
export PROJECT_ROOT

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

# --- Parse args ---
WORLD="walk_across_then_approach"
REAP_STALE=true
while [[ $# -gt 0 ]]; do
  case $1 in
    --world)
      if [ -z "${2:-}" ] || [[ "$2" == --* ]]; then
        echo -e "${RED}Error: --world requires a name${NC}" >&2
        exit 2
      fi
      WORLD="$2"; shift 2 ;;
    --no-reap-stale) REAP_STALE=false; shift ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo -e "${RED}Unknown arg: $1${NC}" >&2; exit 2 ;;
  esac
done

# V5/T-05-05-01: world-name allowlist (alphanumeric + underscore only).
if ! [[ "$WORLD" =~ ^[a-z0-9_]+$ ]]; then
  echo -e "${RED}Invalid --world '$WORLD' (lowercase alphanumeric + underscore only)${NC}" >&2
  exit 2
fi

WORLD_FILE="$ROVER_WORLDS/$WORLD.sdf"
if [ ! -f "$WORLD_FILE" ]; then
  echo -e "${RED}World not found: $WORLD_FILE${NC}" >&2
  echo "Available worlds in $ROVER_WORLDS:" >&2
  for f in "$ROVER_WORLDS"/*.sdf; do
    [ -f "$f" ] && echo "  $(basename "$f" .sdf)" >&2
  done
  exit 2
fi

# --- Preflight ---
if ! command -v gz >/dev/null 2>&1; then
  echo -e "${RED}'gz' CLI missing. Install Gazebo Garden:${NC}" >&2
  echo "  sudo ./install.sh --rover  (handles apt repo + gz-garden)" >&2
  echo "  See sim/rover/README.md if your box runs Harmonic." >&2
  exit 3
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo -e "${RED}/opt/ros/humble/setup.bash missing. Install ROS 2 Humble:${NC}" >&2
  echo "  sudo ./install.sh --rover  (installs ros-humble-ros-base etc.)" >&2
  exit 4
fi
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

# --- Preflight: reap stale processes from a prior run ---
# Mirrors sim/start_sim.sh:80-117.  Patterns scoped to repo so unrelated
# gz / ros2 instances on the machine aren't touched.
preflight_reap_stale() {
  local patterns=(
    "gz sim.*$ROVER_WORLDS"
    "ros_gz_bridge parameter_bridge"
    "$BRIDGE_SCRIPT"
  )
  local pids
  pids=$(for pat in "${patterns[@]}"; do
    pgrep -f -- "$pat" 2>/dev/null || true
  done | sort -u | grep -vE "^($$|$PPID)$" || true)

  [ -z "$pids" ] && return 0

  echo -e "${YELLOW}Stale processes detected from a prior run:${NC}" >&2
  while IFS= read -r pid; do
    [ -z "$pid" ] && continue
    ps -p "$pid" -o pid=,args= 2>/dev/null || echo "  pid=$pid (gone)" >&2
  done <<<"$pids" >&2

  if ! $REAP_STALE; then
    echo -e "${RED}Refusing to start while old instance is alive.${NC}" >&2
    echo "Kill them manually, or rerun without --no-reap-stale." >&2
    exit 5
  fi

  while IFS= read -r pid; do
    [ -z "$pid" ] && continue
    kill "$pid" 2>/dev/null || true
  done <<<"$pids"
  sleep 1
  while IFS= read -r pid; do
    [ -z "$pid" ] && continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -9 "$pid" 2>/dev/null || true
    fi
  done <<<"$pids"
}
preflight_reap_stale

# --- Resource paths so model:// includes resolve ---
# sim/rover/ for model://rover (Plan 05-01); sim/worlds/ for the shared
# Walking actor mesh (existing drone-sim asset, reused per RSIM-03).
export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR:$SIM_DIR/worlds:${GZ_SIM_RESOURCE_PATH:-}"

# --- Track children for cleanup ---
BG_PIDS=()
cleanup() {
  echo -e "${YELLOW}Shutting down rover sim...${NC}" >&2
  for pid in "${BG_PIDS[@]}"; do
    # Kill the whole process group (negative pid). gz sim + ros2 spawn
    # children; without setsid+`kill -- -PID` they survive Ctrl+C and the
    # next launch fails with "World already loaded" (PITFALLS Pitfall E).
    kill -- "-$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  done
  for pid in "${BG_PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo -e "${GREEN}Starting rover sim ($WORLD)${NC}"
echo "  World:   $WORLD_FILE"
echo "  cmd_vel: ROS /cmd_vel <--> GZ /cmd_vel (bidirectional)"
echo "  camera:  GZ /camera --> udp://127.0.0.1:5600 (H.264 RTP)"
echo

# --- Launch Gazebo Garden ---
# setsid so children form their own process group (cleanup uses `kill -- -PID`).
setsid gz sim -r "$WORLD_FILE" &
BG_PIDS+=($!)
sleep 3   # give gz sim time to register topics before parameter_bridge attaches

# --- Launch ros_gz_bridge ---
# Bidirectional /cmd_vel bridge.  Garden uses the gz.msgs.* prefix (NOT the
# pre-Garden ign prefix).  CORRECTED form: bidirectional @ separator, no
# trailing ]; REQUIREMENTS.md RSIM-04 wording is a typo (RESEARCH § ros_gz_bridge
# syntax).
setsid ros2 run ros_gz_bridge parameter_bridge \
    /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist &
BG_PIDS+=($!)

# --- Launch video bridge ---
# Reuses sim/bridge/video_bridge.py VERBATIM (RSIM-06).  PROTOCOL_BUFFERS env
# var avoids a protobuf C++/python ABI crash (video_bridge.py:14-16).
setsid env PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python \
    python3 "$BRIDGE_SCRIPT" --topic /camera --host 127.0.0.1 --port 5600 &
BG_PIDS+=($!)

echo -e "${GREEN}Smoke-test from another terminal:${NC}"
echo "  gz topic -l                                           # /cmd_vel, /camera should appear"
echo "  ros2 topic pub --once /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'"
echo "  ffplay -fflags nobuffer udp://0.0.0.0:5600            # camera feed"
echo
echo "Press Ctrl+C to stop."
wait
