#!/bin/bash
# Start PX4 SITL + Gazebo Garden with the x500_vision drone (includes camera).
#
# Usage: sim/start_sim.sh [--bridge] [--world NAME]
#
# Options:
#   --bridge        Also start the video bridge (Gazebo camera -> UDP 5600)
#   --world NAME    Load a custom world from drone_follow/sdf_examples/ (e.g. 2_person_world)
#                   Defaults to PX4's built-in default world.
#
# Environment variables:
#   HEADLESS=1      Run Gazebo without GUI (useful for CI / remote machines)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PX4_DIR="$SCRIPT_DIR/PX4-Autopilot"
SDF_EXAMPLES="$PROJECT_ROOT/drone_follow/sdf_examples"
BRIDGE_SCRIPT="$SCRIPT_DIR/bridge/video_bridge.py"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Parse flags
START_BRIDGE=false
WORLD=""
while [[ $# -gt 0 ]]; do
    case $1 in
        --bridge) START_BRIDGE=true; shift ;;
        --world)
            if [ -z "$2" ] || [[ "$2" == --* ]]; then
                echo -e "${RED}Error: --world requires a world name${NC}"
                echo "Available worlds:"
                ls "$SDF_EXAMPLES"/*.sdf 2>/dev/null | xargs -I{} basename {} .sdf | sed 's/^/  /'
                exit 1
            fi
            WORLD="$2"; shift 2 ;;
        *)
            echo -e "${RED}Unknown argument: $1${NC}"
            echo "Usage: $0 [--bridge] [--world NAME]"
            exit 1 ;;
    esac
done

# Preflight checks
if [ ! -d "$PX4_DIR/build/px4_sitl_default" ]; then
    echo -e "${RED}Error: PX4 SITL not built. Run sim/setup_sim.sh first.${NC}"
    exit 1
fi

# Set GZ_SIM_RESOURCE_PATH so Gazebo can find custom models (e.g. "Walking actor")
export GZ_SIM_RESOURCE_PATH="${SDF_EXAMPLES}:${GZ_SIM_RESOURCE_PATH:-}"

# Install custom world into PX4's worlds directory (symlink)
# PX4's gz_env.sh hardcodes PX4_GZ_WORLDS to its own worlds dir,
# so we symlink our SDF files there instead of overriding the env var.
PX4_WORLDS_DIR="$PX4_DIR/Tools/simulation/gz/worlds"
if [ -n "$WORLD" ]; then
    WORLD_FILE="$SDF_EXAMPLES/${WORLD}.sdf"
    if [ ! -f "$WORLD_FILE" ]; then
        echo -e "${RED}Error: World not found: $WORLD_FILE${NC}"
        echo "Available worlds:"
        ls "$SDF_EXAMPLES"/*.sdf 2>/dev/null | xargs -I{} basename {} .sdf | sed 's/^/  /'
        exit 1
    fi
    ln -sf "$WORLD_FILE" "$PX4_WORLDS_DIR/${WORLD}.sdf"
    export PX4_GZ_WORLD="$WORLD"
fi

echo -e "${GREEN}Starting PX4 SITL + Gazebo (x500_vision with camera)...${NC}"
echo "  PX4:        $PX4_DIR"
echo "  SDF models: $SDF_EXAMPLES"
if [ -n "$WORLD" ]; then
    echo "  World:      $WORLD"
else
    echo "  World:      default"
fi
echo "  MAVLink:    udp://localhost:14540"

if $START_BRIDGE; then
    echo "  Bridge:     video_bridge.py -> udp://127.0.0.1:5600"
else
    echo "  Camera:     use --bridge or run sim/bridge/video_bridge.py separately"
fi
echo ""

# Start video bridge in background if requested
BRIDGE_PID=""
if $START_BRIDGE; then
    echo -e "${GREEN}Starting video bridge...${NC}"
    PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python python3 "$BRIDGE_SCRIPT" &
    BRIDGE_PID=$!
fi

# Ensure bridge is killed when this script exits
cleanup() {
    if [ -n "$BRIDGE_PID" ]; then
        kill "$BRIDGE_PID" 2>/dev/null || true
        wait "$BRIDGE_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

cd "$PX4_DIR"
make px4_sitl gz_x500_vision
