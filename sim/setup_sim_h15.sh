#!/bin/bash
# Setup PX4 SITL for H15 drone-follow testing.
# Uses an external PX4 checkout (symlinked at sim/PX4-Autopilot) on main branch
# with Gazebo Harmonic. No sim camera needed — H15 provides real detections.
#
# Usage: sim/setup_sim_h15.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$SCRIPT_DIR/PX4-Autopilot"
PX4_BRANCH="main"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "========================================="
echo "  PX4 SITL Setup (H15 drone-follow)"
echo "========================================="
echo ""

# Step 1: Resolve PX4 directory (symlink or submodule)
echo -e "${GREEN}[1/3] Preparing PX4-Autopilot ($PX4_BRANCH)...${NC}"

if [ -L "$PX4_DIR" ]; then
    PX4_REAL="$(readlink -f "$PX4_DIR")"
    echo -e "  Using symlinked PX4: $PX4_REAL"
elif [ -d "$PX4_DIR/.git" ]; then
    PX4_REAL="$PX4_DIR"
    echo -e "  Using PX4 checkout at: $PX4_REAL"
else
    echo -e "${RED}Error: PX4-Autopilot not found at $PX4_DIR${NC}"
    echo -e "  Create a symlink:  ln -s /path/to/PX4-Autopilot $PX4_DIR"
    exit 1
fi

cd "$PX4_REAL"

# Checkout correct branch
CURRENT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
if [ "$CURRENT_BRANCH" != "$PX4_BRANCH" ]; then
    echo -e "${YELLOW}  Switching from $CURRENT_BRANCH to $PX4_BRANCH...${NC}"
    git checkout -f "$PX4_BRANCH"
fi
git pull --ff-only 2>/dev/null || true

# Init PX4's own submodules
echo -e "  Initialising PX4 internal submodules..."
git submodule update --init --recursive

echo -e "  Version: $(git describe --tags 2>/dev/null || git rev-parse --short HEAD)"

# Step 2: Clean previous build
echo ""
echo -e "${GREEN}[2/3] Cleaning previous build...${NC}"
rm -rf build/px4_sitl_default
echo -e "  Done"

# Step 3: Build PX4 SITL
echo ""
echo -e "${GREEN}[3/3] Building PX4 SITL firmware...${NC}"
make px4_sitl_default

echo ""
echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  1. Start the simulator:  sim/start_sim_h15.sh"
echo "  2. On H15:  PYTHONPATH=/home/root python3 -m drone_follow.drone_follow_h15 \\"
echo "                --connection udp://10.0.0.2:14540 --takeoff-landing"
