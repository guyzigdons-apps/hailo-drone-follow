#!/bin/bash
# Setup script for PX4 SITL simulation
# Initialises the PX4-Autopilot git submodule (v1.14.0) and builds the SITL firmware.
#
# Usage: sim/setup_sim.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
PX4_DIR="$SCRIPT_DIR/PX4-Autopilot"
PX4_VERSION="v1.14.0"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "========================================="
echo "  PX4 SITL Simulation Setup"
echo "========================================="
echo ""

# Step 1: Init submodule and checkout correct version
echo -e "${GREEN}[1/3] Initialising PX4-Autopilot submodule ($PX4_VERSION)...${NC}"
cd "$PROJECT_ROOT"
git submodule update --init sim/PX4-Autopilot

if [ ! -d "$PX4_DIR" ]; then
    echo -e "${RED}Error: PX4-Autopilot not found at $PX4_DIR after submodule init.${NC}"
    exit 1
fi

# Ensure we're on the correct version
cd "$PX4_DIR"
CURRENT=$(git describe --tags 2>/dev/null || echo "unknown")
if [ "$CURRENT" != "$PX4_VERSION" ]; then
    echo -e "${YELLOW}  Checking out $PX4_VERSION (currently on $CURRENT)...${NC}"
    git checkout -f "$PX4_VERSION"
fi

# Init PX4's own recursive submodules
echo -e "  Initialising PX4 internal submodules (this may take a few minutes)..."
git submodule update --init --recursive

echo -e "  PX4-Autopilot at: $PX4_DIR"
echo -e "  Version: $(git describe --tags 2>/dev/null || echo 'unknown')"

# Step 2: Apply camera patch to x500_vision model
echo ""
echo -e "${GREEN}[2/3] Applying camera patch to x500_vision model...${NC}"
PATCH_FILE="$SCRIPT_DIR/patches/x500_vision_camera.patch"
if [ -f "$PATCH_FILE" ]; then
    # Apply only if not already applied
    if git apply --check "$PATCH_FILE" 2>/dev/null; then
        git apply "$PATCH_FILE"
        echo -e "  Camera sensor added to x500_vision model."
    else
        echo -e "${YELLOW}  Patch already applied or conflicts — skipping.${NC}"
    fi
else
    echo -e "${RED}  Error: Patch file not found at $PATCH_FILE${NC}"
    exit 1
fi

# Step 3: Build PX4 SITL
echo ""
echo -e "${GREEN}[3/3] Building PX4 SITL firmware (this may take 10-20 minutes on first build)...${NC}"
make px4_sitl_default

echo ""
echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  1. Start the simulator:  sim/start_sim.sh --bridge"
echo "  2. In another terminal:  source setup_env.sh"
echo "     drone-follow --input udp://0.0.0.0:5600 --world 2_person_world --takeoff-landing --ui"
