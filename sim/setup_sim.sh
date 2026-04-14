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

# Step 0: Check / install Gazebo dependencies
# PX4 v1.14's gz_bridge requires gz-transport12.  Gazebo Harmonic ships
# gz-transport13, so the v12 compat package must be installed separately.
# We also need gz-sim7 (Gazebo Garden) if Harmonic (gz-sim8) is the default.
echo -e "${GREEN}[0/4] Checking Gazebo dependencies...${NC}"

MISSING_DEPS=()
for pkg in libgz-transport12-dev; do
    if ! dpkg -s "$pkg" &>/dev/null; then
        MISSING_DEPS+=("$pkg")
    fi
done

# If Gazebo Harmonic (gz-sim8) is installed but Garden (gz-sim7) is not,
# we need Garden so that start_sim.sh can force the correct transport version.
if dpkg -s gz-sim8-cli &>/dev/null && ! dpkg -s gz-sim7-cli &>/dev/null; then
    MISSING_DEPS+=("gz-garden")
fi

if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    echo -e "${YELLOW}  Installing missing packages: ${MISSING_DEPS[*]}${NC}"
    sudo apt-get update -qq
    sudo apt-get install -y "${MISSING_DEPS[@]}"
else
    echo -e "  All Gazebo dependencies present."
fi

# Step 1: Init submodule and checkout correct version
echo -e "${GREEN}[1/4] Initialising PX4-Autopilot submodule ($PX4_VERSION)...${NC}"
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
echo -e "${GREEN}[2/4] Applying camera patch to x500_vision model...${NC}"
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

# Step 3: Verify gz_bridge will be built
echo ""
echo -e "${GREEN}[3/4] Verifying gz_bridge cmake configuration...${NC}"

# If a previous build exists but gz-transport was not found, wipe the cmake
# cache so it gets re-detected with the newly installed package.
if [ -f "$PX4_DIR/build/px4_sitl_default/CMakeCache.txt" ]; then
    if grep -q "gz-transport_DIR:PATH=.*NOTFOUND" "$PX4_DIR/build/px4_sitl_default/CMakeCache.txt" 2>/dev/null; then
        echo -e "${YELLOW}  Stale cmake cache (gz-transport was not found) — reconfiguring...${NC}"
        rm -rf "$PX4_DIR/build/px4_sitl_default"
    fi
fi

# Step 4: Build PX4 SITL
echo ""
echo -e "${GREEN}[4/4] Building PX4 SITL firmware (this may take 10-20 minutes on first build)...${NC}"
make px4_sitl_default

# Post-build check: make sure gz_bridge was compiled
if ! [ -f "$PX4_DIR/build/px4_sitl_default/bin/px4-gz_bridge" ]; then
    echo ""
    echo -e "${RED}Warning: gz_bridge was NOT built — Gazebo simulation will not work.${NC}"
    echo -e "${RED}Make sure libgz-transport12-dev is installed, then delete the build dir and re-run:${NC}"
    echo -e "${RED}  rm -rf $PX4_DIR/build/px4_sitl_default${NC}"
    echo -e "${RED}  sim/setup_sim.sh${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Setup complete!${NC}"
echo ""
echo "Next steps:"
echo "  1. Start the simulator:  sim/start_sim.sh --bridge --world 2_person_world"
echo "  2. In another terminal:  source setup_env.sh"
echo "     drone-follow --input udp://0.0.0.0:5600 --takeoff-landing --ui"
