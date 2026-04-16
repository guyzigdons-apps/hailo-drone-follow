#!/bin/bash
# Start PX4 SITL + Gazebo for H15 drone-follow testing.
# No video bridge needed — H15 provides real camera detections.
#
# Usage: sim/start_sim_h15.sh
#
# Environment variables:
#   HEADLESS=1      Run Gazebo without GUI (useful for SSH / remote machines)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$SCRIPT_DIR/PX4-Autopilot"

# Resolve symlink
if [ -L "$PX4_DIR" ]; then
    PX4_DIR="$(readlink -f "$PX4_DIR")"
fi

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m'

if [ ! -d "$PX4_DIR/build/px4_sitl_default" ]; then
    echo -e "${RED}Error: PX4 SITL not built. Run sim/setup_sim_h15.sh first.${NC}"
    exit 1
fi

H15_IP="${H15_IP:-10.0.0.1}"

echo -e "${GREEN}Starting PX4 SITL + Gazebo (x500_vision)...${NC}"
echo "  PX4:      $PX4_DIR"
echo "  MAVLink:  udp → ${H15_IP}:14540"
echo ""
echo "  On H15 run:"
echo "    PYTHONPATH=/home/root python3 -m drone_follow.drone_follow_h15 --takeoff-landing"
echo ""

# Build first (if needed), then inject the .post file into the build directory.
cd "$PX4_DIR"
make px4_sitl_default

# Add a MAVLink instance that sends to the H15.
# PX4's rcS sources <airframe>.post after airframe init.
POST_FILE="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes/4005_gz_x500_vision.post"
cat > "$POST_FILE" << EOF
mavlink start -x -u 14590 -o 14540 -t ${H15_IP} -r 4000000
EOF
echo -e "  Injected H15 MAVLink link → ${H15_IP}:14540"

# Now launch (won't rebuild, just runs)
make px4_sitl gz_x500_vision
