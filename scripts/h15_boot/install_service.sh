#!/bin/bash
# Install the drone-follow SysV init service on a Hailo15 target.
#
# Usage:
#   ./install.sh                       # uses default target root@10.0.0.1
#   ./install.sh root@<ip>             # custom target
#   TARGET=root@<ip> ./install.sh      # via env var

set -e

TARGET=${1:-${TARGET:-root@10.0.0.1}}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Installing drone-follow init service on $TARGET ==="

echo "[1/4] Copying init script and defaults config..."
scp "$SCRIPT_DIR/drone-follow" "$TARGET:/etc/init.d/drone-follow"
scp "$SCRIPT_DIR/drone-follow.default" "$TARGET:/etc/default/drone-follow"

echo "[2/4] Setting permissions..."
ssh "$TARGET" "chmod +x /etc/init.d/drone-follow"

echo "[3/4] Enabling auto-start on boot (rcS.d/S99)..."
ssh "$TARGET" "ln -sf /etc/init.d/drone-follow /etc/rcS.d/S99drone-follow"

echo "[4/4] Starting service now..."
ssh "$TARGET" "/etc/init.d/drone-follow restart"
sleep 2
ssh "$TARGET" "/etc/init.d/drone-follow status"

echo ""
echo "Done. Useful commands on the target:"
echo "  /etc/init.d/drone-follow {start|stop|restart|status}"
echo "  tail -f /var/log/drone-follow.log"
echo "  nano /etc/default/drone-follow    # edit args, then restart"
