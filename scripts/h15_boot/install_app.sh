#!/bin/bash
# Sync the robot_follow app code to a Hailo15 target.
#
# Usage:
#   ./install_app.sh                       # uses default target root@10.0.0.1
#   ./install_app.sh root@<ip>
#   TARGET=root@<ip> ./install_app.sh

set -e

TARGET=${1:-${TARGET:-root@10.0.0.1}}
REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"

echo "=== Syncing robot_follow app to $TARGET ==="
echo "  from: $REPO_ROOT/robot_follow/"
echo "  to:   $TARGET:/home/root/robot_follow/"

rsync -av \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='*.egg-info' \
    --exclude='ui/node_modules' \
    --exclude='ui/src' \
    --exclude='ui/*.json' \
    --exclude='ui/index.html' \
    --exclude='ui/vite.config.js' \
    "$REPO_ROOT/robot_follow/" "$TARGET:/home/root/robot_follow/"

# Sanity check: ui/build must exist (else web UI will be missing)
if [ ! -d "$REPO_ROOT/robot_follow/ui/build" ]; then
    echo ""
    echo "WARNING: robot_follow/ui/build/ does not exist locally."
    echo "  Web UI will not be served. To build it, run:"
    echo "    cd robot_follow/ui && npm install && npm run build"
fi

# One-time cleanup: if a previous deploy left /home/root/drone_follow/ on
# the target, remove it so stale .pyc/__pycache__ entries can't be picked
# up ahead of the new robot_follow/ tree by Python's import system.
ssh "$TARGET" "[ -d /home/root/drone_follow ] && rm -rf /home/root/drone_follow && echo 'Removed stale /home/root/drone_follow/' || true"

echo "Done."
