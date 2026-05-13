#!/bin/bash
# Uninstall the drone-follow SysV init service from a Hailo15 target.
#
# Usage: ./uninstall.sh [root@<ip>]

set -e

TARGET=${1:-${TARGET:-root@10.0.0.1}}

echo "=== Uninstalling drone-follow init service from $TARGET ==="

ssh "$TARGET" "
    [ -f /etc/init.d/drone-follow ] && /etc/init.d/drone-follow stop 2>/dev/null
    rm -f /etc/rcS.d/S99drone-follow
    rm -f /etc/init.d/drone-follow
    rm -f /etc/default/drone-follow
    rm -f /var/run/drone-follow.pid
    echo 'Removed init script, config, and boot symlink.'
    echo 'Log preserved at /var/log/drone-follow.log'
"

echo "Done."
