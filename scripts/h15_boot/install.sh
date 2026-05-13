#!/bin/bash
# Full install: copy app code and set up the boot service on a Hailo15 target.
# Calls install_app.sh and install_service.sh in sequence.
#
# Usage:
#   ./install.sh                       # uses default target root@10.0.0.1
#   ./install.sh root@<ip>
#   TARGET=root@<ip> ./install.sh

set -e

TARGET=${1:-${TARGET:-root@10.0.0.1}}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "===== Full install on $TARGET ====="
echo ""

"$SCRIPT_DIR/install_app.sh" "$TARGET"
echo ""
"$SCRIPT_DIR/install_service.sh" "$TARGET"

echo ""
echo "===== Full install complete ====="
