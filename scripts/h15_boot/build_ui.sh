#!/bin/bash
# Build the React UI (drone_follow/ui/build/).
# Run this whenever drone_follow/ui/src/ changes.
#
# Usage: ./build_ui.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
UI_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/drone_follow/ui"

if [ ! -d "$UI_DIR/src" ]; then
    echo "Error: $UI_DIR/src not found"
    exit 1
fi

echo "=== Building UI in $UI_DIR ==="

if [ ! -d "$UI_DIR/node_modules" ]; then
    echo "[1/2] Installing npm dependencies (one-time)..."
    (cd "$UI_DIR" && npm install)
fi

echo "[2/2] Building production bundle..."
(cd "$UI_DIR" && npm run build)

echo "Done. Built files in $UI_DIR/build/"
ls -la "$UI_DIR/build/assets/" 2>/dev/null
