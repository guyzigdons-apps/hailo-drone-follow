#!/usr/bin/env bash
# gst-hailo-cache install entry point.
#
# Task 1 of Plan 5 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
# minimal idempotent build + install. Harder hardening (--uninstall, --check,
# --reconfigure, sqlite3 version pre-flight) lands in Task 13.
#
# Usage:
#   bash gst-hailo-cache/install.sh
#
# Requires sudo for `ninja -C build install` (drops libgsthailocache.so into
# the system GStreamer plugin dir resolved via `pkg-config`). Re-running is
# safe — meson skips an already-configured build dir and ninja only rebuilds
# what changed.

set -euo pipefail

# Resolve script dir so the script works from any CWD.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$SCRIPT_DIR"

# --- Pre-flight: required build tools and dev headers ---------------------
missing=()
for cmd in meson ninja pkg-config gst-inspect-1.0; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        missing+=("$cmd")
    fi
done
if ((${#missing[@]} > 0)); then
    echo "ERROR: missing required commands: ${missing[*]}" >&2
    echo "Hint: sudo apt install -y meson ninja-build pkg-config gstreamer1.0-tools" >&2
    exit 1
fi

for pc in gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 sqlite3; do
    if ! pkg-config --exists "$pc"; then
        echo "ERROR: pkg-config can't find '$pc' — missing -dev package?" >&2
        echo "Hint: sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libsqlite3-dev" >&2
        exit 1
    fi
done

# --- Configure --------------------------------------------------------------
if [[ ! -d build ]]; then
    echo "==> meson setup build"
    meson setup build
else
    echo "==> meson build dir already exists, skipping setup"
fi

# --- Build ------------------------------------------------------------------
echo "==> ninja -C build"
ninja -C build

# --- Install (requires sudo) -----------------------------------------------
echo "==> sudo ninja -C build install"
sudo ninja -C build install

# --- Flush the user's GStreamer registry cache so the new plugin is picked up
# Per .claude/memory/hailotilecropper_dynamic.md, stale registry can cause the
# wrong .so to load.
echo "==> rm -f ~/.cache/gstreamer-1.0/registry.*.bin"
rm -f ~/.cache/gstreamer-1.0/registry.*.bin

echo
echo "Done. Verify with:"
echo "    gst-inspect-1.0 hailocache"
