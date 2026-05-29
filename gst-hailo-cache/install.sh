#!/usr/bin/env bash
# gst-hailo-cache install entry point.
#
# Plan 5 Task 13 (docs/superpowers/plans/2026-05-28-gst-cache-plugins.md):
# hardened idempotent install. Modes:
#   bash install.sh              build + install (sudo only for the install step)
#   bash install.sh --check      verify the plugin is registered with GStreamer
#   bash install.sh --uninstall  remove the installed .so and flush the registry
#   bash install.sh --help       print this help
#
# The build steps (`meson setup`, `ninja`) always run unprivileged. Only the
# final `ninja install` is invoked under sudo, since it drops
# libgsthailocache.so into the system GStreamer plugin dir
# (`pkg-config --variable=pluginsdir gstreamer-1.0`).
#
# After install/uninstall we always remove the per-user GStreamer registry
# cache (`~/.cache/gstreamer-1.0/registry.*.bin`) so the next `gst-inspect-1.0`
# picks up the new state. Per `.claude/memory/hailotilecropper_dynamic.md`, a
# stale registry can cause the wrong .so to load.

set -euo pipefail

# Resolve script dir so the script works from any CWD.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
cd "$SCRIPT_DIR"

# --- Arg parsing ------------------------------------------------------------
MODE="install"
for arg in "$@"; do
    case "$arg" in
        --check)     MODE="check" ;;
        --uninstall) MODE="uninstall" ;;
        -h|--help)
            sed -n '2,18p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "ERROR: unknown argument: $arg" >&2
            echo "Usage: $0 [--check|--uninstall|--help]" >&2
            exit 2
            ;;
    esac
done

flush_registry() {
    # Match what hailo-apps/install.sh does. Safe even if the files are absent.
    echo "==> rm -f ~/.cache/gstreamer-1.0/registry.*.bin"
    rm -f ~/.cache/gstreamer-1.0/registry.*.bin
}

resolve_gst_plugins_dir() {
    if ! command -v pkg-config >/dev/null 2>&1; then
        echo "ERROR: pkg-config not found; cannot resolve gstreamer pluginsdir" >&2
        exit 1
    fi
    local dir
    dir="$(pkg-config --variable=pluginsdir gstreamer-1.0 2>/dev/null || true)"
    if [[ -z "$dir" ]]; then
        echo "ERROR: pkg-config returned an empty pluginsdir for gstreamer-1.0" >&2
        exit 1
    fi
    printf '%s' "$dir"
}

# --- --check mode -----------------------------------------------------------
if [[ "$MODE" == "check" ]]; then
    if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
        echo "ERROR: gst-inspect-1.0 not found (install gstreamer1.0-tools)" >&2
        exit 1
    fi
    echo "==> gst-inspect-1.0 hailocache"
    if ! gst-inspect-1.0 hailocache; then
        echo
        echo "ERROR: 'hailocache' plugin is not registered." >&2
        echo "Hint: run 'bash $0' to build + install it." >&2
        exit 1
    fi
    exit 0
fi

# --- --uninstall mode -------------------------------------------------------
if [[ "$MODE" == "uninstall" ]]; then
    GST_PLUGINS_DIR="$(resolve_gst_plugins_dir)"
    TARGET="${GST_PLUGINS_DIR}/libgsthailocache.so"
    if [[ -e "$TARGET" ]]; then
        echo "==> sudo rm -f $TARGET"
        sudo rm -f "$TARGET"
    else
        echo "==> $TARGET not present, nothing to remove"
    fi
    flush_registry
    echo
    echo "Done. Verify removal with:"
    echo "    gst-inspect-1.0 hailocache   # expected: not found / exit non-zero"
    exit 0
fi

# --- install mode -----------------------------------------------------------
# Pre-flight: required build tools and dev headers.
missing=()
for cmd in meson ninja pkg-config gst-inspect-1.0 sqlite3; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        missing+=("$cmd")
    fi
done
if ((${#missing[@]} > 0)); then
    echo "ERROR: missing required commands: ${missing[*]}" >&2
    echo "Hint: sudo apt install -y meson ninja-build pkg-config gstreamer1.0-tools sqlite3" >&2
    exit 1
fi

for pc in gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0 sqlite3; do
    if ! pkg-config --exists "$pc"; then
        echo "ERROR: pkg-config can't find '$pc' — missing -dev package?" >&2
        echo "Hint: sudo apt install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libsqlite3-dev" >&2
        exit 1
    fi
done

# sqlite3 >= 3.31 required for the Plan 4 cache layer's WITHOUT ROWID schema
# behaviour (see docs/superpowers/plans/2026-05-28-gst-cache-plugins.md Task 13
# pre-flight item). 3.31 was the first release where WITHOUT ROWID + generated
# columns + UPSERT all work together the way the schema expects.
SQLITE_VER="$(sqlite3 -version | awk '{print $1}')"
sqlite_ok="$(awk -v v="$SQLITE_VER" 'BEGIN {
    split(v, a, ".");
    major = a[1] + 0; minor = a[2] + 0;
    if (major > 3 || (major == 3 && minor >= 31)) print "yes"; else print "no";
}')"
if [[ "$sqlite_ok" != "yes" ]]; then
    echo "ERROR: sqlite3 version ${SQLITE_VER} is too old; need >= 3.31" >&2
    echo "Hint: sudo apt install -y sqlite3 libsqlite3-dev   # or build a newer one from source" >&2
    exit 1
fi
echo "==> sqlite3 ${SQLITE_VER} OK (>= 3.31)"

# --- Configure --------------------------------------------------------------
# Idempotent: if build/ already exists, use --reconfigure so meson re-reads the
# meson.build but keeps the build dir; otherwise do a fresh setup.
if [[ -d build ]]; then
    echo "==> meson setup --reconfigure build"
    meson setup --reconfigure build
else
    echo "==> meson setup build"
    meson setup build
fi

# --- Build (unprivileged) ---------------------------------------------------
echo "==> ninja -C build"
ninja -C build

# --- Install (sudo for THIS step only) --------------------------------------
echo "==> sudo ninja -C build install"
sudo ninja -C build install

# --- Flush the user's GStreamer registry cache so the new plugin is picked up
flush_registry

echo
echo "Done. Verify with:"
echo "    bash $0 --check"
echo "    # or directly:"
echo "    gst-inspect-1.0 hailocache"
