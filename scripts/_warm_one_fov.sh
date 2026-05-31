#!/usr/bin/env bash
# Manager orchestration helper (Plan 6 A3 workaround): warm one FOV's full
# grid set by invoking warm_gst_cache.py ONCE PER GRID in a fresh subprocess
# (avoids the in-process multi-grid GStreamer deadlock). All grids append to
# ONE cache file (idempotent via A1). Not a committed deliverable — a runner.
#
# Usage: _warm_one_fov.sh <video> <out-cache> [max-frames]
set -u
REPO=/home/giladn/tappas_apps/repos/hailo-drone-follow
cd "$REPO" || exit 2
VIDEO="$1"; OUT="$2"; MAXF="${3:-0}"
HEF=/usr/local/hailo/resources/models/hailo10h/hailo_yolov8n_4_classes_vga.hef
POST=/usr/local/hailo/resources/so/libyolo_hailortpp_postprocess.so
PY="$REPO/hailo-apps/venv_hailo_apps/bin/python"
export HAILO_CHIP=1
export GST_PLUGIN_PATH="$REPO/gst-hailo-cache/build/src"

GRIDS=(1x1:0.0 2x2:0.25 3x2:0.25 3x3:0.25 4x3:0.25 6x4:0.25 8x6:0.25 12x9:0.25)
echo "[fov-warm] video=$VIDEO out=$OUT maxf=$MAXF"
for g in "${GRIDS[@]}"; do
  echo "[fov-warm] === grid $g (fresh process) $(date +%T) ==="
  timeout 1200 "$PY" scripts/warm_gst_cache.py \
    --video "$VIDEO" --hef "$HEF" --post-so "$POST" \
    --out-cache "$OUT" --grids "$g" \
    --source-width 3840 --source-height 2160 --max-frames "$MAXF"
  rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "[fov-warm] GRID $g FAILED rc=$rc — aborting this FOV"
    exit "$rc"
  fi
done
echo "[fov-warm] DONE $OUT $(date +%T)"
