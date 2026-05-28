#!/usr/bin/env bash
# Run the dynamic tile scheduler over the benchmark video with real inference.
#   ./dynamic_tiling/run_dynamic.sh [--budget 300] [--max-frames 300]
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# shellcheck disable=SC1091
source "$REPO_ROOT/setup_env.sh" >/dev/null 2>&1
cd "$REPO_ROOT"
exec python -m dynamic_tiling.run_dynamic \
  --video "/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4" \
  --gt "tiling_benchmark/pxt_runs/pxt_GT-12x9-25-multi.frames.json" \
  "$@"
