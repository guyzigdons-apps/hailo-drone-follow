#!/usr/bin/env bash
# Launch the tiling-benchmark overlay viewer with the standard run set
# (GT + native/+vga3x pairs). Run from anywhere:
#
#   ./tiling_benchmark/run_viewer.sh
#
# Options:
#   -v, --video PATH    Override the source video.
#   -r, --runs-dir DIR  Directory holding pxt_<label>.frames.json (default: pxt_runs).
#   --all               Load every pxt_*.frames.json in the runs dir instead
#                       of the curated default set.
#   Any extra args after `--` are passed straight to overlay_viewer.py.
#
# Example:
#   ./tiling_benchmark/run_viewer.sh --all -- --conf-min 0.4
set -euo pipefail

# --- locate ourselves + repo root ------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

VIDEO="/home/giladn/Videos/Drone/Training/DJI_20260430103421_0010_D_rotated.MP4"
RUNS_DIR="$SCRIPT_DIR/pxt_runs"
LOAD_ALL=0
EXTRA_ARGS=()

# --- parse args -------------------------------------------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    -v|--video)    VIDEO="$2"; shift 2 ;;
    -r|--runs-dir) RUNS_DIR="$2"; shift 2 ;;
    --all)         LOAD_ALL=1; shift ;;
    --)            shift; EXTRA_ARGS=("$@"); break ;;
    -h|--help)
      # Print the leading comment block (skip the shebang, stop at first
      # non-comment line).
      awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
      exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 1 ;;
  esac
done

# --- environment ------------------------------------------------------------
# Activate the hailo-apps venv + paths (sets PYTHONPATH, HAILO_APPS_PATH, …).
# shellcheck disable=SC1091
source "$REPO_ROOT/setup_env.sh" >/dev/null 2>&1

# GUI over SSH / headless login: default to the attached display.
export DISPLAY="${DISPLAY:-:1}"
if [[ -z "${XAUTHORITY:-}" ]]; then
  cand="/run/user/$(id -u)/gdm/Xauthority"
  [[ -f "$cand" ]] && export XAUTHORITY="$cand"
fi

# --- sanity checks ----------------------------------------------------------
[[ -f "$VIDEO" ]]    || { echo "video not found: $VIDEO" >&2; exit 1; }
[[ -d "$RUNS_DIR" ]] || { echo "runs dir not found: $RUNS_DIR" >&2; exit 1; }

# --- build the --frames list ------------------------------------------------
FRAMES_ARGS=()
add_frame() {  # add_frame <label>
  local label="$1"
  local f="$RUNS_DIR/pxt_${label}.frames.json"
  if [[ -f "$f" ]]; then
    FRAMES_ARGS+=(--frames "${f}:${label}")
  else
    echo "  (skip ${label} — not found)" >&2
  fi
}

if [[ "$LOAD_ALL" -eq 1 ]]; then
  shopt -s nullglob
  for f in "$RUNS_DIR"/pxt_*.frames.json; do
    base="$(basename "$f")"; base="${base#pxt_}"; base="${base%.frames.json}"
    FRAMES_ARGS+=(--frames "${f}:${base}")
  done
  shopt -u nullglob
else
  # Curated default: GT + each grid native vs +vga3x.
  add_frame "GT-12x9-25-multi"
  for grid in 2x2 3x3 4x3 6x4 8x6; do
    add_frame "${grid}-native"
    add_frame "${grid}-native+vga3x"
  done
fi

if [[ ${#FRAMES_ARGS[@]} -eq 0 ]]; then
  echo "no run frames.json files found under $RUNS_DIR" >&2
  exit 1
fi

# --- launch -----------------------------------------------------------------
echo "video:   $VIDEO"
echo "runs:    $RUNS_DIR  (DISPLAY=$DISPLAY)"
echo "loading: $(( ${#FRAMES_ARGS[@]} / 2 )) runs"
exec python3 "$SCRIPT_DIR/overlay_viewer.py" \
  --video "$VIDEO" \
  "${FRAMES_ARGS[@]}" \
  "${EXTRA_ARGS[@]}"
