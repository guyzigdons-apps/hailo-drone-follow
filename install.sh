#!/usr/bin/env bash
# robot-follow standalone installer.
#
# Bootstraps hailo-apps-infra as a git submodule at ./hailo-apps, runs the
# parent installer to lay down the venv + system bits, then installs the
# robot-follow Python package editable into that venv, downloads ReID HEFs,
# and builds the React UI.
#
# Legacy note: the `drone-follow` console-script alias is preserved permanently
# (see pyproject.toml [project.scripts]) so the boot service unit, the
# ~/Desktop/drone-follow.conf file, and existing deployed units keep working
# unchanged. `pip uninstall drone-follow -y` runs below to clear stale metadata
# from the legacy `drone-follow` distribution name on already-deployed units.
#
# Idempotent: re-running picks up missing steps and skips ones already done.
#
# Run as your normal user; the parent hailo-apps installer is invoked with
# sudo automatically (it needs root for apt + /usr/local/hailo/resources).
#
# Flags:
#   --skip-submodule   skip git submodule update (assumes ./hailo-apps is ready)
#   --skip-apps        skip running ./hailo-apps/install.sh (assumes parent installed)
#   --skip-python      skip pip install -e .
#   --skip-hefs        skip ReID HEF downloads
#   --skip-ui          skip npm install + UI build
#   --rover            install ROS 2 Humble + Gazebo Garden apt deps (rover sim)

set -euo pipefail

# --- Parse flags first (before SCRIPT_DIR setup, so rover-only invocations
# --- via stripped PATH can short-circuit to the Step-6 preflights without
# --- tripping over dirname / cd on PATH=/tmp test paths).
SKIP_SUBMODULE=false
SKIP_APPS=false
SKIP_PYTHON=false
SKIP_HEFS=false
SKIP_UI=false
ROVER_DEPS=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-submodule) SKIP_SUBMODULE=true; shift ;;
    --skip-apps)      SKIP_APPS=true; shift ;;
    --skip-python)    SKIP_PYTHON=true; shift ;;
    --skip-hefs)      SKIP_HEFS=true; shift ;;
    --skip-ui)        SKIP_UI=true; shift ;;
    --rover)          ROVER_DEPS=true; shift ;;
    -h|--help)
      sed -n '2,26p' "$0"
      exit 0
      ;;
    *) echo "Unknown flag: $1" >&2; exit 2 ;;
  esac
done

# --- Step 6: Rover sim apt deps (only if --rover passed) ---------------------
# RSIM-05: install ros-humble-ros-base, ros-humble-geometry-msgs, and the
# Garden-suffixed ros-gz bridge (ros-humble-ros-gzgarden-bridge).  The
# no-suffix form is the Fortress binding and must NOT be used here
# (PITFALLS Pitfall 5).  See sim/rover/README.md for the Harmonic-migration
# recipe.
#
# Placed BEFORE the hailo-apps SCRIPT_DIR setup so that a rover-only
# invocation (--rover plus all five --skip-* flags) runs through Step-6
# preflights without needing the hailo-apps submodule context.  When --rover
# is NOT passed (default), this block is skipped silently and the existing
# 5-step flow runs unchanged.
if $ROVER_DEPS; then
  echo "==> [6/6] Installing ROS 2 Humble + Gazebo Garden bridge (rover sim)"

  # Preflight 1: apt-get must exist (Ubuntu/Debian only).
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "ERROR: --rover requires apt-get (Ubuntu/Debian only)." >&2
    echo "       This rover-sim install path is sim-only and Linux-only;" >&2
    echo "       see sim/rover/README.md for the Harmonic-migration note." >&2
    exit 6
  fi

  # Preflight 2: osrfoundation apt repo must be configured.
  # ros-humble-ros-gzgarden-bridge ships from packages.osrfoundation.org,
  # NOT packages.ros.org.  Visible to apt-cache iff the repo is configured.
  # (PITFALLS Pitfall 7 / RSIM-05; verified package name from apt-cache
  # search on the v1.1 dev box 2026-05-20.)
  if ! apt-cache search ros-humble-ros-gzgarden-bridge 2>/dev/null \
       | grep -q '^ros-humble-ros-gzgarden-bridge '; then
    echo "ERROR: ros-humble-ros-gzgarden-bridge not visible to apt-cache." >&2
    echo "       Add the osrfoundation apt repo first:" >&2
    echo >&2
    echo "         sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \\" >&2
    echo "              -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg" >&2
    echo "         echo \"deb [signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \\" >&2
    echo "               http://packages.osrfoundation.org/gazebo/ubuntu-stable \\" >&2
    echo "               \$(lsb_release -cs) main\" \\" >&2
    echo "              | sudo tee /etc/apt/sources.list.d/gazebo-stable.list" >&2
    echo "         sudo apt update" >&2
    echo >&2
    echo "       If your machine already runs Gazebo Harmonic instead of Garden," >&2
    echo "       see sim/rover/README.md \"Migration to Harmonic\" for the" >&2
    echo "       s/gzgarden/gzharmonic/ apt-name substitution." >&2
    exit 7
  fi

  # Apt install.  Pin exact package names — RSIM-05 + PITFALLS Pitfall 5:
  # the no-suffix gz-bridge form is the Fortress binding and breaks
  # DiffDrive silently on Garden.
  sudo apt-get install -y \
       ros-humble-ros-base \
       ros-humble-geometry-msgs \
       ros-humble-ros-gzgarden-bridge \
       gz-garden

  # Preflight 3: gz CLI must end up on PATH (sanity check the install).
  if ! command -v gz >/dev/null 2>&1; then
    echo "ERROR: 'gz' CLI not found after apt install -- check apt logs." >&2
    echo "       Re-run with: sudo apt-get install -y gz-garden" >&2
    exit 8
  fi

  echo "==> Rover sim deps installed.  See sim/rover/README.md to launch."

  # Rover-only mode: when all five hailo --skip-* flags are also set, exit
  # cleanly so a contributor on a non-hailo box (rover-sim-only dev) doesn't
  # trip over the hailo-apps submodule / venv assertions below.
  if $SKIP_SUBMODULE && $SKIP_APPS && $SKIP_PYTHON && $SKIP_HEFS && $SKIP_UI; then
    echo
    echo "==> Rover-only install complete.  See sim/rover/README.md to launch."
    exit 0
  fi
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$SCRIPT_DIR/hailo-apps"
RESOURCES_HEF_DIR="/usr/local/hailo/resources/models/hailo8"

# --- Step 1: Init/update the hailo-apps submodule ----------------------------
if ! $SKIP_SUBMODULE; then
  echo "==> [1/5] Initialising hailo-apps submodule"
  (cd "$SCRIPT_DIR" && git submodule update --init --recursive hailo-apps)
fi

if [[ ! -d "$APPS_DIR" ]] || [[ -z "$(ls -A "$APPS_DIR" 2>/dev/null)" ]]; then
  echo "ERROR: $APPS_DIR is missing or empty — did the submodule init succeed?" >&2
  exit 1
fi

# --- Step 2: Run the parent hailo-apps installer -----------------------------
# The parent installer requires sudo (apt, /usr/local/hailo/resources/, ...).
# It uses $SUDO_USER internally to chown the venv back to the invoking user,
# so the remaining drone-follow steps below run unprivileged as expected.
if ! $SKIP_APPS; then
  echo "==> [2/5] Running ./hailo-apps/install.sh (creates venv_hailo_apps + .env, may take a while)"
  if [[ $EUID -eq 0 ]]; then
    "$APPS_DIR/install.sh"
  else
    sudo -E "$APPS_DIR/install.sh"
  fi
fi

VENV="$APPS_DIR/venv_hailo_apps"
if [[ ! -d "$VENV" ]]; then
  echo "ERROR: $VENV not present after parent installer ran." >&2
  echo "       Check $APPS_DIR/logs/ for details, then re-run with --skip-submodule." >&2
  exit 1
fi

# --- Step 3: Activate the venv and install drone-follow editable -------------
# shellcheck disable=SC1091
source "$VENV/bin/activate"

python -c "import hailo_apps" >/dev/null 2>&1 || {
  echo "ERROR: hailo_apps not importable inside $VENV." >&2
  exit 1
}

if ! $SKIP_PYTHON; then
  # Idempotent: remove any prior installation of the legacy 'drone-follow'
  # distribution so pip metadata + the old `drone-follow` console-script shim
  # don't linger after the v1.1 rename. No-op on fresh installs. Runs as the
  # invoking user (we're inside the activated venv from above).
  echo "==> Removing any prior 'drone-follow' distribution (legacy name)"
  pip uninstall drone-follow -y >/dev/null 2>&1 || true

  echo "==> [3/5] pip install -e $SCRIPT_DIR"
  pip install --upgrade pip
  pip install -e "$SCRIPT_DIR"
fi

# --- Step 4: Download ReID HEFs (idempotent) ---------------------------------
if ! $SKIP_HEFS; then
  echo "==> [4/5] ReID HEFs into $RESOURCES_HEF_DIR"
  REID_BASE_URL="https://hailo-model-zoo.s3.eu-west-2.amazonaws.com/ModelZoo/Compiled/v2.18.0/hailo8"
  declare -A HEFS=(
    [repvgg_a0_person_reid_512.hef]="$REID_BASE_URL/repvgg_a0_person_reid_512.hef"
    [osnet_x1_0.hef]="$REID_BASE_URL/osnet_x1_0.hef"
  )
  if [[ ! -d "$RESOURCES_HEF_DIR" ]]; then
    sudo mkdir -p "$RESOURCES_HEF_DIR"
    sudo chown -R "$USER:$USER" "$(dirname "$(dirname "$RESOURCES_HEF_DIR")")"
  fi
  for hef in "${!HEFS[@]}"; do
    target="$RESOURCES_HEF_DIR/$hef"
    if [[ -f "$target" ]]; then
      echo "    $hef already present, skip"
      continue
    fi
    echo "    downloading $hef"
    wget -q --show-progress -O "$target" "${HEFS[$hef]}"
  done
fi

# --- Step 5: Build the React UI ---------------------------------------------
if ! $SKIP_UI; then
  echo "==> [5/5] React UI"
  if command -v npm >/dev/null 2>&1; then
    pushd "$SCRIPT_DIR/robot_follow/ui" >/dev/null
    if [[ ! -f build/index.html ]] || [[ src/App.jsx -nt build/index.html ]]; then
      npm install
      npm run build
    else
      echo "    UI build is up-to-date, skip"
    fi
    popd >/dev/null
  else
    echo "    WARN: npm not found — skipping UI build. Install Node 20+ and rerun."
  fi
fi

echo
echo "==> robot-follow install done. Next:"
echo "    source setup_env.sh"
echo "    robot-follow --help    # 'drone-follow --help' (alias) also works"
