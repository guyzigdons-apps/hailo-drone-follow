#!/usr/bin/env bash
# drone-follow standalone installer.
#
# Bootstraps hailo-apps-infra as a git submodule at ./hailo-apps, runs the
# parent installer to lay down the venv + system bits, then installs the
# drone-follow Python package editable into that venv, downloads ReID HEFs,
# and builds the React UI.
#
# Idempotent: re-running picks up missing steps and skips ones already done.
#
# Flags:
#   --skip-submodule   skip git submodule update (assumes ./hailo-apps is ready)
#   --skip-apps        skip running ./hailo-apps/install.sh (assumes parent installed)
#   --skip-python      skip pip install -e .
#   --skip-hefs        skip ReID HEF downloads
#   --skip-ui          skip npm install + UI build

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APPS_DIR="$SCRIPT_DIR/hailo-apps"
RESOURCES_HEF_DIR="/usr/local/hailo/resources/models/hailo8"

SKIP_SUBMODULE=false
SKIP_APPS=false
SKIP_PYTHON=false
SKIP_HEFS=false
SKIP_UI=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-submodule) SKIP_SUBMODULE=true; shift ;;
    --skip-apps)      SKIP_APPS=true; shift ;;
    --skip-python)    SKIP_PYTHON=true; shift ;;
    --skip-hefs)      SKIP_HEFS=true; shift ;;
    --skip-ui)        SKIP_UI=true; shift ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *) echo "Unknown flag: $1" >&2; exit 2 ;;
  esac
done

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
if ! $SKIP_APPS; then
  echo "==> [2/5] Running ./hailo-apps/install.sh (creates venv_hailo_apps + .env, may take a while)"
  "$APPS_DIR/install.sh"
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
    pushd "$SCRIPT_DIR/drone_follow/ui" >/dev/null
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
echo "==> drone-follow install done. Next:"
echo "    source setup_env.sh"
echo "    drone-follow --help"
