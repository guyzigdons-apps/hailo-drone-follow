SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
# Source hailo-apps setup_env.sh (activates venv, sets PYTHONPATH, loads .env)
# It uses $(pwd) to resolve paths, so we cd into hailo-apps first
ORIG_DIR="$(pwd)"
cd "$SCRIPT_DIR/hailo-apps"
source "$SCRIPT_DIR/hailo-apps/setup_env.sh"
cd "$ORIG_DIR"
