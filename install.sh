#!/bin/bash
set -e

# Install script for drone-follow
# Installs: hailo-apps (base platform), drone-specific Python deps, and UI
#
# Expects hailo-apps to be cloned inside the project directory (./hailo-apps).
# Override with --hailo-apps-dir or HAILO_APPS_DIR env var.
#
# Usage: ./install.sh [OPTIONS]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

# Defaults
HAILO_APPS_DIR="${HAILO_APPS_DIR:-$SCRIPT_DIR/hailo-apps}"
SKIP_HAILO_APPS=false
SKIP_UI=false
SKIP_PYTHON=false

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --hailo-apps-dir DIR   Path to hailo-apps checkout (default: ./hailo-apps)"
    echo "  --skip-hailo-apps      Skip hailo-apps installation (assume already installed)"
    echo "  --skip-ui              Skip UI npm install and build"
    echo "  --skip-python          Skip Python dependency installation"
    echo "  --help, -h             Show this help message"
    echo ""
    echo "Environment variables:"
    echo "  HAILO_APPS_DIR         Same as --hailo-apps-dir"
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --hailo-apps-dir)   HAILO_APPS_DIR="$2"; shift 2 ;;
        --hailo-apps-dir=*) HAILO_APPS_DIR="${1#*=}"; shift ;;
        --skip-hailo-apps)  SKIP_HAILO_APPS=true; shift ;;
        --skip-ui)          SKIP_UI=true; shift ;;
        --skip-python)      SKIP_PYTHON=true; shift ;;
        --help|-h)          usage; exit 0 ;;
        *)                  echo -e "${RED}Unknown argument: $1${NC}"; usage; exit 1 ;;
    esac
done

HAILO_APPS_DIR="$(cd "$HAILO_APPS_DIR" 2>/dev/null && pwd || echo "$HAILO_APPS_DIR")"

echo "========================================="
echo "  drone-follow installer"
echo "========================================="
echo ""

# ─── Step 1: Clone & install hailo-apps (full installer) ─────────────
if ! $SKIP_HAILO_APPS; then
    echo -e "${GREEN}[1/3] Installing hailo-apps base platform...${NC}"

    if [ ! -d "$HAILO_APPS_DIR" ]; then
        echo -e "${YELLOW}  hailo-apps not found at: $HAILO_APPS_DIR${NC}"
        echo -e "  Cloning hailo-apps from github (branch: dev)..."
        git clone -b dev https://github.com/hailocs/hailo-apps-internal.git "$HAILO_APPS_DIR"
    fi

    echo -e "  Using hailo-apps at: ${CYAN}$HAILO_APPS_DIR${NC}"

    # Run hailo-apps' own installer (handles system deps, venv, python
    # bindings, resources, and post-install — requires sudo)
    echo -e "  Running hailo-apps install.sh..."
    sudo "$HAILO_APPS_DIR/install.sh"
else
    echo -e "${YELLOW}[1/3] Skipping hailo-apps installation (--skip-hailo-apps)${NC}"
fi

# Resolve hailo-apps venv path (name comes from hailo-apps config.yaml)
HAILO_VENV_NAME="venv_hailo_apps"
HAILO_VENV_DIR="$HAILO_APPS_DIR/$HAILO_VENV_NAME"

# ─── Step 2: Install drone-follow into hailo-apps venv ───────────────
if ! $SKIP_PYTHON; then
    echo -e "${GREEN}[2/3] Installing drone-follow Python package...${NC}"

    if [ ! -d "$HAILO_VENV_DIR" ]; then
        echo -e "${RED}  Error: hailo-apps venv not found at $HAILO_VENV_DIR${NC}"
        echo -e "  Run without --skip-hailo-apps first, or check your hailo-apps installation."
        exit 1
    fi

    # Activate hailo-apps venv and install drone-follow into it
    echo -e "  Activating hailo-apps venv at ${CYAN}$HAILO_VENV_DIR${NC}..."
    source "$HAILO_VENV_DIR/bin/activate"

    echo -e "  Installing drone-follow (editable)..."
    pip install -e "$SCRIPT_DIR"

    echo -e "${GREEN}  drone-follow installed into hailo-apps venv.${NC}"
else
    echo -e "${YELLOW}[2/3] Skipping Python dependencies (--skip-python)${NC}"
fi

# ─── Step 3: Install and build UI ────────────────────────────────────
if ! $SKIP_UI; then
    echo -e "${GREEN}[3/3] Installing and building UI...${NC}"

    UI_DIR="$SCRIPT_DIR/drone_follow/ui"
    if [ ! -f "$UI_DIR/package.json" ]; then
        echo -e "${YELLOW}  No package.json found, skipping UI.${NC}"
    else
        if ! command -v npm &> /dev/null; then
            echo -e "${RED}  Error: npm is not installed. Install Node.js first:${NC}"
            echo -e "    sudo apt install nodejs npm"
            exit 1
        fi

        echo -e "  Running npm install..."
        (cd "$UI_DIR" && npm install)

        echo -e "  Building UI..."
        (cd "$UI_DIR" && npm run build)

        echo -e "${GREEN}  UI built successfully.${NC}"
    fi
else
    echo -e "${YELLOW}[3/3] Skipping UI installation (--skip-ui)${NC}"
fi

# Regenerate setup_env.sh (portable, using SCRIPT_DIR)
cat > "$SCRIPT_DIR/setup_env.sh" << 'SETUP_EOF'
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"
# Source hailo-apps setup_env.sh (activates venv, sets PYTHONPATH, loads .env)
# It uses $(pwd) to resolve paths, so we cd into hailo-apps first
ORIG_DIR="$(pwd)"
cd "$SCRIPT_DIR/hailo-apps"
source "$SCRIPT_DIR/hailo-apps/setup_env.sh"
cd "$ORIG_DIR"
SETUP_EOF
chmod +x "$SCRIPT_DIR/setup_env.sh"

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "Next steps:"
echo "  source setup_env.sh"
echo "  drone-follow --input rpi --ui"
