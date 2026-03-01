#!/bin/bash
set -e

# Install script for drone-follow
# Installs: hailo-apps (base platform), drone-specific Python deps, and UI
#
# Expects hailo-apps to be cloned as a sibling directory (../hailo-apps).
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
HAILO_APPS_DIR="${HAILO_APPS_DIR:-$(dirname "$SCRIPT_DIR")/hailo-apps}"
SKIP_HAILO_APPS=false
SKIP_UI=false
SKIP_PYTHON=false

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --hailo-apps-dir DIR   Path to hailo-apps checkout (default: ../hailo-apps)"
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

# ─── Step 1: Install hailo-apps (base platform) ──────────────────────
if ! $SKIP_HAILO_APPS; then
    echo -e "${GREEN}[1/3] Installing hailo-apps base platform...${NC}"

    if [ ! -d "$HAILO_APPS_DIR" ]; then
        echo -e "${RED}  Error: hailo-apps not found at: $HAILO_APPS_DIR${NC}"
        echo -e "  Clone it first:  git clone https://github.com/hailo-ai/hailo-apps.git $HAILO_APPS_DIR"
        echo -e "  Or specify path: $0 --hailo-apps-dir /path/to/hailo-apps"
        exit 1
    fi

    if [ ! -f "$HAILO_APPS_DIR/install.sh" ]; then
        echo -e "${RED}  Error: install.sh not found in $HAILO_APPS_DIR${NC}"
        exit 1
    fi

    echo -e "  Using hailo-apps at: ${CYAN}$HAILO_APPS_DIR${NC}"
    echo -e "  Running hailo-apps install.sh..."
    (cd "$HAILO_APPS_DIR" && sudo bash install.sh)
    echo -e "${GREEN}  hailo-apps installed successfully.${NC}"
else
    echo -e "${YELLOW}[1/3] Skipping hailo-apps installation (--skip-hailo-apps)${NC}"
fi

# ─── Step 2: Install drone-follow Python dependencies ────────────────
if ! $SKIP_PYTHON; then
    echo -e "${GREEN}[2/3] Installing Python dependencies...${NC}"

    # Install hailo-apps as local editable package (avoids pip cloning from git)
    if [ -d "$HAILO_APPS_DIR" ] && [ -f "$HAILO_APPS_DIR/pyproject.toml" ]; then
        echo -e "  Installing hailo-apps (editable) from ${CYAN}$HAILO_APPS_DIR${NC}..."
        pip install -e "$HAILO_APPS_DIR"
    fi

    # Install drone-follow and its dependencies
    echo -e "  Installing drone-follow (editable)..."
    pip install -e "$SCRIPT_DIR"

    echo -e "${GREEN}  Python dependencies installed successfully.${NC}"
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

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "Next steps:"
echo "  source $HAILO_APPS_DIR/setup_env.sh"
echo "  drone-follow --input rpi --ui"
