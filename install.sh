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
HAILO_APPS_DIR="${HAILO_APPS_DIR:-$SCRIPT_DIR/hailo-apps}"
SKIP_HAILO_APPS=false
SKIP_UI=false
SKIP_PYTHON=false
VENV_DIR="$SCRIPT_DIR/venv"

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
    echo -e "${GREEN}[1/3] Checking hailo-apps base platform...${NC}"

    if [ ! -d "$HAILO_APPS_DIR" ]; then
        echo -e "${YELLOW}  hailo-apps not found at: $HAILO_APPS_DIR${NC}"
        echo -e "  Cloning hailo-apps from github (branch: feature/combined-fixes-and-features)..."
        git clone -b feature/combined-fixes-and-features https://github.com/hailocs/hailo-apps-internal.git "$HAILO_APPS_DIR"
    fi

    echo -e "  Using hailo-apps at: ${CYAN}$HAILO_APPS_DIR${NC}"
else
    echo -e "${YELLOW}[1/3] Skipping hailo-apps check (--skip-hailo-apps)${NC}"
fi

# ─── Step 1.5: Install System Dependencies ───────────────────────────
if ! $SKIP_HAILO_APPS; then
    echo -e "${GREEN}[1.5/3] Installing system dependencies...${NC}"
    # List from hailo-apps config.yaml
    SYSTEM_PACKAGES=(
        "meson"
        "ninja-build"
        "portaudio19-dev"
        "python3-gi"
        "python3-gi-cairo"
        "python3-venv"
        "libbz2-dev"
        "liblzma-dev"
        "libelf-dev"
        "libunwind-dev"
        "libdw-dev"
    )
    echo -e "  Installing: ${SYSTEM_PACKAGES[*]}"
    sudo apt-get update -qq
    sudo apt-get install -y "${SYSTEM_PACKAGES[@]}"
fi

# ─── Step 2: Install drone-follow Python dependencies ────────────────
if ! $SKIP_PYTHON; then
    echo -e "${GREEN}[2/3] Installing Python dependencies...${NC}"

    # Create venv if not exists (WITH system site packages for gi)
    if [ ! -d "$VENV_DIR" ]; then
        echo -e "  Creating virtual environment in ${CYAN}$VENV_DIR${NC} (with system-site-packages)..."
        python3 -m venv --system-site-packages "$VENV_DIR"
    fi

    # Activate venv
    echo -e "  Activating virtual environment..."
    source "$VENV_DIR/bin/activate"

    # Upgrade pip
    pip install --upgrade pip

    # Install hailo-apps as local editable package (avoids pip cloning from git)
    if [ -d "$HAILO_APPS_DIR" ] && [ -f "$HAILO_APPS_DIR/pyproject.toml" ]; then
        echo -e "  Installing hailo-apps (editable) from ${CYAN}$HAILO_APPS_DIR${NC}..."
        pip install -e "$HAILO_APPS_DIR"
    elif [ -d "$HAILO_APPS_DIR" ] && [ -f "$HAILO_APPS_DIR/setup.py" ]; then
        echo -e "  Installing hailo-apps (editable) from ${CYAN}$HAILO_APPS_DIR${NC}..."
        pip install -e "$HAILO_APPS_DIR"
    else
        echo -e "${YELLOW}  Warning: hailo-apps python package not found in $HAILO_APPS_DIR.${NC}"
    fi

    # Install drone-follow and its dependencies
    echo -e "  Installing drone-follow (editable)..."
    pip install -e "$SCRIPT_DIR"

    # ─── Install Hailo Python Bindings (HailoRT, TAPPAS) ───
    echo -e "  Checking Hailo Python bindings..."
    
    # Detect Architecture using hailo-apps script
    CHECK_SCRIPT="$HAILO_APPS_DIR/scripts/check_installed_packages.sh"
    HAILO_ARCH="unknown"
    
    if [ -f "$CHECK_SCRIPT" ]; then
        # Run check script and extract architecture from summary
        # We use || true because grep might fail if no match
        SUMMARY=$($CHECK_SCRIPT 2>/dev/null | grep "SUMMARY:" || true)
        if [[ -n "$SUMMARY" ]]; then
            HAILO_ARCH=$(echo "$SUMMARY" | grep -o "hailo_arch=[^ ]*" | cut -d= -f2)
            echo -e "  Detected Architecture: ${CYAN}$HAILO_ARCH${NC}"
        else
            echo -e "${YELLOW}  Could not determine architecture from check script.${NC}"
            echo -e "  (Summary was empty or script failed)"
        fi
    else
        echo -e "${YELLOW}  Warning: check_installed_packages.sh not found at $CHECK_SCRIPT${NC}"
    fi

    # Install bindings if arch is valid
    INSTALLER_SCRIPT="$HAILO_APPS_DIR/scripts/hailo_installer_python.sh"
    ARCH_ARG=""
    
    if [[ "$HAILO_ARCH" == "hailo8" || "$HAILO_ARCH" == "hailo8l" ]]; then
        ARCH_ARG="hailo8"
    elif [[ "$HAILO_ARCH" == "hailo10h" ]]; then
        ARCH_ARG="hailo10h"
    else
        echo -e "${YELLOW}  Unknown or unsupported architecture '$HAILO_ARCH'. Skipping auto-install of bindings.${NC}"
        echo -e "  Please install hailort/tappas python wheels manually."
    fi

    if [[ -n "$ARCH_ARG" ]] && [ -f "$INSTALLER_SCRIPT" ]; then
        echo -e "  Installing Hailo Python bindings for $ARCH_ARG..."
        # Execute the installer script in the current shell/venv
        # It installs hailort and tappas-core wheels
        bash "$INSTALLER_SCRIPT" "$ARCH_ARG"
    elif [[ -n "$ARCH_ARG" ]]; then
        echo -e "${RED}  Error: hailo_installer_python.sh not found at $INSTALLER_SCRIPT${NC}"
    fi

    # ─── Run hailo-post-install ───
    # This downloads resources (HEF files) and compiles C++ bits
    if command -v hailo-post-install &> /dev/null; then
        echo -e "  Running hailo-post-install (downloading resources)..."
        # We run with --all to ensure all models are present
        hailo-post-install --all || echo -e "${YELLOW}  Warning: hailo-post-install failed. You may need to run it manually.${NC}"
    else
        echo -e "${YELLOW}  hailo-post-install command not found. Verify hailo-apps installation.${NC}"
    fi

    echo -e "${GREEN}  Python dependencies installed successfully in venv.${NC}"
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

# Create setup_env.sh for convenience
echo "export PYTHONPATH=\"$SCRIPT_DIR:\$PYTHONPATH\"" > "$SCRIPT_DIR/setup_env.sh"
echo "source \"$VENV_DIR/bin/activate\"" >> "$SCRIPT_DIR/setup_env.sh"
chmod +x "$SCRIPT_DIR/setup_env.sh"

echo ""
echo -e "${GREEN}Installation complete!${NC}"
echo ""
echo "Next steps:"
echo "  source setup_env.sh"
echo "  drone-follow --input rpi --ui"
