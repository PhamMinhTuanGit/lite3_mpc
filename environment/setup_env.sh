#!/usr/bin/env bash
# ============================================================================
# Script setup moi truong cho Lite3 RL Deploy (CMPC)
#
# Usage:
#   chmod +x environment/setup_env.sh
#   ./environment/setup_env.sh               # Full setup (can sudo)
#   ./environment/setup_env.sh --check-only  # Chi kiem tra, khong cai gi
#   ./environment/setup_env.sh --skip-apt    # Bo qua apt packages
#   ./environment/setup_env.sh --skip-python # Bo qua Python venv
#   ./environment/setup_env.sh --skip-build  # Bo qua build C++
#
# Moi truong dich:
#   - Ubuntu 24.04 / 22.04
#   - GCC >= 11, CMake >= 3.10
#   - Python 3.10+
# ============================================================================

set -uo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

# --- Colors ----------------------------------------------------------------
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m'
BOLD='\033[1m'

# --- State -----------------------------------------------------------------
PASS=0
FAIL=0
WARN=0
SKIP=0

pass() { PASS=$((PASS+1)); echo -e "  ${GREEN}[PASS]${NC} $1"; }
fail() { FAIL=$((FAIL+1)); echo -e "  ${RED}[FAIL]${NC} $1"; }
warn() { WARN=$((WARN+1)); echo -e "  ${YELLOW}[WARN]${NC} $1"; }
skip() { SKIP=$((SKIP+1)); echo -e "  ${CYAN}[SKIP]${NC} $1"; }
header() { echo -e "\n${BOLD}${MAGENTA}=== $1 ===${NC}\n"; }
cmd_exists() { command -v "$1" &>/dev/null; }

# --- Parse flags -----------------------------------------------------------
CHECK_ONLY=false
SKIP_APT=false
SKIP_PYTHON=false
SKIP_BUILD=false
for arg in "$@"; do
  case "$arg" in
    --check-only) CHECK_ONLY=true ;;
    --skip-apt)   SKIP_APT=true ;;
    --skip-python) SKIP_PYTHON=true ;;
    --skip-build) SKIP_BUILD=true ;;
    *) echo "Unknown option: $arg"; exit 1 ;;
  esac
done

echo -e "${BOLD}${CYAN}"
echo "=============================================="
echo "  Lite3 RL Deploy (CMPC) - Environment Setup "
echo "=============================================="
echo -e "${NC}"
echo "Project dir: ${PROJECT_DIR}"
echo "Mode:        $($CHECK_ONLY && echo 'check only' || echo 'full setup')"
echo ""

# ============================================================================
# 1. BASIC TOOLS
# ============================================================================
header "1. Basic tools (shell, git, ...)"

for tool in bash git wget curl; do
  if cmd_exists "$tool"; then
    pass "$tool: $(which $tool)"
  else
    fail "$tool: NOT FOUND"
  fi
done

# ============================================================================
# 2. SYSTEM PACKAGES (APT)
# ============================================================================
header "2. System packages (apt)"

APT_PKGS=(
  build-essential
  cmake
  libeigen3-dev
  libglfw3-dev
  libboost-all-dev
)

APT_MISSING=()
for pkg in "${APT_PKGS[@]}"; do
  if dpkg -s "$pkg" &>/dev/null; then
    pass "$pkg: installed"
  else
    fail "$pkg: NOT INSTALLED"
    APT_MISSING+=("$pkg")
  fi
done

# Optional packages
for opt_pkg in mold clang-format clang-tidy; do
  if dpkg -s "$opt_pkg" &>/dev/null; then
    pass "$opt_pkg: installed (optional)"
  else
    warn "$opt_pkg: not installed (optional)"
  fi
done

if [ ${#APT_MISSING[@]} -gt 0 ] && [ "$CHECK_ONLY" = false ] && [ "$SKIP_APT" = false ]; then
  echo ""
  echo -e "${YELLOW}Installing apt packages: ${APT_MISSING[*]}${NC}"
  sudo apt update
  sudo apt install -y "${APT_MISSING[@]}"
  echo -e "${GREEN}Done installing apt packages${NC}"
elif [ ${#APT_MISSING[@]} -gt 0 ]; then
  echo ""
  echo -e "${YELLOW}Run: sudo apt install -y ${APT_MISSING[*]}${NC}"
fi

# ============================================================================
# 3. GIT SUBMODULES
# ============================================================================
header "3. Git submodules"

if [ -f "${PROJECT_DIR}/.gitmodules" ]; then
  cd "${PROJECT_DIR}"

  SUBMODULE_COUNT=$(git submodule status 2>/dev/null | wc -l)
  if [ "$SUBMODULE_COUNT" -gt 0 ]; then
    UNINIT_COUNT=$(git submodule status 2>/dev/null | grep -c '^-' || true)
    if [ "$UNINIT_COUNT" -gt 0 ]; then
      warn "$UNINIT_COUNT submodule(s) not initialized"
      if [ "$CHECK_ONLY" = false ]; then
        echo -e "${YELLOW}git submodule update --init --recursive${NC}"
        git submodule update --init --recursive
        echo -e "${GREEN}Submodules initialized${NC}"
      else
        echo -e "${YELLOW}Run: git submodule update --init --recursive${NC}"
      fi
    else
      pass "All $SUBMODULE_COUNT submodule(s) initialized"
    fi

    echo "  Submodule status:"
    git submodule status | awk '{print "    " $2 " -> " $3}'
  else
    pass "No submodules configured"
  fi
else
  skip "No .gitmodules file"
fi

# ============================================================================
# 4. THIRD-PARTY COMPONENTS
# ============================================================================
header "4. Third-party dependencies"

cd "${PROJECT_DIR}"
TP_DIR="${PROJECT_DIR}/third_party"

# 4a. Eigen
if [ -f "${TP_DIR}/eigen/CMakeLists.txt" ]; then
  pass "eigen: found (header-only)"
else
  fail "eigen: MISSING - copy eigen to third_party/eigen"
fi

# 4b. Gamepad
if [ -f "${TP_DIR}/gamepad/CMakeLists.txt" ]; then
  pass "gamepad: found"
else
  fail "gamepad: MISSING"
fi

# 4c. Lite3_MotionSDK
if [ -d "${TP_DIR}/Lite3_MotionSDK/include" ] && [ -d "${TP_DIR}/Lite3_MotionSDK/lib" ]; then
  pass "Lite3_MotionSDK: found (include + lib)"
else
  fail "Lite3_MotionSDK: MISSING"
fi

# 4d. MuJoCo (prebuilt)
for plat in x86 arm; do
  if [ -d "${TP_DIR}/mujoco/${plat}/include" ] && [ -d "${TP_DIR}/mujoco/${plat}/lib" ]; then
    pass "mujoco/${plat}: found"
  else
    fail "mujoco/${plat}: MISSING"
  fi
done

# 4e. onnxruntime (prebuilt, optional)
ONNXRUNTIME_MISSING=false
for plat in x86 arm; do
  if [ -d "${TP_DIR}/onnxruntime/${plat}/include" ] && [ -d "${TP_DIR}/onnxruntime/${plat}/lib" ]; then
    pass "onnxruntime/${plat}: found"
  else
    warn "onnxruntime/${plat}: missing (optional - not used in code yet)"
    ONNXRUNTIME_MISSING=true
  fi
done

if [ "$ONNXRUNTIME_MISSING" = true ] && [ "$CHECK_ONLY" = false ]; then
  echo ""
  echo -e "${YELLOW}onnxruntime is missing. Download now? (y/n, default: n)${NC}"
  echo -e "  (not required for compilation)"
  read -r -p "  > " REPLY </dev/tty
  if [[ "$REPLY" =~ ^[Yy]$ ]]; then
    ONNX_VERSION="1.20.1"
    for pair in "x86|x64" "arm|aarch64"; do
      plat="${pair%|*}"; arch="${pair#*|}"
      url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-${arch}-${ONNX_VERSION}.tgz"
      echo -e "${YELLOW}Downloading onnxruntime for ${arch}...${NC}"
      mkdir -p "${TP_DIR}/onnxruntime/${plat}"
      if wget -q --show-progress "${url}" -O "/tmp/onnxruntime-${arch}.tgz"; then
        tar -xzf "/tmp/onnxruntime-${arch}.tgz" -C "${TP_DIR}/onnxruntime/${plat}" --strip-components=1
        echo -e "${GREEN}onnxruntime/${plat} done${NC}"
      else
        echo -e "${RED}Failed to download onnxruntime for ${arch}${NC}"
        echo -e "${YELLOW}  Try manually: ${url}${NC}"
        rm -rf "${TP_DIR}/onnxruntime/${plat}"
      fi
    done
    rm -f /tmp/onnxruntime-*.tgz 2>/dev/null
  else
    echo -e "${CYAN}Skipping onnxruntime${NC}"
  fi
elif [ "$ONNXRUNTIME_MISSING" = true ]; then
  echo ""
  echo -e "${YELLOW}onnxruntime missing. If needed, download from:${NC}"
  echo "   https://github.com/microsoft/onnxruntime/releases"
fi

# 4f. Boost (header-only in third_party)
if [ -f "${TP_DIR}/boost/boost/version.hpp" ]; then
  pass "boost/boost: found (header-only)"
else
  warn "boost/boost: not in third_party (system /usr/include/boost works via libboost-all-dev)"
fi

# 4g. Pinocchio
if [ -d "${TP_DIR}/pinocchio" ]; then
  pass "pinocchio: found"
else
  warn "pinocchio: not found (may need to copy from another machine)"
fi

# 4h. deep_robotics_model
if [ -d "${TP_DIR}/deep_robotics_model" ]; then
  pass "deep_robotics_model: found"
else
  fail "deep_robotics_model: MISSING - run: git submodule update --init"
fi

# ============================================================================
# 5. PYTHON ENVIRONMENT
# ============================================================================
header "5. Python environment"

PYTHON_REQUIREMENTS=(
  "numpy"
  "mujoco"
  "matplotlib"
  "pandas"
  "colorama"
  "PyYAML"
)

PYTHON_OPTIONAL=(
  "pybullet"
)

if cmd_exists python3; then
  PY_VER=$(python3 --version 2>&1)
  pass "python3: ${PY_VER}"
else
  fail "python3: NOT FOUND"
fi

if cmd_exists pip3; then
  pass "pip3: found"
else
  fail "pip3: NOT FOUND"
fi

PY_MISSING=()
for pkg in "${PYTHON_REQUIREMENTS[@]}"; do
  if python3 -c "import ${pkg%%[=<>]}" 2>/dev/null; then
    pass "python: ${pkg} - installed"
  else
    fail "python: ${pkg} - NOT INSTALLED"
    PY_MISSING+=("${pkg}")
  fi
done

for pkg in "${PYTHON_OPTIONAL[@]}"; do
  if python3 -c "import ${pkg%%[=<>]}" 2>/dev/null; then
    pass "python: ${pkg} - installed (optional)"
  else
    warn "python: ${pkg} - not installed (optional)"
  fi
done

if [ ${#PY_MISSING[@]} -gt 0 ] && [ "$CHECK_ONLY" = false ] && [ "$SKIP_PYTHON" = false ]; then
  echo ""
  echo -e "${YELLOW}Installing Python packages: ${PY_MISSING[*]}${NC}"

  if [ ! -d "${PROJECT_DIR}/.venv" ]; then
    echo -e "${YELLOW}Creating venv at ${PROJECT_DIR}/.venv${NC}"
    python3 -m venv "${PROJECT_DIR}/.venv"
  fi

  source "${PROJECT_DIR}/.venv/bin/activate"
  pip install -U pip
  pip install "${PY_MISSING[@]}"

  for pkg in "${PYTHON_OPTIONAL[@]}"; do
    if ! python3 -c "import ${pkg%%[=<>]}" 2>/dev/null; then
      pip install "${pkg}"
    fi
  done

  pip freeze > "${PROJECT_DIR}/environment/requirements.txt"
  echo -e "${GREEN}Python packages installed${NC}"
  echo ""
  echo -e "${CYAN}Activate venv later: source ${PROJECT_DIR}/.venv/bin/activate${NC}"
elif [ ${#PY_MISSING[@]} -gt 0 ]; then
  echo ""
  echo -e "${YELLOW}Run: pip install ${PY_MISSING[*]}${NC}"
fi

# ============================================================================
# 6. BUILD C++ PROJECT
# ============================================================================
header "6. Build C++ project (cmake + make)"

if [ "$CHECK_ONLY" = false ] && [ "$SKIP_BUILD" = false ]; then
  cd "${PROJECT_DIR}"

  ARCH=$(uname -m)
  if [ "$ARCH" = "x86_64" ]; then
    BUILD_PLATFORM="x86"
  elif [ "$ARCH" = "aarch64" ]; then
    BUILD_PLATFORM="arm"
  else
    BUILD_PLATFORM="x86"
    warn "Unknown arch ${ARCH}, defaulting to x86"
  fi

  echo -e "${YELLOW}Platform: ${BUILD_PLATFORM}${NC}"
  echo -e "${YELLOW}Build with simulation: ON${NC}"
  echo -e "${YELLOW}Parallel jobs: $(nproc)${NC}"

  mkdir -p build
  cd build

  cmake .. \
    -DBUILD_PLATFORM="${BUILD_PLATFORM}" \
    -DBUILD_SIM=ON \
    -DUSE_PYBULLET=ON

  echo ""
  echo -e "${YELLOW}Running make...${NC}"
  make -j"$(nproc)"

  if [ -f "${PROJECT_DIR}/build/cmpc_deploy" ]; then
    echo ""
    pass "BUILD SUCCESS: ${PROJECT_DIR}/build/cmpc_deploy"
  else
    fail "BUILD FAILED: cmpc_deploy not found in build/"
  fi
else
  skip "Build (--skip-build or --check-only)"
  echo ""
  echo -e "${CYAN}Manual build:${NC}"
  echo "    cd ${PROJECT_DIR}"
  echo "    mkdir -p build && cd build"
  echo "    cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON"
  echo "    make -j\$(nproc)"
fi

# ============================================================================
# SUMMARY
# ============================================================================
header "RESULTS"

TOTAL=$((PASS + FAIL + WARN + SKIP))
echo -e "  ${GREEN}Pass:${NC}  $PASS"
echo -e "  ${RED}Fail:${NC}  $FAIL"
echo -e "  ${YELLOW}Warn:${NC}  $WARN"
echo -e "  ${CYAN}Skip:${NC}  $SKIP"
echo ""

if [ "$FAIL" -eq 0 ]; then
  echo -e "${GREEN}${BOLD}All good! Environment is ready.${NC}"
else
  echo -e "${RED}${BOLD}$FAIL item(s) need attention.${NC}"
fi

echo ""
echo -e "${CYAN}Tip: To copy this environment to another machine:${NC}"
echo "  1. git push (including third_party if tracked)"
echo "  2. On new machine: git clone + git submodule update --init"
echo "  3. On new machine: ./environment/setup_env.sh"
echo "  4. Or rsync the whole directory if on same network:"
echo "     rsync -avz --progress ${PROJECT_DIR}/ user@host:/path/to/dest"
echo ""