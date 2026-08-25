#!/usr/bin/env bash
# ==============================================================================
# Script Cross-Compile qua Docker Ubuntu 20.04 (GLIBC 2.31) cho robot Lite3
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REMOTE_HOST="lite3"
REMOTE_DEST_DIR="/home/ysc/workspaces/tuanpm/lite3_mpc/build"
IMAGE_NAME="lite3-builder:20.04-v2"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

echo -e "\n${BOLD}${CYAN}================================================================${NC}"
echo -e "${BOLD}${CYAN}  1. Kiem tra va Chuan bi Docker Image (${IMAGE_NAME})           ${NC}"
echo -e "${BOLD}${CYAN}================================================================${NC}"

# 1. Kiem tra xem Image da co san chua, neu chua thi build image (chi ton vai giay lan dau)
if ! docker image inspect "${IMAGE_NAME}" &>/dev/null; then
    echo -e "${YELLOW}>>> Dang tao Docker image ${IMAGE_NAME} (cai compiler ARM64 & eigen3)...${NC}"
    docker build -t "${IMAGE_NAME}" - <<'EOF'
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    cmake \
    make \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
EOF
    echo -e "${GREEN}>>> Docker Image ${IMAGE_NAME} da san sang!${NC}"
else
    echo -e "${GREEN}>>> Su dung Docker Image da cache: ${IMAGE_NAME}${NC}"
fi

echo -e "\n${BOLD}${CYAN}================================================================${NC}"
echo -e "${BOLD}${CYAN}  2. Cross-compiling ARM64 tren may Host (GLIBC 2.31)            ${NC}"
echo -e "${BOLD}${CYAN}================================================================${NC}"

USER_UID=$(id -u)
USER_GID=$(id -g)

# 2. Chay build ben trong container (in day du log compile)
docker run --rm \
    -v "${PROJECT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_NAME}" \
    bash -c "
        set -e
        mkdir -p build_arm20 && cd build_arm20
        rm -f CMakeCache.txt
        echo '--- [CMake Config] ---'
        cmake .. \
            -DCMAKE_TOOLCHAIN_FILE=../cmake/toolchain_aarch64.cmake \
            -DBUILD_PLATFORM=arm \
            -DBUILD_SIM=OFF \
            -DBUILD_TESTS=ON
        
        echo '--- [Compiling with \$(nproc) cores] ---'
        make -j\$(nproc)
        
        chown -R ${USER_UID}:${USER_GID} /workspace/build_arm20
    "

echo -e "${GREEN}>>> Build thanh cong tat ca binaries ARM64 cho Ubuntu 20.04!${NC}"

echo -e "\n${YELLOW}>>> [Xac thuc kien truc binary]:${NC}"
file "${PROJECT_DIR}/build_arm20/pinocchio_smoke"
file "${PROJECT_DIR}/build_arm20/lite3_pinocchio_model_smoke"
file "${PROJECT_DIR}/build_arm20/lite3_gmo_smoke"
file "${PROJECT_DIR}/build_arm20/cmpc_deploy"

echo -e "\n${BOLD}${CYAN}================================================================${NC}"
echo -e "${BOLD}${CYAN}  3. Chuyen binaries sang robot ${REMOTE_HOST}                   ${NC}"
echo -e "${BOLD}${CYAN}================================================================${NC}"


echo -e "${YELLOW}>>> Dang ket noi SSH toi ${REMOTE_HOST}...${NC}"
ssh -o ConnectTimeout=8 "${REMOTE_HOST}" "mkdir -p ${REMOTE_DEST_DIR}"

rsync -avz --progress \
    "${PROJECT_DIR}/build_arm20/pinocchio_smoke" \
    "${PROJECT_DIR}/build_arm20/lite3_pinocchio_model_smoke" \
    "${PROJECT_DIR}/build_arm20/lite3_gmo_smoke" \
    "${PROJECT_DIR}/build_arm20/cmpc_deploy" \
    "${REMOTE_HOST}:${REMOTE_DEST_DIR}/"

# Sync ca thu vien MotionSDK vao thu muc chay
rsync -avz --progress \
    "${PROJECT_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so" \
    "${REMOTE_HOST}:${REMOTE_DEST_DIR}/"


echo -e "${GREEN}>>> Da chuyen toan bo binary sang Lite3 thanh cong!${NC}"

echo -e "\n${BOLD}${CYAN}================================================================${NC}"
echo -e "${BOLD}${CYAN}  4. Chay kiem thu Pinocchio tren robot ${REMOTE_HOST}           ${NC}"
echo -e "${BOLD}${CYAN}================================================================${NC}"

ssh -t "${REMOTE_HOST}" "bash -lc '
    cd ${REMOTE_DEST_DIR}
    echo \"\n\033[1;32m=== [1/3] RUNNING PINOCCHIO CORE TEST ===\033[0m\"
    ./pinocchio_smoke

    echo \"\n\033[1;32m=== [2/3] RUNNING LITE3 MODEL DYNAMICS TEST ===\033[0m\"
    ./lite3_pinocchio_model_smoke

    echo \"\n\033[1;32m=== [3/3] RUNNING LITE3 GMO/GRF TEST ===\033[0m\"
    ./lite3_gmo_smoke
'"

echo -e "\n${BOLD}${GREEN}================================================================${NC}"
echo -e "${BOLD}${GREEN}  HOAN TAT: Pinocchio va Boost Header-Only da verify tren Lite3! ${NC}"
echo -e "${BOLD}${GREEN}================================================================${NC}\n"
