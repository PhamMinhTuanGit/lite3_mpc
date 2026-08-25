#!/usr/bin/env bash
# ==============================================================================
# Script dong bo (sync) va test tren robot Lite3 su dung SSH alias 'lite3'
#
# Cac che do chay (Usage):
#   ./scripts/sync_to_lite3.sh --src           # Sync ma nguon len robot Lite3
#   ./scripts/sync_to_lite3.sh --bin           # Sync cac file binary da build len robot
#   ./scripts/sync_to_lite3.sh --build-remote  # Sync source + goi cmake/make tren Lite3
#   ./scripts/sync_to_lite3.sh --test          # Chay verify test Pinocchio & dynamics tren Lite3
#   ./scripts/sync_to_lite3.sh --all           # Thuc hien sync src -> build tren robot -> test
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

REMOTE_HOST="lite3"
REMOTE_SRC_DIR="${REMOTE_SRC_DIR:-/home/ysc/workspaces/tuanpm/lite3_mpc}"
REMOTE_BIN_DIR="/home/ysc/cmpc_deploy/bin"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

MODE="${1:---all}"

sync_source() {
    echo -e "${CYAN}>>> Dang sync ma nguon toi ${REMOTE_HOST}:${REMOTE_SRC_DIR}...${NC}"
    ssh "${REMOTE_HOST}" "mkdir -p ${REMOTE_SRC_DIR}"
    
    rsync -avz --progress \
        --exclude='.git' \
        --exclude='build' \
        --exclude='.venv' \
        --exclude='__pycache__' \
        --exclude='*.o' \
        --exclude='*.a' \
        --exclude='*.so' \
        --exclude='data/*.csv' \
        "${PROJECT_DIR}/" "${REMOTE_HOST}:${REMOTE_SRC_DIR}/"
    echo -e "${GREEN}>>> Sync ma nguon thanh cong!${NC}"
}

sync_binaries() {
    echo -e "${CYAN}>>> Dang sync cac file thuc thi (binaries) toi ${REMOTE_HOST}:${REMOTE_BIN_DIR}...${NC}"
    ssh "${REMOTE_HOST}" "mkdir -p ${REMOTE_BIN_DIR}"
    
    if [ ! -d "${PROJECT_DIR}/build" ]; then
        echo -e "${RED}Loi: Thu muc build/ chua ton tai tren may local.${NC}"
        exit 1
    fi
    
    # Sync cac file test va deploy binary neu co
    for bin_file in cmpc_deploy pinocchio_smoke lite3_pinocchio_model_smoke; do
        if [ -f "${PROJECT_DIR}/build/${bin_file}" ]; then
            rsync -avz --progress "${PROJECT_DIR}/build/${bin_file}" "${REMOTE_HOST}:${REMOTE_BIN_DIR}/${bin_file}"
        else
            echo -e "${YELLOW}Bo qua ${bin_file} (khong tim thay trong build/)${NC}"
        fi
    done
    echo -e "${GREEN}>>> Sync binaries thanh cong!${NC}"
}

build_remote() {
    echo -e "${CYAN}>>> Dang build native tren robot ${REMOTE_HOST}...${NC}"
    ssh -t "${REMOTE_HOST}" "bash -lc '
        set -e
        cd ${REMOTE_SRC_DIR}
        mkdir -p build && cd build
        cmake .. -DBUILD_PLATFORM=arm -DBUILD_SIM=OFF -DBUILD_TESTS=ON
        make -j\$(nproc)
    '"
    echo -e "${GREEN}>>> Build tren robot hoan tat!${NC}"
}

test_remote() {
    echo -e "${CYAN}>>> Dang chay cac bai test verify Pinocchio header-only tren robot ${REMOTE_HOST}...${NC}"
    ssh -t "${REMOTE_HOST}" "bash -lc '
        set -e
        echo \"================== 1. TEST PINOCCHIO CORE ==================\"
        if [ -f ${REMOTE_SRC_DIR}/build/pinocchio_smoke ]; then
            ${REMOTE_SRC_DIR}/build/pinocchio_smoke
        elif [ -f ${REMOTE_BIN_DIR}/pinocchio_smoke ]; then
            ${REMOTE_BIN_DIR}/pinocchio_smoke
        else
            echo \"Khong tim thay pinocchio_smoke!\"
            exit 1
        fi

        echo \"\"
        echo \"================== 2. TEST LITE3 DYNAMICS MODEL =============\"
        if [ -f ${REMOTE_SRC_DIR}/build/lite3_pinocchio_model_smoke ]; then
            ${REMOTE_SRC_DIR}/build/lite3_pinocchio_model_smoke
        elif [ -f ${REMOTE_BIN_DIR}/lite3_pinocchio_model_smoke ]; then
            ${REMOTE_BIN_DIR}/lite3_pinocchio_model_smoke
        else
            echo \"Khong tim thay lite3_pinocchio_model_smoke!\"
            exit 1
        fi
    '"
    echo -e "${GREEN}>>> Tat ca cac test Pinocchio header-only da vuot qua tren Lite3!${NC}"
}

cross_compile() {
    echo -e "${CYAN}>>> Dang bien dich cheo (Cross-compile) cho ARM64 tren may Host...${NC}"
    
    # Kiem tra trinh bien dich aarch64
    if ! command -v aarch64-linux-gnu-g++ &>/dev/null; then
        echo -e "${RED}Loi: Chua cai dat aarch64-linux-gnu-g++.${NC}"
        echo -e "${YELLOW}Vui long chay: sudo apt install -y gcc-aarch64-linux-gnu g++-aarch64-linux-gnu${NC}"
        exit 1
    fi

    mkdir -p "${PROJECT_DIR}/build_arm"
    cd "${PROJECT_DIR}/build_arm"
    
    cmake .. \
        -DCMAKE_TOOLCHAIN_FILE="${PROJECT_DIR}/cmake/toolchain_aarch64.cmake" \
        -DBUILD_PLATFORM=arm \
        -DBUILD_SIM=OFF \
        -DBUILD_TESTS=ON
        
    make -j"$(nproc)"
    echo -e "${GREEN}>>> Build ARM64 hoan tat thanh cong tren may Host!${NC}"

    echo -e "${CYAN}>>> Dang chuyen binaries sang robot ${REMOTE_HOST}:${REMOTE_SRC_DIR}/build/...${NC}"
    ssh "${REMOTE_HOST}" "mkdir -p ${REMOTE_SRC_DIR}/build"
    for bin_file in cmpc_deploy pinocchio_smoke lite3_pinocchio_model_smoke; do
        if [ -f "${PROJECT_DIR}/build_arm/${bin_file}" ]; then
            rsync -avz --progress "${PROJECT_DIR}/build_arm/${bin_file}" "${REMOTE_HOST}:${REMOTE_SRC_DIR}/build/${bin_file}"
        fi
    done
    echo -e "${GREEN}>>> Chuyen binaries sang robot thanh cong!${NC}"
}

case "${MODE}" in
    --src)
        sync_source
        ;;
    --bin)
        sync_binaries
        ;;
    --cross)
        cross_compile
        ;;
    --cross-test)
        cross_compile
        test_remote
        ;;
    --build-remote)
        sync_source
        build_remote
        ;;
    --test)
        test_remote
        ;;
    --all)
        sync_source
        build_remote
        test_remote
        ;;
    *)
        echo "Cach dung: $0 [--cross | --cross-test | --src | --bin | --build-remote | --test | --all]"
        exit 1
        ;;
esac

