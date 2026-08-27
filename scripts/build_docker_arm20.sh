#!/usr/bin/env bash
# ==============================================================================
# Cross-compile Lite3 ARM64 binaries in Ubuntu 20.04 (GLIBC 2.31), optionally
# deploy them to the robot, and run this repository's smoke tests remotely.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${PROJECT_DIR}/build_arm20"

REMOTE_HOST="${REMOTE_HOST:-lite3}"
REMOTE_DEST_DIR="${REMOTE_DEST_DIR:-/home/ysc/workspaces/tuanpm/lite3_mpc/build}"
IMAGE_NAME="${IMAGE_NAME:-lite3-builder:20.04-v3}"
DEPLOY=1

if [[ "${1:-}" == "--build-only" ]]; then
    DEPLOY=0
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--build-only]" >&2
    exit 2
fi

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

section() {
    echo -e "\n${BOLD}${CYAN}================================================================${NC}"
    echo -e "${BOLD}${CYAN}  $1${NC}"
    echo -e "${BOLD}${CYAN}================================================================${NC}"
}

require_command() {
    if ! command -v "$1" >/dev/null 2>&1; then
        echo -e "${RED}Missing required command: $1${NC}" >&2
        exit 1
    fi
}

require_command docker
require_command file
if [[ ${DEPLOY} -eq 1 ]]; then
    require_command ssh
    require_command rsync
fi

section "1. Prepare Docker image (${IMAGE_NAME})"

if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    echo -e "${YELLOW}>>> Building Ubuntu 20.04 ARM64 cross-compiler image...${NC}"
    docker build -t "${IMAGE_NAME}" - <<'DOCKERFILE'
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    cmake \
    make \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*
DOCKERFILE
else
    echo -e "${GREEN}>>> Reusing cached image ${IMAGE_NAME}${NC}"
fi

section "2. Cross-compile ARM64 binaries (Ubuntu 20.04 / GLIBC 2.31)"

HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

docker run --rm \
    -e HOST_UID="${HOST_UID}" \
    -e HOST_GID="${HOST_GID}" \
    -v "${PROJECT_DIR}:/workspace" \
    -w /workspace \
    "${IMAGE_NAME}" \
    bash -ceu '
        mkdir -p /workspace/build_arm20
        rm -f /workspace/build_arm20/CMakeCache.txt
        cmake -E remove_directory /workspace/build_arm20/CMakeFiles

        echo "--- [CMake configure] ---"
        cmake -S /workspace -B /workspace/build_arm20 \
            -DCMAKE_SYSTEM_NAME=Linux \
            -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
            -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
            -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
            -DBUILD_PLATFORM=arm \
            -DBUILD_SIM=OFF \
            -DUSE_MJCPP=OFF \
            -DBUILD_TESTS=ON

        echo "--- [Build with $(nproc) cores] ---"
        cmake --build /workspace/build_arm20 --parallel "4"
        chown -R "${HOST_UID}:${HOST_GID}" /workspace/build_arm20
    '

BINARIES=(
    cmpc_deploy
    pinocchio_smoke
    lite3_pinocchio_model_smoke
    robot_model_smoke
    wbic_types_smoke
    wbic_dynamics_smoke
    wbic_kin_smoke
    wbic_qp_smoke
    wbic_mapping_smoke
    wbic_safety_benchmark_smoke
    wbic_contact_schedule_smoke
    standing_contact_estimator_smoke
    wbic_internal_mapping_smoke
    wbic_qp_contact_transition_smoke
    wbic_integration_smoke
)

RUNTIME_LIBS=(
    "${BUILD_DIR}/cmpc_ctrl/src/JCQP/libJCQP.so"
    "${BUILD_DIR}/cmpc_ctrl/src/qpOASES/libs/libqpOASES.so"
    "${BUILD_DIR}/cmpc_ctrl/src/qpOASES/libs/libqpOASES.so.3.2"
    "${BUILD_DIR}/cmpc_ctrl/src/osqp/out/libosqp.so"
    "${PROJECT_DIR}/third_party/Lite3_MotionSDK/lib/libdeeprobotics_legged_sdk_aarch64.so"
)

TRANSFER_FILES=()
echo -e "\n${YELLOW}>>> Verifying ARM64 artifacts:${NC}"
for binary in "${BINARIES[@]}"; do
    artifact="${BUILD_DIR}/${binary}"
    if [[ ! -x "${artifact}" ]]; then
        echo -e "${RED}Missing build artifact: ${artifact}${NC}" >&2
        exit 1
    fi
    file "${artifact}"
    TRANSFER_FILES+=("${artifact}")
done

for library in "${RUNTIME_LIBS[@]}"; do
    if [[ ! -f "${library}" ]]; then
        echo -e "${RED}Missing runtime library: ${library}${NC}" >&2
        exit 1
    fi
    file "${library}"
    TRANSFER_FILES+=("${library}")
done

echo -e "${GREEN}>>> ARM64 build completed successfully.${NC}"

if [[ ${DEPLOY} -eq 0 ]]; then
    echo -e "${YELLOW}>>> --build-only selected; skipping robot deployment and tests.${NC}"
    exit 0
fi

section "3. Deploy artifacts to ${REMOTE_HOST}:${REMOTE_DEST_DIR}"

ssh -o ConnectTimeout=8 "${REMOTE_HOST}" "mkdir -p '${REMOTE_DEST_DIR}'"
rsync -avz --progress "${TRANSFER_FILES[@]}" "${REMOTE_HOST}:${REMOTE_DEST_DIR}/"

echo -e "${GREEN}>>> Deployment completed.${NC}"

section "4. Run ARM64 smoke tests on ${REMOTE_HOST}"

REMOTE_TESTS=(
    pinocchio_smoke
    lite3_pinocchio_model_smoke
    robot_model_smoke
    wbic_types_smoke
    wbic_dynamics_smoke
    wbic_kin_smoke
    wbic_qp_smoke
    wbic_mapping_smoke
    wbic_safety_benchmark_smoke
    wbic_contact_schedule_smoke
    standing_contact_estimator_smoke
    wbic_internal_mapping_smoke
    wbic_qp_contact_transition_smoke
    wbic_integration_smoke
)

REMOTE_COMMAND="cd '${REMOTE_DEST_DIR}' && export LD_LIBRARY_PATH='${REMOTE_DEST_DIR}':\${LD_LIBRARY_PATH:-}"
for test_name in "${REMOTE_TESTS[@]}"; do
    REMOTE_COMMAND+=" && printf '\\n=== RUNNING ${test_name} ===\\n' && ./${test_name}"
done

ssh -t "${REMOTE_HOST}" "bash -lc \"${REMOTE_COMMAND}\""

echo -e "\n${BOLD}${GREEN}================================================================${NC}"
echo -e "${BOLD}${GREEN}  DONE: ARM64 build, deploy, and smoke tests passed.              ${NC}"
echo -e "${BOLD}${GREEN}================================================================${NC}\n"
