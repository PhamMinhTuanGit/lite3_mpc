#!/usr/bin/env bash

set -e

SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/"
# Tuỳ chọn đích đến trên robot (thay đổi IP nếu cần):
ROBOT_USER="ysc"
ROBOT_IP="192.168.2.1"
ROBOT_DEST_DIR="/home/ysc/workspaces/tuanpm/lite3_mpc"

# Hoặc nếu bạn đã cấu hình alias "lite3" trong ~/.ssh/config:
# DEST="lite3:/home/ysc/workspaces/tuanpm/Lite3_rl_deploy/"
DEST="${ROBOT_USER}@${ROBOT_IP}:${ROBOT_DEST_DIR}"

echo "=========================================="
echo "Đang đồng bộ code -> Robot Lite3 ($DEST)..."
echo "=========================================="

rsync -avz --progress "$SOURCE_DIR" "$DEST" \
    --exclude ".git/" \
    --exclude ".codegraph/" \
    --exclude ".venv/" \
    --exclude ".vscode/" \
    --exclude "build*/" \
    --exclude "data/*.csv" \
    --exclude "__pycache__/" \
    --exclude "*.o" \
    --exclude "*.so"

echo "=========================================="
echo "Đồng bộ hoàn tất thành công!"
echo "=========================================="
