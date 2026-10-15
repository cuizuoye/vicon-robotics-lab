#!/usr/bin/env bash
# =============================================================================
# 下载 QGroundControl (AppImage)
#   用途: 刷固件、机架选择、传感器/遥控器校准、看飞行状态
# 用法:  bash scripts/60_download_qgc.sh [版本号]   默认 v5.1.4
# 产物:  tools/QGroundControl.AppImage
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${1:-v5.1.4}"
URL="https://github.com/mavlink/qgroundcontrol/releases/download/${VERSION}/QGroundControl-x86_64.AppImage"
OUT_DIR="${ROOT_DIR}/tools"
OUT="${OUT_DIR}/QGroundControl.AppImage"

mkdir -p "${OUT_DIR}"
echo "==> 下载 ${URL}"
curl -L --fail --progress-bar -o "${OUT}.part" "${URL}"
mv -f "${OUT}.part" "${OUT}"
chmod +x "${OUT}"
mkdir -p "${OUT_DIR}/qgc_user"

echo "==> 完成: ${OUT}"
echo "    启动:  ${OUT}   (如果报 FUSE 错误: 加 --appimage-extract-and-run)"
