#!/usr/bin/env bash
# =============================================================================
# 编译飞控固件（用于烧录到实物飞控）
#
# 用法:
#   bash scripts/50_build_firmware.sh                    # 默认 Pixhawk 6X
#   bash scripts/50_build_firmware.sh px4_fmu-v6c_default
#   TARGET=px4_fmu-v5_default bash scripts/50_build_firmware.sh
#
# 机型对应关系（按你最终确认的飞控硬件选择）:
#   Pixhawk 6X / 6X Pro   -> px4_fmu-v6x_default
#   Pixhawk 6C / 6C Mini  -> px4_fmu-v6c_default
#   Pixhawk 4 / CUAV v5   -> px4_fmu-v5_default
#   Holybro Kakute H7     -> holybro_kakuteh7_default
#   CUAV X7 / N7          -> cuav_x7pro_default / cuav_nora_default
#
# 产物:  px4_firmware/build/<TARGET>/<TARGET>.px4
# 烧录:  用 QGroundControl「Vehicle Setup -> Firmware」选择该 .px4 文件，
#        或接入 USB 后执行  make <TARGET> upload
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/px4_firmware"
TARGET="${1:-${TARGET:-px4_fmu-v6x_default}}"

cd "${PX4_DIR}"
echo "==> 编译固件: make ${TARGET}"
make "${TARGET}"

FW_FILE="${PX4_DIR}/build/${TARGET}/${TARGET}.px4"
if [[ -f "${FW_FILE}" ]]; then
  echo "==> 固件已生成: ${FW_FILE}  ($(du -h "${FW_FILE}" | cut -f1))"
fi
