#!/usr/bin/env bash
# =============================================================================
# 启动 PX4 SITL + Gazebo 四旋翼仿真
#
# 用法:
#   bash scripts/40_run_sitl.sh                 # 默认机型 x500
#   bash scripts/40_run_sitl.sh gz_x500_mono_cam
#   MODEL=gz_x500_depth bash scripts/40_run_sitl.sh
#
# 说明: 该脚本只负责 "飞控 + Gazebo"。ROS 2 侧（Agent + 桥）请另开终端执行
#       scripts/41_run_ros2_bridge.sh，二者通过 uXRCE-DDS / gz-transport 通信。
#
# 首次运行: PX4 会把模型放到 ~/.simulation-gazebo；仓库里 Tools/simulation/gz
#   已经带了整套模型/世界，本脚本会先本地拷贝过去，避免再去 GitHub 下载。
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4_DIR="${ROOT_DIR}/px4_firmware"
MODEL="${1:-${MODEL:-gz_x500}}"
GZ_MODEL_STORE="${HOME}/.simulation-gazebo"

if [[ ! -d "${PX4_DIR}" ]]; then
  echo "找不到 PX4 源码目录 ${PX4_DIR}" >&2
  exit 1
fi

# 找到 gz 可执行文件：先看 PATH，再看 ros_gz 的 vendor 目录
#   （不同 ros_gz 版本把 gz 放在 opt/gz_tools_vendor/bin 或 opt/gz_sim_vendor/bin）
if ! command -v gz >/dev/null 2>&1; then
  for GZ_VENDOR_BIN in \
    "/opt/ros/${ROS_DISTRO:-jazzy}/opt/gz_tools_vendor/bin" \
    "/opt/ros/${ROS_DISTRO:-jazzy}/opt/gz_sim_vendor/bin"; do
    if [[ -x "${GZ_VENDOR_BIN}/gz" ]]; then
      export PATH="${GZ_VENDOR_BIN}:${PATH}"
      break
    fi
  done
fi
if ! command -v gz >/dev/null 2>&1; then
  echo "找不到 gz 命令：请先执行 sudo bash scripts/10_install_apt_deps.sh" >&2
  exit 1
fi

# 本地准备 Gazebo 模型（避免 PX4 首次运行时联网下载）
if [[ ! -d "${GZ_MODEL_STORE}/models" ]]; then
  LOCAL_GZ_DIR="${PX4_DIR}/Tools/simulation/gz"
  if [[ -d "${LOCAL_GZ_DIR}/models" ]]; then
    echo "==> 拷贝 Gazebo 模型到 ${GZ_MODEL_STORE}（来自本地 PX4 源码）"
    mkdir -p "${GZ_MODEL_STORE}"
    cp -r "${LOCAL_GZ_DIR}/models" "${LOCAL_GZ_DIR}/worlds" "${GZ_MODEL_STORE}/"
    cp -f "${LOCAL_GZ_DIR}/server.config" "${GZ_MODEL_STORE}/" 2>/dev/null || true
  else
    echo "警告: ${LOCAL_GZ_DIR}/models 不存在，PX4 首次运行会自行联网下载模型" >&2
  fi
fi

cd "${PX4_DIR}"
echo "==> 启动 PX4 SITL: make px4_sitl ${MODEL}"
exec make px4_sitl "${MODEL}"
