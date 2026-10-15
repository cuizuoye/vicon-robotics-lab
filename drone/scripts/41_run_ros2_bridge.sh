#!/usr/bin/env bash
# =============================================================================
# 启动 ROS 2 侧通信:
#   1. MicroXRCEAgent  (PX4 <-> ROS 2)
#   2. ros_gz_bridge   (Gazebo <-> ROS 2, 只有仿真才需要)
#
# 用法:
#   仿真（默认）:
#     bash scripts/41_run_ros2_bridge.sh              # Agent(udp4:8888) + Gazebo 桥
#     bash scripts/41_run_ros2_bridge.sh --with-rviz  # 再打开 RViz2
#
#   真机（Agent 走串口连飞控，不启动 Gazebo 桥）:
#     AGENT_TRANSPORT=serial AGENT_DEV=/dev/ttyUSB0 AGENT_BAUD=921600 \
#       bash scripts/41_run_ros2_bridge.sh
#     （飞控侧对应设置 UXRCE_DDS_CFG=102(TELEM2) 且 SER_TEL2_BAUD 与这里一致）
#
#   真机走以太网:
#     AGENT_TRANSPORT=udp4 AGENT_PORT=8888 bash scripts/41_run_ros2_bridge.sh --no-gz
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
AGENT_BIN="${ROOT_DIR}/build/bin/MicroXRCEAgent"
WS_SETUP="${ROOT_DIR}/ros2_ws/install/setup.bash"
WITH_RVIZ=0
TRANSPORT="${AGENT_TRANSPORT:-udp4}"
USE_GZ=1

for arg in "$@"; do
  case "${arg}" in
    --with-rviz) WITH_RVIZ=1 ;;
    --no-gz)     USE_GZ=0 ;;
    --with-gz)   USE_GZ=1 ;;
    *) echo "未知参数: ${arg}" >&2; exit 1 ;;
  esac
done

# 串口模式下默认不起 Gazebo 桥（真机没有 Gazebo）
if [[ "${TRANSPORT}" == "serial" && "${USE_GZ}" == "1" && " $* " != *" --with-gz "* ]]; then
  USE_GZ=0
fi

if [[ "${TRANSPORT}" == "serial" ]]; then
  AGENT_ARGS=(serial --dev "${AGENT_DEV:-/dev/ttyUSB0}" -b "${AGENT_BAUD:-921600}")
else
  AGENT_ARGS=(udp4 -p "${AGENT_PORT:-8888}")
fi

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "${WS_SETUP}" ]]; then
  # shellcheck disable=SC1091
  source "${WS_SETUP}"
else
  echo "警告: 未找到 ${WS_SETUP}，请先运行 scripts/30_build_ros2_ws.sh" >&2
fi
set -u

if [[ ! -x "${AGENT_BIN}" ]]; then
  echo "警告: 未找到 ${AGENT_BIN}，请先运行 scripts/20_build_micro_xrce_dds.sh" >&2
else
  echo "==> 启动 MicroXRCEAgent ${AGENT_ARGS[*]}"
  "${AGENT_BIN}" "${AGENT_ARGS[@]}" > "${ROOT_DIR}/build/xrce_agent.log" 2>&1 &
  AGENT_PID=$!
  trap 'kill ${AGENT_PID} 2>/dev/null || true' EXIT
fi

sleep 1
if [[ "${USE_GZ}" == "1" ]]; then
  echo "==> 启动 Gazebo <-> ROS 2 桥"
  ros2 launch uav_bringup gz_bridge.launch.py &
fi

if [[ ${WITH_RVIZ} -eq 1 ]]; then
  sleep 2
  echo "==> 启动 RViz2"
  rviz2 -d "${ROOT_DIR}/ros2_ws/src/uav_bringup/config/uav_sim.rviz" &
fi

wait
