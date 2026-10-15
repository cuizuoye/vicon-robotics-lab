#!/usr/bin/env bash
# =============================================================================
# ROS 2 -> PX4 坐标指令下发演示（官方 px4_ros_com offboard 例程）
#
# 前提: PX4 SITL 已运行(40_run_sitl.sh)、MicroXRCEAgent 已运行(41_run_ros2_bridge.sh)
#       并且 pre_flight_checks_pass 必须为 true，否则解锁会被拒、飞机不动：
#         - 开 QGroundControl（PX4 v1.17 的 SITL 地面站端口是 18570），或
#         - 在 PX4 终端执行 param set NAV_DLL_ACT 0，或
#         - python3 scripts/91_fake_gcs_heartbeat.py --port 18570 --seconds 120
#       详见 docs/SETUP.md 3.2 节。
# 现象: 飞控起飞后按脚本里的轨迹飞行，说明 ROS 2 坐标系指令成功传到 PX4。
# 用法: bash scripts/42_run_offboard_demo.sh
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

echo "==> 运行 offboard_control（Ctrl+C 停止）"
exec ros2 run px4_ros_com offboard_control
