#!/usr/bin/env bash
# =============================================================================
# 一键演示: ROS 2 -> PX4 起飞 / 悬停 / 降落
#
# 运行内容: offboard 位置控制，升到 3m，悬停 6s，再 AUTO_LAND 降落并自动上锁
#
# 前提（三个终端都就绪）:
#   1) PX4 SITL + Gazebo         : bash scripts/40_run_sitl.sh
#   2) 地面站连接(否则不能解锁)  : python3 scripts/91_fake_gcs_heartbeat.py --port 18570
#                                  或开 QGroundControl（UDP 端口填 18570）
#   3) Agent + 桥                : bash scripts/41_run_ros2_bridge.sh
#                                  （只跑本脚本也可以，但桥能让你同时看 RViz）
#
# 用法:
#   bash scripts/43_run_takeoff_land_demo.sh
#   HEIGHT=2.0 HOLD=10.0 bash scripts/43_run_takeoff_land_demo.sh
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HEIGHT="${HEIGHT:-3.0}"
HOLD="${HOLD:-6.0}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

echo "==> 起飞 ${HEIGHT} m -> 悬停 ${HOLD} s -> 自动降落"
exec ros2 run uav_coord_bridge offboard_takeoff_land --ros-args \
  -p takeoff_height:="${HEIGHT}" -p hold_seconds:="${HOLD}"
