#!/usr/bin/env bash
# =============================================================================
# 编译 ROS 2 工作区 (px4_msgs / px4_ros_com / uav_bringup)
#
# 用法:  bash scripts/30_build_ros2_ws.sh
# 产物:  ros2_ws/install/setup.bash
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="${ROOT_DIR}/ros2_ws"
ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

cd "${WS_DIR}"

# px4_msgs 是 px4_ros_com 和后续自研节点的消息依赖，必须同一批编译
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --packages-up-to px4_msgs px4_ros_com uav_bringup 2>/dev/null \
  || colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "==> 完成。使用前:  source ${WS_DIR}/install/setup.bash"
