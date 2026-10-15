#!/usr/bin/env bash
# =============================================================================
# 在 PC 上"看到真机在飞"：把 ROS 位姿镜像到本地 Gazebo 的模型上
#
# 原理: 订阅一个位姿话题（默认 /px4/odometry_enu，ENU），通过 ros_gz 的
#       /world/<world>/set_pose 服务移动 Gazebo 里的模型。
#
# 用法:
#   bash scripts/44_run_gz_mirror.sh                 # Gazebo 已经在跑，只起 桥+镜像
#   bash scripts/44_run_gz_mirror.sh --with-gazebo   # 顺带起空世界并放入 x500 模型
#   bash scripts/44_run_gz_mirror.sh --with-gazebo --headless   # 不开 GUI（测试用）
#
# 可调环境变量:
#   POSE_TOPIC=/px4/odometry_enu   话题名（也可以填 /vicon/<刚体>/<段>）
#   POSE_TYPE=odometry             odometry | pose_stamped
#   ENTITY=x500_0                  Gazebo 里的模型名
#   WORLD=default                  世界名
#
# 注意: 位姿话题由机载电脑通过 ROS 2 网络发布，PC 必须在同一个 ROS_DOMAIN_ID。
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
POSE_TOPIC="${POSE_TOPIC:-/px4/odometry_enu}"
POSE_TYPE="${POSE_TYPE:-odometry}"
ENTITY="${ENTITY:-x500_0}"
WORLD="${WORLD:-default}"
WITH_GAZEBO=0
HEADLESS=0

for arg in "$@"; do
  case "${arg}" in
    --with-gazebo) WITH_GAZEBO=1 ;;
    --headless)    HEADLESS=1 ;;
    *) echo "未知参数: ${arg}" >&2; exit 1 ;;
  esac
done

MODEL_STORE="${HOME}/.simulation-gazebo"
export GZ_SIM_RESOURCE_PATH="${MODEL_STORE}/models:${GZ_SIM_RESOURCE_PATH:-}"

set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
# shellcheck disable=SC1091
source "${ROOT_DIR}/ros2_ws/install/setup.bash"
set -u

PIDS=()
cleanup() {
  echo
  echo "==> 收尾"
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

if [[ "${WITH_GAZEBO}" == "1" ]]; then
  WORLD_FILE="${MODEL_STORE}/worlds/${WORLD}.sdf"
  if [[ ! -f "${WORLD_FILE}" ]]; then
    echo "找不到世界文件 ${WORLD_FILE}（先用 scripts/40_run_sitl.sh 跑过一次生成模型库）" >&2
    exit 1
  fi
  echo "==> 启动 Gazebo 世界 ${WORLD}"
  if [[ "${HEADLESS}" == "1" ]]; then
    gz sim -r -s -v1 "${WORLD_FILE}" > /tmp/gz_mirror_server.log 2>&1 &
  else
    gz sim -r -v1 "${WORLD_FILE}" > /tmp/gz_mirror_server.log 2>&1 &
  fi
  PIDS+=($!)
  sleep 8
  echo "==> 放入模型 ${ENTITY}"
  ros2 run ros_gz_sim create -world "${WORLD}" -name "${ENTITY}" \
    -file "${MODEL_STORE}/models/x500/model.sdf" 2>&1 | tail -1
  sleep 2
fi

echo "==> 桥出 set_pose 服务（注意要写满 4 段，含 gz 响应类型）"
ros2 run ros_gz_bridge parameter_bridge \
  "/world/${WORLD}/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean" \
  > /tmp/gz_mirror_bridge.log 2>&1 &
PIDS+=($!)
sleep 4

echo "==> 启动镜像节点: ${POSE_TOPIC} (${POSE_TYPE}) -> ${ENTITY}"
ros2 run uav_coord_bridge gz_pose_mirror --ros-args \
  -p pose_topic:="${POSE_TOPIC}" \
  -p pose_type:="${POSE_TYPE}" \
  -p entity_name:="${ENTITY}" \
  -p service_name:="/world/${WORLD}/set_pose" \
  -p relative_to_first:="${RELATIVE_TO_FIRST:-true}" &
PIDS+=($!)

echo
echo "==> 已启动。真机（或仿真）动起来时，Gazebo 里的 ${ENTITY} 会跟着动。"
echo "    Ctrl+C 停止。"
wait
