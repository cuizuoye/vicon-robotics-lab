#!/usr/bin/env bash
# =============================================================================
# 编译 eProsima Micro XRCE-DDS Agent
#
# 作用: PX4 与 ROS 2 之间的通信桥梁（uXRCE-DDS）。
#       PX4 内部跑 xrce_client <-> 本 Agent <-> ROS 2 DDS，实现 /fmu/in 与
#       /fmu/out 上的 px4_msgs 话题。
#
# 版本说明(关键): PX4 v1.17 固件里的 Micro-XRCE-DDS **Client 是 v2.x**，
#   与 Agent v3.x 不兼容（见 px4_firmware/docs/en/middleware/uxrce_dds.md），
#   ROS 2 Jazzy 对应 Agent v2.4.3；链接系统 Fast DDS 2.14 / Fast CDR / spdlog，
#   这样 Agent 与 ROS 2 用的是同一套中间件。
#
# 用法:
#   bash scripts/20_build_micro_xrce_dds.sh                # 默认：快速、离线
#   AGENT_WITH_LOGS=1 bash scripts/20_build_micro_xrce_dds.sh   # 带日志（官方 superbuild，需联网）
#
# 关于日志: Agent v2.4.3 的日志模块依赖 spdlog 1.9/fmt 8 的隐式 ostream 格式化，
#   Ubuntu 24.04 自带的是 spdlog 1.12 + fmt 9，两者的组合编译不过（fmt 9 需要显式
#   formatter）。因此默认关闭日志模块（UXR_AGENT_LOG_* 变成空操作），
#   想看到 Agent 的连接/建话题日志就用 AGENT_WITH_LOGS=1 走 superbuild。
#   注意: superbuild 会从 GitHub 下载并编译 Fast DDS + spdlog，耗时较长(约 10 分钟)。
#
# 产物:  build/bin/MicroXRCEAgent
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${ROOT_DIR}/Micro-XRCE-DDS-Agent"
BUILD_DIR="${ROOT_DIR}/build/Micro-XRCE-DDS-Agent"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"

if [[ ! -d "${SRC_DIR}" ]]; then
  echo "找不到源码目录 ${SRC_DIR}，请先克隆 Micro-XRCE-DDS-Agent" >&2
  exit 1
fi

# 固定到 v2.4.3（与 PX4 v1.17 的客户端兼容）
CURRENT_TAG="$(git -C "${SRC_DIR}" describe --tags --exact-match 2>/dev/null || echo '')"
if [[ "${CURRENT_TAG}" != "v2.4.3" ]]; then
  echo "==> 切换 Micro-XRCE-DDS-Agent 到 v2.4.3（当前: ${CURRENT_TAG:-unknown}）"
  git -C "${SRC_DIR}" fetch --depth 1 origin tag v2.4.3
  if [[ -n "$(git -C "${SRC_DIR}" status --porcelain)" ]]; then
    echo "Agent源码有本地修改，拒绝切换版本；请先保留成果。" >&2
    exit 2
  fi
  git -C "${SRC_DIR}" checkout --detach v2.4.3
fi

# ROS 的 setup.bash 会引用未定义变量，先关掉 -u
set +u
# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

if [[ "${AGENT_WITH_LOGS:-0}" == "1" ]]; then
  echo "==> superbuild 模式（含日志，需联网）"
  cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUAGENT_SUPERBUILD=ON \
    -DUAGENT_BUILD_EXECUTABLE=ON \
    -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/build/micro_xrce_dds_install"
else
  echo "==> 快速模式（关闭日志，链接 ROS 2 的 Fast DDS / Fast CDR）"
  cmake -S "${SRC_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DUAGENT_SUPERBUILD=OFF \
    -DUAGENT_USE_SYSTEM_FASTDDS=ON \
    -DUAGENT_USE_SYSTEM_FASTCDR=ON \
    -DUAGENT_USE_SYSTEM_LOGGER=OFF \
    -DUAGENT_LOGGER_PROFILE=OFF \
    -DUAGENT_BUILD_EXECUTABLE=ON \
    -DUAGENT_P2P_PROFILE=OFF \
    -DUAGENT_SOCKETCAN_PROFILE=OFF \
    -DUAGENT_CED_PROFILE=OFF \
    -DCMAKE_INSTALL_PREFIX="${ROOT_DIR}/build/micro_xrce_dds_install"
fi

cmake --build "${BUILD_DIR}" -j"$(nproc)"

mkdir -p "${ROOT_DIR}/build/bin"
cp -f "${BUILD_DIR}/MicroXRCEAgent" "${ROOT_DIR}/build/bin/MicroXRCEAgent"

echo "==> 完成: ${ROOT_DIR}/build/bin/MicroXRCEAgent"
"${ROOT_DIR}/build/bin/MicroXRCEAgent" -h 2>&1 | head -6
