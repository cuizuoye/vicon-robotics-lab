#!/usr/bin/env bash
# =============================================================================
# 环境自检: 逐项确认仿真/飞控开发所需组件是否就绪
# 用法:  bash scripts/99_verify_env.sh
# =============================================================================
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO_NAME="${ROS_DISTRO:-jazzy}"
PASS=0
FAIL=0

ok()   { printf "  [ OK ] %s\n" "$1"; PASS=$((PASS + 1)); }
bad()  { printf "  [ !! ] %s\n" "$1"; FAIL=$((FAIL + 1)); }
warn() { printf "  [ ?? ] %s\n" "$1"; }
section() { printf "\n== %s ==\n" "$1"; }

section "ROS 2"
if [[ -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]]; then
  ok "ROS 2 ${ROS_DISTRO_NAME} 已安装 (/opt/ros/${ROS_DISTRO_NAME})"
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
else
  bad "未找到 /opt/ros/${ROS_DISTRO_NAME}"
fi

section "Gazebo (Harmonic / gz-sim8)"
GZ_VENDOR_BIN="/opt/ros/${ROS_DISTRO_NAME}/opt/gz_sim_vendor/bin"
GZ_BIN=""
if command -v gz >/dev/null 2>&1; then
  GZ_BIN="$(command -v gz)"
else
  for GZ_VENDOR_BIN in \
    "/opt/ros/${ROS_DISTRO_NAME}/opt/gz_tools_vendor/bin" \
    "/opt/ros/${ROS_DISTRO_NAME}/opt/gz_sim_vendor/bin"; do
    if [[ -x "${GZ_VENDOR_BIN}/gz" ]]; then
      GZ_BIN="${GZ_VENDOR_BIN}/gz"
      export PATH="${GZ_VENDOR_BIN}:${PATH}"
      break
    fi
  done
fi

if [[ -n "${GZ_BIN}" ]]; then
  ok "gz 可执行文件: ${GZ_BIN}"
  "${GZ_BIN}" sim --version 2>&1 | head -2 | sed 's/^/        /'
else
  bad "未找到 gz 命令（ros_gz vendor 包或 gz-harmonic 未安装）"
fi

if [[ -d "/opt/ros/${ROS_DISTRO_NAME}/opt/gz_sim_vendor" ]]; then
  ok "ros_gz vendor 版 Gazebo 已安装"
else
  warn "未见 gz_sim_vendor；若用系统版 gz-harmonic 可忽略"
fi

section "ros_gz 桥"
for exe in ros_gz_bridge ros_gz_sim; do
  if [[ -n "$(ros2 pkg prefix ${exe} 2>/dev/null)" ]]; then
    ok "ROS 包 ${exe} 可用"
  else
    bad "缺少 ROS 包 ${exe}（apt install ros-${ROS_DISTRO_NAME}-${exe//_/-}）"
  fi
done

section "uXRCE-DDS Agent (PX4 <-> ROS 2)"
if [[ -x "${ROOT_DIR}/build/bin/MicroXRCEAgent" ]]; then
  ok "已编译: build/bin/MicroXRCEAgent"
elif command -v MicroXRCEAgent >/dev/null 2>&1; then
  ok "系统已安装: $(command -v MicroXRCEAgent)"
else
  bad "缺 MicroXRCEAgent（bash scripts/20_build_micro_xrce_dds.sh）"
fi

section "ROS 2 工作区"
if [[ -f "${ROOT_DIR}/ros2_ws/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${ROOT_DIR}/ros2_ws/install/setup.bash"
  set -u
  for pkg in px4_msgs uav_bringup uav_coord_bridge; do
    if [[ -n "$(ros2 pkg prefix ${pkg} 2>/dev/null)" ]]; then
      ok "ROS 包 ${pkg} 已编译"
    else
      bad "ROS 包 ${pkg} 未编译（bash scripts/30_build_ros2_ws.sh）"
    fi
  done
else
  bad "工作区未编译（bash scripts/30_build_ros2_ws.sh）"
fi

section "PX4 源码"
if [[ -f "${ROOT_DIR}/px4_firmware/Makefile" ]]; then
  ok "px4_firmware 存在 ($(cd "${ROOT_DIR}/px4_firmware" && git describe --tags --always 2>/dev/null || echo 'unknown'))"
  if git -C "${ROOT_DIR}/px4_firmware" submodule status --recursive >/dev/null 2>&1; then
    if [[ -z "$(git -C "${ROOT_DIR}/px4_firmware" submodule status --recursive | grep '^-' || true)" ]]; then
      ok "子模块已就绪"
    else
      bad "子模块未完整拉取（git submodule update --init --recursive --depth 1）"
    fi
  fi
else
  bad "缺少 px4_firmware"
fi

section "编译/烧录工具链"
if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  ok "arm-none-eabi-gcc: $(arm-none-eabi-gcc -dumpversion)"
else
  bad "缺少 gcc-arm-none-eabi（无法编译飞控固件）"
fi
for tool in cmake ninja ccache python3; do
  if command -v "${tool}" >/dev/null 2>&1; then
    ok "${tool}: $(command -v ${tool})"
  else
    bad "缺少 ${tool}"
  fi
done

section "PX4 构建依赖 (Python)"
python3 - <<'PY' || true
import importlib
# 注意: Debian/Ubuntu 的 python3-empy 提供的模块名是 em（不是 empy）
mods = ['kconfiglib', 'jinja2', 'yaml', 'toml', 'numpy', 'serial', 'packaging', 'em', 'jsonschema',
        'nunavut', 'pymavlink', 'pyulog']
missing = []
for m in mods:
    try:
        importlib.import_module(m)
        print(f"  [ OK ] python 模块 {m}")
    except Exception as exc:
        missing.append(m)
        print(f"  [ !! ] python 模块 {m} 缺失 ({exc.__class__.__name__})")
print(f"  -> 缺失 {len(missing)} 个" if missing else "  -> 全部就绪")
PY

section "串口/烧录权限"
if id -nG "${USER}" | tr ' ' '\n' | grep -qx dialout; then
  ok "用户 ${USER} 在 dialout 组（可访问飞控串口）"
else
  bad "用户 ${USER} 不在 dialout 组（sudo usermod -aG dialout ${USER} 后重新登录）"
fi

section "磁盘"
AVAIL_GB="$(df --output=avail -BG / | tail -1 | tr -dc '0-9')"
if [[ "${AVAIL_GB}" -gt 25 ]]; then
  ok "根分区可用 ${AVAIL_GB}G（PX4 编译约需 10~20G）"
else
  warn "根分区仅剩 ${AVAIL_GB}G，编译固件可能空间不足"
fi

printf "\n== 结果: %d 项通过, %d 项待处理 ==\n" "${PASS}" "${FAIL}"
exit 0
