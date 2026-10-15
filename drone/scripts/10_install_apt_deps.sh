#!/usr/bin/env bash
# =============================================================================
# 四旋翼仿真环境 —— 系统依赖安装（需要 root 权限）
#
# 用法:   sudo bash scripts/10_install_apt_deps.sh
# 平台:   Ubuntu 24.04 (noble) + ROS 2 Jazzy
#
# 安装内容:
#   1. Gazebo (Harmonic, 通过 ROS vendor 包) + ros_gz 桥
#   2. PX4 编译/仿真依赖 (cmake/ninja/px4 python 依赖/gstreamer/geographiclib...)
#   3. ARM 交叉编译链 (用于把固件编译成 .px4 并烧录到飞控)
#   4. QGroundControl 运行依赖 + 串口权限组
# =============================================================================
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "请使用 root 运行:  sudo bash $0" >&2
  exit 1
fi

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-jazzy}"
TARGET_USER="${SUDO_USER:-$(logname 2>/dev/null || echo "")}"

echo "==> [1/5] apt update"
apt-get update

echo "==> [2/5] Gazebo Harmonic + ros_gz (ROS 2 <-> Gazebo 桥)"
# ROS 2 Jazzy 的 ros_gz 使用 gz vendor 包自带 Gazebo Harmonic (gz-sim8)，
# 这样 Gazebo 与桥接插件来自同一套构建，避免版本/ABI 混用问题。
apt-get install -y \
  ros-${ROS_DISTRO_NAME}-ros-gz \
  ros-${ROS_DISTRO_NAME}-ros-gz-sim \
  ros-${ROS_DISTRO_NAME}-ros-gz-bridge \
  ros-${ROS_DISTRO_NAME}-ros-gz-image \
  ros-${ROS_DISTRO_NAME}-ros-gz-interfaces

echo "==> [3/5] ROS 2 补充包 (机器人模型 / 手柄 / 可视化)"
apt-get install -y \
  ros-${ROS_DISTRO_NAME}-xacro \
  ros-${ROS_DISTRO_NAME}-joint-state-publisher \
  ros-${ROS_DISTRO_NAME}-robot-state-publisher \
  ros-${ROS_DISTRO_NAME}-joy \
  ros-${ROS_DISTRO_NAME}-teleop-twist-joy \
  ros-${ROS_DISTRO_NAME}-tf2-tools \
  ros-${ROS_DISTRO_NAME}-tf-transformations

echo "==> [4/5] PX4 编译 + ARM 烧录工具链"
apt-get install -y \
  build-essential gcc g++ gcc-multilib g++-multilib make cmake ninja-build ccache git \
  gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi \
  libstdc++-arm-none-eabi-newlib binutils-dev \
  gdb gdb-multiarch lldb lld openocd minicom screen \
  python3-pip python3-venv python3-dev python3-setuptools \
  python3-wheel python3-full \
  python3-serial python3-packaging python3-yaml python3-toml python3-numpy \
  python3-jinja2 python3-jsonschema python3-kconfiglib python3-future python3-empy \
  python3-geographiclib python3-lxml python3-pytest \
  python3-lark python3-matplotlib python3-pandas python3-psutil python3-sympy \
  python3-pygments python3-argcomplete python3-cerberus python3-pycryptodome \
  protobuf-compiler libeigen3-dev libxml2-dev libxml2-utils \
  geographiclib-tools libgeographiclib-dev \
  genromfs kconfig-frontends u-boot-tools cppzmq-dev \
  libssl-dev libunwind-dev libelf-dev libncurses-dev pkg-config \
  libimage-exiftool-perl dmidecode \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-alsa \
  astyle cppcheck shellcheck lcov file bc rsync unzip zip

echo "==> [5/5] QGroundControl 运行依赖 + 串口/USB 权限"
# Ubuntu 24.04 把 libfuse2 改名为 libfuse2t64（64 位 time_t 迁移），这里自动选
if apt-cache policy libfuse2t64 2>/dev/null | grep -q 'Candidate: [0-9]'; then
  FUSE_PKG=libfuse2t64
else
  FUSE_PKG=libfuse2
fi
apt-get install -y \
  "${FUSE_PKG}" libxcb-xinerama0 libxkbcommon-x11-0 libxcb-cursor0 \
  libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-randr0 \
  libxcb-render-util0 libxcb-shape0 libx11-xcb1 libgl1-mesa-dri \
  libsdl2-2.0-0 libsdl2-image-2.0-0 \
  modemmanager

if [[ -n "${TARGET_USER}" && -n "$(getent passwd "${TARGET_USER}")" ]]; then
  usermod -aG dialout,plugdev,video "${TARGET_USER}" || true
  echo "    已把用户 ${TARGET_USER} 加入 dialout / plugdev / video 组（重新登录后生效）"
fi

# 用完即走：关掉 ModemManager 对飞控串口的抢占（PX4/QGC 常见连接失败原因）
systemctl disable --now ModemManager 2>/dev/null || true

echo
echo "==> 完成。下一步："
echo "    1) 重新登录（使用户组生效），然后:  source /opt/ros/${ROS_DISTRO_NAME}/setup.bash"
echo "    2) 安装 PX4 的 Python 依赖:  bash scripts/21_install_px4_python_deps.sh"
echo "    3) 自检:               bash scripts/99_verify_env.sh"
echo "    4) 编译 ROS 2 工作区:  bash scripts/30_build_ros2_ws.sh"
echo "    5) 编译飞控固件:       bash scripts/50_build_firmware.sh"
