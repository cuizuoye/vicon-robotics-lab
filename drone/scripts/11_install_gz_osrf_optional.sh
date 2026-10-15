#!/usr/bin/env bash
# =============================================================================
# 【可选】安装系统级 Gazebo Harmonic (OSRF 官方源)
#
# 什么时候需要:
#   只有当你希望把 Gazebo 作为系统独立组件使用（例如自己写世界文件、用 gazebo
#   的 GUI 工具链、或者 PX4 SITL 在编译时找不到 gz-sim8）才需要执行本脚本。
#   正常情况下 10_install_apt_deps.sh 里 ros_gz 的 vendor 包已足够。
#
# 注意: 执行后系统里会同时存在两套 Gazebo（系统版 + ROS vendor 版），
#       运行时请只启动其中一套，避免两个版本互相干扰。
#
# 用法:  sudo bash scripts/11_install_gz_osrf_optional.sh
# =============================================================================
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "请使用 root 运行:  sudo bash $0" >&2
  exit 1
fi

apt-get update
apt-get install -y curl gnupg lsb-release

curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
  -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/gazebo-stable.list

apt-get update
apt-get install -y gz-harmonic

echo "==> 完成: $(gz sim --version 2>/dev/null || echo '请重新打开终端后执行 gz sim --version 验证')"
