#!/usr/bin/env bash
# =============================================================================
# 安装 PX4 构建所需的 Python 依赖 (px4_firmware/Tools/setup/requirements.txt)
#
# 不要用 sudo 运行: 依赖装到当前用户的 ~/.local，
#    sudo 会把依赖装到 /root/.local 导致后面仍然找不到。
#
# 注意: PX4 的 requirements.txt 会连带升级 numpy / setuptools，
#   而 ROS 2 Jazzy 依赖 numpy 1.x（scipy 等）与 setuptools<80（colcon），
#   所以这里用约束把它们钉住，避免 ~/.local 覆盖系统版本后把 ROS 环境搞坏。
#
# 用法:  bash scripts/21_install_px4_python_deps.sh
# =============================================================================
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REQ="${ROOT_DIR}/px4_firmware/Tools/setup/requirements.txt"

if [[ ${EUID} -eq 0 ]]; then
  echo "请不要用 root/sudo 运行本脚本（依赖要装到普通用户目录）。" >&2
  exit 1
fi

if [[ ! -f "${REQ}" ]]; then
  echo "找不到 ${REQ}" >&2
  exit 1
fi

CONSTRAINTS="${ROOT_DIR}/scripts/pip-constraints.txt"

echo "==> 使用 ${REQ} 安装 Python 依赖到 ~/.local（约束: ${CONSTRAINTS}）"
python3 -m pip install --user --break-system-packages \
  --upgrade-strategy only-if-needed \
  --constraint "${CONSTRAINTS}" \
  -r "${REQ}"

echo "==> 完成。检查:"
python3 -c "import numpy, setuptools; print(f'    numpy {numpy.__version__} / setuptools {setuptools.__version__}')"
