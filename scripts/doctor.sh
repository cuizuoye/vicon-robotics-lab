#!/usr/bin/env bash
# Read-only local environment inventory. No ROS graph, device or network access.
set -euo pipefail
handoff_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
printf 'Checkout: %s\n' "$handoff_root"
git -C "$handoff_root" status --short --branch
if [[ -f /etc/os-release ]]; then . /etc/os-release; printf 'OS: %s\n' "$PRETTY_NAME"; fi
printf 'Architecture: %s\n' "$(uname -m)"
python3 --version
for executable in git gcc cmake ninja colcon arm-none-eabi-gcc idf.py; do
    if command -v "$executable" >/dev/null; then
        printf 'FOUND %s\n' "$executable"
    else
        printf 'OPTIONAL/MISSING %s\n' "$executable"
    fi
done
[[ ! -f /opt/ros/jazzy/setup.bash ]] || printf 'ROS Jazzy setup is present\n'
printf 'Hardware state is NOT inferred from software; no device was opened.\n'
