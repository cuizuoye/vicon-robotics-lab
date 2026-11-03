#!/usr/bin/env bash
set -euo pipefail
entry=/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_browser_direct_20261030/entry.py
if [[ "${1:-}" == "--check" ]]; then
    exec /usr/bin/python3 -B "$entry" --check
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=191
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -B "$entry" "$@"
