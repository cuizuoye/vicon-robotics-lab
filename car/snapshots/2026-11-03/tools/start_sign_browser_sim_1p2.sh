#!/usr/bin/env bash
# Real Sign -> private SIM12 only. No motor bridge or firmware interface.
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
task_browser_entry=/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_browser_sim12_20261030/entry.py
if [[ "${1:-}" == --check ]]; then exec /usr/bin/python3 "$task_browser_entry" "$@"; fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
exec /usr/bin/python3 "$task_browser_entry" "$@"
