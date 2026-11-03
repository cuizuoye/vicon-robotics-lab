#!/usr/bin/env bash
set -euo pipefail
task_adaptive_entry=/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_adaptive_servo_20261030/entry.py
if [[ "${1:-}" == --check ]]; then exec /usr/bin/python3 -B "$task_adaptive_entry" --check; fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -B "$task_adaptive_entry" "$@"
