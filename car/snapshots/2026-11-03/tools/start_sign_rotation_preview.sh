#!/usr/bin/env bash
# Read-only Sign orientation and geometry. No MCU / UDP / motor outputs.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$task_root/candidates/wand_rotation_preview/main.py" "$@"
