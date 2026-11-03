#!/usr/bin/env bash
# SIM ONLY: distinct domain/scope; never calls the original real-car entry.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=0 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_sync_tuning_20261029/dashboard.py "$@"
