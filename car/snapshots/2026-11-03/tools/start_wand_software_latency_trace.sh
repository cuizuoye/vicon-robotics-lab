#!/usr/bin/env bash
# Independent SOFTWARE-only profiler; no hardware/network setting mutations.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONDONTWRITEBYTECODE=1
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_software_latency_trace_20261029/console.py "$@"
