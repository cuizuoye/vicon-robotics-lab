#!/usr/bin/env bash
# Independent SOFTWARE ONLY entry; never calls the real-car launcher.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=191 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_sim12_20261030/entry12.py "$@"
