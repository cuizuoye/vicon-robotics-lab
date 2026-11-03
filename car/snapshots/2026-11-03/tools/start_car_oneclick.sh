#!/usr/bin/env bash
# A single command opens the novice UI. Nothing automatically enables motion.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/docs/wand_servo_4ws8_2026-10-26/oneclick_v1/ready/runtime/install/local_setup.bash"
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec python3 "$task_root/candidates/wand_oneclick_v1/oneclick_app.py" "$@"
