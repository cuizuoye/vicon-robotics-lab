#!/usr/bin/env bash
# Guided auto-open/prepare. Motion requires a NEW real Space click.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_candidate="$task_root/docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_candidate/runtime/install/local_setup.bash"
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec python3 "$task_candidate/oneclick_app.py" "$@"
