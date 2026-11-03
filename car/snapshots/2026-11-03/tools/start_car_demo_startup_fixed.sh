#!/usr/bin/env bash
# Original XY hardware demo, fixed startup preparation. NEW human click for motion.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_candidate="$task_root/docs/wand_demo_startup_fixed_2026-10-26/prepared"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_candidate/runtime/install/local_setup.bash"
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$task_candidate/oneclick_app.py" "$@"
