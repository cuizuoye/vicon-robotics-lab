#!/usr/bin/env bash
# Independent candidate. Read-only X11 probe does not construct any ROS node.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_candidate="$task_root/docs/wand_remote_startup_v2_2026-10-27/prepared"
if [ "$#" -eq 0 ]; then set -- --check; fi
source "$task_root/candidates/wand_tracking_autostart/display_environment.sh"
configure_tracking_display "$@"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_candidate/runtime/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
if [ "$1" = --x11-check ]; then
    shift
    export WAND_REMOTE_X11=1
    exec python3 "$task_candidate/remote_probe.py" "$@"
fi
exec python3 "$task_candidate/oneclick_app.py" "$@"
