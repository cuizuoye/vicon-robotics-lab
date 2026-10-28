#!/usr/bin/env bash
# Independent 4WS8 no-fixed-fence host. No flash. Default is a file check.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_candidate="$task_root/docs/wand_tracking_workspace_2026-10-27/prepared"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_candidate/runtime/install/local_setup.bash"
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
export PYTHONDONTWRITEBYTECODE=1
if [ "$#" -eq 0 ]; then set -- --check; fi
exec python3 "$task_candidate/oneclick_app.py" "$@"
