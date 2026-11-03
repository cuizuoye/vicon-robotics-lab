#!/usr/bin/env bash
# Independent 4WSC host; default file check only. No firmware/network writes.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
task_ws=/home/PRIMARY_USER/vicon_mecanum_ws
if [[ " $* " == *" --launch "* || " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source "$task_ws/install/setup.bash"
fi
exec /usr/bin/python3 "$task_ws/candidates/wand_4wsc_browser_20261102/entry.py" "$@"
