#!/usr/bin/env bash
# Independent persistent startup page; default/--check is file-only.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --launch "* || " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
fi
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_tracker_startup_fix_20261102/startup_entry.py "$@"
