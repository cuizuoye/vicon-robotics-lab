#!/usr/bin/env bash
# Independent offline STOPFIX; no hardware launch path.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
fi
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_stopfix_20261102/ground_entry.py "$@"
