#!/usr/bin/env bash
# Source-intake repair; old standalone/rectangle entries remain frozen.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --launch "* || " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
fi
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_static_source_20261102/startup_entry.py "$@"
