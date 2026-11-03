#!/usr/bin/env bash
# Explicit, independent stage0 ground commissioning; never flash or change Wi-Fi.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --launch "* || " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
fi
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_4wsc_ground_20261102/ground_entry.py "$@"
