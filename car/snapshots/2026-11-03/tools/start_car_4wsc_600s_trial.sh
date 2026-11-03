#!/usr/bin/env bash
# Independent operator-requested trial, not a measured/certified long-demo release.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --launch "* || " $* " == *" --rehearsal "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source /home/PRIMARY_USER/vicon_receiver_ws/install/setup.bash
fi
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_4wsc_stage6_600s_trial_v2_20261102/ground_entry.py "$@"
