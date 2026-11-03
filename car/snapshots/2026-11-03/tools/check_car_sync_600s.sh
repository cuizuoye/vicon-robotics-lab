#!/usr/bin/env bash
# File check only. No ROS, Wi-Fi, serial, flash or car control is initialized.
set -eo pipefail
exec python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_sync_600s/cli.py "$@"
