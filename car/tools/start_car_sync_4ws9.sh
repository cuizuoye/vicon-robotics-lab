#!/usr/bin/env bash
# New-image staged candidate. No arguments = file check, NOT a hardware start.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
task_prepared="$task_root/docs/wand_sync_4ws9_2026-10-27/prepared"
source "$task_prepared/runtime/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1
exec python3 "$task_root/candidates/wand_sync_4ws9/cli.py" "$@"
