#!/usr/bin/env bash
# Independent new-image host. No arguments: file-only check, no hardware IO.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
if [ "$#" -eq 0 ]; then set -- --check; fi
export PYTHONDONTWRITEBYTECODE=1
if [[ " $* " == *" --launch "* ]]; then
    source /opt/ros/jazzy/setup.bash
    source "$task_root/install/setup.bash"
    source "$task_root/docs/wand_4wsb_host_2026-10-30/prepared_v9/runtime/install/local_setup.bash"
fi
exec python3 "$task_root/candidates/wand_4wsb_host_20261030/cli.py" "$@"
