#!/usr/bin/env bash
# Default file-only check. Independent entry; never programs firmware.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_ready=$task_root/docs/wand_servo_4ws8_2026-10-23/ready
task_cli=$task_root/stm32/mecanum_f407/candidates/servo_4ws8/launch_cli.py
if [[ ${1:-} != --launch ]]; then
  exec python3 "$task_cli" --manifest "$task_ready/build_manifest.json" --purpose sign "$@"
fi
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_ready/runtime/install/local_setup.bash"
exec python3 "$task_cli" --manifest "$task_ready/build_manifest.json" --purpose sign "$@"
