#!/usr/bin/env bash
# Accepted bounded-demo entry. Default: file-only check, never enables the car.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_ready=$task_root/docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready
task_action=${1:---check}
if [[ $# -gt 1 || ( "$task_action" != --check && "$task_action" != --launch ) ]]; then
  printf 'Usage: bash %s [--check|--launch]\n' "$0" >&2
  exit 2
fi
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_ready/runtime/install/local_setup.bash"
if [[ "$task_action" == --check ]]; then
  exec python3 "$task_ready/launch_demo.py" --manifest "$task_ready/build_manifest.json"
fi
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
# Fresh physical-site confirmation is required by the launcher every time.
# Actual motion still requires the human's held Space key in the console.
exec python3 "$task_ready/launch_demo.py" --manifest "$task_ready/build_manifest.json" \
  --flash-record "$task_root/stm32/mecanum_f407/backups/pre_4ws8_staged_20261023_1q2wgy/POST_FLASH_RECORD.json" --launch
