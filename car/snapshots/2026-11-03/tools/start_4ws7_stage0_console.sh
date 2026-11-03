#!/usr/bin/env bash
# Default FILE CHECK ONLY. No firmware/network changes or automatic enable.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_ready=$task_root/docs/wand_4ws7_stage0_2026-10-23/host_fix_20261023/verified_v2
task_cli=$task_root/stm32/mecanum_f407/candidates/servo_4ws7_stage0/host/launch_cli.py
task_mode=${1:---check}
if [[ "$task_mode" == --check && $# -le 1 ]]; then
  exec python3 "$task_cli" --manifest "$task_ready/build_manifest.json" --purpose sign
fi
if [[ "$task_mode" != --launch || $# -ne 2 ]]; then
  echo 'Usage: bash tools/start_4ws7_stage0_console.sh [--check | --launch POST_FLASH_RECORD.json]' >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_ready/runtime/install/local_setup.bash"
exec python3 "$task_cli" --manifest "$task_ready/build_manifest.json" --purpose sign --launch --flash-record "$2"
