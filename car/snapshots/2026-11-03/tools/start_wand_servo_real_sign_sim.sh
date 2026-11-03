#!/usr/bin/env bash
# Real Sign input is read-only. All actuation is SOFTWARE ONLY in domain114.
set -eo pipefail
task_start=corner
if [[ $# -eq 1 && "$1" == --center-start ]]; then
  task_start=center
elif [[ $# -ne 0 ]]; then
  echo 'Usage: bash tools/start_wand_servo_real_sign_sim.sh [--center-start] (SIM ONLY)' >&2
  exit 2
fi
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/docs/wand_low_latency_servo_2026-10-23/verified_runtime/install/local_setup.bash"
task_runtime=$task_root/docs/wand_low_latency_servo_2026-10-23/verified_runtime
for task_module in continuous_servo.py servo_config.py; do
  cmp "$task_root/candidates/wand_low_latency_servo/$task_module" \
      "$task_runtime/src/mecanum_wand_servo_control/mecanum_wand_servo_control/$task_module"
done
task_parent=$(mktemp -d /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/live_sign_XXXXXX)
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec python3 candidates/wand_low_latency_servo/live_sign_sim.py --output "$task_parent/session" --sim-start "$task_start"
