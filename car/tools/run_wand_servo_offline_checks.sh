#!/usr/bin/env bash
# Synthetic inputs only. No ROS nodes, Vicon, socket, serial or firmware tools.
set -eo pipefail
if [[ $# -gt 0 ]]; then
  echo 'Usage: bash tools/run_wand_servo_offline_checks.sh (OFFLINE ONLY)' >&2
  exit 2
fi
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_artifacts=$task_root/docs/wand_low_latency_servo_2026-10-23
task_runtime=$task_artifacts/verified_runtime
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_runtime/install/local_setup.bash"
set -u
for task_module in continuous_servo.py servo_config.py offline_validation.py; do
  cmp "$task_root/candidates/wand_low_latency_servo/$task_module" \
      "$task_runtime/src/mecanum_wand_servo_control/mecanum_wand_servo_control/$task_module"
done
task_evidence=$(mktemp -d /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_low_latency_servo_2026-10-23/check_XXXXXX)
export SERVO_GENERATED_ROOT=$task_runtime/src
echo "OFFLINE ONLY; synthetic inputs; evidence: $task_evidence"
python3 -m pytest -q candidates/wand_low_latency_servo/test_continuous_servo.py \
    --junitxml="$task_evidence/tests.xml"
python3 -m mecanum_wand_servo_control.offline_validation \
    --profile "$task_runtime/src/mecanum_wand_servo_hardware/config/servo.3x3_1to1.OFFLINE_ONLY.json" \
    --output "$task_evidence/comparison"
