#!/usr/bin/env bash
# Default is DATA CHECK ONLY. --launch asks fresh site confirmation in a TTY.
# Never flashes firmware, changes networking, synthesizes Sign or presses keys.
set -eo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_mode=${1:---check}
if [[ $# -gt 1 || ( "$task_mode" != --check && "$task_mode" != --launch ) ]]; then
  echo 'Usage: bash tools/start_wand_1to1_3x3.sh [--check|--launch]' >&2
  exit 2
fi
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/install/setup.bash
source /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_one_to_one_3x3_2026-10-23/verified_runtime/install/local_setup.bash
set -u
task_review=$task_root/stm32/mecanum_f407/candidates/first_stage_4w20/host/review_arena_profile.py
python3 "$task_review"
if [[ "$task_mode" == --check ]]; then
  exit 0
fi
if [[ ! -t 0 ]]; then
  echo 'Launch requires a human terminal and fresh site confirmation.' >&2
  exit 2
fi
if pgrep -af '[w]and_acceptance_transport|[c]md_vel_udp_bridge|[r]ectangle_run_guard'; then
  echo 'An existing car control entry is running. Release/stop it and close it first.' >&2
  exit 2
fi
echo '确认：原起点和原朝向；前3m/左3m目标区及四边50cm缓冲空旷且有Vicon覆盖；有人立即断电。'
echo '车体外轮廓另计，空旷范围约4.19m×4.21m；保留20cm余量，硬停会早于缓冲外缘。'
echo '速度仍≤0.20m/s、35%PWM。未完成更高速停车/电流/温升验收。'
echo '全部确认后输入 3X3_CLEAR_TRACKED_POWER_CUT_READY；不满足则Ctrl+C取消：'
read -r task_confirmation
if [[ "$task_confirmation" != 3X3_CLEAR_TRACKED_POWER_CUT_READY ]]; then
  echo 'Not confirmed. No control entry launched.' >&2
  exit 2
fi
task_session=$(mktemp -d /home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_one_to_one_3x3_2026-10-23/session_XXXXXX)
python3 "$task_review" --output "$task_session/approved" --human-confirmation "$task_confirmation"
task_profile=$task_session/approved/site_confirmed.json
task_hash=$(sha256sum "$task_profile" | cut -d' ' -f1)
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
export ROS_DOMAIN_ID=97 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=$task_session/ros_logs
mkdir -m 700 "$ROS_LOG_DIR"
unset WAND_ONE_TO_ONE_RAW_RECEIVE_TRACE
echo "Session evidence: $task_session"
echo '操作台启动不运动；轻按并松开空格解锁→准备→就绪后按住空格移动杆。松键停车。'
exec ros2 launch mecanum_wand_one_to_one_hardware wand_one_to_one_acceptance.launch.py \
  mode:=hardware gui:=true "profile:=$task_profile" "approved_sha256:=$task_hash" \
  hardware_consent:=ONE_TO_ONE_3X3_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY
