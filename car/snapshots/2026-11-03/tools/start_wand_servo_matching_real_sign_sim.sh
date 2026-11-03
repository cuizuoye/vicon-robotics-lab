#!/usr/bin/env bash
# Separate .70m/s / 2m/s2 REAL Sign -> SOFTWARE CAR ONLY, fixed domain115.
set -eo pipefail
if [[ $# -ne 0 ]]; then
  echo 'Usage: bash tools/start_wand_servo_matching_real_sign_sim.sh (SIM ONLY, center start)' >&2
  exit 2
fi
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/docs/wand_low_latency_servo_2026-10-23/verified_runtime/install/local_setup.bash"
task_runtime=$task_root/docs/wand_servo_matching_sim_2026-10-23/live_runtime_v1
export PYTHONPATH="$task_root/candidates/wand_servo_matching_sim${PYTHONPATH:+:$PYTHONPATH}"
python3 - "$task_runtime" <<'PY'
import hashlib,json,sys
from pathlib import Path
root=Path(sys.argv[1]);report=json.loads((root/'generation.json').read_text())
for name,sha in report['inputs_sha256'].items():
    if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=sha:raise SystemExit('SIM adapter input drift: '+name)
if hashlib.sha256((root/'live_matching_sim.py').read_bytes()).hexdigest()!=report['output_sha256']:
    raise SystemExit('SIM adapter generated code drift')
PY
task_parent=$(mktemp -d "$task_root/docs/wand_servo_matching_sim_2026-10-23/real_sign_XXXXXX")
export DISPLAY=${DISPLAY:-:1}
export XAUTHORITY=${XAUTHORITY:-/run/user/1000/gdm/Xauthority}
exec python3 "$task_runtime/live_matching_sim.py" --output "$task_parent/session" --sim-start center
