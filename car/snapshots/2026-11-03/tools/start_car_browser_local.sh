#!/usr/bin/env bash
# Browser is the default new SIM console. --readonly retains the old display trial.
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --readonly ]]; then
    shift
    exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_local_browser_20261030/launch.py "$@"
fi
exec bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_sign_browser_sim_1p2.sh "$@"
