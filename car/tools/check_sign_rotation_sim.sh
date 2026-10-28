#!/usr/bin/env bash
# Checks saved SOFTWARE-ONLY results. This is NOT a real-car console launcher.
set -eo pipefail
export PYTHONDONTWRITEBYTECODE=1
exec python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_rotation_sim/report.py "$@"
