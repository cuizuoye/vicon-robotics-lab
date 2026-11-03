#!/usr/bin/env bash
# New file-only diagnostic/candidate check, never a motion entry.
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 /home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_latency_fast_follow_20261029/check.py "$@"
