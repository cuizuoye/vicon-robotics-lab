#!/usr/bin/env bash
# Observation only. Never construct ROS or physical output.
set -euo pipefail
task_probe=/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_probe_2026-10-27/prepared
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --check ]]; then exec /usr/bin/python3 "$task_probe/check.py"; fi
/usr/bin/python3 "$task_probe/check.py"
task_node="$(command -v node || true)"
if [[ -z "$task_node" ]]; then task_node=/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node; fi
exec "$task_node" "$task_probe/server.cjs" "$@"
