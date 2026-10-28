#!/usr/bin/env bash
# Independent candidate. No firmware, Tenda configuration or physical-car mode.
set -euo pipefail
task_ready=/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_browser_fast_2026-10-28/prepared
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --check ]]; then exec /usr/bin/python3 "$task_ready/check.py"; fi
/usr/bin/python3 "$task_ready/check.py"
task_node="$(command -v node || true)"
if [[ -z "$task_node" ]]; then task_node=/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node; fi
exec "$task_node" "$task_ready/server.cjs" "$@"
