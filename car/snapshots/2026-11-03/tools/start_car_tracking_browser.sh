#!/usr/bin/env bash
# Independent browser frontend. This round has no physical-car entry.
set -euo pipefail
task_root=/home/PRIMARY_USER/vicon_mecanum_ws
task_ready="$task_root/docs/wand_browser_ssh_2026-10-27/prepared"
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --check ]]; then
    exec /usr/bin/python3 "$task_ready/check.py"
fi
/usr/bin/python3 "$task_ready/check.py"
task_mode=readonly
for task_arg in "$@"; do
    if [[ "$task_arg" == --software ]]; then task_mode=software; fi
done
if [[ "$task_mode" == software ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    source "$task_root/install/setup.bash"
    set -u
fi
task_node="$(command -v node || true)"
if [[ -z "$task_node" ]]; then
    task_node=/home/PRIMARY_USER/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node
fi
exec "$task_node" "$task_ready/server.cjs" "$@"
