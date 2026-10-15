#!/usr/bin/env bash
# Explicit opt-in network downloads, isolated to this checkout. No sudo/OS changes.
set -euo pipefail
handoff_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
handoff_python=${HANDOFF_PYTHON:-/usr/bin/python3}
"$handoff_python" -c 'import sys; assert sys.version_info[:2] == (3,12), "Use Python 3.12 for ROS Jazzy compatibility"'
mkdir -p "$handoff_root/.local"
if [[ ! -x "$handoff_root/.local/venv/bin/python" ]]; then
    "$handoff_python" -m venv --system-site-packages "$handoff_root/.local/venv"
fi
"$handoff_root/.local/venv/bin/python" -m pip install -r "$handoff_root/scripts/requirements-offline.txt"
printf 'Local Python environment ready. Run bash scripts/test_offline.sh\n'
