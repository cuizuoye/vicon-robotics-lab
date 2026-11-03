#!/usr/bin/env bash
# Offline source tests only; no archived launcher or deployment generator.
set -euo pipefail
update_repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
update_out="$update_repo/.local/car-20261103"
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi
export PYTHONDONTWRITEBYTECODE=1
mkdir -p "$update_out"
cmake -S "$update_repo/scripts/car_4wsc_native" -B "$update_out/native" -G Ninja
cmake --build "$update_out/native" -j2
ctest --test-dir "$update_out/native" --output-on-failure --output-junit "$update_out/native.xml"
/usr/bin/python3 "$update_repo/scripts/test_car_update_20261103.py"
printf 'PASS: 4WSC native safety suites and static-source units; no hardware/ROS motion output.\n'
