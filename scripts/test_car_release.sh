#!/usr/bin/env bash
# Portable source tests only. Does not call an archived prepare/launch script.
set -euo pipefail
snapshot_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
snapshot_car="$snapshot_root/car"
snapshot_release="$snapshot_car/docs/wand_sync_600s_2026-10-28/prepared"
snapshot_out="$snapshot_root/.local/4wsa-offline"
snapshot_python=${HANDOFF_PYTHON:-/usr/bin/python3}
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi
export PUBLIC_CAR_ROOT="$snapshot_car" SYNC_OUTPUT="$snapshot_out"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=187 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$snapshot_car/src/mecanum_vicon_control:$snapshot_release/runtime/src/mecanum_wand_4wsa_control:$snapshot_release/runtime/src/mecanum_wand_4wsa_hardware:${PYTHONPATH:-}"
mkdir -p "$snapshot_out/runtime"
if [[ ! -e "$snapshot_out/runtime/src" ]]; then ln -s "$snapshot_release/runtime/src" "$snapshot_out/runtime/src"; fi
cp "$snapshot_release/public_offline_manifest.json" "$snapshot_out/build_manifest.json"
cp "$snapshot_release/test_reused_sync.py" "$snapshot_out/test_reused_sync.py"
cmake -S "$snapshot_release/firmware" -B "$snapshot_out/native" -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build "$snapshot_out/native" -j2
ctest --test-dir "$snapshot_out/native" --output-on-failure
cd "$snapshot_car"
"$snapshot_python" -m pytest -q -p no:cacheprovider "$snapshot_out/test_reused_sync.py" \
    "$snapshot_car/candidates/wand_sync_600s/test_long_session.py" \
    --junitxml="$snapshot_out/host.xml"
printf 'PASS: exported 4WSA source only; no ROS graph, device, network or motion was started.\n'
