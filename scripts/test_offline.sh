#!/usr/bin/env bash
# Bounded offline tests only. Never invoke flashing, serial probes or flight launch.
set -euo pipefail
handoff_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
handoff_mode=${1:-all}
[[ "$handoff_mode" == all || "$handoff_mode" == car || "$handoff_mode" == drone ]] || {
    printf 'Usage: bash scripts/test_offline.sh [all|car|drone]\n' >&2; exit 2;
}
handoff_python=${HANDOFF_PYTHON:-$handoff_root/.local/venv/bin/python}
[[ -x "$handoff_python" ]] || handoff_python=/usr/bin/python3
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi
# These variables are process-local and do not alter any NIC or system setting.
export ROS_DOMAIN_ID=121
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export PYTHONPATH="$handoff_root/car/src/mecanum_vicon_control:$handoff_root/car/src/mecanum_wand_control:$handoff_root/car/src/mecanum_wand_hardware:$handoff_root/car/stm32/mecanum_f407/tools:$handoff_root/drone/ros2_ws/src/uav_coord_bridge:${PYTHONPATH:-}"
cd "$handoff_root"
mkdir -p .local/test-results
if [[ "$handoff_mode" == all || "$handoff_mode" == car ]]; then
    "$handoff_python" -c 'import rclpy, geometry_msgs, launch; print("ROS Python prerequisites OK")'
    # Launch-structure tests need the ament index for the description package.
    # Build it here instead of accidentally relying on a primary-PC install.
    colcon --log-base "$handoff_root/.local/log/car-description" build \
        --base-paths "$handoff_root/car/src" --packages-select mecanum_description mecanum_vicon_control \
        --build-base "$handoff_root/.local/build/car-description" \
        --install-base "$handoff_root/.local/install/car-description" --symlink-install
    set +u
    source "$handoff_root/.local/install/car-description/setup.bash"
    set -u
    "$handoff_python" -m pytest -q car/src/mecanum_vicon_control/test \
        car/stm32/mecanum_f407/tools car/esp32/mecanum_wifi_bridge/tests \
        --junitxml=.local/test-results/car.xml
    "$handoff_python" -m pytest -q car/src/mecanum_wand_control/test \
        --junitxml=.local/test-results/wand-control.xml
    bash car/stm32/mecanum_f407/tests/run_motion_tests.sh
    bash car/stm32/mecanum_f407/tests/run_rectangle_tests.sh
fi
if [[ "$handoff_mode" == all || "$handoff_mode" == drone ]]; then
    "$handoff_python" -m pytest -q drone/tools/gamepad_rc/test \
        drone/ros2_ws/src/uav_coord_bridge/test --junitxml=.local/test-results/drone.xml
fi
printf 'PASS: offline source tests only. No firmware, parameter or motor state was changed.\n'
