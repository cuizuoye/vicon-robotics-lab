#!/usr/bin/env bash
# Native only: never opens hardware, network sockets or a flasher.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
motion_test_dir=$(mktemp -d /tmp/mecanum_motion_tests.XXXXXXXX)
for profile in suspended ground ground2 ground3 ground4 ground5 ground6 ground7 ground8 ground9 grounda groundb; do
    definitions=(-DFOUR_LINK_MOTION=1 -DDRIVE_BENCH=1 -DDRIVE_STARTUP_OBSERVE=1)
    if [[ "$profile" == ground ]]; then definitions+=(-DFOUR_LINK_GROUND=1); fi
    if [[ "$profile" == ground2 ]]; then definitions+=(-DFOUR_LINK_GROUND2=1); fi
    if [[ "$profile" == ground3 ]]; then definitions+=(-DFOUR_LINK_GROUND3=1); fi
    if [[ "$profile" == ground4 ]]; then definitions+=(-DFOUR_LINK_GROUND4=1); fi
    if [[ "$profile" == ground5 ]]; then definitions+=(-DFOUR_LINK_GROUND5=1); fi
    if [[ "$profile" == ground6 ]]; then definitions+=(-DFOUR_LINK_GROUND6=1); fi
    if [[ "$profile" == ground7 ]]; then definitions+=(-DFOUR_LINK_GROUND7=1); fi
    if [[ "$profile" == ground8 ]]; then definitions+=(-DFOUR_LINK_GROUND8=1); fi
    if [[ "$profile" == ground9 ]]; then definitions+=(-DFOUR_LINK_GROUND9=1); fi
    if [[ "$profile" == grounda ]]; then definitions+=(-DFOUR_LINK_GROUNDA=1); fi
    if [[ "$profile" == groundb ]]; then definitions+=(-DFOUR_LINK_GROUNDB=1); fi
    "${CC:-gcc}" -std=c11 -Wall -Wextra -Wpedantic -Werror -g \
        -fsanitize=address,undefined -fno-omit-frame-pointer "${definitions[@]}" \
        -IInc -I../../protocol tests/test_four_link_motion.c \
        Src/four_link_motion.c Src/four_drive_group.c Src/four_wheel_speed.c \
        Src/drive_bench_control.c Src/mecanum_drive_plan.c Src/mecanum_kinematics.c \
        Src/mc_protocol.c -lm -o "$motion_test_dir/motion_$profile"
    "$motion_test_dir/motion_$profile"
done
printf 'Native executables retained at %s\n' "$motion_test_dir"
