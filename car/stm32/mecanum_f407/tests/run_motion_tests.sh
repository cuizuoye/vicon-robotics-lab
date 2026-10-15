#!/usr/bin/env bash
# Native only: never opens hardware, network sockets or a flasher.
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
motion_test_dir=$(mktemp -d /tmp/mecanum_motion_tests.XXXXXXXX)
"${CC:-gcc}" -std=c11 -Wall -Wextra -Wpedantic -Werror -g \
    -fsanitize=address,undefined -fno-omit-frame-pointer \
    -DFOUR_LINK_MOTION=1 -DDRIVE_BENCH=1 -DDRIVE_STARTUP_OBSERVE=1 \
    -IInc -I../../protocol tests/test_four_link_motion.c \
    Src/four_link_motion.c Src/four_drive_group.c Src/four_wheel_speed.c \
    Src/drive_bench_control.c Src/mecanum_drive_plan.c Src/mecanum_kinematics.c \
    Src/mc_protocol.c -lm -o "$motion_test_dir/motion"
"$motion_test_dir/motion"
printf 'Native executable retained at %s\n' "$motion_test_dir/motion"
