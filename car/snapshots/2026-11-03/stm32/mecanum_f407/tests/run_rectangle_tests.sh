#!/usr/bin/env bash
# 本机纯算法测试：不打开串口、不调用烧录器、不发送运动指令。
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
rectangle_test_dir=$(mktemp -d /tmp/mecanum_rectangle_tests.XXXXXXXX)
compiler=${CC:-gcc}
flags=(-std=c11 -Wall -Wextra -Wpedantic -Werror -g
       -fsanitize=address,undefined -fno-omit-frame-pointer -IInc -I../../protocol)
algorithms=(Src/four_drive_group.c Src/four_wheel_speed.c
            Src/mecanum_drive_plan.c Src/mecanum_kinematics.c)
observe=(-DDRIVE_BENCH=1 -DDRIVE_STARTUP_OBSERVE=1 -DSPEED_UART_FRAME_BYTES=1152U)
build_and_run() {
    local name=$1
    shift
    "$compiler" "${flags[@]}" "$@" -lm -o "$rectangle_test_dir/$name"
    "$rectangle_test_dir/$name"
}
build_and_run rectangle -DAUTONOMOUS_RECTANGLE=1 -DDRIVE_STARTUP_OBSERVE=1 \
    tests/test_rectangle_demo.c Src/rectangle_demo.c "${algorithms[@]}"
build_and_run rectangle_trace -DAUTONOMOUS_RECTANGLE=1 -DDRIVE_STARTUP_OBSERVE=1 \
    tests/test_rectangle_startup_trace.c Src/rectangle_startup_trace.c \
    Src/rectangle_demo.c "${algorithms[@]}"
build_and_run speed tests/test_four_wheel_speed.c Src/four_wheel_speed.c
build_and_run drive_group tests/test_four_drive_group.c "${algorithms[@]}"
build_and_run startup "${observe[@]}" tests/test_drive_startup_observe.c \
    Src/drive_bench_control.c Src/speed_uart_queue.c "${algorithms[@]}"
build_and_run wireless "${observe[@]}" tests/test_four_link_bench.c \
    Src/four_link_bench.c Src/mc_protocol.c Src/drive_bench_control.c \
    Src/speed_uart_queue.c "${algorithms[@]}"
python3 tools/test_rectangle_startup_trace_decode.py
printf 'PASS: offline tests only; executables retained at %s\n' "$rectangle_test_dir"
