#!/usr/bin/env bash
# Build only. Deliberately contains no programmer, reset or device command.
set -euo pipefail
handoff_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ -n "${STM32_TOOLCHAIN_BIN:-}" ]]; then export PATH="$STM32_TOOLCHAIN_BIN:$PATH"; fi
command -v arm-none-eabi-gcc >/dev/null || { printf 'Put the pinned ARM GCC on PATH first.\n' >&2; exit 2; }
cmake -S "$handoff_root/car/stm32/mecanum_f407" -B "$handoff_root/.local/build/stm32" \
    -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build "$handoff_root/.local/build/stm32" --target \
    mecanum_f407_four_link_readonly mecanum_f407_four_link_bench mecanum_f407_four_link_motion -j 4
printf 'Built only. Do not flash a movement image without a fresh physical confirmation.\n'
