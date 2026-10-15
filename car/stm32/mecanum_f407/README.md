# STM32 source snapshot

Start with the parent `car/README.md` and root handoff. All historical bench images
remain separate; the main PC's actual firmware is the no-PWM read-only image, not
the default CMake executable and not the new streaming candidate.

Use root `scripts/build_car_stm32.sh` for an offline build. Tests are HAL-free and
run via `tests/run_motion_tests.sh` and `tests/run_rectangle_tests.sh`.
The copied `ThirdParty/STM32CubeF4` files retain their original HAL/CMSIS licenses.
No full Flash backups or historical motion binaries were copied. A successful
build does not authorize flashing, powering a driver, or touching wheels.
