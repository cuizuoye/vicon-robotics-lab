cmake_minimum_required(VERSION 3.20)
project(public_4wsc_source_checks C)
enable_testing()
get_filename_component(REPO "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
set(CAR "${REPO}/car/snapshots/2026-11-03")
set(BASE "${CAR}/stm32/mecanum_f407")
set(FW "${CAR}/docs/wand_stopfix_2026-11-02/prepared_v1/firmware")
set(ALGORITHMS
  ${FW}/Src/four_wheel_speed.c ${FW}/Src/four_drive_group.c
  ${FW}/Src/drive_bench_control.c ${FW}/Src/four_link_motion.c
  ${FW}/Src/servo_policy.c ${FW}/Src/mecanum_drive_plan.c
  ${BASE}/Src/mecanum_kinematics.c ${BASE}/Src/mc_protocol.c)
foreach(name stages controller rotation long_run stage7 new_stages stop_safety)
  add_executable(test_${name} ${FW}/tests/test_${name}.c ${ALGORITHMS})
  target_include_directories(test_${name} BEFORE PRIVATE ${FW}/Inc ${BASE}/candidates/servo_4ws7/Inc)
  target_include_directories(test_${name} PRIVATE ${BASE}/Inc ${CAR}/protocol)
  target_compile_definitions(test_${name} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUNDB=1
    DRIVE_BENCH=1 DRIVE_STARTUP_OBSERVE=1 FOUR_SERVO_CANDIDATE=1 SERVO_STAGED_COMMISSIONING=1)
  target_compile_options(test_${name} PRIVATE -std=c11 -Wall -Wextra -Wpedantic -Werror
    -g -include${FW}/Inc/four_speed_pwm_profile.h
    -fsanitize=address,undefined -fno-omit-frame-pointer -fno-pie)
  target_link_options(test_${name} PRIVATE -fsanitize=address,undefined -no-pie)
  target_link_libraries(test_${name} PRIVATE m)
  if(name MATCHES "^(rotation|stage7|new_stages)$")
    add_test(NAME ${name} COMMAND test_${name} ${CMAKE_CURRENT_BINARY_DIR})
  else()
    add_test(NAME ${name} COMMAND test_${name})
  endif()
endforeach()
