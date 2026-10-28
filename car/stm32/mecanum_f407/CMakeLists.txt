cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
set(CMAKE_C_COMPILER_FORCED TRUE)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/target.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc-arm-none-eabi.cmake)

project(mecanum_f407 C ASM)

set(STM32CUBE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/ThirdParty/STM32CubeF4")

add_executable(${PROJECT_NAME}
    Src/main.c
    Src/motor_a.c
    Src/encoder_a.c
    Src/speed_controller.c
    Src/command.c
    Src/uart_console.c
    Src/stm32f4xx_hal_msp.c
    Src/stm32f4xx_it.c
    Src/syscall.c
    Src/sysmem.c
    Src/startup_stm32f407xx.S
    ${STM32CUBE_DIR}/CMSIS/Device/Source/system_stm32f4xx.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_cortex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_gpio.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_tim.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_tim_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_uart.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    Inc
    ${STM32CUBE_DIR}/CMSIS/Core/Include
    ${STM32CUBE_DIR}/CMSIS/Device/Include
    ${STM32CUBE_DIR}/HAL/Inc
)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    STM32F407xx
    USE_HAL_DRIVER
)

target_compile_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter
    $<$<CONFIG:Debug>:-O0;-g3>
    $<$<CONFIG:Release>:-Os>
)

target_link_options(${PROJECT_NAME} PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f407xg_flash.ld
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)

set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${PROJECT_NAME}>
            ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating HEX/BIN images and reporting image size"
)

# Safe communications-only image. It initializes USART2 and PB2 only: no
# encoder timer, motor timer or PWM output is configured.
set(LINK_TEST_TARGET mecanum_f407_link_test)
add_executable(${LINK_TEST_TARGET}
    Src/link_test_main.c
    Src/mc_link_test.c
    Src/mc_protocol.c
    Src/stm32f4xx_hal_msp.c
    Src/stm32f4xx_it.c
    Src/syscall.c
    Src/sysmem.c
    Src/startup_stm32f407xx.S
    ${STM32CUBE_DIR}/CMSIS/Device/Source/system_stm32f4xx.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_cortex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_gpio.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_uart.c
)

target_include_directories(${LINK_TEST_TARGET} PRIVATE
    Inc
    ../../protocol
    ${STM32CUBE_DIR}/CMSIS/Core/Include
    ${STM32CUBE_DIR}/CMSIS/Device/Include
    ${STM32CUBE_DIR}/HAL/Inc
)
target_compile_definitions(${LINK_TEST_TARGET} PRIVATE STM32F407xx USE_HAL_DRIVER)
target_compile_options(${LINK_TEST_TARGET} PRIVATE
    ${CPU_FLAGS}
    -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter
    $<$<CONFIG:Debug>:-O0;-g3>
    $<$<CONFIG:Release>:-Os>
)
target_link_options(${LINK_TEST_TARGET} PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f407xg_flash.ld
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${LINK_TEST_TARGET}.map
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
set_target_properties(${LINK_TEST_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${LINK_TEST_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${LINK_TEST_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${LINK_TEST_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${LINK_TEST_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${LINK_TEST_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${LINK_TEST_TARGET}>
    COMMENT "Generating safe UART link-test HEX/BIN images"
)

# Binary MC-Link single-motor image. With the default zero encoder calibration
# it initializes the peripherals but refuses every enabled motion command.
set(MOTOR_LINK_TARGET mecanum_f407_motor_link)
add_executable(${MOTOR_LINK_TARGET}
    Src/motor_link_main.c
    Src/motor_link_control.c
    Src/mc_uart_transport.c
    Src/mc_protocol.c
    Src/motor_a.c
    Src/encoder_a.c
    Src/speed_controller.c
    Src/mecanum_kinematics.c
    Src/stm32f4xx_hal_msp.c
    Src/stm32f4xx_it.c
    Src/syscall.c
    Src/sysmem.c
    Src/startup_stm32f407xx.S
    ${STM32CUBE_DIR}/CMSIS/Device/Source/system_stm32f4xx.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_cortex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_dma_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_gpio.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_pwr_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_rcc_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_tim.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_tim_ex.c
    ${STM32CUBE_DIR}/HAL/Src/stm32f4xx_hal_uart.c
)
target_include_directories(${MOTOR_LINK_TARGET} PRIVATE
    Inc
    ../../protocol
    ${STM32CUBE_DIR}/CMSIS/Core/Include
    ${STM32CUBE_DIR}/CMSIS/Device/Include
    ${STM32CUBE_DIR}/HAL/Inc
)
target_compile_definitions(${MOTOR_LINK_TARGET} PRIVATE STM32F407xx USE_HAL_DRIVER)
target_compile_options(${MOTOR_LINK_TARGET} PRIVATE
    ${CPU_FLAGS}
    -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter
    $<$<CONFIG:Debug>:-O0;-g3>
    $<$<CONFIG:Release>:-Os>
)
target_link_options(${MOTOR_LINK_TARGET} PRIVATE
    ${CPU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32f407xg_flash.ld
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${MOTOR_LINK_TARGET}.map
    --specs=nosys.specs
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
set_target_properties(${MOTOR_LINK_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${MOTOR_LINK_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${MOTOR_LINK_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${MOTOR_LINK_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${MOTOR_LINK_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${MOTOR_LINK_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${MOTOR_LINK_TARGET}>
    COMMENT "Generating guarded MC-Link motor HEX/BIN images"
)

# Separate explicit-channel bench image; keep known-good single-A sources intact.
set(DUAL_BENCH_TARGET mecanum_f407_dual_bench)
get_target_property(DUAL_BENCH_BASE_SOURCES ${PROJECT_NAME} SOURCES)
list(REMOVE_ITEM DUAL_BENCH_BASE_SOURCES
    Src/main.c Src/motor_a.c Src/speed_controller.c Src/command.c
    Src/stm32f4xx_hal_msp.c)
add_executable(${DUAL_BENCH_TARGET}
    ${DUAL_BENCH_BASE_SOURCES}
    Src/bench_dual_main.c
    Src/bench_dual_control.c
    Src/bench_dual_msp.c
)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(DUAL_BENCH_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${DUAL_BENCH_TARGET} PROPERTY ${property} ${DUAL_BENCH_PROPERTY})
endforeach()

# Give the new target its own linker map rather than overwrite the single-A map.
get_target_property(DUAL_BENCH_LINK_OPTIONS ${DUAL_BENCH_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM DUAL_BENCH_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND DUAL_BENCH_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${DUAL_BENCH_TARGET}.map)
set_property(TARGET ${DUAL_BENCH_TARGET} PROPERTY LINK_OPTIONS ${DUAL_BENCH_LINK_OPTIONS})
set_target_properties(${DUAL_BENCH_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${DUAL_BENCH_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex
            $<TARGET_FILE:${DUAL_BENCH_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${DUAL_BENCH_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary
            $<TARGET_FILE:${DUAL_BENCH_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${DUAL_BENCH_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${DUAL_BENCH_TARGET}>
    COMMENT "Generating explicit-channel dual bench HEX/BIN images"
)

# 独立30秒展示镜像；只有本目标启用长时限，不放宽dual_v2台架。
set(FRONT_DEMO_TARGET mecanum_f407_front_demo30)
get_target_property(FRONT_DEMO_SOURCES ${DUAL_BENCH_TARGET} SOURCES)
add_executable(${FRONT_DEMO_TARGET} ${FRONT_DEMO_SOURCES})
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FRONT_DEMO_PROPERTY ${DUAL_BENCH_TARGET} ${property})
    set_property(TARGET ${FRONT_DEMO_TARGET} PROPERTY ${property} ${FRONT_DEMO_PROPERTY})
endforeach()
target_compile_definitions(${FRONT_DEMO_TARGET} PRIVATE BENCH_FRONT_DEMO30=1)
get_target_property(FRONT_DEMO_LINK_OPTIONS ${FRONT_DEMO_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FRONT_DEMO_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${DUAL_BENCH_TARGET}.map)
list(APPEND FRONT_DEMO_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FRONT_DEMO_TARGET}.map)
set_property(TARGET ${FRONT_DEMO_TARGET} PROPERTY LINK_OPTIONS ${FRONT_DEMO_LINK_OPTIONS})
set_target_properties(${FRONT_DEMO_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FRONT_DEMO_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FRONT_DEMO_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FRONT_DEMO_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FRONT_DEMO_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FRONT_DEMO_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FRONT_DEMO_TARGET}>
    COMMENT "Generating separately guarded front-wheel demo30 HEX/BIN images"
)

# 纯读取双编码器镜像：PE5/6/9/11始终GPIO低，无PWM或运动命令。
set(FRONT_ENCODER_TARGET mecanum_f407_front_encoder_readonly)
add_executable(${FRONT_ENCODER_TARGET} ${FRONT_DEMO_SOURCES} Src/encoder_b.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FRONT_ENCODER_PROPERTY ${DUAL_BENCH_TARGET} ${property})
    set_property(TARGET ${FRONT_ENCODER_TARGET} PROPERTY ${property} ${FRONT_ENCODER_PROPERTY})
endforeach()
target_compile_definitions(${FRONT_ENCODER_TARGET} PRIVATE BENCH_FRONT_ENCODERS_READONLY=1)
get_target_property(FRONT_ENCODER_LINK_OPTIONS ${FRONT_ENCODER_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FRONT_ENCODER_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${DUAL_BENCH_TARGET}.map)
list(APPEND FRONT_ENCODER_LINK_OPTIONS
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FRONT_ENCODER_TARGET}.map)
set_property(TARGET ${FRONT_ENCODER_TARGET} PROPERTY LINK_OPTIONS ${FRONT_ENCODER_LINK_OPTIONS})
set_target_properties(${FRONT_ENCODER_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FRONT_ENCODER_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FRONT_ENCODER_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FRONT_ENCODER_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FRONT_ENCODER_TARGET}>
            ${CMAKE_CURRENT_BINARY_DIR}/${FRONT_ENCODER_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FRONT_ENCODER_TARGET}>
    COMMENT "Generating motion-disabled front encoder HEX/BIN images"
)

# 独立四轮单通道台架，不放宽旧双轮/Demo/只读镜像。
set(FOUR_BENCH_TARGET mecanum_f407_four_bench)
set(FOUR_BENCH_SOURCES ${DUAL_BENCH_BASE_SOURCES})
list(REMOVE_ITEM FOUR_BENCH_SOURCES Src/encoder_a.c)
add_executable(${FOUR_BENCH_TARGET} ${FOUR_BENCH_SOURCES}
    Src/bench_four_main.c Src/bench_four_control.c Src/bench_four_msp.c Src/encoder_accumulator.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FOUR_BENCH_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${FOUR_BENCH_TARGET} PROPERTY ${property} ${FOUR_BENCH_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_BENCH_TARGET} PRIVATE BENCH_FOUR_WHEELS=1)
get_target_property(FOUR_BENCH_LINK_OPTIONS ${FOUR_BENCH_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_BENCH_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_BENCH_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_BENCH_TARGET}.map)
set_property(TARGET ${FOUR_BENCH_TARGET} PROPERTY LINK_OPTIONS ${FOUR_BENCH_LINK_OPTIONS})
set_target_properties(${FOUR_BENCH_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_BENCH_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FOUR_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_BENCH_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_BENCH_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_BENCH_TARGET}>
    COMMENT "Generating single-wheel-only four-channel bench HEX/BIN images"
)

# 四路速度算法仅构建为库，不链接当前台架镜像、不生成新可烧录固件。
add_library(mecanum_four_speed_algorithms STATIC Src/four_wheel_speed.c)
target_include_directories(mecanum_four_speed_algorithms PUBLIC Inc)
target_compile_options(mecanum_four_speed_algorithms PRIVATE
    ${CPU_FLAGS} -Wall -Wextra -Wpedantic -Werror)

# 新速度台架独立目标：先验证只读版，保留所有已验收旧镜像。
set(SPEED_BENCH_SOURCES ${FOUR_BENCH_SOURCES})
list(REMOVE_ITEM SPEED_BENCH_SOURCES Src/uart_console.c)
foreach(SPEED_TARGET mecanum_f407_speed_readonly mecanum_f407_speed_bench)
    add_executable(${SPEED_TARGET} ${SPEED_BENCH_SOURCES}
        Src/speed_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
        Src/four_wheel_speed.c Src/speed_bench_control.c
        Src/speed_uart_queue.c Src/speed_uart_transport.c)
    foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
        get_target_property(SPEED_PROPERTY ${PROJECT_NAME} ${property})
        set_property(TARGET ${SPEED_TARGET} PROPERTY ${property} ${SPEED_PROPERTY})
    endforeach()
    get_target_property(SPEED_LINK_OPTIONS ${SPEED_TARGET} LINK_OPTIONS)
    list(REMOVE_ITEM SPEED_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
    list(APPEND SPEED_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${SPEED_TARGET}.map)
    set_property(TARGET ${SPEED_TARGET} PROPERTY LINK_OPTIONS ${SPEED_LINK_OPTIONS})
    target_link_libraries(${SPEED_TARGET} PRIVATE m)
    if(SPEED_TARGET STREQUAL "mecanum_f407_speed_readonly")
        target_compile_definitions(${SPEED_TARGET} PRIVATE SPEED_BENCH_READONLY=1)
    else()
        target_compile_definitions(${SPEED_TARGET} PRIVATE SPEED_BENCH_READONLY=0)
    endif()
    set_target_properties(${SPEED_TARGET} PROPERTIES SUFFIX ".elf")
    add_custom_command(TARGET ${SPEED_TARGET} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${SPEED_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${SPEED_TARGET}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${SPEED_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${SPEED_TARGET}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${SPEED_TARGET}>
        COMMENT "Generating isolated speed-bench images")
endforeach()

# 独立四轮悬空前进闭环镜像；不改旧单轮/只读镜像或默认目标。
set(FORWARD_BENCH_TARGET mecanum_f407_forward_bench)
add_executable(${FORWARD_BENCH_TARGET} ${SPEED_BENCH_SOURCES}
    Src/forward_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_wheel_speed.c Src/four_forward_group.c Src/forward_bench_control.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FORWARD_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${FORWARD_BENCH_TARGET} PROPERTY ${property} ${FORWARD_PROPERTY})
endforeach()
get_target_property(FORWARD_LINK_OPTIONS ${FORWARD_BENCH_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FORWARD_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FORWARD_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FORWARD_BENCH_TARGET}.map)
set_property(TARGET ${FORWARD_BENCH_TARGET} PROPERTY LINK_OPTIONS ${FORWARD_LINK_OPTIONS})
target_link_libraries(${FORWARD_BENCH_TARGET} PRIVATE m)
set_target_properties(${FORWARD_BENCH_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FORWARD_BENCH_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FORWARD_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FORWARD_BENCH_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FORWARD_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FORWARD_BENCH_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FORWARD_BENCH_TARGET}>
    COMMENT "Generating one-shot four-wheel forward speed-bench images")

# 整车速度接口先只编译为HAL-free库；不链接/替换已验证前进固件、不烧录。
add_library(mecanum_drive_planning STATIC Src/mecanum_drive_plan.c Src/mecanum_kinematics.c)
target_include_directories(mecanum_drive_planning PUBLIC Inc)
target_compile_options(mecanum_drive_planning PRIVATE
    ${CPU_FLAGS} -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(mecanum_drive_planning PUBLIC mecanum_four_speed_algorithms m)

# 受限三向组控制独立镜像；默认不使能，每次启动仅接受一个固定5秒动作。
set(DRIVE_BENCH_TARGET mecanum_f407_drive_bench)
add_executable(${DRIVE_BENCH_TARGET} ${SPEED_BENCH_SOURCES}
    Src/forward_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_drive_group.c Src/drive_bench_control.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(DRIVE_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${DRIVE_BENCH_TARGET} PROPERTY ${property} ${DRIVE_PROPERTY})
endforeach()
target_compile_definitions(${DRIVE_BENCH_TARGET} PRIVATE DRIVE_BENCH=1 SPEED_UART_FRAME_BYTES=1152U)
get_target_property(DRIVE_LINK_OPTIONS ${DRIVE_BENCH_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM DRIVE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND DRIVE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_BENCH_TARGET}.map)
set_property(TARGET ${DRIVE_BENCH_TARGET} PROPERTY LINK_OPTIONS ${DRIVE_LINK_OPTIONS})
target_link_libraries(${DRIVE_BENCH_TARGET} PRIVATE mecanum_drive_planning m)
set_target_properties(${DRIVE_BENCH_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${DRIVE_BENCH_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${DRIVE_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_BENCH_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${DRIVE_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_BENCH_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${DRIVE_BENCH_TARGET}>
    COMMENT "Generating guarded one-shot body-velocity bench images (not flashing)")

# 独立8%/200ms启动、10秒观察镜像；所有算法以同一宏重新编译，不混用旧库结构。
set(DRIVE_OBSERVE_TARGET mecanum_f407_drive_startup_observe)
add_executable(${DRIVE_OBSERVE_TARGET} ${SPEED_BENCH_SOURCES}
    Src/forward_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_drive_group.c Src/drive_bench_control.c Src/four_wheel_speed.c
    Src/mecanum_drive_plan.c Src/mecanum_kinematics.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(DRIVE_OBSERVE_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${DRIVE_OBSERVE_TARGET} PROPERTY ${property} ${DRIVE_OBSERVE_PROPERTY})
endforeach()
target_compile_definitions(${DRIVE_OBSERVE_TARGET} PRIVATE
    DRIVE_BENCH=1 DRIVE_STARTUP_OBSERVE=1 SPEED_UART_FRAME_BYTES=1152U)
get_target_property(DRIVE_OBSERVE_LINK_OPTIONS ${DRIVE_OBSERVE_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM DRIVE_OBSERVE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND DRIVE_OBSERVE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_OBSERVE_TARGET}.map)
set_property(TARGET ${DRIVE_OBSERVE_TARGET} PROPERTY LINK_OPTIONS ${DRIVE_OBSERVE_LINK_OPTIONS})
target_link_libraries(${DRIVE_OBSERVE_TARGET} PRIVATE m)
set_target_properties(${DRIVE_OBSERVE_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${DRIVE_OBSERVE_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${DRIVE_OBSERVE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_OBSERVE_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${DRIVE_OBSERVE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${DRIVE_OBSERVE_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${DRIVE_OBSERVE_TARGET}>
    COMMENT "Generating isolated bounded-startup observation images (not flashing)")

# RR单轮低速诊断：仅新镜像，复用外设/PI/安全层；100Hz记录停机后导出。
set(RR_DIAG_TARGET mecanum_f407_rr_diagnostic)
add_executable(${RR_DIAG_TARGET} ${SPEED_BENCH_SOURCES}
    Src/forward_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_drive_group.c Src/drive_bench_control.c Src/four_wheel_speed.c
    Src/mecanum_drive_plan.c Src/mecanum_kinematics.c Src/rr_diagnostic_trace.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(RR_DIAG_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${RR_DIAG_TARGET} PROPERTY ${property} ${RR_DIAG_PROPERTY})
endforeach()
target_compile_definitions(${RR_DIAG_TARGET} PRIVATE
    DRIVE_BENCH=1 DRIVE_STARTUP_OBSERVE=1 RR_DIAGNOSTIC=1 SPEED_UART_FRAME_BYTES=1152U)
get_target_property(RR_DIAG_LINK_OPTIONS ${RR_DIAG_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM RR_DIAG_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND RR_DIAG_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RR_DIAG_TARGET}.map)
set_property(TARGET ${RR_DIAG_TARGET} PROPERTY LINK_OPTIONS ${RR_DIAG_LINK_OPTIONS})
target_link_libraries(${RR_DIAG_TARGET} PRIVATE m)
set_target_properties(${RR_DIAG_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${RR_DIAG_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RR_DIAG_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_DIAG_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RR_DIAG_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_DIAG_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RR_DIAG_TARGET}>
    COMMENT "Generating RR-only fixed-speed diagnostic images (not flashing)")

# 独立RR精细PWM对照版：不覆盖旧目标，目标/增益/保护及10kHz频率不变。
set(RR_FINE_TARGET mecanum_f407_rr_pwm_fine)
add_executable(${RR_FINE_TARGET} ${SPEED_BENCH_SOURCES}
    Src/forward_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_drive_group.c Src/drive_bench_control.c Src/four_wheel_speed.c
    Src/mecanum_drive_plan.c Src/mecanum_kinematics.c Src/rr_diagnostic_trace.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(RR_FINE_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${RR_FINE_TARGET} PROPERTY ${property} ${RR_FINE_PROPERTY})
endforeach()
target_compile_definitions(${RR_FINE_TARGET} PRIVATE
    DRIVE_BENCH=1 DRIVE_STARTUP_OBSERVE=1 RR_DIAGNOSTIC=1 RR_PWM_FINE=1 SPEED_UART_FRAME_BYTES=1152U)
get_target_property(RR_FINE_LINK_OPTIONS ${RR_FINE_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM RR_FINE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND RR_FINE_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RR_FINE_TARGET}.map)
set_property(TARGET ${RR_FINE_TARGET} PROPERTY LINK_OPTIONS ${RR_FINE_LINK_OPTIONS})
target_link_libraries(${RR_FINE_TARGET} PRIVATE m)
set_target_properties(${RR_FINE_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${RR_FINE_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RR_FINE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_FINE_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RR_FINE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_FINE_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RR_FINE_TARGET}>
    COMMENT "Generating isolated RR 0.1-percent PWM comparison images (not flashing)")

# 仅RR的2倍目标对照，独立产物；10%输出/10秒/心跳/单次额度不变。
set(RR_SPEED2_TARGET mecanum_f407_rr_speed2)
get_target_property(RR_SPEED2_SOURCES ${RR_FINE_TARGET} SOURCES)
add_executable(${RR_SPEED2_TARGET} ${RR_SPEED2_SOURCES})
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(RR_SPEED2_PROPERTY ${RR_FINE_TARGET} ${property})
    set_property(TARGET ${RR_SPEED2_TARGET} PROPERTY ${property} ${RR_SPEED2_PROPERTY})
endforeach()
target_compile_definitions(${RR_SPEED2_TARGET} PRIVATE RR_SPEED2=1)
get_target_property(RR_SPEED2_LINK_OPTIONS ${RR_SPEED2_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM RR_SPEED2_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RR_FINE_TARGET}.map)
list(APPEND RR_SPEED2_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RR_SPEED2_TARGET}.map)
set_property(TARGET ${RR_SPEED2_TARGET} PROPERTY LINK_OPTIONS ${RR_SPEED2_LINK_OPTIONS})
target_link_libraries(${RR_SPEED2_TARGET} PRIVATE m)
set_target_properties(${RR_SPEED2_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${RR_SPEED2_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RR_SPEED2_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_SPEED2_TARGET}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RR_SPEED2_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RR_SPEED2_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RR_SPEED2_TARGET}>
    COMMENT "Generating isolated RR 328cps comparison images (not flashing)")

# 四轮真实反馈MC-Link只读镜像：电机脚GPIO低，不配置任何PWM定时器。
set(FOUR_RO_TARGET mecanum_f407_four_link_readonly)
add_executable(${FOUR_RO_TARGET} ${SPEED_BENCH_SOURCES}
    Src/speed_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/speed_bench_control.c Src/four_wheel_speed.c Src/four_link_readonly.c
    Src/mc_protocol.c Src/mc_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FOUR_RO_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${FOUR_RO_TARGET} PROPERTY ${property} ${FOUR_RO_PROPERTY})
endforeach()
target_include_directories(${FOUR_RO_TARGET} PRIVATE ../../protocol)
target_compile_definitions(${FOUR_RO_TARGET} PRIVATE SPEED_BENCH_READONLY=1 FOUR_LINK_READONLY=1)
get_target_property(FOUR_RO_LINK_OPTIONS ${FOUR_RO_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_RO_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_RO_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_RO_TARGET}.map)
set_property(TARGET ${FOUR_RO_TARGET} PROPERTY LINK_OPTIONS ${FOUR_RO_LINK_OPTIONS})
target_link_libraries(${FOUR_RO_TARGET} PRIVATE m)
set_target_properties(${FOUR_RO_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_RO_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_RO_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_RO_TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FOUR_RO_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_RO_TARGET}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_RO_TARGET}>
    COMMENT "Generating four-wheel MC-Link readonly image (no flashing)")

# 无线固定F50悬空试验；复用四轮PI/8%启动/10秒/10%/单次额度，不开放连续轨迹。
# 独立启动诊断镜像：只增加SWD可读错误计数，不改变故障锁存或PWM禁用。
set(STARTUP_RO_TARGET mecanum_f407_four_link_startup_diag)
add_executable(${STARTUP_RO_TARGET} ${SPEED_BENCH_SOURCES}
    Src/speed_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/speed_bench_control.c Src/four_wheel_speed.c Src/four_link_readonly.c
    Src/mc_protocol.c Src/mc_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(STARTUP_RO_PROPERTY ${FOUR_RO_TARGET} ${property})
    set_property(TARGET ${STARTUP_RO_TARGET} PROPERTY ${property} ${STARTUP_RO_PROPERTY})
endforeach()
target_compile_definitions(${STARTUP_RO_TARGET} PRIVATE MC_UART_DIAGNOSTICS=1)
get_target_property(STARTUP_RO_LINK_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM STARTUP_RO_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND STARTUP_RO_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${STARTUP_RO_TARGET}.map)
set_property(TARGET ${STARTUP_RO_TARGET} PROPERTY LINK_OPTIONS ${STARTUP_RO_LINK_OPTIONS})
target_link_libraries(${STARTUP_RO_TARGET} PRIVATE m)
set_target_properties(${STARTUP_RO_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${STARTUP_RO_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${STARTUP_RO_TARGET}> ${STARTUP_RO_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${STARTUP_RO_TARGET}>
    COMMENT "Generating readonly startup diagnosis (no flashing)")

set(FOUR_LINK_BENCH_TARGET mecanum_f407_four_link_bench)
add_executable(${FOUR_LINK_BENCH_TARGET} ${SPEED_BENCH_SOURCES}
    Src/four_link_bench_main.c Src/bench_four_msp.c Src/encoder_accumulator.c
    Src/four_link_bench.c Src/drive_bench_control.c Src/four_drive_group.c
    Src/four_wheel_speed.c Src/mecanum_drive_plan.c Src/mecanum_kinematics.c
    Src/mc_protocol.c Src/mc_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(FOUR_LINK_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${FOUR_LINK_BENCH_TARGET} PROPERTY ${property} ${FOUR_LINK_PROPERTY})
endforeach()
target_include_directories(${FOUR_LINK_BENCH_TARGET} PRIVATE ../../protocol)
target_compile_definitions(${FOUR_LINK_BENCH_TARGET} PRIVATE DRIVE_BENCH=1 DRIVE_STARTUP_OBSERVE=1)
get_target_property(FOUR_LINK_OPTIONS ${FOUR_LINK_BENCH_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_LINK_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_LINK_BENCH_TARGET}.map)
set_property(TARGET ${FOUR_LINK_BENCH_TARGET} PROPERTY LINK_OPTIONS ${FOUR_LINK_OPTIONS})
target_link_libraries(${FOUR_LINK_BENCH_TARGET} PRIVATE m)
set_target_properties(${FOUR_LINK_BENCH_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_LINK_BENCH_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_LINK_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_LINK_BENCH_TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${FOUR_LINK_BENCH_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_LINK_BENCH_TARGET}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_LINK_BENCH_TARGET}>
    COMMENT "Generating one-shot fixed-forward wireless bench image (no flashing)")

# 无ESP32独立矩形：KEY明确触发、默认不动、单圈、分段/全程时限。旧镜像不替换。
## New streaming image, deliberately excluded from the default build. Never flashed here.
set(FOUR_MOTION_TARGET mecanum_f407_four_link_motion)
get_target_property(FOUR_MOTION_SOURCES ${FOUR_LINK_BENCH_TARGET} SOURCES)
list(REMOVE_ITEM FOUR_MOTION_SOURCES Src/four_link_bench.c)
add_executable(${FOUR_MOTION_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_MOTION_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_MOTION_TARGET} PROPERTY ${property} ${FOUR_MOTION_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_MOTION_TARGET} PRIVATE FOUR_LINK_MOTION=1)
get_target_property(FOUR_MOTION_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_MOTION_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_MOTION_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_MOTION_TARGET}.map)
set_property(TARGET ${FOUR_MOTION_TARGET} PROPERTY LINK_OPTIONS ${FOUR_MOTION_OPTIONS})
target_link_libraries(${FOUR_MOTION_TARGET} PRIVATE m)
set_target_properties(${FOUR_MOTION_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_MOTION_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_MOTION_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_MOTION_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_MOTION_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded streaming image; NO FLASHING")

# Separate ground-commissioning image.  The suspended 4WM1 binary and target
# remain unchanged and independently reproducible.
set(FOUR_GROUND_TARGET mecanum_f407_four_link_ground)
add_executable(${FOUR_GROUND_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND_TARGET} PROPERTY ${property} ${FOUR_GROUND_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND=1)
get_target_property(FOUR_GROUND_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND_TARGET}.map)
set_property(TARGET ${FOUR_GROUND_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND_OPTIONS})
target_link_libraries(${FOUR_GROUND_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground image; NO FLASHING")

# Second, still bounded ground profile.  Kept as its own target so 4WG1 stays
# reproducible and its flashed/read-back evidence is never overwritten.
set(FOUR_GROUND2_TARGET mecanum_f407_four_link_ground2)
add_executable(${FOUR_GROUND2_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND2_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND2_TARGET} PROPERTY ${property} ${FOUR_GROUND2_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND2_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND2=1)
get_target_property(FOUR_GROUND2_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND2_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND2_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND2_TARGET}.map)
set_property(TARGET ${FOUR_GROUND2_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND2_OPTIONS})
target_link_libraries(${FOUR_GROUND2_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND2_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND2_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND2_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND2_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND2_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground-v2 image; NO FLASHING")

# Same 35% envelope as 4WG2, with a bumpless startup-to-PI handoff.
set(FOUR_GROUND3_TARGET mecanum_f407_four_link_ground3)
add_executable(${FOUR_GROUND3_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND3_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND3_TARGET} PROPERTY ${property} ${FOUR_GROUND3_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND3_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND3=1)
get_target_property(FOUR_GROUND3_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND3_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND3_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND3_TARGET}.map)
set_property(TARGET ${FOUR_GROUND3_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND3_OPTIONS})
target_link_libraries(${FOUR_GROUND3_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND3_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND3_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND3_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND3_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND3_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground-v3 image; NO FLASHING")

# Same proven motor envelope as 4WG3.  Idle disabled-zero lease expiry is
# recoverable, while an owned permit or active output still uses the unchanged
# 150 ms fail-closed command deadline.
set(FOUR_GROUND4_TARGET mecanum_f407_four_link_ground4)
add_executable(${FOUR_GROUND4_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND4_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND4_TARGET} PROPERTY ${property} ${FOUR_GROUND4_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND4_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND4=1)
get_target_property(FOUR_GROUND4_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND4_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND4_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND4_TARGET}.map)
set_property(TARGET ${FOUR_GROUND4_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND4_OPTIONS})
target_link_libraries(${FOUR_GROUND4_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND4_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND4_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND4_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND4_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND4_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground-v4 image; NO FLASHING")

# Same fail-closed envelope as 4WG4, but do not let a tiny streamed ramp target
# cancel the bounded startup assist after the first encoder pulse.
set(FOUR_GROUND5_TARGET mecanum_f407_four_link_ground5)
add_executable(${FOUR_GROUND5_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND5_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND5_TARGET} PROPERTY ${property} ${FOUR_GROUND5_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND5_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND5=1)
get_target_property(FOUR_GROUND5_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND5_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND5_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND5_TARGET}.map)
set_property(TARGET ${FOUR_GROUND5_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND5_OPTIONS})
target_link_libraries(${FOUR_GROUND5_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND5_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND5_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND5_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND5_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND5_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground-v5 image; NO FLASHING")

# Same output, startup and fail-closed limits as 4WG5. Mecanum cancellation
# terms below 20 cps are explicitly zeroed instead of being treated as a
# trackable wheel demand and later producing a false stall.
set(FOUR_GROUND6_TARGET mecanum_f407_four_link_ground6)
add_executable(${FOUR_GROUND6_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND6_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND6_TARGET} PROPERTY ${property} ${FOUR_GROUND6_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND6_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND6=1)
get_target_property(FOUR_GROUND6_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND6_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND6_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND6_TARGET}.map)
set_property(TARGET ${FOUR_GROUND6_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND6_OPTIONS})
target_link_libraries(${FOUR_GROUND6_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND6_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND6_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND6_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND6_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND6_TARGET}>
    COMMENT "Generating OFFLINE-ONLY bounded ground-v6 image; NO FLASHING")

# Mature-controller replacement for the unflashed 4WG6 draft: per-wheel
# kS+kV feedforward, reduced low-speed PI, slew-bounded breakaway, immediate
# release on encoder progress, and target/duty-qualified stall detection.
set(FOUR_GROUND7_TARGET mecanum_f407_four_link_ground7)
add_executable(${FOUR_GROUND7_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND7_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND7_TARGET} PROPERTY ${property} ${FOUR_GROUND7_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND7_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND7=1)
get_target_property(FOUR_GROUND7_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND7_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND7_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND7_TARGET}.map)
set_property(TARGET ${FOUR_GROUND7_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND7_OPTIONS})
target_link_libraries(${FOUR_GROUND7_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND7_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND7_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND7_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND7_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND7_TARGET}>
    COMMENT "Generating OFFLINE-ONLY feedforward+PI ground-v7 image; NO FLASHING")

# Large-course candidate.  The accepted 4WG7 target remains byte-reproducible;
# 4WG8 alone raises the kinematic/encoder envelope to 0.20 m/s and the absolute
# run budget to 150 s while retaining the 35% PWM and 150 ms lease limits.
set(FOUR_GROUND8_TARGET mecanum_f407_four_link_ground8)
add_executable(${FOUR_GROUND8_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND8_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND8_TARGET} PROPERTY ${property} ${FOUR_GROUND8_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND8_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND8=1)
get_target_property(FOUR_GROUND8_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND8_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND8_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND8_TARGET}.map)
set_property(TARGET ${FOUR_GROUND8_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND8_OPTIONS})
target_link_libraries(${FOUR_GROUND8_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND8_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND8_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND8_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND8_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND8_TARGET}>
    COMMENT "Generating OFFLINE-ONLY 0.20 m/s ground-v8 image; NO FLASHING")

# 4WG9 keeps the exact 4WG8 motor/velocity envelope.  Its only new authority
# is an explicit run-budget rearm after disabled zero, fault-free feedback and
# 500 ms measured standstill; it never clears a real fault.
set(FOUR_GROUND9_TARGET mecanum_f407_four_link_ground9)
add_executable(${FOUR_GROUND9_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUND9_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUND9_TARGET} PROPERTY ${property} ${FOUR_GROUND9_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUND9_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUND9=1)
get_target_property(FOUR_GROUND9_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUND9_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUND9_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND9_TARGET}.map)
set_property(TARGET ${FOUR_GROUND9_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUND9_OPTIONS})
target_link_libraries(${FOUR_GROUND9_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUND9_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUND9_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUND9_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUND9_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUND9_TARGET}>
    COMMENT "Generating OFFLINE-ONLY safe-rearm ground-v9 image; NO FLASHING")

# Recovery-demo profile.  It retains the 4WG9 actuator and safe-rearm logic,
# caps body translation at 0.10 m/s, and uses a fixed 600 ms MCU lease to
# tolerate the measured intermittent Wi-Fi receive stalls.  It is a distinct
# identity so a 4WG9 host can never silently acquire the wider lease.
set(FOUR_GROUNDA_TARGET mecanum_f407_four_link_grounda)
add_executable(${FOUR_GROUNDA_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUNDA_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUNDA_TARGET} PROPERTY ${property} ${FOUR_GROUNDA_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUNDA_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUNDA=1)
get_target_property(FOUR_GROUNDA_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUNDA_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUNDA_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUNDA_TARGET}.map)
set_property(TARGET ${FOUR_GROUNDA_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUNDA_OPTIONS})
target_link_libraries(${FOUR_GROUNDA_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUNDA_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUNDA_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUNDA_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUNDA_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUNDA_TARGET}>
    COMMENT "Generating OFFLINE-ONLY 0.10 m/s recovery-demo ground-vA image; NO FLASHING")

# Faster recovery-demo profile.  It changes only the body-axis ceiling to
# 0.15 m/s; 4WGA's 600 ms lease, 35% PWM envelope, safe rearm and 150 s
# independent run budget remain unchanged.  The distinct tag prevents a host
# configured for either speed from silently accepting the other image.
set(FOUR_GROUNDB_TARGET mecanum_f407_four_link_groundb)
add_executable(${FOUR_GROUNDB_TARGET} EXCLUDE_FROM_ALL ${FOUR_MOTION_SOURCES} Src/four_link_motion.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS)
    get_target_property(FOUR_GROUNDB_PROPERTY ${FOUR_LINK_BENCH_TARGET} ${property})
    set_property(TARGET ${FOUR_GROUNDB_TARGET} PROPERTY ${property} ${FOUR_GROUNDB_PROPERTY})
endforeach()
target_compile_definitions(${FOUR_GROUNDB_TARGET} PRIVATE FOUR_LINK_MOTION=1 FOUR_LINK_GROUNDB=1)
get_target_property(FOUR_GROUNDB_OPTIONS ${PROJECT_NAME} LINK_OPTIONS)
list(REMOVE_ITEM FOUR_GROUNDB_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND FOUR_GROUNDB_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUNDB_TARGET}.map)
set_property(TARGET ${FOUR_GROUNDB_TARGET} PROPERTY LINK_OPTIONS ${FOUR_GROUNDB_OPTIONS})
target_link_libraries(${FOUR_GROUNDB_TARGET} PRIVATE m)
set_target_properties(${FOUR_GROUNDB_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${FOUR_GROUNDB_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${FOUR_GROUNDB_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${FOUR_GROUNDB_TARGET}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${FOUR_GROUNDB_TARGET}>
    COMMENT "Generating OFFLINE-ONLY 0.15 m/s recovery-demo ground-vB image; NO FLASHING")

set(RECTANGLE_DEMO_TARGET mecanum_f407_rectangle_demo)
add_executable(${RECTANGLE_DEMO_TARGET} ${SPEED_BENCH_SOURCES}
    Src/rectangle_demo_main.c Src/rectangle_demo.c Src/bench_four_msp.c
    Src/encoder_accumulator.c Src/four_drive_group.c Src/four_wheel_speed.c
    Src/mecanum_drive_plan.c Src/mecanum_kinematics.c
    Src/speed_uart_queue.c Src/speed_uart_transport.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(RECTANGLE_PROPERTY ${PROJECT_NAME} ${property})
    set_property(TARGET ${RECTANGLE_DEMO_TARGET} PROPERTY ${property} ${RECTANGLE_PROPERTY})
endforeach()
target_compile_definitions(${RECTANGLE_DEMO_TARGET} PRIVATE
    AUTONOMOUS_RECTANGLE=1 DRIVE_STARTUP_OBSERVE=1 SPEED_UART_FRAME_BYTES=768U)
get_target_property(RECTANGLE_OPTIONS ${RECTANGLE_DEMO_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM RECTANGLE_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map)
list(APPEND RECTANGLE_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_DEMO_TARGET}.map)
set_property(TARGET ${RECTANGLE_DEMO_TARGET} PROPERTY LINK_OPTIONS ${RECTANGLE_OPTIONS})
target_link_libraries(${RECTANGLE_DEMO_TARGET} PRIVATE m)
set_target_properties(${RECTANGLE_DEMO_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${RECTANGLE_DEMO_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RECTANGLE_DEMO_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_DEMO_TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RECTANGLE_DEMO_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_DEMO_TARGET}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RECTANGLE_DEMO_TARGET}>
    COMMENT "Generating KEY-triggered single rectangle (no flashing)")

# 独立启动诊断：不换参数，只记录真实CCR/四轮反馈，首段最多5秒且禁止换段。
set(RECTANGLE_TRACE_TARGET mecanum_f407_rectangle_startup_trace)
get_target_property(RECTANGLE_TRACE_SOURCES ${RECTANGLE_DEMO_TARGET} SOURCES)
add_executable(${RECTANGLE_TRACE_TARGET} ${RECTANGLE_TRACE_SOURCES} Src/rectangle_startup_trace.c)
foreach(property INCLUDE_DIRECTORIES COMPILE_DEFINITIONS COMPILE_OPTIONS LINK_OPTIONS)
    get_target_property(RECTANGLE_TRACE_PROPERTY ${RECTANGLE_DEMO_TARGET} ${property})
    set_property(TARGET ${RECTANGLE_TRACE_TARGET} PROPERTY ${property} ${RECTANGLE_TRACE_PROPERTY})
endforeach()
target_compile_definitions(${RECTANGLE_TRACE_TARGET} PRIVATE RECTANGLE_STARTUP_TRACE=1)
get_target_property(RECTANGLE_TRACE_OPTIONS ${RECTANGLE_TRACE_TARGET} LINK_OPTIONS)
list(REMOVE_ITEM RECTANGLE_TRACE_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_DEMO_TARGET}.map)
list(APPEND RECTANGLE_TRACE_OPTIONS -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_TRACE_TARGET}.map)
set_property(TARGET ${RECTANGLE_TRACE_TARGET} PROPERTY LINK_OPTIONS ${RECTANGLE_TRACE_OPTIONS})
target_link_libraries(${RECTANGLE_TRACE_TARGET} PRIVATE m)
set_target_properties(${RECTANGLE_TRACE_TARGET} PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET ${RECTANGLE_TRACE_TARGET} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${RECTANGLE_TRACE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_TRACE_TARGET}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${RECTANGLE_TRACE_TARGET}> ${CMAKE_CURRENT_BINARY_DIR}/${RECTANGLE_TRACE_TARGET}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${RECTANGLE_TRACE_TARGET}>
    COMMENT "Generating first-leg-only KEY startup recorder (no flashing)")
