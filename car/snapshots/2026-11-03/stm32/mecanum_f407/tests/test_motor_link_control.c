#include "app_config.h"
#include "mc_protocol.h"
#include "motor_link_control.h"

#include <assert.h>
#include <stdio.h>

static size_t build_frame(
    uint8_t type,
    uint16_t flags,
    uint32_t sequence,
    uint32_t session,
    const void *payload,
    uint16_t payload_length,
    uint8_t *frame)
{
    size_t length = 0U;
    assert(mc_build_frame(
        type, flags, sequence, session, 0U, payload, payload_length,
        frame, MC_MAX_RAW_FRAME_SIZE, &length));
    return length;
}

static void accept_hello(
    motor_link_control_t *control,
    uint32_t sequence,
    uint32_t session)
{
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
    const size_t length = build_frame(
        MC_MSG_HELLO, 0U, sequence, session, NULL, 0U, frame);
    assert(motor_link_control_accept(control, frame, length, 10U)
           == MOTOR_LINK_EVENT_HELLO);
}

static motor_link_event_t accept_velocity(
    motor_link_control_t *control,
    uint32_t sequence,
    uint32_t session,
    uint16_t flags,
    int16_t vx,
    int16_t vy,
    int16_t wz,
    uint32_t now_ms)
{
    const mc_cmd_vel_v1_t command = {
        .vx_mm_s = vx,
        .vy_mm_s = vy,
        .wz_mrad_s = wz,
        .timeout_ms = 150U,
        .control_mode = 0U,
        .reserved = 0U,
    };
    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
    const size_t length = build_frame(
        MC_MSG_CMD_VEL, flags, sequence, session,
        &command, sizeof(command), frame);
    return motor_link_control_accept(control, frame, length, now_ms);
}

static void test_uncalibrated_never_enables(void)
{
    motor_link_control_t control;
    motor_link_control_init(&control, false);
    assert(control.fault_bits == FAULT_CONFIG_UNCALIBRATED);
    accept_hello(&control, 1U, 7U);
    assert(accept_velocity(
        &control, 2U, 7U, MC_FLAG_ENABLED, 100, 0, 0, 20U)
        == MOTOR_LINK_EVENT_COMMAND);
    assert(!control.output_enabled);
    assert(motor_link_control_target_cps(&control) == 0);
}

static void test_sequence_timeout_and_new_session(void)
{
    motor_link_control_t control;
    motor_link_control_init(&control, true);
    accept_hello(&control, 10U, 0x1234U);
    assert(accept_velocity(
        &control, 11U, 0x1234U, MC_FLAG_ENABLED, 100, 0, 0, 100U)
        == MOTOR_LINK_EVENT_COMMAND);
    assert(control.output_enabled);
    assert(motor_link_control_target_cps(&control) > 0);

    assert(accept_velocity(
        &control, 11U, 0x1234U, MC_FLAG_ENABLED, 200, 0, 0, 110U)
        == MOTOR_LINK_EVENT_NONE);
    assert(control.target_vx_mm_s == 100);
    assert(!motor_link_control_tick(&control, 250U));
    assert(motor_link_control_tick(&control, 251U));
    assert(!control.output_enabled);
    assert(control.timeout_latched);
    assert((control.fault_bits & FAULT_COMMAND_TIMEOUT) != 0U);

    accept_hello(&control, 1U, 0x5678U);
    assert(!control.timeout_latched);
    assert((control.fault_bits & FAULT_COMMAND_TIMEOUT) == 0U);
}

static void test_estop_requires_explicit_clear(void)
{
    motor_link_control_t control;
    motor_link_control_init(&control, true);
    accept_hello(&control, 1U, 9U);

    uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
    size_t length = build_frame(
        MC_MSG_ESTOP, 0U, 2U, 9U, NULL, 0U, frame);
    assert(motor_link_control_accept(&control, frame, length, 20U)
           == MOTOR_LINK_EVENT_ESTOP);
    assert((control.fault_bits & FAULT_ESTOP) != 0U);

    accept_hello(&control, 3U, 10U);
    assert((control.fault_bits & FAULT_ESTOP) != 0U);
    const mc_control_enable_v1_t clear = {
        .enabled = 0U,
        .clear_latched_faults = 1U,
        .reserved = 0U,
    };
    length = build_frame(
        MC_MSG_CONTROL_ENABLE, 0U, 4U, 10U,
        &clear, sizeof(clear), frame);
    assert(motor_link_control_accept(&control, frame, length, 30U)
           == MOTOR_LINK_EVENT_CONTROL);
    assert(control.fault_bits == FAULT_NONE);
}

static void test_single_motor_rejects_lateral_or_yaw_motion(void)
{
    motor_link_control_t control;
    motor_link_control_init(&control, true);
    accept_hello(&control, 1U, 5U);
    assert(accept_velocity(
        &control, 2U, 5U, MC_FLAG_ENABLED, 0, 1, 0, 20U)
        == MOTOR_LINK_EVENT_COMMAND);
    assert(!control.output_enabled);
    assert((control.fault_bits & FAULT_UNSUPPORTED_MOTION) != 0U);
}

int main(void)
{
    test_uncalibrated_never_enables();
    test_sequence_timeout_and_new_session();
    test_estop_requires_explicit_clear();
    test_single_motor_rejects_lateral_or_yaw_motion();
    puts("motor_link_control_test: PASS");
    return 0;
}
