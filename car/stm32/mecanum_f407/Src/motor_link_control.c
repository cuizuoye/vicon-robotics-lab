#include "motor_link_control.h"

#include "app_config.h"
#include "mc_protocol.h"

#include <string.h>

static void disable_output(motor_link_control_t *control)
{
    control->output_enabled = false;
    control->target_vx_mm_s = 0;
}

void motor_link_control_init(motor_link_control_t *control, bool calibrated)
{
    memset(control, 0, sizeof(*control));
    control->calibrated = calibrated;
    control->command_timeout_ms = 150U;
    if (!calibrated) {
        control->fault_bits |= FAULT_CONFIG_UNCALIBRATED;
    }
}

static motor_link_event_t accept_hello(
    motor_link_control_t *control,
    const mc_frame_header_v1_t *header)
{
    if (header->payload_length != 0U || header->session_id == 0U) {
        return MOTOR_LINK_EVENT_NONE;
    }
    disable_output(control);
    control->session_active = true;
    control->timeout_latched = false;
    control->session_id = header->session_id;
    control->last_sequence = header->sequence;
    control->fault_bits &= ~FAULT_COMMAND_TIMEOUT;
    return MOTOR_LINK_EVENT_HELLO;
}

static bool valid_active_sequence(
    const motor_link_control_t *control,
    const mc_frame_header_v1_t *header)
{
    return control->session_active
        && header->session_id == control->session_id
        && mc_sequence_is_newer(header->sequence, control->last_sequence);
}

static motor_link_event_t accept_cmd_vel(
    motor_link_control_t *control,
    const mc_frame_header_v1_t *header,
    const uint8_t *payload,
    uint32_t now_ms)
{
    if (header->payload_length != sizeof(mc_cmd_vel_v1_t)) {
        return MOTOR_LINK_EVENT_NONE;
    }
    mc_cmd_vel_v1_t command;
    memcpy(&command, payload, sizeof(command));
    if (command.timeout_ms < 20U || command.timeout_ms > 2000U
        || command.control_mode != 0U || command.reserved != 0U) {
        return MOTOR_LINK_EVENT_NONE;
    }

    control->last_sequence = header->sequence;
    control->last_command_ms = now_ms;
    control->command_timeout_ms = command.timeout_ms;

    if ((header->flags & MC_FLAG_ENABLED) == 0U) {
        disable_output(control);
        return MOTOR_LINK_EVENT_COMMAND;
    }
    if (command.vy_mm_s != 0 || command.wz_mrad_s != 0) {
        control->fault_bits |= FAULT_UNSUPPORTED_MOTION;
        disable_output(control);
        return MOTOR_LINK_EVENT_COMMAND;
    }
    if (control->fault_bits != FAULT_NONE || control->timeout_latched) {
        disable_output(control);
        return MOTOR_LINK_EVENT_COMMAND;
    }

    control->target_vx_mm_s = command.vx_mm_s;
    control->output_enabled = true;
    return MOTOR_LINK_EVENT_COMMAND;
}

static motor_link_event_t accept_control_enable(
    motor_link_control_t *control,
    const mc_frame_header_v1_t *header,
    const uint8_t *payload)
{
    if (header->payload_length != sizeof(mc_control_enable_v1_t)) {
        return MOTOR_LINK_EVENT_NONE;
    }
    mc_control_enable_v1_t command;
    memcpy(&command, payload, sizeof(command));
    if (command.enabled > 1U || command.clear_latched_faults > 1U
        || command.reserved != 0U) {
        return MOTOR_LINK_EVENT_NONE;
    }
    control->last_sequence = header->sequence;
    if (command.enabled == 0U) {
        disable_output(control);
        if (command.clear_latched_faults != 0U) {
            control->fault_bits &= FAULT_CONFIG_UNCALIBRATED;
            control->timeout_latched = false;
        }
    }
    return MOTOR_LINK_EVENT_CONTROL;
}

motor_link_event_t motor_link_control_accept(
    motor_link_control_t *control,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t now_ms)
{
    mc_frame_header_v1_t header;
    if (!mc_validate_frame(frame, frame_length, &header)) {
        return MOTOR_LINK_EVENT_NONE;
    }
    if (header.message_type == MC_MSG_HELLO) {
        return accept_hello(control, &header);
    }
    if (!valid_active_sequence(control, &header)) {
        return MOTOR_LINK_EVENT_NONE;
    }

    const uint8_t *payload = frame + sizeof(header);
    switch (header.message_type) {
    case MC_MSG_CMD_VEL:
        return accept_cmd_vel(control, &header, payload, now_ms);
    case MC_MSG_CONTROL_ENABLE:
        return accept_control_enable(control, &header, payload);
    case MC_MSG_ESTOP:
        if (header.payload_length != 0U) {
            return MOTOR_LINK_EVENT_NONE;
        }
        control->last_sequence = header.sequence;
        control->fault_bits |= FAULT_ESTOP;
        disable_output(control);
        return MOTOR_LINK_EVENT_ESTOP;
    case MC_MSG_HEARTBEAT:
        if (header.payload_length != 0U) {
            return MOTOR_LINK_EVENT_NONE;
        }
        control->last_sequence = header.sequence;
        return MOTOR_LINK_EVENT_CONTROL;
    default:
        return MOTOR_LINK_EVENT_NONE;
    }
}

bool motor_link_control_tick(motor_link_control_t *control, uint32_t now_ms)
{
    if (!control->output_enabled) {
        return false;
    }
    if ((uint32_t)(now_ms - control->last_command_ms)
        <= control->command_timeout_ms) {
        return false;
    }
    disable_output(control);
    control->timeout_latched = true;
    control->fault_bits |= FAULT_COMMAND_TIMEOUT;
    return true;
}

void motor_link_control_force_fault(
    motor_link_control_t *control, uint32_t fault_bits)
{
    control->fault_bits |= fault_bits;
    disable_output(control);
}

int32_t motor_link_control_target_cps(const motor_link_control_t *control)
{
    if (!control->calibrated || !control->output_enabled
        || MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV == 0U) {
        return 0;
    }
    const int64_t numerator =
        (int64_t)control->target_vx_mm_s
        * MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV * 10000;
    const int64_t denominator =
        (int64_t)MOTOR_A_WHEEL_DIAMETER_MM * 31416;
    return (int32_t)(numerator / denominator);
}

uint16_t motor_link_control_state_flags(const motor_link_control_t *control)
{
    uint16_t flags = 0U;
    if (control->session_active) {
        flags |= MOTOR_LINK_STATE_SESSION_ACTIVE;
    }
    if (control->output_enabled) {
        flags |= MOTOR_LINK_STATE_OUTPUT_ENABLED;
    } else {
        flags |= MOTOR_LINK_STATE_HIGH_Z;
    }
    if (control->calibrated) {
        flags |= MOTOR_LINK_STATE_CALIBRATED;
    }
    if (control->timeout_latched) {
        flags |= MOTOR_LINK_STATE_TIMEOUT_LATCHED;
    }
    return flags;
}
