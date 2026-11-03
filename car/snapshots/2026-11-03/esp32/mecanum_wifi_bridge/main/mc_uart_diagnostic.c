#include "mc_uart_diagnostic.h"
#include <string.h>

bool mc_uart_ack_matches(const uint8_t *frame, size_t length,
                         uint32_t session, uint32_t sequence)
{
    mc_frame_header_v1_t h;
    return mc_validate_frame(frame, length, &h)
        && h.message_type == MC_MSG_HELLO_ACK && h.flags == MC_FLAG_RESPONSE
        && h.session_id == session && h.sequence == sequence && h.payload_length == 0;
}

bool mc_uart_diagnostic_accept(mc_uart_diagnostic_t *s,
                              const uint8_t *frame, size_t length,
                              uint32_t session)
{
    mc_frame_header_v1_t h;
    if (!s) return false;
    if (!mc_validate_frame(frame, length, &h) || h.session_id != session
        || (h.message_type != MC_MSG_WHEEL_TELEMETRY && h.message_type != MC_MSG_LINK_STATUS)
        || (s->sequence_seen && !mc_sequence_is_newer(h.sequence, s->last_sequence))) {
        ++s->invalid_frames;
        return false;
    }
    if ((h.message_type == MC_MSG_WHEEL_TELEMETRY && h.payload_length != sizeof(s->wheel))
        || (h.message_type == MC_MSG_LINK_STATUS && h.payload_length != sizeof(s->status))) {
        ++s->invalid_frames;
        return false;
    }
    s->last_sequence = h.sequence;
    s->sequence_seen = true;
    if (h.flags != 0) s->unsafe_seen = true;
    const uint8_t *payload = frame + sizeof(h);
    if (h.message_type == MC_MSG_WHEEL_TELEMETRY) {
        memcpy(&s->wheel, payload, sizeof(s->wheel));
        ++s->wheel_frames;
        /* SESSION + CALIBRATED + HIGH_Z + READONLY + FOUR_WHEELS, no ENABLED. */
        if (s->wheel.state_flags != 109 || s->wheel.fault_bits || s->wheel.last_command_sequence)
            s->unsafe_seen = true;
        for (unsigned i = 0; i < 4; ++i)
            if (s->wheel.wheel_rpm_x10[i]) s->unsafe_seen = true;
    } else {
        memcpy(&s->status, payload, sizeof(s->status));
        ++s->status_frames;
        if (s->status.tag != UINT32_C(0x314f5234) || s->status.gpio_low != 1
            || s->status.watchdog_reset || s->status.dt_min != 10 || s->status.dt_max != 10
            || s->status.ticks != s->status.updates) s->unsafe_seen = true;
        for (unsigned i = 0; i < 8; ++i)
            if (s->status.compares[i]) s->unsafe_seen = true;
    }
    return true;
}

bool mc_uart_diagnostic_safe(const mc_uart_diagnostic_t *s)
{
    return s && s->wheel_frames && s->status_frames && !s->unsafe_seen && !s->invalid_frames;
}
