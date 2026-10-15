#ifndef MC_UART_PACKET_H
#define MC_UART_PACKET_H

#include "mc_protocol.h"

/* HELLO begins with an empty delimiter to synchronize an errored receiver.
 * All other packets retain COBS(raw)+delimiter. No new wire message is added. */
static inline bool mc_uart_packet_encode(const uint8_t *raw, size_t raw_length,
    uint8_t *out, size_t capacity, size_t *length)
{
    mc_frame_header_v1_t header;
    if (out == NULL || length == NULL
        || !mc_validate_frame(raw, raw_length, &header)) return false;
    const size_t prefix = header.message_type == MC_MSG_HELLO ? 1u : 0u;
    if (capacity <= prefix) return false;
    size_t encoded = 0u;
    if (!mc_cobs_encode(raw, raw_length, out + prefix,
                        capacity - prefix - 1u, &encoded)) return false;
    if (prefix) out[0] = 0u;
    *length = prefix + encoded;
    out[(*length)++] = 0u;
    return true;
}

#endif
