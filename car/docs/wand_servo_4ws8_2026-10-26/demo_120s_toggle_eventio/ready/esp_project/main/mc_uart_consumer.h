#ifndef MC_UART_CONSUMER_H
#define MC_UART_CONSUMER_H

#include "mc_protocol.h"

/* Session zero means no reader. This only gates received telemetry; it never
 * generates commands, acknowledges HELLO, or clears an MCU fault. */
static inline bool mc_uart_consumer_accepts(
    uint32_t consumer_session, const mc_frame_header_v1_t *header)
{
    return consumer_session != 0u && header != NULL
        && header->session_id == consumer_session;
}

#endif
