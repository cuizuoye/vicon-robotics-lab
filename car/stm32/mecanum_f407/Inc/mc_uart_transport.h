#ifndef MC_UART_TRANSPORT_H
#define MC_UART_TRANSPORT_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef MC_UART_DIAGNOSTICS
/* SWD-only observations; no protocol or fault-clear behavior is changed. */
typedef struct {
    uint32_t tag, rx_bytes, queued_frames, queue_overflows, packet_overflows;
    uint32_t error_callbacks, last_error_code, last_sr, last_error_ms;
    uint32_t rearm_failures, invalid_cobs, tx_failures;
    uint32_t irq_error_count, irq_error_sr, irq_error_ms;
} mc_uart_diagnostics_t;
extern volatile mc_uart_diagnostics_t mc_uart_diagnostics;
void mc_uart_diagnostics_capture_irq(UART_HandleTypeDef *uart);
#endif

void mc_uart_transport_init(UART_HandleTypeDef *uart);
bool mc_uart_transport_take_frame(
    uint8_t *raw, size_t capacity, size_t *raw_length);
bool mc_uart_transport_take_fault(void);
bool mc_uart_transport_send_frame(
    uint8_t message_type,
    uint16_t flags,
    uint32_t sequence,
    uint32_t session_id,
    uint32_t sender_time_ms,
    const void *payload,
    uint16_t payload_length);

#endif
