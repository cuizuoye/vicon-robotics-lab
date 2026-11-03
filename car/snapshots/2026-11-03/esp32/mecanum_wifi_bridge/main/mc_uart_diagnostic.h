#ifndef MC_UART_DIAGNOSTIC_H
#define MC_UART_DIAGNOSTIC_H
#include "mc_protocol.h"

typedef struct MC_PACKED {
    uint32_t tag, ticks, updates, dt_min, dt_max, irq_us_max;
    uint32_t compares[8];
    uint16_t gpio_low, watchdog_reset;
} mc_readonly_status_t;
_Static_assert(sizeof(mc_readonly_status_t) == 60, "readonly status size");

typedef struct {
    uint32_t wheel_frames, status_frames, invalid_frames, last_sequence;
    bool sequence_seen, unsafe_seen;
    mc_wheel_telemetry_v1_t wheel;
    mc_readonly_status_t status;
} mc_uart_diagnostic_t;

bool mc_uart_ack_matches(const uint8_t *frame, size_t length,
                         uint32_t session, uint32_t sequence);
bool mc_uart_diagnostic_accept(mc_uart_diagnostic_t *state,
                              const uint8_t *frame, size_t length,
                              uint32_t session);
bool mc_uart_diagnostic_safe(const mc_uart_diagnostic_t *state);
#endif
