#ifndef FOUR_LINK_READONLY_H
#define FOUR_LINK_READONLY_H

#include "mc_protocol.h"
#include "speed_bench_control.h"

enum {
    FOUR_RO_TIMEOUT=1U, FOUR_RO_UART=2U, FOUR_RO_ESTOP=4U,
    FOUR_RO_CONFIG=8U, FOUR_RO_FORBIDDEN=16U, FOUR_RO_SAMPLE=256U
};
enum { FOUR_RO_SESSION=1U, FOUR_RO_CALIBRATED=4U, FOUR_RO_HIGH_Z=8U,
       FOUR_RO_READONLY=32U, FOUR_RO_FOUR_WHEELS=64U, FOUR_RO_EXPIRED=16U };
#define FOUR_RO_STATUS_TAG UINT32_C(0x314f5234) /* little endian: 4RO1 */
typedef struct {
    uint32_t session_id, last_sequence, last_command_sequence, last_command_ms, fault;
    bool session_active, command_seen, expired;
} four_ro_control_t;
typedef enum { FOUR_RO_NONE, FOUR_RO_HELLO, FOUR_RO_COMMAND, FOUR_RO_CONTROL } four_ro_event_t;
/* LINK_STATUS只读诊断扩展；正式40字节WHEEL_TELEMETRY布局不变。 */
typedef struct MC_PACKED {
    uint32_t tag, ticks, updates, dt_min, dt_max, irq_us_max;
    uint32_t compares[8];
    uint16_t gpio_low, watchdog_reset;
} four_ro_status_t;
_Static_assert(sizeof(four_ro_status_t)==60, "four-wheel readonly status size");

void four_ro_init(four_ro_control_t *control);
four_ro_event_t four_ro_accept(four_ro_control_t *control, const uint8_t *frame,
                               size_t length, uint32_t now);
bool four_ro_tick(four_ro_control_t *control, uint32_t now);
bool four_ro_payloads(const four_ro_control_t *control, const speed_bench_t *bench,
                      const speed_bench_diagnostics_t *diagnostics,
                      mc_wheel_telemetry_v1_t *telemetry, four_ro_status_t *status);
#endif
