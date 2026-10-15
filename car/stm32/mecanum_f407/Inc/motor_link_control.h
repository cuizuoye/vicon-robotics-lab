#ifndef MOTOR_LINK_CONTROL_H
#define MOTOR_LINK_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    MOTOR_LINK_EVENT_NONE = 0,
    MOTOR_LINK_EVENT_HELLO,
    MOTOR_LINK_EVENT_COMMAND,
    MOTOR_LINK_EVENT_ESTOP,
    MOTOR_LINK_EVENT_CONTROL,
} motor_link_event_t;

typedef struct {
    bool session_active;
    bool output_enabled;
    bool calibrated;
    bool timeout_latched;
    uint32_t session_id;
    uint32_t last_sequence;
    uint32_t last_command_ms;
    uint16_t command_timeout_ms;
    int16_t target_vx_mm_s;
    uint32_t fault_bits;
} motor_link_control_t;

void motor_link_control_init(motor_link_control_t *control, bool calibrated);
motor_link_event_t motor_link_control_accept(
    motor_link_control_t *control,
    const uint8_t *frame,
    size_t frame_length,
    uint32_t now_ms);
bool motor_link_control_tick(motor_link_control_t *control, uint32_t now_ms);
void motor_link_control_force_fault(
    motor_link_control_t *control, uint32_t fault_bits);
int32_t motor_link_control_target_cps(const motor_link_control_t *control);
uint16_t motor_link_control_state_flags(const motor_link_control_t *control);

#define MOTOR_LINK_STATE_SESSION_ACTIVE  (1U << 0)
#define MOTOR_LINK_STATE_OUTPUT_ENABLED  (1U << 1)
#define MOTOR_LINK_STATE_CALIBRATED      (1U << 2)
#define MOTOR_LINK_STATE_HIGH_Z          (1U << 3)
#define MOTOR_LINK_STATE_TIMEOUT_LATCHED (1U << 4)

#endif
