#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t target_cps;
    int64_t integral_cps_ms;
    uint32_t previous_update_ms;
    bool enabled;
} speed_controller_t;

void speed_controller_init(speed_controller_t *controller, uint32_t now_ms);
void speed_controller_set_target(
    speed_controller_t *controller, int32_t target_cps, uint32_t now_ms);
void speed_controller_disable(speed_controller_t *controller);
bool speed_controller_tick(
    speed_controller_t *controller,
    int32_t measured_cps,
    uint32_t now_ms,
    int16_t *duty_permille);
int32_t speed_controller_target(const speed_controller_t *controller);
bool speed_controller_enabled(const speed_controller_t *controller);

#endif
