#include "speed_controller.h"

#include "app_config.h"

#include <stddef.h>

static int32_t clamp_i32(int32_t value, int32_t lower, int32_t upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

void speed_controller_init(speed_controller_t *controller, uint32_t now_ms)
{
    controller->target_cps = 0;
    controller->integral_cps_ms = 0;
    controller->previous_update_ms = now_ms;
    controller->enabled = false;
}

void speed_controller_set_target(
    speed_controller_t *controller, int32_t target_cps, uint32_t now_ms)
{
    const int32_t clamped_target = clamp_i32(
        target_cps, -SPEED_TARGET_MAX_CPS, SPEED_TARGET_MAX_CPS);
    if (!controller->enabled || controller->target_cps != clamped_target) {
        controller->integral_cps_ms = 0;
        controller->previous_update_ms = now_ms;
    }
    controller->target_cps = clamped_target;
    controller->enabled = true;
}

void speed_controller_disable(speed_controller_t *controller)
{
    controller->target_cps = 0;
    controller->integral_cps_ms = 0;
    controller->enabled = false;
}

bool speed_controller_tick(
    speed_controller_t *controller,
    int32_t measured_cps,
    uint32_t now_ms,
    int16_t *duty_permille)
{
    if (!controller->enabled || duty_permille == NULL) {
        return false;
    }

    const uint32_t elapsed_ms = now_ms - controller->previous_update_ms;
    if (elapsed_ms < ENCODER_SAMPLE_PERIOD_MS) {
        return false;
    }
    controller->previous_update_ms = now_ms;

    if (controller->target_cps == 0) {
        controller->integral_cps_ms = 0;
        *duty_permille = 0;
        return true;
    }

    const int32_t error = controller->target_cps - measured_cps;
    const int64_t integral_limit =
        ((int64_t)MOTOR_BRINGUP_MAX_PERMILLE * SPEED_KI_DENOMINATOR) /
        SPEED_KI_NUMERATOR;
    controller->integral_cps_ms += (int64_t)error * elapsed_ms;
    if (controller->integral_cps_ms > integral_limit) {
        controller->integral_cps_ms = integral_limit;
    } else if (controller->integral_cps_ms < -integral_limit) {
        controller->integral_cps_ms = -integral_limit;
    }

    const int64_t proportional =
        ((int64_t)error * SPEED_KP_NUMERATOR) / SPEED_KP_DENOMINATOR;
    const int64_t integral =
        (controller->integral_cps_ms * SPEED_KI_NUMERATOR) /
        SPEED_KI_DENOMINATOR;
    const int32_t output = clamp_i32(
        (int32_t)(proportional + integral),
        -MOTOR_BRINGUP_MAX_PERMILLE,
        MOTOR_BRINGUP_MAX_PERMILLE);
    *duty_permille = (int16_t)output;
    return true;
}

int32_t speed_controller_target(const speed_controller_t *controller)
{
    return controller->target_cps;
}

bool speed_controller_enabled(const speed_controller_t *controller)
{
    return controller->enabled;
}
