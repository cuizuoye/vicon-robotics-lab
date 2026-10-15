#include "motor_a.h"

#include "app_config.h"
#include "board.h"

#include <stdbool.h>

static TIM_HandleTypeDef *motor_timer;
static int16_t applied_command;
static int16_t pending_command;
static uint32_t pending_at_ms;
static bool pending_valid;
static bool output_high_z;

static int16_t clamp_command(int16_t command)
{
    if (command > MOTOR_BRINGUP_MAX_PERMILLE) {
        return MOTOR_BRINGUP_MAX_PERMILLE;
    }
    if (command < -MOTOR_BRINGUP_MAX_PERMILLE) {
        return -MOTOR_BRINGUP_MAX_PERMILLE;
    }
    return command;
}

static void apply_high_z(void)
{
    __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_2, 0U);
    applied_command = 0;
    output_high_z = true;
}

static void apply_brake(void)
{
    __HAL_TIM_SET_COMPARE(
        motor_timer, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_COUNTS);
    __HAL_TIM_SET_COMPARE(
        motor_timer, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_COUNTS);
    applied_command = 0;
    output_high_z = false;
}

static void apply_immediately(int16_t command)
{
    const uint32_t magnitude = (uint32_t)(command < 0 ? -command : command);
    const uint32_t drive_counts =
        (magnitude * MOTOR_PWM_PERIOD_COUNTS) / 1000U;
    const uint32_t decay_compare = MOTOR_PWM_PERIOD_COUNTS - drive_counts;

    /* D157B unipolar slow decay; the low phase of PWM1 applies drive. */
    if (command > 0) {
        __HAL_TIM_SET_COMPARE(
            motor_timer, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_COUNTS);
        __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_2, decay_compare);
    } else if (command < 0) {
        __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_1, decay_compare);
        __HAL_TIM_SET_COMPARE(
            motor_timer, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_COUNTS);
    } else {
        apply_brake();
        return;
    }
    applied_command = command;
    output_high_z = false;
}

void motor_a_init(TIM_HandleTypeDef *timer)
{
    motor_timer = timer;
    applied_command = 0;
    pending_command = 0;
    pending_at_ms = 0U;
    pending_valid = false;
    output_high_z = true;

    __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(motor_timer, TIM_CHANNEL_2, 0U);
    if (HAL_TIM_PWM_Start(motor_timer, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(motor_timer, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
}

void motor_a_request(int16_t duty_permille, uint32_t now_ms)
{
    const int16_t command = clamp_command(duty_permille);
    const bool reversing =
        (applied_command > 0 && command < 0) ||
        (applied_command < 0 && command > 0);

    if (reversing) {
        apply_high_z();
        pending_command = command;
        pending_at_ms = now_ms + MOTOR_DIRECTION_DEADTIME_MS;
        pending_valid = true;
        return;
    }

    pending_valid = false;
    apply_immediately(command);
}

void motor_a_tick(uint32_t now_ms)
{
    if (pending_valid && (int32_t)(now_ms - pending_at_ms) >= 0) {
        const int16_t command = pending_command;
        pending_valid = false;
        apply_immediately(command);
    }
}

void motor_a_brake(void)
{
    pending_valid = false;
    apply_brake();
}

void motor_a_stop(void)
{
    pending_valid = false;
    apply_high_z();
}

int16_t motor_a_command(void)
{
    return applied_command;
}

bool motor_a_is_high_z(void)
{
    return output_high_z;
}
