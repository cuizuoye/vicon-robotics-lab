#ifndef FOUR_WHEEL_SPEED_H
#define FOUR_WHEEL_SPEED_H

#include <stdbool.h>
#include <stdint.h>
#include "four_speed_pwm_profile.h"

enum { FOUR_SPEED_FL = 0, FOUR_SPEED_FR, FOUR_SPEED_RL, FOUR_SPEED_RR,
       FOUR_SPEED_WHEELS };
/* Wheel indexes are FL=0, FR=1, RL=2, RR=3. */
typedef struct {
    float counts_per_rev;
    float diameter_m;
    int8_t motor_sign;
    int8_t encoder_sign;
    bool calibration_confirmed;
    float kp_permille_per_cps;
    float ki_permille_per_count;
} four_speed_wheel_config_t;

typedef struct {
    four_speed_wheel_config_t wheel[FOUR_SPEED_WHEELS];
    float target_limit_cps;
    float feedback_limit_cps;
    float ramp_cps_per_s;
    float feedback_filter_ms;
    int16_t duty_limit_permille;
    uint32_t sample_period_ms;
    uint32_t max_sample_gap_ms;
    uint32_t command_timeout_ms;
    uint32_t run_limit_ms;
} four_speed_config_t;

typedef struct {
    float forward_cps;
    float rpm;
    float speed_m_s;
} four_speed_measurement_t;

typedef struct {
    int32_t previous_count;
    float requested_cps;
    float ramped_cps;
    float integral_permille;
    float filtered_cps;
    four_speed_measurement_t measured;
    int16_t raw_duty_permille;
} four_speed_wheel_state_t;

enum {
    FOUR_SPEED_FAULT_CONFIG = 1U << 0,
    FOUR_SPEED_FAULT_TIMING = 1U << 1,
    FOUR_SPEED_FAULT_COMMAND = 1U << 2,
    FOUR_SPEED_FAULT_DURATION = 1U << 3,
    FOUR_SPEED_FAULT_FEEDBACK = 1U << 4,
    FOUR_SPEED_FAULT_STALL = 1U << 5
};

/* HAL-free algorithm component, not a flashed motor controller or ISR. */
typedef struct {
    four_speed_config_t config;
    four_speed_wheel_state_t wheel[FOUR_SPEED_WHEELS];
    uint32_t fault;
    uint32_t last_sample_ms;
    uint32_t last_command_ms;
    uint32_t started_ms;
    uint32_t last_progress_ms;
    bool progress_seen;
    bool enabled;
    uint8_t selected;
    int8_t run_direction;
} four_speed_controller_t;

void four_speed_defaults(four_speed_config_t *config);
bool four_speed_counts_to_measurement(const four_speed_wheel_config_t *config,
                                     int32_t raw_delta, uint32_t elapsed_ms,
                                     four_speed_measurement_t *measurement);
bool four_speed_m_s_to_cps(const four_speed_wheel_config_t *config,
                          float speed_m_s, float *target_cps);
bool four_speed_init(four_speed_controller_t *controller,
                     const four_speed_config_t *config);
/* 首次台架只允许单轮；未确认CPR或未设置增益时拒绝使能。 */
bool four_speed_arm(four_speed_controller_t *controller, uint8_t wheel,
                    const int32_t raw_counts[FOUR_SPEED_WHEELS], uint32_t now_ms);
bool four_speed_set_target(four_speed_controller_t *controller,
                           float forward_cps, uint32_t now_ms);
void four_speed_stop(four_speed_controller_t *controller, uint32_t fault);
bool four_speed_tick(four_speed_controller_t *controller,
                     const int32_t raw_counts[FOUR_SPEED_WHEELS], uint32_t now_ms);
/* 慢衰减映射；默认整1%，RR_FINE隔离版整0.1%，停机均为两路LOW。 */
bool four_speed_compare(int16_t raw_duty_permille, uint32_t *first,
                        uint32_t *second);

#endif
