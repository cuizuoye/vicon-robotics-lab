#include "four_wheel_speed.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define WHEEL_PI 3.14159265358979323846f
#define INITIAL_PROGRESS_MS 800U
#define LOST_PROGRESS_MS 500U
#ifdef FOUR_LINK_MOTION
/* New bounded streaming image only; no change to the field-tested bench images. */
#define MAX_RUN_LIMIT_MS 120000U
#elif defined(AUTONOMOUS_RECTANGLE)
/* 仅独立离线矩形镜像；旧5/10秒台架及无线镜像不放宽。 */
#define MAX_RUN_LIMIT_MS 30000U
#elif defined(DRIVE_STARTUP_OBSERVE)
#define MAX_RUN_LIMIT_MS 10000U
#else
#define MAX_RUN_LIMIT_MS 5000U
#endif

static float clamp(float value, float lower, float upper)
{
    return value < lower ? lower : (value > upper ? upper : value);
}

static bool geometry_valid(const four_speed_wheel_config_t *c)
{
    return c != NULL && isfinite(c->counts_per_rev) && c->counts_per_rev > 0.0f &&
           isfinite(c->diameter_m) && c->diameter_m > 0.0f &&
           (c->encoder_sign == 1 || c->encoder_sign == -1) &&
           (c->motor_sign == 1 || c->motor_sign == -1);
}

void four_speed_defaults(four_speed_config_t *c)
{
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    const int8_t signs[FOUR_SPEED_WHEELS] = {1, -1, 1, -1};
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        c->wheel[i].diameter_m = 0.060f;
        c->wheel[i].motor_sign = signs[i];
        c->wheel[i].encoder_sign = signs[i];
        /* CPR、确认标记、增益均保留为0，不把手转初测自动变成运行参数。 */
    }
    c->target_limit_cps = 500.0f;
    c->feedback_limit_cps = 10000.0f;
    c->ramp_cps_per_s = 200.0f;
    c->feedback_filter_ms = 50.0f;
    c->duty_limit_permille = 100;
    c->sample_period_ms = 10U;
    c->max_sample_gap_ms = 30U;
    c->command_timeout_ms = 150U;
    c->run_limit_ms = 5000U;
}

bool four_speed_counts_to_measurement(const four_speed_wheel_config_t *c,
                                     int32_t delta, uint32_t elapsed_ms,
                                     four_speed_measurement_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!geometry_valid(c) || elapsed_ms == 0U) return false;
    const float cps = (float)delta * (float)c->encoder_sign * (1000.0f / (float)elapsed_ms);
    const float rpm = cps * 60.0f / c->counts_per_rev;
    const float m_s = cps * WHEEL_PI * c->diameter_m / c->counts_per_rev;
    if (!isfinite(cps) || !isfinite(rpm) || !isfinite(m_s)) return false;
    out->forward_cps = cps;
    out->rpm = rpm;
    out->speed_m_s = m_s;
    return true;
}

bool four_speed_m_s_to_cps(const four_speed_wheel_config_t *c,
                          float m_s, float *out)
{
    if (out == NULL) return false;
    *out = 0.0f;
    if (!geometry_valid(c) || !isfinite(m_s)) return false;
    const float cps = m_s * c->counts_per_rev / (WHEEL_PI * c->diameter_m);
    if (!isfinite(cps)) return false;
    *out = cps;
    return true;
}

static bool config_valid(const four_speed_config_t *c)
{
    if (c == NULL || !isfinite(c->target_limit_cps) || c->target_limit_cps <= 0.0f ||
        !isfinite(c->feedback_limit_cps) || c->feedback_limit_cps < c->target_limit_cps ||
        !isfinite(c->ramp_cps_per_s) || c->ramp_cps_per_s <= 0.0f ||
        !isfinite(c->feedback_filter_ms) || c->feedback_filter_ms < 0.0f ||
        c->feedback_filter_ms > 100.0f ||
        c->duty_limit_permille <= 0 || c->duty_limit_permille > 100 ||
        c->duty_limit_permille % 10 != 0 || c->sample_period_ms != 10U ||
        c->max_sample_gap_ms < c->sample_period_ms || c->max_sample_gap_ms > 30U ||
        c->command_timeout_ms == 0U || c->command_timeout_ms > 150U ||
        c->run_limit_ms == 0U || c->run_limit_ms > MAX_RUN_LIMIT_MS) return false;
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        const four_speed_wheel_config_t *w = &c->wheel[i];
        if (!geometry_valid(w) || !w->calibration_confirmed ||
            !isfinite(w->kp_permille_per_cps) || w->kp_permille_per_cps < 0.0f ||
            !isfinite(w->ki_permille_per_count) || w->ki_permille_per_count < 0.0f ||
            (w->kp_permille_per_cps == 0.0f && w->ki_permille_per_count == 0.0f)) return false;
    }
    return true;
}

void four_speed_stop(four_speed_controller_t *c, uint32_t fault)
{
    if (c == NULL) return;
    c->enabled = false;
    c->fault |= fault;
    c->run_direction = 0;
    c->progress_seen = false;
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        c->wheel[i].requested_cps = 0.0f;
        c->wheel[i].ramped_cps = 0.0f;
        c->wheel[i].integral_permille = 0.0f;
        c->wheel[i].raw_duty_permille = 0;
        c->wheel[i].filtered_cps = 0.0f;
    }
}

bool four_speed_init(four_speed_controller_t *c, const four_speed_config_t *config)
{
    if (c == NULL) return false;
    memset(c, 0, sizeof(*c));
    if (!config_valid(config)) {
        c->fault = FOUR_SPEED_FAULT_CONFIG;
        return false;
    }
    c->config = *config;
    return true;
}

bool four_speed_arm(four_speed_controller_t *c, uint8_t selected,
                    const int32_t raw[FOUR_SPEED_WHEELS], uint32_t now)
{
    if (c == NULL) return false;
    if (c->enabled || c->fault != 0U || raw == NULL || selected >= FOUR_SPEED_WHEELS ||
        !config_valid(&c->config)) {
        four_speed_stop(c, FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    four_speed_stop(c, 0U);
    c->selected = selected;
    c->last_sample_ms = c->last_command_ms = c->started_ms = c->last_progress_ms = now;
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        c->wheel[i].previous_count = raw[i];
        memset(&c->wheel[i].measured, 0, sizeof(c->wheel[i].measured));
    }
    c->enabled = true;
    return true;
}

bool four_speed_set_target(four_speed_controller_t *c, float target, uint32_t now)
{
    if (c == NULL || !c->enabled) return false;
    if (!isfinite(target) || fabsf(target) > c->config.target_limit_cps ||
        (uint32_t)(now - c->last_command_ms) >= c->config.command_timeout_ms ||
        (uint32_t)(now - c->started_ms) >= c->config.run_limit_ms) {
        four_speed_stop(c, FOUR_SPEED_FAULT_COMMAND);
        return false;
    }
    const int8_t direction = target > 0.0f ? 1 : (target < 0.0f ? -1 : 0);
    /* 单次试转不接受在线反向；必须先停机，重新确认后才能再使能。 */
    if (direction != 0 && c->run_direction != 0 && direction != c->run_direction) {
        four_speed_stop(c, FOUR_SPEED_FAULT_COMMAND);
        return false;
    }
    if (direction != 0 && c->run_direction == 0) c->run_direction = direction;
    c->last_command_ms = now;
    four_speed_wheel_state_t *s = &c->wheel[c->selected];
    s->requested_cps = target;
    if (direction == 0) {
        s->ramped_cps = s->integral_permille = 0.0f;
        s->raw_duty_permille = 0;
    }
    return true;
}

static int32_t delta_i32(int32_t current, int32_t previous)
{
    const uint32_t delta = (uint32_t)current - (uint32_t)previous;
    return delta <= INT32_MAX ? (int32_t)delta :
        (int32_t)((int64_t)delta - INT64_C(4294967296));
}

bool four_speed_tick(four_speed_controller_t *c,
                     const int32_t raw[FOUR_SPEED_WHEELS], uint32_t now)
{
    if (c == NULL || !c->enabled) return false;
    if (raw == NULL) {
        four_speed_stop(c, FOUR_SPEED_FAULT_FEEDBACK);
        return false;
    }
    const uint32_t dt_ms = now - c->last_sample_ms;
    if (now - c->last_command_ms >= c->config.command_timeout_ms) {
        four_speed_stop(c, FOUR_SPEED_FAULT_COMMAND);
        return false;
    }
    if (now - c->started_ms >= c->config.run_limit_ms) {
        four_speed_stop(c, FOUR_SPEED_FAULT_DURATION);
        return false;
    }
    if (dt_ms > c->config.max_sample_gap_ms) {
        four_speed_stop(c, FOUR_SPEED_FAULT_TIMING);
        return false;
    }
    if (dt_ms < c->config.sample_period_ms) return false;
    four_speed_wheel_state_t *s = &c->wheel[c->selected];
    /* 截止时间先于新计数：迟到的变化不能刷新已到期的无计数保护。 */
    if (s->requested_cps != 0.0f &&
        (uint32_t)(now - c->last_progress_ms) >=
        (c->progress_seen ? LOST_PROGRESS_MS : INITIAL_PROGRESS_MS)) {
        four_speed_stop(c, FOUR_SPEED_FAULT_STALL);
        return false;
    }
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        four_speed_wheel_state_t *state = &c->wheel[i];
        const int32_t delta = delta_i32(raw[i], state->previous_count);
        if (!four_speed_counts_to_measurement(&c->config.wheel[i], delta, dt_ms, &state->measured) ||
            fabsf(state->measured.forward_cps) > c->config.feedback_limit_cps ||
            (i == c->selected && delta != 0 && s->requested_cps != 0.0f &&
             ((delta > 0 ? 1 : -1) * c->config.wheel[i].encoder_sign != c->run_direction)) ||
            (i != c->selected && delta != 0)) {
            four_speed_stop(c, FOUR_SPEED_FAULT_FEEDBACK);
            return false;
        }
        state->previous_count = raw[i];
        /* 10ms低速计数有量化阶梯：保留原始测速，PI另用按实测dt计算的低通值。 */
        const float alpha = (float)dt_ms / (c->config.feedback_filter_ms + (float)dt_ms);
        state->filtered_cps += alpha * (state->measured.forward_cps - state->filtered_cps);
        if (i == c->selected && delta != 0) {
            c->last_progress_ms = now;
            c->progress_seen = true;
        }
    }
    c->last_sample_ms = now;
    if (s->requested_cps == 0.0f) {
        s->ramped_cps = s->integral_permille = 0.0f;
        s->raw_duty_permille = 0;
        c->progress_seen = false;
        c->last_progress_ms = now;
        return true;
    }
    const float dt_s = (float)dt_ms / 1000.0f;
    const float step = c->config.ramp_cps_per_s * dt_s;
    s->ramped_cps += clamp(s->requested_cps - s->ramped_cps, -step, step);
    const float error = s->ramped_cps - s->filtered_cps;
    const four_speed_wheel_config_t *config = &c->config.wheel[c->selected];
    const float limit = (float)c->config.duty_limit_permille;
    /* 超速时降到0，不在一次固定方向的试转中主动反转制动。 */
    const float lower = s->ramped_cps > 0.0f ? 0.0f : -limit;
    const float upper = s->ramped_cps > 0.0f ? limit : 0.0f;
    const float p = config->kp_permille_per_cps * error;
    const float increment = config->ki_permille_per_count * error * dt_s;
    const float candidate_i = clamp(s->integral_permille + increment, -limit, limit);
    const float candidate_u = p + candidate_i;
    if (!isfinite(p) || !isfinite(increment) || !isfinite(candidate_u)) {
        four_speed_stop(c, FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    /* 饱和时仅允许把输出往可用区间拉回的积分变化，停机则清积分。 */
    if (!((candidate_u > upper && increment > 0.0f) ||
          (candidate_u < lower && increment < 0.0f))) s->integral_permille = candidate_i;
    const float u = clamp(p + s->integral_permille, lower, upper);
    const int16_t steps = (int16_t)(fabsf(u) / (float)FOUR_SPEED_DUTY_QUANTUM + 0.5f);
    s->raw_duty_permille = (int16_t)(steps * FOUR_SPEED_DUTY_QUANTUM *
                                   (u < 0.0f ? -1 : 1) * config->motor_sign);
    return true;
}

bool four_speed_compare(int16_t duty, uint32_t *first, uint32_t *second)
{
    if (first == NULL || second == NULL) return false;
    *first = *second = 0U;
    if (duty < -100 || duty > 100 || duty % FOUR_SPEED_DUTY_QUANTUM != 0) return false;
#ifdef RR_PWM_FINE
    /* 向下取实际通电tick，不因CCR舍入超过请求或10%硬上限。 */
    const uint32_t magnitude = duty < 0 ? (uint32_t)(-duty) : (uint32_t)duty;
    const uint32_t active = FOUR_SPEED_PWM_COUNTS * magnitude / 1000U;
    if (duty > 0) {
        *first = FOUR_SPEED_PWM_COUNTS;
        *second = FOUR_SPEED_PWM_COUNTS - active;
    } else if (duty < 0) {
        *first = FOUR_SPEED_PWM_COUNTS - active;
        *second = FOUR_SPEED_PWM_COUNTS;
    }
#else
    if (duty > 0) {
        *first = 100U;
        *second = 100U - (uint32_t)duty / 10U;
    } else if (duty < 0) {
        *first = 100U - (uint32_t)(-duty) / 10U;
        *second = 100U;
    }
#endif
    return true;
}
