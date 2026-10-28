/* Isolated 4WGC candidate. Never compiled by the accepted rectangle targets. */
#include "four_wheel_speed.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define WHEEL_PI 3.14159265358979323846f
#define INITIAL_PROGRESS_MS 800U
#define LOST_PROGRESS_MS 500U
#if defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
#define MAX_RUN_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_MOTION)
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
    c->target_limit_cps = FOUR_DRIVE_TARGET_LIMIT_CPS;
    c->feedback_limit_cps = 10000.0f;
    c->ramp_cps_per_s = 200.0f;
    c->feedback_filter_ms = 50.0f;
    c->duty_limit_permille = FOUR_SPEED_DUTY_LIMIT_PERMILLE;
    c->sample_period_ms = 10U;
    c->max_sample_gap_ms = 30U;
    c->command_timeout_ms = FOUR_DRIVE_COMMAND_TIMEOUT_MS;
    c->run_limit_ms = 5000U;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    c->stall_target_floor_cps = FOUR_DRIVE_START_THRESHOLD_CPS;
    c->stall_duty_floor_permille = 200;
    c->launch_duty_permille = FOUR_DRIVE_STARTUP_DUTY_PERMILLE;
    c->launch_slew_permille_per_s = 1200.0f;
    c->launch_timeout_ms = FOUR_DRIVE_STARTUP_MS;
#endif
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
        c->ramp_cps_per_s != 800.0f ||
        !isfinite(c->ramp_cps_per_s) || c->ramp_cps_per_s <= 0.0f ||
        !isfinite(c->feedback_filter_ms) || c->feedback_filter_ms < 0.0f ||
        c->feedback_filter_ms > 100.0f ||
        c->duty_limit_permille <= 0 ||
        c->duty_limit_permille > FOUR_SPEED_DUTY_LIMIT_PERMILLE ||
        c->duty_limit_permille % 10 != 0 || c->sample_period_ms != 10U ||
        c->max_sample_gap_ms < c->sample_period_ms || c->max_sample_gap_ms > 30U ||
#if defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
        c->command_timeout_ms != FOUR_DRIVE_COMMAND_TIMEOUT_MS ||
#else
        c->command_timeout_ms == 0U || c->command_timeout_ms > 150U ||
#endif
        c->run_limit_ms == 0U || c->run_limit_ms > MAX_RUN_LIMIT_MS) return false;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    if (!isfinite(c->stall_target_floor_cps) || c->stall_target_floor_cps < 20.0f ||
        c->stall_target_floor_cps > c->target_limit_cps ||
        c->stall_duty_floor_permille <= 0 ||
        c->stall_duty_floor_permille > c->duty_limit_permille ||
        c->launch_duty_permille < c->stall_duty_floor_permille ||
        c->launch_duty_permille > c->duty_limit_permille ||
        !isfinite(c->launch_slew_permille_per_s) ||
        c->launch_slew_permille_per_s <= 0.0f ||
        c->launch_timeout_ms < 100U || c->launch_timeout_ms > INITIAL_PROGRESS_MS)
        return false;
#endif
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        const four_speed_wheel_config_t *w = &c->wheel[i];
        if (!geometry_valid(w) || !w->calibration_confirmed ||
            !isfinite(w->kp_permille_per_cps) || w->kp_permille_per_cps < 0.0f ||
            !isfinite(w->ki_permille_per_count) || w->ki_permille_per_count < 0.0f ||
            (w->kp_permille_per_cps == 0.0f && w->ki_permille_per_count == 0.0f)) return false;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
        if (!isfinite(w->ff_static_permille) || w->ff_static_permille < 0.0f ||
            w->ff_static_permille > c->duty_limit_permille ||
            !isfinite(w->ff_velocity_permille_per_cps) ||
            w->ff_velocity_permille_per_cps < 0.0f ||
            w->ff_velocity_permille_per_cps > 2.0f ||
            !isfinite(w->low_speed_trust_cps) ||
            w->low_speed_trust_cps < c->stall_target_floor_cps ||
            w->low_speed_trust_cps > c->target_limit_cps ||
            (w->ff_static_permille == 0.0f && w->ff_velocity_permille_per_cps == 0.0f))
            return false;
#endif
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
    c->launch_active = false;
    c->reversing = c->opposite_pending = false;
    c->neutral_ms = c->opposite_counts = 0U;
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
    c->launch_started_ms = now;
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
    if (c->selected >= FOUR_SPEED_WHEELS) {
        four_speed_stop(c, FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    if (!isfinite(target) || fabsf(target) > c->config.target_limit_cps ||
        (uint32_t)(now - c->last_command_ms) >= c->config.command_timeout_ms ||
        (uint32_t)(now - c->started_ms) >= c->config.run_limit_ms) {
        four_speed_stop(c, FOUR_SPEED_FAULT_COMMAND);
        return false;
    }
    const int8_t direction = target > 0.0f ? 1 : (target < 0.0f ? -1 : 0);
    /* run_direction is a per-wheel DRIVE permission, not the latest demand.
     * A new sign must pass the measured-neutral state machine below. */
    if (direction != 0 && c->run_direction != 0 && direction != c->run_direction) {
        if (!c->reversing) {
            c->reversing = true;
            c->reversal_started_ms = now;
            c->reversal_budget_ms = (uint32_t)ceilf(
                fabsf(c->wheel[c->selected].ramped_cps)*1000.0f/c->config.ramp_cps_per_s)
                + FOUR_CONTINUOUS_REVERSE_LIMIT_MS;
            c->neutral_ms = 0U;
        }
        c->launch_active = false;
        c->wheel[c->selected].integral_permille = 0.0f;
    } else {
        c->reversing = false;
        c->neutral_ms = 0U;
    }
    if (direction != 0 && c->run_direction == 0) c->run_direction = direction;
    c->last_command_ms = now;
    four_speed_wheel_state_t *s = &c->wheel[c->selected];
    s->requested_cps = target;
    if (direction == 0) {
        s->ramped_cps = s->integral_permille = 0.0f;
        s->raw_duty_permille = 0;
        c->launch_active = false;
        c->progress_seen = false;
        c->last_progress_ms = now;
    }
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    else if (fabsf(target) < c->config.stall_target_floor_cps) {
        c->launch_active = false;
    }
#endif
    return true;
}

static int32_t delta_i32(int32_t current, int32_t previous)
{
    const uint32_t delta = (uint32_t)current - (uint32_t)previous;
    return delta <= INT32_MAX ? (int32_t)delta :
        (int32_t)((int64_t)delta - INT64_C(4294967296));
}

static bool feedback_fault(four_speed_controller_t *c, uint16_t reason,
                           uint32_t now, uint32_t dt, int32_t delta, float measured)
{
    four_speed_wheel_state_t *s = &c->wheel[c->selected];
    if (!c->first_feedback_fault.valid) {
        c->first_feedback_fault = (four_feedback_snapshot_t){
            .valid=true, .reason=reason, .ms=now, .dt_ms=dt, .raw_delta=delta,
            .requested_cps=s->requested_cps, .effective_cps=s->ramped_cps,
            .measured_cps=measured, .duty_permille=s->raw_duty_permille,
            .opposite_counts=c->opposite_counts};
    }
    four_speed_stop(c, FOUR_SPEED_FAULT_FEEDBACK);
    return false;
}

bool four_speed_tick(four_speed_controller_t *c,
                     const int32_t raw[FOUR_SPEED_WHEELS], uint32_t now)
{
    if (c == NULL || !c->enabled) return false;
    if (c->selected >= FOUR_SPEED_WHEELS) {
        four_speed_stop(c, FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    if (raw == NULL) {
        return feedback_fault(c, FOUR_FEEDBACK_INVALID, now, 0U, 0, 0.0f);
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
    const int32_t selected_delta=delta_i32(raw[c->selected], s->previous_count);
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    const bool launch_expired = c->launch_active &&
        (uint32_t)(now - c->launch_started_ms) >= c->config.launch_timeout_ms;
    const bool tracking_stalled = !c->reversing && !c->launch_active && c->progress_seen &&
        fabsf(s->requested_cps) >= c->config.stall_target_floor_cps &&
        (s->raw_duty_permille < 0 ? -s->raw_duty_permille : s->raw_duty_permille) >=
            c->config.stall_duty_floor_permille &&
        (uint32_t)(now - c->last_progress_ms) >= LOST_PROGRESS_MS;
    if (launch_expired || tracking_stalled) {
        four_speed_stop(c, FOUR_SPEED_FAULT_STALL);
        return false;
    }
#else
    if (s->requested_cps != 0.0f &&
        (uint32_t)(now - c->last_progress_ms) >=
        (c->progress_seen ? LOST_PROGRESS_MS : INITIAL_PROGRESS_MS)) {
        four_speed_stop(c, FOUR_SPEED_FAULT_STALL);
        return false;
    }
#endif
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        four_speed_wheel_state_t *state = &c->wheel[i];
        const int32_t delta = delta_i32(raw[i], state->previous_count);
        if (!four_speed_counts_to_measurement(&c->config.wheel[i], delta, dt_ms, &state->measured))
            return feedback_fault(c, FOUR_FEEDBACK_INVALID, now, dt_ms, delta, 0.0f);
        if (fabsf(state->measured.forward_cps) > c->config.feedback_limit_cps)
            return feedback_fault(c, FOUR_FEEDBACK_OVERSPEED, now, dt_ms, delta, state->measured.forward_cps);
        if (i != c->selected && delta != 0)
            return feedback_fault(c, FOUR_FEEDBACK_OTHER_WHEEL, now, dt_ms, delta, state->measured.forward_cps);
        if (i == c->selected) {
            const four_speed_wheel_config_t *wc = &c->config.wheel[i];
            const int direction = (s->ramped_cps>0.0f)-(s->ramped_cps<0.0f);
            const int64_t directed = (int64_t)delta * wc->encoder_sign * direction;
            /* Near cancellation the wheel may be passively back-driven by
             * the chassis. Direction evidence requires both meaningful
             * effective demand AND actual drive authority from the PRIOR
             * output. Reversal is bounded by required deceleration time
             * plus at most 750ms of neutral settling. */
            const bool authority = !c->reversing && direction != 0 &&
                fabsf(s->ramped_cps) >= wc->low_speed_trust_cps &&
                abs(s->raw_duty_permille) >= (int)fmaxf(50.0f, wc->ff_static_permille);
            if (authority && directed < 0) {
                const uint32_t magnitude = (uint32_t)(-directed);
                if (!c->opposite_pending) {
                    c->opposite_pending=true; c->opposite_since_ms=now;
                }
                c->opposite_counts = UINT32_MAX-c->opposite_counts < magnitude ?
                    UINT32_MAX : c->opposite_counts+magnitude;
                /* Small count-level transients need persistent evidence,
                 * not an immediate three-tick fault during load pickup.
                 * A large backwards excursion still trips immediately. */
                if (c->opposite_counts >= FOUR_CONTINUOUS_LARGE_OPPOSED_COUNTS ||
                    (c->opposite_counts >= FOUR_CONTINUOUS_OPPOSED_COUNTS &&
                     now-c->opposite_since_ms >= FOUR_CONTINUOUS_OPPOSED_SETTLE_MS) ||
                    now-c->opposite_since_ms >= FOUR_CONTINUOUS_OPPOSED_MS)
                    return feedback_fault(c, FOUR_FEEDBACK_OPPOSED, now, dt_ms, delta, state->measured.forward_cps);
            } else if (directed > 0) {
                const uint32_t credit=(uint32_t)directed;
                c->opposite_counts = credit>=c->opposite_counts ? 0U : c->opposite_counts-credit;
                if (!c->opposite_counts) c->opposite_pending=false;
            }
        }
        state->previous_count = raw[i];
        /* 10ms低速计数有量化阶梯：保留原始测速，PI另用按实测dt计算的低通值。 */
        const float alpha = (float)dt_ms / (c->config.feedback_filter_ms + (float)dt_ms);
        state->filtered_cps += alpha * (state->measured.forward_cps - state->filtered_cps);
        if (i == c->selected && s->requested_cps*c->run_direction>0.0f &&
            !c->reversing && (int64_t)delta*c->config.wheel[i].encoder_sign*c->run_direction>0) {
            c->last_progress_ms = now;
            c->progress_seen = true;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
            c->launch_active = false;
#endif
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
    if (c->reversing) {
        if (now-c->reversal_started_ms >= c->reversal_budget_ms)
            return feedback_fault(c, FOUR_FEEDBACK_REVERSAL_TIMEOUT, now, dt_ms,
                selected_delta, s->measured.forward_cps);
        s->ramped_cps += clamp(-s->ramped_cps, -step, step);
        s->integral_permille=0.0f;
        c->launch_active=false;
        if (s->ramped_cps==0.0f) {
            const bool neutral_output=s->raw_duty_permille==0;
            s->raw_duty_permille=0;
            c->progress_seen=false; c->last_progress_ms=now;
            const bool slow = fabsf(s->filtered_cps)<=FOUR_CONTINUOUS_REVERSE_FILTER_CPS &&
                fabsf(s->measured.forward_cps)<=FOUR_CONTINUOUS_REVERSE_RAW_CPS;
            c->neutral_ms = neutral_output && slow ? c->neutral_ms+dt_ms : 0U;
            if (c->neutral_ms >= FOUR_CONTINUOUS_NEUTRAL_MS) {
                c->run_direction=(s->requested_cps>0.0f)-(s->requested_cps<0.0f);
                c->reversing=false; c->neutral_ms=0U;
                c->opposite_counts=0U; c->opposite_pending=false;
                c->progress_seen=false; c->last_progress_ms=now;
            }
            return true;
        }
    } else {
        s->ramped_cps += clamp(s->requested_cps - s->ramped_cps, -step, step);
        if (!c->progress_seen && !c->launch_active &&
            fabsf(s->requested_cps)>=c->config.stall_target_floor_cps) {
            c->launch_active=true; c->launch_started_ms=c->last_progress_ms=now;
        }
    }
    const float error = s->ramped_cps - s->filtered_cps;
    const four_speed_wheel_config_t *config = &c->config.wheel[c->selected];
    const float limit = (float)c->config.duty_limit_permille;
    /* 超速时降到0，不在一次固定方向的试转中主动反转制动。 */
    const float lower = s->ramped_cps > 0.0f ? 0.0f : -limit;
    const float upper = s->ramped_cps > 0.0f ? limit : 0.0f;
    float feedback_scale = 1.0f;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    if (fabsf(s->ramped_cps) < config->low_speed_trust_cps) feedback_scale = 0.25f;
    const bool untrusted_command =
        fabsf(s->requested_cps) < c->config.stall_target_floor_cps;
    if (untrusted_command) s->integral_permille = 0.0f;
#endif
    const float p = config->kp_permille_per_cps * error * feedback_scale;
    float increment = config->ki_permille_per_count * error * dt_s * feedback_scale;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    if (untrusted_command) increment = 0.0f;
#endif
    const float candidate_i = clamp(s->integral_permille + increment, -limit, limit);
    float feedforward = 0.0f;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    const float magnitude = fabsf(s->ramped_cps);
    const float static_scale = clamp(magnitude / config->low_speed_trust_cps, 0.0f, 1.0f);
    feedforward = copysignf(config->ff_static_permille * static_scale +
                            config->ff_velocity_permille_per_cps * magnitude,
                            s->ramped_cps);
#endif
    const float candidate_u = feedforward + p + candidate_i;
    if (!isfinite(p) || !isfinite(increment) || !isfinite(candidate_u)) {
        four_speed_stop(c, FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    /* 饱和时仅允许把输出往可用区间拉回的积分变化，停机则清积分。 */
    if (!((candidate_u > upper && increment > 0.0f) ||
          (candidate_u < lower && increment < 0.0f))) s->integral_permille = candidate_i;
    float u = clamp(feedforward + p + s->integral_permille, lower, upper);
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
    if (c->launch_active) {
        const float previous = fabsf((float)s->raw_duty_permille);
        const float launch = fminf((float)c->config.launch_duty_permille,
            fmaxf(fabsf(u), previous + c->config.launch_slew_permille_per_s * dt_s));
        u = copysignf(launch, s->ramped_cps);
        s->integral_permille = 0.0f;
    }
#endif
    /* Bound every driven-output increase/decrease, not only breakaway.
     * Explicit zero/stop may still remove power immediately. */
    const float previous=(float)s->raw_duty_permille*config->motor_sign;
    const float output_step=c->config.launch_slew_permille_per_s*dt_s;
    u=clamp(clamp(u,previous-output_step,previous+output_step),lower,upper);
    const int16_t steps = (int16_t)(fabsf(u) / (float)FOUR_SPEED_DUTY_QUANTUM + 0.5f);
    s->raw_duty_permille = (int16_t)(steps * FOUR_SPEED_DUTY_QUANTUM *
                                   (u < 0.0f ? -1 : 1) * config->motor_sign);
    return true;
}

bool four_speed_compare(int16_t duty, uint32_t *first, uint32_t *second)
{
    if (first == NULL || second == NULL) return false;
    *first = *second = 0U;
    if (duty < -FOUR_SPEED_DUTY_LIMIT_PERMILLE ||
        duty > FOUR_SPEED_DUTY_LIMIT_PERMILLE ||
        duty % FOUR_SPEED_DUTY_QUANTUM != 0) return false;
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
