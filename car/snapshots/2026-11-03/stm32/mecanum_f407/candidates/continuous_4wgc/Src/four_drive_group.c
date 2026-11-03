#include "four_drive_group.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

static int32_t delta_i32(int32_t current, int32_t previous)
{
    const uint32_t d = (uint32_t)current - (uint32_t)previous;
    return d <= INT32_MAX ? (int32_t)d : (int32_t)((int64_t)d - INT64_C(4294967296));
}

#ifdef FOUR_LINK_MOTION
void four_drive_normalize_plan(mecanum_drive_plan_t *plan)
{
    if (plan == NULL) return;
#if defined(FOUR_LINK_GROUND6) || defined(FOUR_LINK_GROUND7) || \
    defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
    for (unsigned i = 0; i < 4; ++i) {
        if (fabsf(plan->forward_cps[i]) < FOUR_DRIVE_MIN_TRACKABLE_CPS) {
            plan->forward_cps[i] = 0.0f;
            plan->wheel_m_s[i] = 0.0f;
        }
    }
#endif
}
#endif

void four_drive_stop(four_drive_group_t *g, uint32_t fault, uint32_t now)
{
    if (g == NULL) return;
    g->fault |= fault;
    g->enabled = false;
    g->stationary_ms = 0U;
    g->stopped_ms = now;
    g->heartbeat_seen = false;
#ifdef DRIVE_STARTUP_OBSERVE
    g->startup_mask = 0;
#endif
    for (unsigned i = 0; i < 4; ++i) {
        four_speed_stop(&g->loop[i], g->fault);
        g->raw_duty[i] = 0;
    }
    /* plan保留作诊断，已停止的PI请求/输出仍全部为0。 */
}

bool four_drive_init(four_drive_group_t *g, const mecanum_drive_geometry_t *geometry,
                     const four_speed_config_t *c, const int32_t counts[4], uint32_t now)
{
    if (g == NULL) return false;
    memset(g, 0, sizeof(*g));
    g->sampled_ms = g->stopped_ms = now;
    const mecanum_body_velocity_t zero = {0};
    if (counts == NULL || !mecanum_drive_plan(
            geometry, c, &zero, FOUR_DRIVE_MAX_WHEEL_M_S, &g->plan)) {
        four_drive_stop(g, FOUR_SPEED_FAULT_CONFIG, now);
        return false;
    }
    g->geometry = *geometry;
#ifdef DRIVE_STARTUP_OBSERVE
    /* 独立矩形按段限30秒；旧观察/无线目标仍严格10秒。 */
#ifdef FOUR_LINK_MOTION
    const uint32_t expected_run_limit = FOUR_DRIVE_RUN_LIMIT_MS;
#elif defined(AUTONOMOUS_RECTANGLE)
    const uint32_t expected_run_limit = 30000U;
#else
    const uint32_t expected_run_limit = 10000U;
#endif
    if (c->run_limit_ms != expected_run_limit || c->ramp_cps_per_s != 800.0f ||
        c->duty_limit_permille != FOUR_SPEED_DUTY_LIMIT_PERMILLE) {
        four_drive_stop(g, FOUR_SPEED_FAULT_CONFIG, now);
        return false;
    }
#endif
    for (unsigned i = 0; i < 4; ++i) {
        /* 只复用初始增益，反馈、CPR、积分和符号仍独立。 */
        if (c->wheel[i].kp_permille_per_cps != c->wheel[0].kp_permille_per_cps ||
            c->wheel[i].ki_permille_per_count != c->wheel[0].ki_permille_per_count ||
            !four_speed_init(&g->loop[i], c)) {
            four_drive_stop(g, FOUR_SPEED_FAULT_CONFIG, now);
            return false;
        }
        g->counts[i] = counts[i];
    }
    return true;
}

static bool deadline_ok(four_drive_group_t *g, uint32_t now)
{
    const four_speed_config_t *c = &g->loop[0].config;
    /* 迟到心跳不能救活过期动作；截止优先于采样和新计数。 */
    if (now - g->heartbeat_ms >= c->command_timeout_ms) {
        four_drive_stop(g, FOUR_SPEED_FAULT_COMMAND, now);
        return false;
    }
    if (now - g->started_ms >= c->run_limit_ms) {
        four_drive_stop(g, FOUR_SPEED_FAULT_DURATION, now);
        return false;
    }
    return true;
}

bool four_drive_heartbeat(four_drive_group_t *g, uint32_t now)
{
    if (g == NULL || g->fault != 0U || (g->run_used && !g->enabled)) return false;
    if (g->enabled) {
        if (!deadline_ok(g, now)) return false;
        for (unsigned i = 0; i < 4; ++i) {
            if (!four_speed_set_target(&g->loop[i], g->plan.forward_cps[i], now)) {
#ifdef DRIVE_STARTUP_OBSERVE
                g->fault_wheel = (uint8_t)(i + 1U);
#endif
                four_drive_stop(g, g->loop[i].fault, now);
                return false;
            }
        }
    }
    g->heartbeat_seen = true;
    g->heartbeat_ms = now;
    return true;
}

#ifdef RR_DIAGNOSTIC
static bool arm_plan(four_drive_group_t *g, const mecanum_drive_plan_t *planned, uint32_t now)
#else
bool four_drive_arm(four_drive_group_t *g, const mecanum_body_velocity_t *request, uint32_t now)
#endif
{
    if (g == NULL || g->enabled || g->fault != 0U || g->run_used ||
        g->stationary_ms < 500U || now - g->stopped_ms < 500U ||
        now - g->sampled_ms > 30U || !g->heartbeat_seen || now - g->heartbeat_ms >= 100U)
        return false;
#ifdef RR_DIAGNOSTIC
    if (planned == NULL) return false;
    const mecanum_drive_plan_t plan = *planned;
#else
    if (request == NULL || fabsf(request->vx_m_s) > FOUR_DRIVE_MAX_LINEAR_M_S ||
        fabsf(request->vy_m_s) > FOUR_DRIVE_MAX_LINEAR_M_S ||
        fabsf(request->wz_rad_s) > FOUR_DRIVE_MAX_YAW_RAD_S) return false;
    mecanum_drive_plan_t plan;
    if (!mecanum_drive_plan(&g->geometry, &g->loop[0].config, request,
                            FOUR_DRIVE_MAX_WHEEL_M_S, &plan))
        return false;
#ifdef FOUR_LINK_MOTION
    four_drive_normalize_plan(&plan);
#endif
#endif
    bool nonzero = false;
    for (unsigned i = 0; i < 4; ++i) {
        const float target = fabsf(plan.forward_cps[i]);
        /* 此初始台架增益不宣称能跟踪任意微小速度；零轮允许，极小非零拒绝。 */
#ifndef FOUR_LINK_MOTION
        if (target != 0.0f && target < 50.0f) return false;
#endif
        nonzero |= target != 0.0f;
    }
    if (!nonzero) return false;
    g->run_used = true; /* 尝试实际使能即消费，不允许部分启动后重试。 */
    g->stationary_ms = 0U;
    g->plan = plan;
#ifdef DRIVE_STARTUP_OBSERVE
    memset(g->startup_counts, 0, sizeof(g->startup_counts));
    g->startup_mask = g->startup_done_mask = g->fault_wheel = 0;
#endif
    for (unsigned i = 0; i < 4; ++i) {
        if (!four_speed_arm(&g->loop[i], (uint8_t)i, g->counts, now) ||
            !four_speed_set_target(&g->loop[i], plan.forward_cps[i], now)) {
#ifdef DRIVE_STARTUP_OBSERVE
            g->fault_wheel = (uint8_t)(i + 1U);
#endif
            four_drive_stop(g, g->loop[i].fault != 0U ? g->loop[i].fault :
                            FOUR_SPEED_FAULT_CONFIG, now);
            return false;
        }
    }
    g->started_ms = g->sampled_ms = now;
    g->enabled = true;
    return true;
}

#ifdef FOUR_LINK_MOTION
bool four_drive_retarget(four_drive_group_t *g, const mecanum_drive_plan_t *p, uint32_t now)
{
    if (!g || !p || !g->enabled || g->fault || !deadline_ok(g, now)) return false;
    mecanum_drive_plan_t normalized = *p;
    four_drive_normalize_plan(&normalized);
    p = &normalized;
    /* Validate all wheels before changing any loop. No partial target commit. */
    for (unsigned i = 0; i < 4; ++i) {
        const float target = p->forward_cps[i];
        if (!isfinite(target) || fabsf(target) > g->loop[i].config.target_limit_cps) return false;
    }
    for (unsigned i = 0; i < 4; ++i) {
        four_speed_controller_t *loop = &g->loop[i];
        four_speed_wheel_state_t *s = &loop->wheel[i];
        if (!four_speed_set_target(loop, p->forward_cps[i], now)) {
            four_drive_stop(g, loop->fault ? loop->fault : FOUR_SPEED_FAULT_COMMAND, now);
            return false;
        }
        /* Nonzero targets are future signed demands. The per-wheel loop
         * decelerates its effective target; never snap across polarity. */
        if (!loop->launch_active) {
            g->startup_mask &= (uint8_t)~(1U << i);
            g->startup_done_mask |= (uint8_t)(1U << i);
        }
        g->raw_duty[i] = s->raw_duty_permille;
    }
    g->plan = *p;
    g->heartbeat_ms = now;
    g->heartbeat_seen = true;
    return true;
}

#if defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
bool four_drive_rearm(four_drive_group_t *g, uint32_t now)
{
    if (g == NULL || g->enabled || g->fault != 0U ||
        g->stationary_ms < 500U || now - g->stopped_ms < 500U ||
        now - g->sampled_ms > 30U)
        return false;
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_controller_t *loop = &g->loop[i];
        const four_speed_wheel_state_t *s = &loop->wheel[i];
        if (loop->enabled || loop->fault != 0U || s->requested_cps != 0.0f ||
            s->ramped_cps != 0.0f || s->integral_permille != 0.0f ||
            s->raw_duty_permille != 0 || g->raw_duty[i] != 0)
            return false;
    }
    g->run_used = false;
    g->heartbeat_seen = false;
    return true;
}
#endif
#endif

#ifdef RR_DIAGNOSTIC
bool four_drive_arm(four_drive_group_t *g, const mecanum_body_velocity_t *request, uint32_t now)
{
    (void)g; (void)request; (void)now;
    return false; /* 单轮诊断镜像完全禁用整车动作入口。 */
}
#ifndef DRIVE_STARTUP_OBSERVE
#error RR diagnostic must retain the bounded startup observation protections.
#endif
bool four_drive_arm_rr_diagnostic(four_drive_group_t *g, uint32_t now)
{
    /* 固定RR轮速对照；body字段为0，不是车体命令。 */
    mecanum_drive_plan_t plan = {0};
    plan.forward_cps[FOUR_SPEED_RR] = RR_DIAGNOSTIC_TARGET_CPS;
    return arm_plan(g, &plan, now);
}
#endif

bool four_drive_compares(const four_drive_group_t *g, uint32_t out[8])
{
    if (out == NULL) return false;
    memset(out, 0, 8U * sizeof(*out));
    if (g == NULL) return false;
#ifdef DRIVE_STARTUP_OBSERVE
    if ((g->startup_mask & 0xf0U) || (g->startup_done_mask & 0xf0U) ||
        (g->startup_mask & g->startup_done_mask) || g->fault_wheel > 4U ||
        (!g->enabled && g->startup_mask != 0)) return false;
#endif
    uint32_t values[8] = {0};
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_controller_t *loop = &g->loop[i];
        const four_speed_wheel_state_t *s = &loop->wheel[i];
        const float target = g->plan.forward_cps[i];
        const int duty = g->raw_duty[i];
#ifdef RR_DIAGNOSTIC
        /* 独立执行器边界；即使串口白名单之外的目标被损坏也不能启动其它轮。 */
        if ((i != FOUR_SPEED_RR && (target != 0.0f || duty != 0)) ||
            (g->enabled && i == FOUR_SPEED_RR && target != RR_DIAGNOSTIC_TARGET_CPS)) return false;
#endif
        if (!isfinite(target) || fabsf(target) > FOUR_DRIVE_TARGET_LIMIT_CPS ||
            !four_speed_compare((int16_t)duty, &values[2U * i], &values[2U * i + 1U]))
            return false;
        if (g->enabled) {
            if (g->fault || !g->run_used || !loop->enabled || loop->fault || loop->selected != i ||
                s->requested_cps != target || s->raw_duty_permille != duty ||
                !isfinite(s->ramped_cps) || !isfinite(s->filtered_cps) ||
                !isfinite(s->integral_permille) ||
                fabsf(s->ramped_cps) > loop->config.target_limit_cps ||
                (s->ramped_cps * target < 0.0f && !loop->reversing) ||
                (s->ramped_cps != 0.0f &&
                 ((s->ramped_cps>0.0f)-(s->ramped_cps<0.0f)) != loop->run_direction) ||
                duty > loop->config.duty_limit_permille || duty < -loop->config.duty_limit_permille ||
                (loop->config.wheel[i].motor_sign != 1 && loop->config.wheel[i].motor_sign != -1) ||
                (target == 0.0f && (duty != 0 || s->integral_permille != 0.0f || s->ramped_cps != 0.0f)) ||
                (s->ramped_cps == 0.0f && duty != 0) ||
                (s->ramped_cps > 0.0f && duty * loop->config.wheel[i].motor_sign < 0) ||
                (s->ramped_cps < 0.0f && duty * loop->config.wheel[i].motor_sign > 0)) return false;
        } else if (duty != 0 || loop->enabled || s->requested_cps != 0.0f ||
                   s->ramped_cps != 0.0f || s->integral_permille != 0.0f ||
                   s->raw_duty_permille != 0) return false;
#ifdef DRIVE_STARTUP_OBSERVE
#if defined(FOUR_LINK_GROUND7) || defined(FOUR_LINK_GROUND8) || \
    defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
        const bool launch_bit = (g->startup_mask & (1U << i)) != 0U;
        const bool armed_before_first_sample = loop->launch_active && !launch_bit &&
            duty == 0 && s->ramped_cps == 0.0f;
        if ((launch_bit != loop->launch_active && !armed_before_first_sample) ||
            (launch_bit && (target == 0.0f ||
             duty > FOUR_DRIVE_STARTUP_DUTY_PERMILLE ||
             duty < -FOUR_DRIVE_STARTUP_DUTY_PERMILLE ||
             s->integral_permille != 0.0f)))
            return false;
#else
        if ((g->startup_mask & (1U << i)) &&
            (target == 0.0f || g->startup_counts[i] >= FOUR_DRIVE_STARTUP_COUNTS ||
             (duty != FOUR_DRIVE_STARTUP_DUTY_PERMILLE &&
              duty != -FOUR_DRIVE_STARTUP_DUTY_PERMILLE) ||
             s->integral_permille != 0.0f))
            return false;
#endif
#endif
    }
    memcpy(out, values, sizeof(values));
    return true;
}

bool four_drive_tick(four_drive_group_t *g, const int32_t raw[4], uint32_t now)
{
    if (g == NULL || g->fault != 0U) return false;
    if (raw == NULL) {
        four_drive_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
        return false;
    }
    if (g->enabled && !deadline_ok(g, now)) return false;
    const four_speed_config_t *c = &g->loop[0].config;
    const uint32_t elapsed = now - g->sampled_ms;
    if (elapsed > c->max_sample_gap_ms) {
        four_drive_stop(g, FOUR_SPEED_FAULT_TIMING, now);
        return false;
    }
    if (elapsed < c->sample_period_ms) return false;
    if (g->enabled) {
#ifdef DRIVE_STARTUP_OBSERVE
        g->startup_mask = 0;
#endif
        for (unsigned i = 0; i < 4; ++i) {
            four_speed_controller_t *loop = &g->loop[i];
#ifndef FOUR_LINK_MOTION
            if (g->plan.forward_cps[i] == 0.0f && raw[i] != g->counts[i]) {
#ifdef DRIVE_STARTUP_OBSERVE
                g->fault_wheel = (uint8_t)(i + 1U);
#endif
                /* 悬空台架零目标轮应静止；不当作失速，也不假定可被其它轮带动。 */
                four_drive_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
                return false;
            }
#endif
            int32_t isolated[4];
            for (unsigned j = 0; j < 4; ++j) isolated[j] = loop->wheel[j].previous_count;
            isolated[i] = raw[i]; /* 每环只隔离其它轮，仍输入本轮真实计数。 */
            if (!four_speed_tick(loop, isolated, now)) {
#ifdef DRIVE_STARTUP_OBSERVE
                g->fault_wheel = (uint8_t)(i + 1U);
#endif
                four_drive_stop(g, loop->fault != 0U ? loop->fault : FOUR_SPEED_FAULT_TIMING, now);
                return false;
            }
#ifdef DRIVE_STARTUP_OBSERVE
#if defined(FOUR_LINK_GROUND7) || defined(FOUR_LINK_GROUND8) || \
    defined(FOUR_LINK_GROUND9) || defined(FOUR_LINK_GROUNDA) || \
    defined(FOUR_LINK_GROUNDB)
            /* Launch is owned by the wheel controller: bounded duty slew,
             * immediate release on encoder progress, and no launch for small
             * mecanum cancellation targets.  The mask remains diagnostic. */
            if (loop->launch_active) {
                g->startup_done_mask &= (uint8_t)~(1U << i);
                g->startup_mask |= (uint8_t)(1U << i);
            } else g->startup_done_mask |= (uint8_t)(1U << i);
#else
            /* 反馈/方向/失速检查先执行，再做一次有界起动补偿；不能救活过期动作。 */
            four_speed_wheel_state_t *s = &loop->wheel[i];
            const int32_t delta = delta_i32(raw[i], g->counts[i]);
            const uint32_t magnitude = delta < 0 ? (uint32_t)(-(int64_t)delta) : (uint32_t)delta;
            if (magnitude >= FOUR_DRIVE_STARTUP_COUNTS - g->startup_counts[i])
                g->startup_counts[i] = FOUR_DRIVE_STARTUP_COUNTS;
            else g->startup_counts[i] += magnitude;
            const float target = g->plan.forward_cps[i];
            const int direction = (target > 0.0f) - (target < 0.0f);
            const bool was_starting = !(g->startup_done_mask & (1U << i));
            /* In 4WG5 the ROS outer loop streams a ramp beginning near zero.
             * A single 30% breakaway pulse can exceed that transient target,
             * but that is not proof that the loaded wheel can sustain motion.
             * Only the per-wheel count budget or the unchanged 500 ms cap may
             * end startup assist; neither raises its 30% output envelope. */
#if defined(FOUR_LINK_GROUND5) || defined(FOUR_LINK_GROUND6)
            const bool reached_speed = false;
#else
            const bool reached_speed = direction != 0 &&
                s->filtered_cps * (float)direction >= fabsf(target);
#endif
            const bool forced_handoff = direction != 0 && !reached_speed &&
                (now - g->started_ms >= FOUR_DRIVE_STARTUP_MS ||
                 g->startup_counts[i] >= FOUR_DRIVE_STARTUP_COUNTS);
            if (direction == 0 || forced_handoff || reached_speed)
                g->startup_done_mask |= (uint8_t)(1U << i);
            if (!(g->startup_done_mask & (1U << i)) &&
                s->raw_duty_permille > -FOUR_DRIVE_STARTUP_DUTY_PERMILLE &&
                s->raw_duty_permille < FOUR_DRIVE_STARTUP_DUTY_PERMILLE) {
                s->raw_duty_permille = (int16_t)(FOUR_DRIVE_STARTUP_DUTY_PERMILLE *
                    direction * loop->config.wheel[i].motor_sign);
                s->integral_permille = 0.0f; /* 补偿期间不继续积分堆积。 */
                g->startup_mask |= (uint8_t)(1U << i);
            } else if (FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE > 0 &&
                       was_starting && forced_handoff) {
                /* Preserve the last startup output across the PI handoff.
                 * The previous profiles deliberately retain their old zero-
                 * integral behavior through a compile-time zero handoff. */
                const float error = s->ramped_cps - s->filtered_cps;
                const float p = loop->config.wheel[i].kp_permille_per_cps * error;
                const float wanted = (float)(FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE * direction);
                float seeded = wanted - p;
                const float limit = (float)loop->config.duty_limit_permille;
                if (seeded > limit) seeded = limit;
                if (seeded < -limit) seeded = -limit;
                s->integral_permille = seeded;
                s->raw_duty_permille = (int16_t)(
                    FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE * direction *
                    loop->config.wheel[i].motor_sign);
            }
#endif
#endif
        }
        for (unsigned i = 0; i < 4; ++i) g->raw_duty[i] = g->loop[i].wheel[i].raw_duty_permille;
        uint32_t compares[8];
        if (!four_drive_compares(g, compares)) {
            four_drive_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
            return false;
        }
    } else {
        bool still = true;
        for (unsigned i = 0; i < 4; ++i) {
            const int32_t delta = delta_i32(raw[i], g->counts[i]);
            four_speed_measurement_t measured;
            if (!four_speed_counts_to_measurement(&c->wheel[i], delta, elapsed, &measured) ||
                fabsf(measured.forward_cps) > c->feedback_limit_cps) {
                four_drive_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
                return false;
            }
            if (delta != 0 || g->raw_duty[i] != 0) still = false;
        }
        g->stationary_ms = still ? (g->stationary_ms + elapsed > 500U ? 500U :
                                   g->stationary_ms + elapsed) : 0U;
    }
    memcpy(g->counts, raw, sizeof(g->counts));
    g->sampled_ms = now;
    return true;
}
