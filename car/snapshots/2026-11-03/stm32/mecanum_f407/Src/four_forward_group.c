#include "four_forward_group.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

void four_forward_stop(four_forward_group_t *g, uint32_t fault, uint32_t now)
{
    if (g == NULL) return;
    g->fault |= fault;
    g->enabled = false;
    g->stationary_ms = 0U;
    g->stopped_ms = now;
    g->heartbeat_seen = false;
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        four_speed_stop(&g->loop[i], g->fault);
        g->raw_duty[i] = 0;
    }
}

bool four_forward_init(four_forward_group_t *g, const four_speed_config_t *config,
                       const int32_t counts[FOUR_SPEED_WHEELS], uint32_t now)
{
    if (g == NULL) return false;
    memset(g, 0, sizeof(*g));
    g->sampled_ms = g->stopped_ms = now;
    if (config == NULL || counts == NULL || config->target_limit_cps > 300.0f) {
        four_forward_stop(g, FOUR_SPEED_FAULT_CONFIG, now);
        return false;
    }
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        /* 复用的是相同初始增益；每轮CPR、方向、积分和反馈仍各自独立。 */
        if (config->wheel[i].kp_permille_per_cps != config->wheel[0].kp_permille_per_cps ||
            config->wheel[i].ki_permille_per_count != config->wheel[0].ki_permille_per_count ||
            !four_speed_init(&g->loop[i], config) ||
            !four_speed_m_s_to_cps(&config->wheel[i], FOUR_FORWARD_SPEED_M_S,
                                  &g->target_cps[i]) ||
            g->target_cps[i] > config->target_limit_cps) {
            four_forward_stop(g, FOUR_SPEED_FAULT_CONFIG, now);
            return false;
        }
        g->counts[i] = counts[i];
    }
    return true;
}

bool four_forward_heartbeat(four_forward_group_t *g, uint32_t now)
{
    if (g == NULL || g->fault != 0U) return false;
    if (g->enabled) {
        for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
            if (!four_speed_set_target(&g->loop[i], g->target_cps[i], now)) {
                four_forward_stop(g, g->loop[i].fault, now);
                return false;
            }
        }
    }
    g->heartbeat_seen = true;
    g->heartbeat_ms = now;
    return true;
}

bool four_forward_arm(four_forward_group_t *g, uint32_t now)
{
    if (g == NULL || g->enabled || g->fault != 0U || g->run_used ||
        g->stationary_ms < 500U || now - g->stopped_ms < 500U ||
        now - g->sampled_ms > 30U || !g->heartbeat_seen ||
        now - g->heartbeat_ms >= 100U) return false;
    /* 尝试启动即消耗额度；部分通道失败不能重复尝试。 */
    g->run_used = true;
    g->stationary_ms = 0U;
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        if (!four_speed_arm(&g->loop[i], (uint8_t)i, g->counts, now) ||
            !four_speed_set_target(&g->loop[i], g->target_cps[i], now)) {
            four_forward_stop(g, g->loop[i].fault, now);
            return false;
        }
    }
    g->sampled_ms = now;
    g->enabled = true;
    return true;
}

static int32_t delta_i32(int32_t current, int32_t previous)
{
    const uint32_t delta = (uint32_t)current - (uint32_t)previous;
    return delta <= INT32_MAX ? (int32_t)delta :
        (int32_t)((int64_t)delta - INT64_C(4294967296));
}

bool four_forward_tick(four_forward_group_t *g, const int32_t raw[FOUR_SPEED_WHEELS],
                       uint32_t now)
{
    if (g == NULL || g->fault != 0U) return false;
    if (raw == NULL) {
        four_forward_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
        return false;
    }
    const four_speed_config_t *config = &g->loop[0].config;
    const uint32_t elapsed = now - g->sampled_ms;
    if (elapsed > config->max_sample_gap_ms) {
        four_forward_stop(g, FOUR_SPEED_FAULT_TIMING, now);
        return false;
    }
    if (elapsed < config->sample_period_ms) return false;
    if (g->enabled) {
        for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
            four_speed_controller_t *loop = &g->loop[i];
            int32_t independent_counts[FOUR_SPEED_WHEELS];
            /* 单轮库会把其它轮运动视作异常；此适配器只向该环输入本轮真实计数。
             * 四个真实通道分别送入各自环，外部遥测必须用g->counts而非适配数组。 */
            for (unsigned j = 0; j < FOUR_SPEED_WHEELS; ++j)
                independent_counts[j] = loop->wheel[j].previous_count;
            independent_counts[i] = raw[i];
            if (!four_speed_tick(loop, independent_counts, now)) {
                four_forward_stop(g, loop->fault != 0U ? loop->fault :
                                  FOUR_SPEED_FAULT_TIMING, now);
                return false;
            }
        }
        /* 四环全部通过才发布新输出；任意环故障全停，禁止逐环直接写PWM。 */
        for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i)
            g->raw_duty[i] = g->loop[i].wheel[i].raw_duty_permille;
    } else {
        bool still = true;
        for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
            const int32_t delta = delta_i32(raw[i], g->counts[i]);
            four_speed_measurement_t measured;
            if (!four_speed_counts_to_measurement(&config->wheel[i], delta, elapsed, &measured) ||
                fabsf(measured.forward_cps) > config->feedback_limit_cps) {
                four_forward_stop(g, FOUR_SPEED_FAULT_FEEDBACK, now);
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
