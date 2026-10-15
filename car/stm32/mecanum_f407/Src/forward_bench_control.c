#include "forward_bench_control.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void forward_bench_init(forward_bench_t *b, const int32_t counts[4], uint32_t now)
{
    memset(b, 0, sizeof(*b));
    b->sampled_ms = now;
    four_speed_defaults(&b->adopted);
    /* 各轮近似CPR保留；四轮共用经FL悬空试验的初始增益，不称充分整定。 */
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    for (unsigned i = 0; i < 4; ++i) {
        b->adopted.wheel[i].counts_per_rev = cpr[i];
        b->adopted.wheel[i].calibration_confirmed = true;
        b->adopted.wheel[i].kp_permille_per_cps = 0.1f;
        b->adopted.wheel[i].ki_permille_per_count = 0.5f;
        b->counts[i] = counts[i];
    }
    b->adopted.target_limit_cps = 300.0f;
    (void)four_forward_init(&b->group, &b->adopted, counts, now);
}

const char *forward_bench_accept(forward_bench_t *b, const char *line, uint32_t now)
{
    if (strcmp(line, "stop") == 0) {
        four_forward_stop(&b->group, 0U, now);
        return "OK,stopped_forward\r\n";
    }
    if (strcmp(line, "status") == 0) return NULL;
    if (strcmp(line, "estop") == 0) {
        four_forward_stop(&b->group, SPEED_BENCH_FAULT_ESTOP, now);
        return "OK,estopped_forward\r\n";
    }
    if (strcmp(line, "hb") == 0)
        return four_forward_heartbeat(&b->group, now) ?
               "OK,hb_forward\r\n" : "ERR,heartbeat_late\r\n";
    if (strcmp(line, FORWARD_BENCH_RUN) == 0) {
        if (b->group.enabled || b->group.fault != 0U) return "ERR,busy_or_fault\r\n";
        if (b->group.run_used) return "ERR,one_shot_used\r\n";
        if (b->group.stationary_ms < 500U || now - b->group.stopped_ms < 500U ||
            now - b->group.sampled_ms > 30U) return "ERR,not_settled\r\n";
        if (!b->group.heartbeat_seen || now - b->group.heartbeat_ms >= 100U)
            return "ERR,no_fresh_heartbeat\r\n";
        return four_forward_arm(&b->group, now) ?
               "OK,running_forward\r\n" : "ERR,run_rejected\r\n";
    }
    /* 旧run/arm/duty/clear、其它速度及延时均不接受，不扩大动作范围。 */
    return "ERR,forward_command\r\n";
}

static int32_t count_delta(int32_t current, int32_t previous)
{
    const uint32_t value = (uint32_t)current - (uint32_t)previous;
    return value <= INT32_MAX ? (int32_t)value :
        (int32_t)((int64_t)value - INT64_C(4294967296));
}

bool forward_bench_tick(forward_bench_t *b, const int32_t counts[4], uint32_t now)
{
    if (counts == NULL) {
        four_forward_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
        return false;
    }
    const uint32_t elapsed = now - b->sampled_ms;
    (void)four_forward_tick(&b->group, counts, now);
    if (elapsed < 10U) return false;
    if (elapsed > 30U) four_forward_stop(&b->group, FOUR_SPEED_FAULT_TIMING, now);
    for (unsigned i = 0; i < 4; ++i) {
        const int32_t delta = count_delta(counts[i], b->counts[i]);
        four_speed_measurement_t m;
        if (!four_speed_counts_to_measurement(&b->adopted.wheel[i], delta, elapsed, &m)) {
            four_forward_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
            return false;
        }
        /* 遥测始终用真实四通道计数，即使全停/故障也继续观测滑行。 */
        b->counts[i] = counts[i];
        const int64_t cps = (int64_t)delta * 1000 / elapsed;
        b->raw_cps[i] = cps > INT32_MAX ? INT32_MAX : cps < INT32_MIN ? INT32_MIN : (int32_t)cps;
        const float rpm = m.rpm * 1000.0f, mm_s = m.speed_m_s * 1000.0f;
        if (!isfinite(rpm) || !isfinite(mm_s) || fabsf(rpm) > 10000000.0f ||
            fabsf(mm_s) > 1000000.0f) {
            four_forward_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
            b->rpm_milli[i] = b->speed_mm_s[i] = 0;
        } else {
            b->rpm_milli[i] = (int32_t)rpm;
            b->speed_mm_s[i] = (int32_t)mm_s;
        }
    }
    b->sampled_ms = now;
    ++b->updates;
    return true;
}

bool forward_bench_format(const forward_bench_t *b, const speed_bench_diagnostics_t *d,
                          const char *prefix, char *out, size_t capacity)
{
    if (capacity == 0U) return false;
    int n = snprintf(out, capacity,
        "%s,fw=forward_pi_v1,ms=%lu,ticks=%lu,updates=%lu,dt_min=%lu,dt_max=%lu,irq_us_max=%lu,"
        "ro=0,gpio_low=%u,wdg_reset=%u,en=%u,sel=%u,fault=0x%08lx",
        prefix, (unsigned long)d->ms, (unsigned long)d->ticks, (unsigned long)b->updates,
        (unsigned long)d->dt_min, (unsigned long)d->dt_max, (unsigned long)d->irq_us_max,
        d->gpio_low ? 1U : 0U, d->watchdog_reset ? 1U : 0U,
        b->group.enabled ? 1U : 0U, b->group.enabled ? 5U : 0U, (unsigned long)b->group.fault);
    if (n < 0 || (size_t)n >= capacity) return false;
    size_t used = (size_t)n;
    const char *const names[] = {"fl", "fr", "rl", "rr"};
    for (unsigned i = 0; i < 4; ++i) {
        const four_speed_wheel_state_t *s = &b->group.loop[i].wheel[i];
        n = snprintf(out + used, capacity - used,
            ",enc_%s=%ld,cps_%s=%ld,fcps_%s=%ld,target_%s=%ld,rpm_%s=%ld,mms_%s=%ld,"
            "d_%s=%d,p_%s1=%lu,p_%s2=%lu,want_%s=%ld",
            names[i], (long)b->counts[i], names[i], (long)b->raw_cps[i],
            names[i], (long)s->filtered_cps, names[i], (long)s->ramped_cps,
            names[i], (long)b->rpm_milli[i], names[i], (long)b->speed_mm_s[i],
            names[i], (int)b->group.raw_duty[i], names[i], (unsigned long)d->compares[2U * i],
            names[i], (unsigned long)d->compares[2U * i + 1U], names[i], (long)s->requested_cps);
        if (n < 0 || (size_t)n >= capacity - used) return false;
        used += (size_t)n;
    }
    n = snprintf(out + used, capacity - used, ",stable_ms=%lu,used=%u",
                 (unsigned long)b->group.stationary_ms, b->group.run_used ? 1U : 0U);
    if (n < 0 || (size_t)n >= capacity - used) return false;
    used += (size_t)n;
    if (used + 3U > capacity) return false;
    memcpy(out + used, "\r\n", 3U);
    return true;
}
