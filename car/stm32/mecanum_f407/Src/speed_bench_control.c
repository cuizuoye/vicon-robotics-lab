#include "speed_bench_control.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

void speed_bench_init(speed_bench_t *b, bool read_only, const int32_t counts[4], uint32_t now)
{
    memset(b, 0, sizeof(*b));
    b->read_only = read_only;
    b->sampled_ms = now;
    b->stopped_ms = now;
    four_speed_defaults(&b->adopted);
    /* 用户采用的台架近似CPR，原始测量记录保留；不代表精确/重复性认证。 */
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    for (unsigned i = 0; i < 4; ++i) {
        b->adopted.wheel[i].counts_per_rev = cpr[i];
        b->adopted.wheel[i].calibration_confirmed = true;
        /* 尚未实机调好的起始增益，只用于限幅、限时、悬空单轮试验。 */
        b->adopted.wheel[i].kp_permille_per_cps = 0.1f;
        b->adopted.wheel[i].ki_permille_per_count = 0.5f;
        b->counts[i] = counts[i];
    }
    b->adopted.target_limit_cps = 300.0f;
    (void)four_speed_init(&b->pi, &b->adopted);
}

static int wheel_number(const char *name)
{
    const char *const names[4] = {"FL", "FR", "RL", "RR"};
    for (unsigned i = 0; i < 4; ++i) if (strcmp(name, names[i]) == 0) return (int)i;
    return -1;
}

const char *speed_bench_accept(speed_bench_t *b, const char *line, uint32_t now)
{
    if (strcmp(line, "stop") == 0) {
        four_speed_stop(&b->pi, 0U);
        b->stationary_ms = 0U;
        b->stopped_ms = now;
        return "OK,stopped_speed\r\n";
    }
    if (strcmp(line, "status") == 0) return NULL;
    if (strcmp(line, "estop") == 0) {
        four_speed_stop(&b->pi, SPEED_BENCH_FAULT_ESTOP);
        return "OK,estopped_speed\r\n";
    }
    if (strcmp(line, "hb") == 0) {
        if (b->pi.enabled && !four_speed_set_target(&b->pi,
                b->pi.wheel[b->pi.selected].requested_cps, now)) return "ERR,heartbeat_late\r\n";
        b->heartbeat_seen = true;
        b->heartbeat_ms = now;
        return "OK,hb_speed\r\n";
    }
    if (strncmp(line, "run ", 4) == 0) {
        /* 精确允许6个目标，不用宽松数值解析；旧arm/duty协议完全不接受。 */
        const int targets[] = {-300, -200, -100, 100, 200, 300};
        const char *const suffix[] = {"-300 S5", "-200 S5", "-100 S5", "100 S5", "200 S5", "300 S5"};
        const size_t length = strlen(line);
        if ((length != 13U && length != 14U) || line[6] != ' ') return "ERR,run_schema\r\n";
        const char name[3] = {line[4], line[5], '\0'};
        const int selected = wheel_number(name);
        int target = 0;
        for (unsigned j = 0; j < 6; ++j) if (strcmp(line + 7, suffix[j]) == 0) target = targets[j];
        if (selected < 0 || target == 0) return "ERR,run_schema\r\n";
        if (b->read_only) return "ERR,read_only\r\n";
        if (b->pi.enabled || b->pi.fault != 0U) return "ERR,busy_or_fault\r\n";
        if (b->run_used) return "ERR,one_shot_used\r\n";
        /* 必须连续观测零输出/无计数变化500ms，且采样仍新鲜。 */
        if (b->stationary_ms < 500U || now - b->stopped_ms < 500U ||
            now - b->sampled_ms > 30U)
            return "ERR,not_settled\r\n";
        if (!b->heartbeat_seen || now - b->heartbeat_ms >= 100U) return "ERR,no_fresh_heartbeat\r\n";
        b->run_used = true;
        b->stationary_ms = 0U;
        if (!four_speed_arm(&b->pi, (uint8_t)selected, b->counts, now) ||
            !four_speed_set_target(&b->pi, (float)target, now)) return "ERR,run_rejected\r\n";
        return "OK,running_speed\r\n";
    }
    return "ERR,speed_command\r\n";
}

static int32_t count_delta(int32_t current, int32_t previous)
{
    const uint32_t value = (uint32_t)current - (uint32_t)previous;
    return value <= INT32_MAX ? (int32_t)value : (int32_t)((int64_t)value - INT64_C(4294967296));
}

bool speed_bench_tick(speed_bench_t *b, const int32_t counts[4], uint32_t now)
{
    const uint32_t elapsed = now - b->sampled_ms;
    const bool was_enabled = b->pi.enabled;
    /* 先检查独立运行时限，迟到心跳不能恢复已过期的运行。 */
    if (b->pi.enabled) (void)four_speed_tick(&b->pi, counts, now);
    if (elapsed < 10U) return false;
    if (elapsed > 30U) four_speed_stop(&b->pi, FOUR_SPEED_FAULT_TIMING);
    bool still = !was_enabled && !b->pi.enabled && b->pi.fault == 0U && elapsed <= 30U;
    for (unsigned i = 0; i < 4; ++i) {
        const int32_t delta = count_delta(counts[i], b->counts[i]);
        if (delta != 0 || b->pi.wheel[i].raw_duty_permille != 0) still = false;
        four_speed_measurement_t measured;
        if (!four_speed_counts_to_measurement(&b->adopted.wheel[i], delta, elapsed, &measured)) {
            four_speed_stop(&b->pi, FOUR_SPEED_FAULT_FEEDBACK);
            return false;
        }
        b->counts[i] = counts[i];
        const int64_t rate = (int64_t)delta * 1000 / elapsed;
        b->raw_cps[i] = rate > INT32_MAX ? INT32_MAX : rate < INT32_MIN ? INT32_MIN : (int32_t)rate;
        const float rpm = measured.rpm * 1000.0f;
        const float mm_s = measured.speed_m_s * 1000.0f;
        /* 异常大计数先拒绝，避免浮点转整数溢出；不让遥测格式化影响ISR。 */
        if (!isfinite(rpm) || !isfinite(mm_s) || fabsf(rpm) > 10000000.0f ||
            fabsf(mm_s) > 1000000.0f) {
            four_speed_stop(&b->pi, FOUR_SPEED_FAULT_FEEDBACK);
            b->rpm_milli[i] = b->speed_mm_s[i] = 0;
        } else {
            b->rpm_milli[i] = (int32_t)rpm;
            b->speed_mm_s[i] = (int32_t)mm_s;
        }
    }
    b->stationary_ms = still ? (b->stationary_ms + elapsed > 500U ? 500U :
                               b->stationary_ms + elapsed) : 0U;
    b->sampled_ms = now;
    ++b->updates;
    return true;
}

bool speed_bench_format(const speed_bench_t *b, const speed_bench_diagnostics_t *d,
                       const char *prefix, char *out, size_t capacity)
{
    if (capacity == 0U) return false;
    int n = snprintf(out, capacity,
        "%s,fw=%s,ms=%lu,ticks=%lu,updates=%lu,dt_min=%lu,dt_max=%lu,irq_us_max=%lu,"
        "ro=%u,gpio_low=%u,wdg_reset=%u,en=%u,sel=%u,fault=0x%08lx",
        prefix, b->read_only ? "speed_ro_v1" : "speed_pi_v2", (unsigned long)d->ms,
        (unsigned long)d->ticks, (unsigned long)b->updates, (unsigned long)d->dt_min,
        (unsigned long)d->dt_max, (unsigned long)d->irq_us_max,
        b->read_only ? 1U : 0U, d->gpio_low ? 1U : 0U, d->watchdog_reset ? 1U : 0U,
        b->pi.enabled ? 1U : 0U, b->pi.enabled ? (unsigned)b->pi.selected + 1U : 0U,
        (unsigned long)b->pi.fault);
    if (n < 0 || (size_t)n >= capacity) return false;
    size_t used = (size_t)n;
    const char *const names[] = {"fl", "fr", "rl", "rr"};
    for (unsigned i = 0; i < 4; ++i) {
        n = snprintf(out + used, capacity - used,
            ",enc_%s=%ld,cps_%s=%ld,fcps_%s=%ld,target_%s=%ld,rpm_%s=%ld,mms_%s=%ld,"
            "d_%s=%d,p_%s1=%lu,p_%s2=%lu",
            names[i], (long)b->counts[i], names[i], (long)b->raw_cps[i],
            names[i], (long)b->pi.wheel[i].filtered_cps,
            names[i], (long)b->pi.wheel[i].ramped_cps,
            names[i], (long)b->rpm_milli[i], names[i], (long)b->speed_mm_s[i],
            names[i], (int)b->pi.wheel[i].raw_duty_permille,
            names[i], (unsigned long)d->compares[2U * i], names[i],
            (unsigned long)d->compares[2U * i + 1U]);
        if (n < 0 || (size_t)n >= capacity - used) return false;
        used += (size_t)n;
    }
    if (!b->read_only) {
        n = snprintf(out + used, capacity - used, ",stable_ms=%lu,used=%u",
                     (unsigned long)b->stationary_ms, b->run_used ? 1U : 0U);
        if (n < 0 || (size_t)n >= capacity - used) return false;
        used += (size_t)n;
    }
    if (used + 3U > capacity) return false;
    memcpy(out + used, "\r\n", 3U);
    return true;
}
