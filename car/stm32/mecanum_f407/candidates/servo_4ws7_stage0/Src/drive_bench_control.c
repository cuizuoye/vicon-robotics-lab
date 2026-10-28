#include "drive_bench_control.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#ifdef FOUR_LINK_GROUNDB
#define DRIVE_FW_NAME "commissioning_4ws7_stage0_v3"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUNDA)
#define DRIVE_FW_NAME "drive_ground35_recovery600_vA"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND9)
#define DRIVE_FW_NAME "drive_ground35_rearm_v9"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND8)
#define DRIVE_FW_NAME "drive_ground35_200mmps_v8"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND7)
#define DRIVE_FW_NAME "drive_ground35_ffpi_v7"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND6)
#define DRIVE_FW_NAME "drive_ground35_deadband20_v6"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND5)
#define DRIVE_FW_NAME "drive_ground35_streamstart_v5"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND4)
#define DRIVE_FW_NAME "drive_ground35_idlelease_v4"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND3)
#define DRIVE_FW_NAME "drive_ground35_handoff30_v3"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND2)
#define DRIVE_FW_NAME "drive_ground35_start30_v2"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(FOUR_LINK_GROUND)
#define DRIVE_FW_NAME "drive_ground20_start15_v1"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(RR_SPEED2)
#define DRIVE_FW_NAME "rr_diag_328_v1"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(RR_PWM_FINE)
#define DRIVE_FW_NAME "rr_diag_pwm01_v1"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(RR_DIAGNOSTIC)
#define DRIVE_FW_NAME "rr_diag_start8_v1"
#define DRIVE_RUN_SUFFIX " S10"
#elif defined(DRIVE_STARTUP_OBSERVE)
#define DRIVE_FW_NAME "drive_pi_start8_v1"
#define DRIVE_RUN_SUFFIX " S10"
#else
#define DRIVE_FW_NAME "drive_pi_v1"
#define DRIVE_RUN_SUFFIX " S5"
#endif

void drive_bench_init(drive_bench_t *b, const int32_t counts[4], uint32_t now)
{
    if (b == NULL) return;
    memset(b, 0, sizeof(*b));
    b->sampled_ms = now;
    four_speed_defaults(&b->adopted);
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    for (unsigned i = 0; i < 4; ++i) {
        b->adopted.wheel[i].counts_per_rev = cpr[i];
        b->adopted.wheel[i].calibration_confirmed = true;
        b->adopted.wheel[i].kp_permille_per_cps = 0.1f;
        b->adopted.wheel[i].ki_permille_per_count = 0.5f;
#ifdef FOUR_DRIVE_FEEDFORWARD_CONTROL
        /* Conservative seed from the existing 4WG5 ground log.  These are
         * intentionally identical until a bounded on-ground identification
         * run can justify per-wheel values; the structure is per-wheel. */
        b->adopted.wheel[i].ff_static_permille = 80.0f;
        b->adopted.wheel[i].ff_velocity_permille_per_cps = 0.15f;
        b->adopted.wheel[i].low_speed_trust_cps = 100.0f;
#endif
        if (counts != NULL) b->counts[i] = counts[i];
    }
    b->adopted.target_limit_cps = FOUR_DRIVE_TARGET_LIMIT_CPS;
    b->adopted.duty_limit_permille=350;
#ifdef DRIVE_STARTUP_OBSERVE
    b->adopted.run_limit_ms = 10000U;
    b->adopted.ramp_cps_per_s = SERVO_WHEEL_SLEW_GUARD_CPS_S;
#endif
#ifdef FOUR_LINK_MOTION
    b->adopted.run_limit_ms = FOUR_DRIVE_RUN_LIMIT_MS;
#endif
    /* 用户已确认整改X、轮心距离不变；仅近似几何，不是Vicon标定。 */
    const mecanum_drive_geometry_t geometry = {{0.030f, 0.061f, 0.085f}, true, true};
    (void)four_drive_init(&b->group, &geometry, &b->adopted, counts, now);
}

const char *drive_bench_accept(drive_bench_t *b, const char *line, uint32_t now)
{
    if (!b || !line) return "ERR,drive_command\r\n";
    if (!strcmp(line,"status")) return NULL;
    if (!strcmp(line,"stop")) {
        four_drive_stop(&b->group,0,now); b->mode=DRIVE_MODE_IDLE;
        return "OK,stopped_drive\r\n";
    }
    return "ERR,4ws7_requires_versioned_session\r\n";
}

static int32_t count_delta(int32_t current, int32_t previous)
{
    const uint32_t d = (uint32_t)current - (uint32_t)previous;
    return d <= INT32_MAX ? (int32_t)d : (int32_t)((int64_t)d - INT64_C(4294967296));
}

bool drive_bench_tick(drive_bench_t *b, const int32_t counts[4], uint32_t now)
{
    if (b == NULL) return false;
    if (counts == NULL) {
        four_drive_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
        b->mode = DRIVE_MODE_IDLE;
        return false;
    }
    const uint32_t elapsed = now - b->sampled_ms;
    (void)four_drive_tick(&b->group, counts, now);
    if (!b->group.enabled) b->mode = DRIVE_MODE_IDLE;
    if (elapsed < 10U) return false;
    if (elapsed > 30U) four_drive_stop(&b->group, FOUR_SPEED_FAULT_TIMING, now);
    for (unsigned i = 0; i < 4; ++i) {
        const int32_t delta = count_delta(counts[i], b->counts[i]);
        four_speed_measurement_t m;
        if (!four_speed_counts_to_measurement(&b->adopted.wheel[i], delta, elapsed, &m)) {
            four_drive_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
            b->mode = DRIVE_MODE_IDLE;
            return false;
        }
        /* 故障/停机后继续报告真实通道，模型不能冒充Vicon位姿。 */
        b->counts[i] = counts[i];
        const int64_t cps = (int64_t)delta * 1000 / elapsed;
        b->raw_cps[i] = cps > INT32_MAX ? INT32_MAX : cps < INT32_MIN ? INT32_MIN : (int32_t)cps;
        const float rpm = m.rpm * 1000.0f, mm_s = m.speed_m_s * 1000.0f;
        if (!isfinite(rpm) || !isfinite(mm_s) || fabsf(rpm) > 10000000.0f ||
            fabsf(mm_s) > 1000000.0f) {
            four_drive_stop(&b->group, FOUR_SPEED_FAULT_FEEDBACK, now);
            b->rpm_milli[i] = b->speed_mm_s[i] = 0;
        } else {
            b->rpm_milli[i] = (int32_t)rpm;
            b->speed_mm_s[i] = (int32_t)mm_s;
        }
    }
    if (!b->group.enabled) b->mode = DRIVE_MODE_IDLE;
    b->sampled_ms = now;
    ++b->updates;
    return true;
}

bool drive_bench_format(const drive_bench_t *b, const speed_bench_diagnostics_t *d,
                        const char *prefix, char *out, size_t capacity)
{
    if (b == NULL || d == NULL || prefix == NULL || out == NULL || capacity == 0U) return false;
    const mecanum_body_velocity_t zero = {0};
    const mecanum_body_velocity_t *body = b->group.enabled ? &b->group.plan.applied_body : &zero;
    int n = snprintf(out, capacity,
        "%s,fw=" DRIVE_FW_NAME ",ms=%lu,ticks=%lu,updates=%lu,dt_min=%lu,dt_max=%lu,irq_us_max=%lu,"
        "ro=0,gpio_low=%u,wdg_reset=%u,en=%u,sel=%u,fault=0x%08lx,mode=%u,vx=%ld,vy=%ld,wz=%ld",
        prefix, (unsigned long)d->ms, (unsigned long)d->ticks, (unsigned long)b->updates,
        (unsigned long)d->dt_min, (unsigned long)d->dt_max, (unsigned long)d->irq_us_max,
        d->gpio_low ? 1U : 0U, d->watchdog_reset ? 1U : 0U,
        b->group.enabled ? 1U : 0U,
#ifdef RR_DIAGNOSTIC
        b->group.enabled ? 4U : 0U,
#else
        b->group.enabled ? 5U : 0U,
#endif
        (unsigned long)b->group.fault,
        b->group.enabled ? b->mode : 0U,
        (long)lroundf(body->vx_m_s * 1000.0f), (long)lroundf(body->vy_m_s * 1000.0f),
        (long)lroundf(body->wz_rad_s * 1000.0f));
    if (n < 0 || (size_t)n >= capacity) return false;
    size_t used = (size_t)n;
#ifdef RR_PWM_FINE
    n = snprintf(out + used, capacity - used, ",pwm_rr_arr=%lu,pwm_rr_psc=%lu",
                 (unsigned long)d->pwm_rr_arr, (unsigned long)d->pwm_rr_psc);
    if (n < 0 || (size_t)n >= capacity - used) return false;
    used += (size_t)n;
#endif
#ifdef DRIVE_STARTUP_OBSERVE
    n = snprintf(out + used, capacity - used, ",start_mask=%u,fault_wheel=%u",
                 (unsigned)b->group.startup_mask, (unsigned)b->group.fault_wheel);
    if (n < 0 || (size_t)n >= capacity - used) return false;
    used += (size_t)n;
#endif
    const char *const names[4] = {"fl", "fr", "rl", "rr"};
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
    n = snprintf(out + used, capacity - used, ",stable_ms=%lu,used=%u\r\n",
                 (unsigned long)b->group.stationary_ms, b->group.run_used ? 1U : 0U);
    return n >= 0 && (size_t)n < capacity - used;
}
