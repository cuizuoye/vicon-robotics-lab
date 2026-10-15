#include "rectangle_demo.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#if !defined(AUTONOMOUS_RECTANGLE) || !defined(DRIVE_STARTUP_OBSERVE)
#error Rectangle demo must use its own bounded startup compilation profile.
#endif

static const mecanum_body_velocity_t directions[RECT_LEGS] = {
    {.vx_m_s = 0.05f}, {.vy_m_s = 0.05f},
    {.vx_m_s = -0.05f}, {.vy_m_s = -0.05f}
};

static int32_t delta_i32(int32_t current, int32_t previous)
{
    const uint32_t d = (uint32_t)current - (uint32_t)previous;
    return d <= INT32_MAX ? (int32_t)d : (int32_t)((int64_t)d - INT64_C(4294967296));
}

void rectangle_demo_stop(rectangle_demo_t *r, uint32_t fault, uint32_t now)
{
    if (r == NULL) return;
    r->fault |= fault;
    r->used = true; /* 停机/故障均不自动重试或接受第二圈。 */
    r->state = r->fault ? RECT_FAULT : RECT_STOPPED;
    four_drive_stop(&r->group, r->fault, now);
}

bool rectangle_demo_init(rectangle_demo_t *r, const int32_t counts[4],
                         uint32_t now, bool watchdog_reset)
{
    if (r == NULL) return false;
    memset(r, 0, sizeof(*r));
    r->sampled_ms = r->phase_ms = now;
    four_speed_defaults(&r->calibration);
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    for (unsigned i = 0; i < 4; ++i) {
        r->calibration.wheel[i].counts_per_rev = cpr[i];
        r->calibration.wheel[i].calibration_confirmed = true;
        r->calibration.wheel[i].kp_permille_per_cps = 0.1f;
        r->calibration.wheel[i].ki_permille_per_count = 0.5f;
    }
    r->calibration.target_limit_cps = FOUR_DRIVE_TARGET_LIMIT_CPS;
    r->calibration.run_limit_ms = RECT_LEG_LIMIT_MS;
    r->calibration.ramp_cps_per_s = 800.0f;
    r->geometry = (mecanum_drive_geometry_t){{0.030f, 0.061f, 0.085f}, true, true};
    if (!four_drive_init(&r->group, &r->geometry, &r->calibration, counts, now)) {
        rectangle_demo_stop(r, FOUR_SPEED_FAULT_CONFIG, now);
        return false;
    }
    memcpy(r->counts, counts, sizeof(r->counts));
    r->state = RECT_BOOT_RELEASE;
    if (watchdog_reset) {
        rectangle_demo_stop(r, RECT_FAULT_WATCHDOG, now);
        return false;
    }
    return true;
}

bool rectangle_demo_compares(const rectangle_demo_t *r, uint32_t out[8])
{
    if (out == NULL) return false;
    memset(out, 0, 8U * sizeof(*out));
    if (r == NULL || r->state > RECT_FAULT || r->completed_legs > RECT_LEGS ||
        (r->group.enabled && (r->state != RECT_MOVE || !r->used || r->fault)) ||
        (r->state == RECT_MOVE && (!r->group.enabled || r->completed_legs >= RECT_LEGS)))
        return false;
    return four_drive_compares(&r->group, out);
}

static bool start_leg(rectangle_demo_t *r, uint32_t now)
{
    if (!r->used || r->completed_legs >= RECT_LEGS || r->fault ||
        r->group.stationary_ms < 500U ||
        !four_drive_heartbeat(&r->group, now) ||
        !four_drive_arm(&r->group, &directions[r->completed_legs], now)) {
        rectangle_demo_stop(r, RECT_FAULT_NOT_SETTLED, now);
        return false;
    }
    r->state = RECT_MOVE;
    r->phase_ms = now;
    r->progress_m = r->lateral_m = r->yaw_rad = 0.0f;
    return true;
}

static bool moving(rectangle_demo_t *r, const int32_t counts[4], uint32_t dt, uint32_t now)
{
    float raw_cps[4];
    for (unsigned i = 0; i < 4; ++i)
        raw_cps[i] = (float)delta_i32(counts[i], r->counts[i]) * 1000.0f / (float)dt;
    mecanum_body_velocity_t measured;
    if (!mecanum_drive_body_from_raw_cps(&r->geometry, &r->calibration, raw_cps, &measured)) {
        rectangle_demo_stop(r, RECT_FAULT_ODOMETRY, now);
        return false;
    }
    const mecanum_body_velocity_t *direction = &directions[r->completed_legs];
    const float seconds = (float)dt / 1000.0f;
    /* 各段按车体前/左方向投影；轮滑移或真实车头偏转不能由此可靠消除。 */
    r->progress_m += (measured.vx_m_s * direction->vx_m_s +
                      measured.vy_m_s * direction->vy_m_s) * seconds / 0.05f;
    r->lateral_m += (-measured.vx_m_s * direction->vy_m_s +
                     measured.vy_m_s * direction->vx_m_s) * seconds / 0.05f;
    r->yaw_rad += measured.wz_rad_s * seconds;
    r->total_yaw_rad += measured.wz_rad_s * seconds;
    if (!isfinite(r->progress_m) || !isfinite(r->lateral_m) || !isfinite(r->yaw_rad) ||
        !isfinite(r->total_yaw_rad) ||
        r->progress_m < -0.01f || r->progress_m > 1.25f ||
        fabsf(r->lateral_m) > 0.15f || fabsf(r->yaw_rad) > 0.35f ||
        fabsf(r->total_yaw_rad) > 0.35f) {
        rectangle_demo_stop(r, RECT_FAULT_ODOMETRY, now);
        return false;
    }
    if (r->progress_m < 1.0f) return true;
    four_drive_stop(&r->group, 0U, now);
    ++r->completed_legs;
    r->phase_ms = now;
    if (r->completed_legs == RECT_LEGS) {
        r->state = RECT_COMPLETE;
    } else {
        /* 只有正常到达本段距离才能换段；外层used及全程时限不会被重置。 */
        if (!four_drive_init(&r->group, &r->geometry, &r->calibration, counts, now)) {
            rectangle_demo_stop(r, FOUR_SPEED_FAULT_CONFIG, now);
            return false;
        }
        r->state = RECT_PAUSE;
    }
    return true;
}

bool rectangle_demo_tick(rectangle_demo_t *r, const int32_t counts[4],
                         bool pressed, uint32_t now)
{
    if (r == NULL) return false;
    if (counts == NULL) {
        rectangle_demo_stop(r, FOUR_SPEED_FAULT_FEEDBACK, now);
        return false;
    }
    if (r->state > RECT_FAULT || r->completed_legs > RECT_LEGS ||
        (r->state == RECT_MOVE && r->completed_legs >= RECT_LEGS)) {
        rectangle_demo_stop(r, RECT_FAULT_STATE, now);
        return false;
    }
    if (r->state == RECT_COMPLETE || r->state == RECT_STOPPED || r->state == RECT_FAULT)
        return false;
    const uint32_t dt = now - r->sampled_ms;
    /* 截止时间优先于迟到反馈/按键，不用新数据救活已过期的动作。 */
    if (dt > 30U) {
        rectangle_demo_stop(r, FOUR_SPEED_FAULT_TIMING, now);
        return false;
    }
    if (r->state >= RECT_COUNTDOWN && r->state <= RECT_PAUSE) {
        if (pressed) {
            rectangle_demo_stop(r, RECT_FAULT_STOP, now);
            return false;
        }
        if (now - r->sequence_ms >= RECT_TOTAL_LIMIT_MS) {
            rectangle_demo_stop(r, RECT_FAULT_TOTAL_TIMEOUT, now);
            return false;
        }
    }
    if (r->state == RECT_MOVE && now - r->phase_ms >= RECT_LEG_LIMIT_MS) {
        rectangle_demo_stop(r, RECT_FAULT_LEG_TIMEOUT, now);
        return false;
    }
    if (dt < 10U) return false;
    if (r->state == RECT_MOVE && !four_drive_heartbeat(&r->group, now)) {
        rectangle_demo_stop(r, r->group.fault ? r->group.fault : FOUR_SPEED_FAULT_COMMAND, now);
        return false;
    }
    if (!four_drive_tick(&r->group, counts, now)) {
        rectangle_demo_stop(r, r->group.fault ? r->group.fault : FOUR_SPEED_FAULT_FEEDBACK, now);
        return false;
    }
    switch (r->state) {
    case RECT_BOOT_RELEASE:
        if (pressed) r->release_seen = false;
        else if (!r->release_seen) { r->release_seen = true; r->release_ms = now; }
        else if (now - r->release_ms >= 500U) r->state = RECT_READY;
        break;
    case RECT_READY:
        if (pressed && r->group.stationary_ms >= 500U) {
            r->state = RECT_HOLD; r->phase_ms = now;
        }
        break;
    case RECT_HOLD:
        if (!pressed) r->state = RECT_READY;
        else if (now - r->phase_ms >= RECT_HOLD_MS) {
            r->used = true; r->release_seen = false;
            r->state = RECT_ARMED_RELEASE; r->phase_ms = now;
        }
        break;
    case RECT_ARMED_RELEASE:
        if (now - r->phase_ms >= RECT_LEG_LIMIT_MS) {
            rectangle_demo_stop(r, RECT_FAULT_NOT_SETTLED, now);
            break;
        }
        if (pressed) r->release_seen = false;
        else if (!r->release_seen) { r->release_seen = true; r->release_ms = now; }
        else if (now - r->release_ms >= 50U) {
            r->state = RECT_COUNTDOWN; r->phase_ms = r->sequence_ms = now;
        }
        break;
    case RECT_COUNTDOWN:
        if (now - r->phase_ms >= RECT_COUNTDOWN_MS) (void)start_leg(r, now);
        break;
    case RECT_MOVE:
        (void)moving(r, counts, dt, now);
        break;
    case RECT_PAUSE:
        if (now - r->phase_ms >= RECT_PAUSE_MS && r->group.stationary_ms >= 500U)
            (void)start_leg(r, now);
        else if (now - r->phase_ms >= 3000U)
            rectangle_demo_stop(r, RECT_FAULT_NOT_SETTLED, now);
        break;
    default:
        rectangle_demo_stop(r, RECT_FAULT_STATE, now);
        break;
    }
    memcpy(r->counts, counts, sizeof(r->counts));
    r->sampled_ms = now;
    return r->state != RECT_FAULT && r->state != RECT_STOPPED;
}
