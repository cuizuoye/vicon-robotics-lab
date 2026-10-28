#include "mecanum_drive_plan.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define DRIVE_PI 3.14159265358979323846f

static bool ready(const mecanum_drive_geometry_t *g, const four_speed_config_t *c)
{
    if (g == NULL || c == NULL || !g->dimensions_confirmed || !g->x_layout_confirmed ||
        !isfinite(c->target_limit_cps) || c->target_limit_cps <= 0.0f ||
        c->target_limit_cps > FOUR_DRIVE_TARGET_LIMIT_CPS || !isfinite(c->feedback_limit_cps) ||
        c->feedback_limit_cps < c->target_limit_cps || c->feedback_limit_cps > 10000.0f)
        return false;
    const float zero[4] = {0};
    mecanum_body_velocity_t check;
    if (!mecanum_forward_si(&g->kinematics, zero, &check)) return false;
    for (unsigned i = 0; i < 4; ++i) {
        float unused;
        const four_speed_wheel_config_t *w = &c->wheel[i];
        if (!w->calibration_confirmed || !four_speed_m_s_to_cps(w, 0.0f, &unused) ||
            w->counts_per_rev < 1.0f || w->counts_per_rev > 10000000.0f ||
            fabsf(w->diameter_m - 2.0f * g->kinematics.wheel_radius_m) > 0.000001f)
            return false;
    }
    return true;
}

bool mecanum_drive_plan(const mecanum_drive_geometry_t *g,
                        const four_speed_config_t *c,
                        const mecanum_body_velocity_t *requested,
                        float max_wheel_m_s, mecanum_drive_plan_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!ready(g, c) || requested == NULL ||
        !isfinite(requested->vx_m_s) || fabsf(requested->vx_m_s) > 1.0f ||
        !isfinite(requested->vy_m_s) || fabsf(requested->vy_m_s) > 1.0f ||
        !isfinite(requested->wz_rad_s) || fabsf(requested->wz_rad_s) > 10.0f ||
        !isfinite(max_wheel_m_s) || max_wheel_m_s <= 0.0f ||
        max_wheel_m_s > FOUR_DRIVE_MAX_WHEEL_M_S)
        return false;
    mecanum_angular_targets_t angular;
    if (!mecanum_inverse_si(&g->kinematics, requested,
        max_wheel_m_s / g->kinematics.wheel_radius_m, &angular)) return false;
    mecanum_drive_plan_t plan = {0};
    float extra_scale = 1.0f;
    for (unsigned i = 0; i < 4; ++i) {
        plan.wheel_m_s[i] = angular.rad_s[i] * g->kinematics.wheel_radius_m;
        if (!four_speed_m_s_to_cps(&c->wheel[i], plan.wheel_m_s[i], &plan.forward_cps[i]))
            return false;
        const float cps = fabsf(plan.forward_cps[i]), speed = fabsf(plan.wheel_m_s[i]);
        if (cps > c->target_limit_cps && c->target_limit_cps / cps < extra_scale)
            extra_scale = c->target_limit_cps / cps;
        if (speed > max_wheel_m_s && max_wheel_m_s / speed < extra_scale)
            extra_scale = max_wheel_m_s / speed;
    }
    /* 共同缩放而不是逐轮截断；浮点限幅向零保守取一格，避免越过PI硬上限。 */
    if (extra_scale < 1.0f) extra_scale = nextafterf(extra_scale, 0.0f);
    plan.common_scale = angular.scale * extra_scale;
    for (unsigned i = 0; i < 4; ++i) {
        plan.wheel_m_s[i] *= extra_scale;
        plan.forward_cps[i] *= extra_scale;
        if (fabsf(plan.forward_cps[i]) > c->target_limit_cps ||
            fabsf(plan.wheel_m_s[i]) > max_wheel_m_s) return false;
        plan.expected_raw_encoder_cps[i] = plan.forward_cps[i] * c->wheel[i].encoder_sign;
        const int8_t direction = plan.forward_cps[i] > 0.0f ? 1 :
                                 plan.forward_cps[i] < 0.0f ? -1 : 0;
        plan.motor_direction_reference[i] = direction * c->wheel[i].motor_sign;
    }
    plan.applied_body.vx_m_s = requested->vx_m_s * plan.common_scale;
    plan.applied_body.vy_m_s = requested->vy_m_s * plan.common_scale;
    plan.applied_body.wz_rad_s = requested->wz_rad_s * plan.common_scale;
    *out = plan;
    return true;
}

bool mecanum_drive_body_from_raw_cps(const mecanum_drive_geometry_t *g,
                                   const four_speed_config_t *c,
                                   const float raw_cps[4], mecanum_body_velocity_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!ready(g, c) || raw_cps == NULL) return false;
    float angular[4];
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(raw_cps[i]) || fabsf(raw_cps[i]) > c->feedback_limit_cps) return false;
        angular[i] = raw_cps[i] * c->wheel[i].encoder_sign *
                     (2.0f * DRIVE_PI / c->wheel[i].counts_per_rev);
    }
    return mecanum_forward_si(&g->kinematics, angular, out);
}
