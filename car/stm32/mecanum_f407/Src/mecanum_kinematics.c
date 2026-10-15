#include "mecanum_kinematics.h"

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MECANUM_MAX_BODY_MM_S 1000000
#define MECANUM_MAX_YAW_MRAD_S 1000000
#define MECANUM_MAX_DIMENSION_SUM_MM 10000U
#define MECANUM_MAX_WHEEL_MM_S 1000000U

static int64_t absolute_i64(int64_t value)
{
    return value < 0 ? -value : value;
}

bool mecanum_inverse_kinematics(
    int32_t vx_mm_s,
    int32_t vy_mm_s,
    int32_t wz_mrad_s,
    uint32_t half_length_mm,
    uint32_t half_width_mm,
    uint32_t max_wheel_mm_s,
    mecanum_wheel_targets_t *targets)
{
    const uint64_t dimension_sum =
        (uint64_t)half_length_mm + (uint64_t)half_width_mm;
    if (targets == NULL
        || absolute_i64(vx_mm_s) > MECANUM_MAX_BODY_MM_S
        || absolute_i64(vy_mm_s) > MECANUM_MAX_BODY_MM_S
        || absolute_i64(wz_mrad_s) > MECANUM_MAX_YAW_MRAD_S
        || dimension_sum == 0U
        || dimension_sum > MECANUM_MAX_DIMENSION_SUM_MM
        || max_wheel_mm_s == 0U
        || max_wheel_mm_s > MECANUM_MAX_WHEEL_MM_S) {
        return false;
    }

    const int64_t lever_mm = (int64_t)dimension_sum;
    const int64_t rotation_mm_s = lever_mm * wz_mrad_s / 1000;
    int64_t raw[MECANUM_WHEEL_COUNT] = {
        (int64_t)vx_mm_s - vy_mm_s - rotation_mm_s,
        (int64_t)vx_mm_s + vy_mm_s + rotation_mm_s,
        (int64_t)vx_mm_s + vy_mm_s - rotation_mm_s,
        (int64_t)vx_mm_s - vy_mm_s + rotation_mm_s,
    };

    int64_t peak = 0;
    for (size_t index = 0; index < MECANUM_WHEEL_COUNT; ++index) {
        const int64_t magnitude = absolute_i64(raw[index]);
        if (magnitude > peak) {
            peak = magnitude;
        }
    }

    uint16_t scale_permille = 1000U;
    if (peak > (int64_t)max_wheel_mm_s) {
        scale_permille = (uint16_t)(((int64_t)max_wheel_mm_s * 1000) / peak);
        for (size_t index = 0; index < MECANUM_WHEEL_COUNT; ++index) {
            raw[index] = raw[index] * max_wheel_mm_s / peak;
        }
    }

    for (size_t index = 0; index < MECANUM_WHEEL_COUNT; ++index) {
        if (raw[index] > INT32_MAX || raw[index] < INT32_MIN) {
            return false;
        }
        targets->wheel_mm_s[index] = (int32_t)raw[index];
    }
    targets->scale_permille = scale_permille;
    return true;
}

static bool si_geometry_valid(const mecanum_geometry_si_t *g)
{
    return g != NULL && isfinite(g->wheel_radius_m) &&
        g->wheel_radius_m >= 0.001f && g->wheel_radius_m <= 1.0f &&
        isfinite(g->half_length_m) && g->half_length_m > 0.0f && g->half_length_m <= 1.0f &&
        isfinite(g->half_width_m) && g->half_width_m > 0.0f && g->half_width_m <= 1.0f;
}

bool mecanum_inverse_si(const mecanum_geometry_si_t *g,
                        const mecanum_body_velocity_t *body,
                        float max_rad_s, mecanum_angular_targets_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!si_geometry_valid(g) || body == NULL ||
        !isfinite(body->vx_m_s) || fabsf(body->vx_m_s) > 1000.0f ||
        !isfinite(body->vy_m_s) || fabsf(body->vy_m_s) > 1000.0f ||
        !isfinite(body->wz_rad_s) || fabsf(body->wz_rad_s) > 1000.0f ||
        !isfinite(max_rad_s) || max_rad_s <= 0.0f || max_rad_s > 1000000.0f) return false;
    const float rotation = (g->half_length_m + g->half_width_m) * body->wz_rad_s;
    float raw[4] = {
        (body->vx_m_s - body->vy_m_s - rotation) / g->wheel_radius_m,
        (body->vx_m_s + body->vy_m_s + rotation) / g->wheel_radius_m,
        (body->vx_m_s + body->vy_m_s - rotation) / g->wheel_radius_m,
        (body->vx_m_s - body->vy_m_s + rotation) / g->wheel_radius_m
    };
    float peak = 0.0f;
    for (unsigned i = 0; i < 4; ++i) {
        if (!isfinite(raw[i])) return false;
        if (fabsf(raw[i]) > peak) peak = fabsf(raw[i]);
    }
    const float scale = peak > max_rad_s ? max_rad_s / peak : 1.0f;
    for (unsigned i = 0; i < 4; ++i) out->rad_s[i] = raw[i] * scale;
    out->scale = scale;
    return true;
}

bool mecanum_forward_si(const mecanum_geometry_si_t *g, const float w[4],
                        mecanum_body_velocity_t *out)
{
    if (out == NULL) return false;
    memset(out, 0, sizeof(*out));
    if (!si_geometry_valid(g) || w == NULL) return false;
    for (unsigned i = 0; i < 4; ++i)
        if (!isfinite(w[i]) || fabsf(w[i]) > 1000000.0f) return false;
    const float r = g->wheel_radius_m, lever = g->half_length_m + g->half_width_m;
    const mecanum_body_velocity_t result = {
        r * (w[0] + w[1] + w[2] + w[3]) / 4.0f,
        r * (-w[0] + w[1] + w[2] - w[3]) / 4.0f,
        r * (-w[0] + w[1] - w[2] + w[3]) / (4.0f * lever)
    };
    if (!isfinite(result.vx_m_s) || !isfinite(result.vy_m_s) ||
        !isfinite(result.wz_rad_s)) return false;
    *out = result;
    return true;
}
