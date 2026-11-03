#include "mecanum_drive_plan.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static const mecanum_drive_geometry_t geometry = {{0.03f, 0.061f, 0.085f}, true, true};

static four_speed_config_t adopted(void)
{
    four_speed_config_t c;
    four_speed_defaults(&c);
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    c.target_limit_cps = 300.0f;
    for (unsigned i = 0; i < 4; ++i) {
        c.wheel[i].counts_per_rev = cpr[i];
        c.wheel[i].calibration_confirmed = true;
    }
    return c;
}

static void close_to(float a, float b) { assert(fabsf(a - b) < 0.0001f); }

static void zero_bytes(const void *p, size_t length)
{
    const unsigned char *bytes = p;
    for (size_t i = 0; i < length; ++i) assert(bytes[i] == 0U);
}

static void directions(void)
{
    four_speed_config_t c = adopted();
    const mecanum_body_velocity_t commands[4] = {
        {0.05f, 0.0f, 0.0f}, {0.0f, 0.03f, 0.0f},
        {0.0f, 0.0f, 0.1f}, {0.0f, 0.0f, 0.0f}
    };
    const int signs[4][4] = {{1,1,1,1}, {-1,1,1,-1}, {-1,1,-1,1}, {0,0,0,0}};
    for (unsigned row = 0; row < 4; ++row) {
        mecanum_drive_plan_t out;
        assert(mecanum_drive_plan(&geometry, &c, &commands[row], 0.05f, &out));
        close_to(out.common_scale, 1.0f);
        for (unsigned i = 0; i < 4; ++i) {
            const int direction = out.forward_cps[i] > 0 ? 1 : out.forward_cps[i] < 0 ? -1 : 0;
            assert(direction == signs[row][i]);
            assert(out.motor_direction_reference[i] == direction * c.wheel[i].motor_sign);
            close_to(out.expected_raw_encoder_cps[i], out.forward_cps[i] * c.wheel[i].encoder_sign);
        }
    }
}

static void common_limits(void)
{
    four_speed_config_t c = adopted();
    const mecanum_body_velocity_t request = {0.4f, 0.2f, 0.8f};
    mecanum_drive_plan_t p;
    assert(mecanum_drive_plan(&geometry, &c, &request, 0.05f, &p));
    close_to(p.common_scale, 0.05f / (0.4f + 0.2f + 0.146f * 0.8f));
    for (unsigned i = 0; i < 4; ++i) assert(fabsf(p.wheel_m_s[i]) <= 0.05f);
    c.wheel[0].counts_per_rev = 5000.0f;
    const mecanum_body_velocity_t forward = {0.05f, 0, 0};
    assert(mecanum_drive_plan(&geometry, &c, &forward, 0.05f, &p));
    assert(p.common_scale < 1.0f);
    assert(p.forward_cps[0] <= 300.0f && p.forward_cps[0] > 299.9f);
    for (unsigned i = 0; i < 4; ++i) {
        close_to(p.wheel_m_s[i], p.wheel_m_s[0]);
        assert(fabsf(p.forward_cps[i]) <= 300.0f);
    }
}

static void feedback_roundtrip(void)
{
    const four_speed_config_t c = adopted();
    for (int x = -10; x <= 10; ++x) for (int y = -10; y <= 10; ++y) {
        const mecanum_body_velocity_t request = {x * 0.023f, y * 0.019f, (x-y) * 0.05f};
        mecanum_drive_plan_t plan;
        mecanum_body_velocity_t recovered;
        assert(mecanum_drive_plan(&geometry, &c, &request, 0.05f, &plan));
        assert(mecanum_drive_body_from_raw_cps(&geometry, &c, plan.expected_raw_encoder_cps, &recovered));
        close_to(recovered.vx_m_s, plan.applied_body.vx_m_s);
        close_to(recovered.vy_m_s, plan.applied_body.vy_m_s);
        close_to(recovered.wz_rad_s, plan.applied_body.wz_rad_s);
    }
}

static void failed_plan_clears(void)
{
    four_speed_config_t c = adopted();
    mecanum_drive_geometry_t g = geometry;
    const mecanum_body_velocity_t request = {0.05f, 0, 0};
    mecanum_drive_plan_t p;
    for (unsigned kind = 0; kind < 9; ++kind) {
        c = adopted(); g = geometry;
        mecanum_body_velocity_t command = request;
        switch (kind) {
        case 0: g.dimensions_confirmed = false; break;
        case 1: g.x_layout_confirmed = false; break;
        case 2: g.kinematics.half_width_m = 0; break;
        case 3: c.wheel[1].calibration_confirmed = false; break;
        case 4: c.wheel[2].counts_per_rev = NAN; break;
        case 5: c.wheel[3].motor_sign = 0; break;
        case 6: c.target_limit_cps = 301; break;
        case 7: c.wheel[2].diameter_m = 0.08f; break;
        default: command.vy_m_s = INFINITY; break;
        }
        memset(&p, 0x55, sizeof(p));
        assert(!mecanum_drive_plan(&g, &c, &command, 0.05f, &p));
        zero_bytes(&p, sizeof(p));
    }
    c = adopted();
    assert(!mecanum_drive_plan(NULL, &c, &request, 0.05f, &p));
    assert(!mecanum_drive_plan(&geometry, NULL, &request, 0.05f, &p));
    assert(!mecanum_drive_plan(&geometry, &c, NULL, 0.05f, &p));
    assert(!mecanum_drive_plan(&geometry, &c, &request, 0.051f, &p));
    assert(!mecanum_drive_plan(&geometry, &c, &request, 0.05f, NULL));
    zero_bytes(&p, sizeof(p));
}

static void invalid_feedback(void)
{
    const four_speed_config_t c = adopted();
    float raw[4] = {0};
    mecanum_body_velocity_t body = {1,2,3};
    raw[1] = NAN;
    assert(!mecanum_drive_body_from_raw_cps(&geometry, &c, raw, &body));
    zero_bytes(&body, sizeof(body));
    raw[1] = 10001;
    assert(!mecanum_drive_body_from_raw_cps(&geometry, &c, raw, &body));
    assert(!mecanum_drive_body_from_raw_cps(&geometry, &c, NULL, &body));
    assert(!mecanum_drive_body_from_raw_cps(&geometry, &c, raw, NULL));
}

static void si_failures(void)
{
    mecanum_geometry_si_t g = geometry.kinematics;
    mecanum_body_velocity_t request = {0.05f, 0, 0}, body;
    mecanum_angular_targets_t angular;
    assert(mecanum_inverse_si(&g, &request, 2.0f, &angular));
    assert(mecanum_forward_si(&g, angular.rad_s, &body));
    close_to(body.vx_m_s, request.vx_m_s);
    assert(!mecanum_inverse_si(&g, &request, NAN, &angular));
    zero_bytes(&angular, sizeof(angular));
    g.wheel_radius_m = 0;
    assert(!mecanum_inverse_si(&g, &request, 2.0f, &angular));
    assert(!mecanum_forward_si(&g, NULL, &body));
    zero_bytes(&body, sizeof(body));
    assert(!mecanum_inverse_si(NULL, &request, 2.0f, &angular));
    assert(!mecanum_forward_si(NULL, NULL, &body));
}

int main(void)
{
    directions(); common_limits(); feedback_roundtrip(); failed_plan_clears();
    invalid_feedback(); si_failures();
    puts("mecanum_drive_plan: 6 groups passed (including 441 model round trips)");
    return 0;
}
