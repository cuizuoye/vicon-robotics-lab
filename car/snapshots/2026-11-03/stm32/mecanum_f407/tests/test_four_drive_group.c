#include "four_drive_group.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const mecanum_drive_geometry_t geometry = {{0.03f, 0.061f, 0.085f}, true, true};
static const mecanum_body_velocity_t motions[] = {
    {0.05f, 0, 0}, {0, 0.05f, 0}, {0, 0, 0.2f}, {0.025f, 0.025f, 0},
    {-0.05f, 0, 0}, {0, -0.05f, 0}, {0, 0, -0.2f}
};

static four_speed_config_t profile(void)
{
    four_speed_config_t c;
    four_speed_defaults(&c);
    const float cpr[4] = {3179.0f / 3.0f, 1049, 3167.0f / 3.0f, 1061};
    c.target_limit_cps = 300;
    for (unsigned i = 0; i < 4; ++i) {
        c.wheel[i].counts_per_rev = cpr[i];
        c.wheel[i].calibration_confirmed = true;
        c.wheel[i].kp_permille_per_cps = 0.1f;
        c.wheel[i].ki_permille_per_count = 0.5f;
    }
    return c;
}

static void zero(const four_drive_group_t *g)
{
    assert(!g->enabled);
    uint32_t cc[8];
    assert(four_drive_compares(g, cc));
    for (unsigned i = 0; i < 4; ++i) {
        assert(!g->loop[i].enabled && g->raw_duty[i] == 0);
        const four_speed_wheel_state_t *s = &g->loop[i].wheel[i];
        assert(s->raw_duty_permille == 0 && s->requested_cps == 0 &&
               s->ramped_cps == 0 && s->integral_permille == 0);
        assert(cc[2 * i] == 0 && cc[2 * i + 1] == 0);
    }
}

static uint32_t settled(four_drive_group_t *g, int32_t raw[4], uint32_t start)
{
    four_speed_config_t c = profile();
    assert(four_drive_init(g, &geometry, &c, raw, start));
    for (uint32_t dt = 10; dt <= 500; dt += 10) assert(four_drive_tick(g, raw, start + dt));
    assert(four_drive_heartbeat(g, start + 500));
    return start + 500;
}

static uint32_t ready(four_drive_group_t *g, int32_t raw[4], unsigned motion, uint32_t start)
{
    uint32_t now = settled(g, raw, start);
    assert(four_drive_arm(g, &motions[motion], now));
    return now;
}

static void progress(four_drive_group_t *g, int32_t raw[4], int excluded)
{
    for (unsigned i = 0; i < 4; ++i) {
        if ((int)i == excluded) continue;
        const float target = g->plan.forward_cps[i];
        if (target != 0) raw[i] += (target > 0 ? 1 : -1) * g->loop[i].config.wheel[i].encoder_sign;
    }
}

static void test_init_fail_closed(void)
{
    four_drive_group_t g;
    int32_t raw[4] = {0};
    four_speed_config_t c = profile();
    assert(four_drive_init(&g, &geometry, &c, raw, 0));
    zero(&g);
    assert(!four_drive_arm(&g, &motions[0], 0));
    mecanum_drive_geometry_t bad = geometry; bad.x_layout_confirmed = false;
    assert(!four_drive_init(&g, &bad, &c, raw, 0)); zero(&g);
    bad = geometry; bad.dimensions_confirmed = false;
    assert(!four_drive_init(&g, &bad, &c, raw, 0)); zero(&g);
    c.wheel[2].calibration_confirmed = false;
    assert(!four_drive_init(&g, &geometry, &c, raw, 0)); zero(&g);
    c = profile(); c.wheel[3].ki_permille_per_count = 0.6f;
    assert(!four_drive_init(&g, &geometry, &c, raw, 0)); zero(&g);
    c = profile(); c.duty_limit_permille = 110;
    assert(!four_drive_init(&g, &geometry, &c, raw, 0)); zero(&g);
    c = profile(); c.command_timeout_ms = 151;
    assert(!four_drive_init(&g, &geometry, &c, raw, 0)); zero(&g);
    c = profile(); c.target_limit_cps = 301;
    assert(!four_drive_init(&g, &geometry, &c, raw, 0)); zero(&g);
    assert(!four_drive_init(NULL, &geometry, &c, raw, 0));
    assert(!four_drive_init(&g, NULL, &c, raw, 0)); zero(&g);
    assert(!four_drive_init(&g, &geometry, NULL, raw, 0)); zero(&g);
    assert(!four_drive_init(&g, &geometry, &c, NULL, 0)); zero(&g);
}

static void test_arm_boundaries(void)
{
    four_drive_group_t g;
    int32_t raw[4] = {0};
    settled(&g, raw, 0);
    g.heartbeat_seen = false;
    assert(!four_drive_arm(&g, &motions[0], 500));
    assert(four_drive_heartbeat(&g, 500));
    assert(!four_drive_arm(&g, &motions[0], 531)); /* stale sample */
    assert(!four_drive_arm(&g, NULL, 500));
    const mecanum_body_velocity_t bad[] = {{0}, {NAN, 0, 0}, {0.051f, 0, 0},
        {0, 0.051f, 0}, {0, 0, 0.201f}, {0.001f, 0, 0}, {INFINITY, 0, 0}};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        assert(!four_drive_arm(&g, &bad[i], 500));
        assert(!g.run_used); zero(&g);
    }
    assert(four_drive_arm(&g, &motions[0], 505));
    assert(!four_drive_arm(&g, &motions[1], 506)); /* no online direction/mode update */
    assert(g.plan.forward_cps[0] > 0);
    assert(four_drive_tick(&g, raw, 515));
    four_drive_stop(&g, 0, 515); zero(&g);
    for (uint32_t t = 525; t <= 1025; t += 10) assert(four_drive_tick(&g, raw, t));
    assert(!four_drive_heartbeat(&g, 1025));
    assert(!four_drive_arm(&g, &motions[1], 1025));
    assert(g.run_used && g.fault == 0);
    /* stationary gate resets on any independent encoder movement */
    settled(&g, raw, 0); raw[1]++;
    assert(four_drive_tick(&g, raw, 510));
    assert(g.stationary_ms == 0 && !four_drive_arm(&g, &motions[0], 510));
}

static void test_all_motion_signs_and_zero_wheels(void)
{
    for (unsigned m = 0; m < sizeof(motions) / sizeof(motions[0]); ++m) {
        four_drive_group_t g;
        int32_t raw[4] = {0};
        uint32_t start = ready(&g, raw, m, 0);
        for (uint32_t dt = 10; dt <= 4900; dt += 10) {
            if (dt > 500) progress(&g, raw, -1);
            if (dt % 50 == 0) assert(four_drive_heartbeat(&g, start + dt));
            assert(four_drive_tick(&g, raw, start + dt));
            uint32_t cc[8]; assert(four_drive_compares(&g, cc));
            for (unsigned i = 0; i < 4; ++i) {
                const float want = g.plan.forward_cps[i];
                const int sign = g.loop[i].config.wheel[i].motor_sign;
                assert(g.counts[i] == raw[i] && abs(g.raw_duty[i]) <= 100);
                if (want == 0) {
                    assert(g.raw_duty[i] == 0 && raw[i] == 0);
                    assert(cc[2 * i] == 0 && cc[2 * i + 1] == 0);
                } else if (dt == 500) {
                    assert((want > 0 ? 1 : -1) * sign * g.raw_duty[i] > 0);
                }
            }
        }
        assert(g.enabled && g.fault == 0);
        four_drive_stop(&g, 0, start + 4900); zero(&g);
    }
}

static void test_zero_wheel_unexpected_motion(void)
{
    for (unsigned i = 0; i < 4; i += 3) {
        four_drive_group_t g;
        int32_t raw[4] = {0};
        ready(&g, raw, 3, 0); /* diagonal: FL/RR exactly zero, no false stall */
        assert(g.plan.forward_cps[0] == 0 && g.plan.forward_cps[3] == 0);
        raw[i] = 1;
        assert(!four_drive_tick(&g, raw, 510));
        assert(g.fault == FOUR_SPEED_FAULT_FEEDBACK); zero(&g);
    }
}

static void test_every_direction_speed_and_stall_fault(void)
{
    for (unsigned m = 0; m < 3; ++m) for (unsigned failed = 0; failed < 4; ++failed) {
        for (unsigned overspeed = 0; overspeed < 2; ++overspeed) {
            four_drive_group_t g; int32_t raw[4] = {0};
            ready(&g, raw, m, 0); progress(&g, raw, -1);
            raw[failed] *= overspeed ? 101 : -1;
            assert(!four_drive_tick(&g, raw, 510));
            assert(g.fault == FOUR_SPEED_FAULT_FEEDBACK); zero(&g);
        }
        for (unsigned had_progress = 0; had_progress < 2; ++had_progress) {
            four_drive_group_t g; int32_t raw[4] = {0};
            uint32_t start = ready(&g, raw, m, 0);
            uint32_t end = had_progress ? 510 : 800;
            for (uint32_t dt = 10; dt <= end; dt += 10) {
                progress(&g, raw, dt == 10 && had_progress ? -1 : (int)failed);
                if (dt % 50 == 0) assert(four_drive_heartbeat(&g, start + dt));
                assert(four_drive_tick(&g, raw, start + dt) == (dt < end));
            }
            assert(g.fault == FOUR_SPEED_FAULT_STALL); zero(&g);
        }
    }
}

static void test_timeouts_and_no_resurrection(void)
{
    four_drive_group_t g; int32_t raw[4] = {0};
    uint32_t start = ready(&g, raw, 1, 0);
    for (uint32_t dt = 10; dt < 150; dt += 10) assert(four_drive_tick(&g, raw, start + dt));
    assert(!four_drive_heartbeat(&g, start + 150));
    assert(g.fault == FOUR_SPEED_FAULT_COMMAND); zero(&g);
    assert(!four_drive_heartbeat(&g, start + 151));
    four_drive_stop(&g, 0, start + 151);
    assert(g.fault && g.run_used); zero(&g);
    start = ready(&g, raw, 2, 0);
    /* deadline must be enforced even before next 10ms sample */
    for (uint32_t dt = 10; dt < 150; dt += 10) assert(four_drive_tick(&g, raw, start + dt));
    assert(!four_drive_tick(&g, raw, start + 150)); zero(&g);
    settled(&g, raw, 0);
    assert(four_drive_arm(&g, &motions[0], 505));
    for (uint32_t t = 515; t <= 645; t += 10) assert(four_drive_tick(&g, raw, t));
    assert(!four_drive_tick(&g, raw, 650)); /* 5ms since sample, 150ms since heartbeat */
    assert(g.fault == FOUR_SPEED_FAULT_COMMAND); zero(&g);
    start = ready(&g, raw, 1, 0);
    for (uint32_t dt = 10; dt < 5000; dt += 10) {
        progress(&g, raw, -1);
        if (dt % 50 == 0) assert(four_drive_heartbeat(&g, start + dt));
        assert(four_drive_tick(&g, raw, start + dt));
    }
    assert(!four_drive_heartbeat(&g, start + 5000));
    assert(g.fault == FOUR_SPEED_FAULT_DURATION); zero(&g);
}

static void test_partial_arm_and_output_corruption(void)
{
    four_drive_group_t g; int32_t raw[4] = {0};
    settled(&g, raw, 0);
    g.loop[3].config.wheel[3].calibration_confirmed = false;
    assert(!four_drive_arm(&g, &motions[1], 500));
    assert(g.run_used && g.fault); zero(&g);
    for (unsigned i = 0; i < 4; ++i) {
        ready(&g, raw, 1, 0);
        g.raw_duty[i] = 10 * (g.plan.forward_cps[i] > 0 ? -1 : 1) * g.loop[i].config.wheel[i].motor_sign;
        g.loop[i].wheel[i].raw_duty_permille = g.raw_duty[i];
        uint32_t cc[8]; memset(cc, 0xff, sizeof(cc));
        assert(!four_drive_compares(&g, cc));
        for (unsigned j = 0; j < 8; ++j) assert(cc[j] == 0);
        four_drive_stop(&g, FOUR_SPEED_FAULT_FEEDBACK, 501); zero(&g);
    }
    ready(&g, raw, 3, 0);
    g.raw_duty[0] = g.loop[0].wheel[0].raw_duty_permille = 10;
    uint32_t cc[8]; assert(!four_drive_compares(&g, cc));
    four_drive_stop(&g, 0, 501); zero(&g);
    ready(&g, raw, 0, 0);
    g.plan.forward_cps[0] = -g.plan.forward_cps[0]; /* online reversal rejected by child PI */
    assert(!four_drive_heartbeat(&g, 550));
    assert(g.fault == FOUR_SPEED_FAULT_COMMAND); zero(&g);
    ready(&g, raw, 0, 0);
    g.loop[0].wheel[0].integral_permille = NAN;
    assert(!four_drive_compares(&g, cc));
    four_drive_stop(&g, FOUR_SPEED_FAULT_CONFIG, 501); zero(&g);
}

static void test_null_timing_and_wrap(void)
{
    four_drive_group_t g; int32_t raw[4] = {INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX};
    uint32_t start = ready(&g, raw, 1, UINT32_MAX - 100);
    const int signs[4] = {-1, -1, 1, 1};
    for (unsigned i = 0; i < 4; ++i) raw[i] = signs[i] > 0 ? INT32_MIN : INT32_MAX;
    assert(four_drive_tick(&g, raw, start + 10));
    for (unsigned i = 0; i < 4; ++i)
        assert(fabsf(g.loop[i].wheel[i].measured.forward_cps) == 100.0f);
    assert(g.enabled); four_drive_stop(&g, 0, start + 10); zero(&g);
    ready(&g, raw, 0, 0);
    assert(!four_drive_tick(&g, raw, 505) && g.enabled && !g.fault);
    assert(!four_drive_tick(&g, raw, 531));
    assert(g.fault == FOUR_SPEED_FAULT_TIMING); zero(&g);
    ready(&g, raw, 0, 0);
    assert(!four_drive_tick(&g, NULL, 510)); zero(&g);
    assert(!four_drive_tick(NULL, raw, 0));
    assert(!four_drive_heartbeat(NULL, 0));
    assert(!four_drive_arm(NULL, &motions[0], 0));
    assert(!four_drive_compares(NULL, (uint32_t[8]){0}));
    assert(!four_drive_compares(&g, NULL));
    four_drive_stop(NULL, 0, 0);
}

static void test_initial_ramp_before_no_progress_deadline(void)
{
    /* 2026-10-10左移未启动的离线边界复现，不模拟/证明真实启动摩擦。 */
    four_drive_group_t g; int32_t raw[4] = {0};
    const uint32_t start = ready(&g, raw, 1, 0);
    int peak_permille = 0;
    for (uint32_t dt = 10; dt < 800; dt += 10) {
        if (dt % 50 == 0) assert(four_drive_heartbeat(&g, start + dt));
        assert(four_drive_tick(&g, raw, start + dt));
        assert(g.enabled && g.fault == 0);
        for (unsigned i = 0; i < 4; ++i) {
            const int duty = abs(g.raw_duty[i]);
            if (duty > peak_permille) peak_permille = duty;
            assert(duty <= 50); /* 到800ms之前最多5%，不是已经输出10%。 */
            assert(fabsf(g.loop[i].wheel[i].ramped_cps) < fabsf(g.plan.forward_cps[i]));
        }
    }
    assert(peak_permille == 50);
    assert(fabsf(fabsf(g.loop[0].wheel[0].ramped_cps) - 158.0f) < 0.001f);
    assert(four_drive_heartbeat(&g, start + 800)); /* 新鲜心跳不能阻止失速保护。 */
    assert(!four_drive_tick(&g, raw, start + 800));
    assert(g.fault == FOUR_SPEED_FAULT_STALL && g.run_used); zero(&g);
    assert(!four_drive_heartbeat(&g, start + 801));
    four_drive_stop(&g, 0, start + 802);
    assert(g.fault == FOUR_SPEED_FAULT_STALL && g.run_used); zero(&g);
    puts("startup no-feedback model: 800ms STALL, peak 5%, all-stop; not hardware root-cause proof");
}

int main(void)
{
    test_init_fail_closed(); test_arm_boundaries(); test_all_motion_signs_and_zero_wheels();
    test_zero_wheel_unexpected_motion(); test_every_direction_speed_and_stall_fault();
    test_timeouts_and_no_resurrection(); test_partial_arm_and_output_corruption();
    test_null_timing_and_wrap(); test_initial_ramp_before_no_progress_deadline();
    puts("four-drive group: 9 offline test groups passed (not hardware)");
    return 0;
}
