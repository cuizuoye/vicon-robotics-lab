#include "four_forward_group.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* 软件台架输入；不是四轮实机速度闭环的验证记录。 */
static four_speed_config_t profile(void)
{
    four_speed_config_t c;
    four_speed_defaults(&c);
    const float cpr[4] = {3179.0f / 3.0f, 1049.0f, 3167.0f / 3.0f, 1061.0f};
    c.target_limit_cps = 300.0f;
    for (unsigned i = 0; i < 4; ++i) {
        c.wheel[i].counts_per_rev = cpr[i];
        c.wheel[i].calibration_confirmed = true;
        c.wheel[i].kp_permille_per_cps = 0.1f;
        c.wheel[i].ki_permille_per_count = 0.5f;
    }
    return c;
}

static void zero(const four_forward_group_t *g)
{
    assert(!g->enabled);
    for (unsigned i = 0; i < 4; ++i) {
        assert(g->raw_duty[i] == 0 && !g->loop[i].enabled);
        for (unsigned j = 0; j < 4; ++j) {
            assert(g->loop[i].wheel[j].raw_duty_permille == 0);
            assert(g->loop[i].wheel[j].integral_permille == 0.0f);
            assert(g->loop[i].wheel[j].requested_cps == 0.0f);
        }
    }
}

static uint32_t ready(four_forward_group_t *g, int32_t raw[4], uint32_t start)
{
    four_speed_config_t c = profile();
    assert(four_forward_init(g, &c, raw, start));
    for (uint32_t dt = 10; dt <= 500; dt += 10)
        assert(four_forward_tick(g, raw, start + dt));
    const uint32_t now = start + 500U;
    assert(four_forward_heartbeat(g, now));
    assert(four_forward_arm(g, now));
    return now;
}

static void progress(int32_t raw[4], int excluded)
{
    const int signs[4] = {1, -1, 1, -1};
    for (unsigned i = 0; i < 4; ++i)
        if ((int)i != excluded) raw[i] += signs[i];
}

static void test_profile_and_lock(void)
{
    four_speed_config_t c = profile();
    four_forward_group_t g;
    int32_t raw[4] = {0};
    assert(four_forward_init(&g, &c, raw, 0));
    zero(&g);
    assert(!four_forward_arm(&g, 0));
    for (unsigned i = 0; i < 4; ++i) {
        four_speed_measurement_t m;
        assert(g.target_cps[i] > 278.0f && g.target_cps[i] < 282.0f);
        assert(four_speed_counts_to_measurement(&c.wheel[i],
                c.wheel[i].encoder_sign * 100, 1000, &m));
        assert(m.speed_m_s > 0.0f);
        assert(g.loop[i].config.wheel[i].counts_per_rev == c.wheel[i].counts_per_rev);
    }
    assert(g.target_cps[0] != g.target_cps[1]);
    c.wheel[2].ki_permille_per_count = 0.6f;
    assert(!four_forward_init(&g, &c, raw, 0));
    zero(&g);
    c = profile(); c.target_limit_cps = 301.0f;
    assert(!four_forward_init(&g, &c, raw, 0));
    c = profile(); c.wheel[3].counts_per_rev = 5000;
    assert(!four_forward_init(&g, &c, raw, 0));
    c = profile(); c.wheel[0].kp_permille_per_cps = NAN;
    assert(!four_forward_init(&g, &c, raw, 0));
    c = profile(); c.wheel[1].calibration_confirmed = false;
    assert(!four_forward_init(&g, &c, raw, 0));
    assert(!four_forward_init(&g, NULL, raw, 0));
    assert(!four_forward_init(&g, &c, NULL, 0));
    assert(!four_forward_init(NULL, &c, raw, 0));
}

static void test_settle_and_one_shot(void)
{
    four_forward_group_t g;
    four_speed_config_t c = profile();
    int32_t raw[4] = {0};
    assert(four_forward_init(&g, &c, raw, 0));
    for (uint32_t t = 10; t <= 500; t += 10) {
        if (t == 250) raw[2] = 1;
        assert(four_forward_tick(&g, raw, t));
    }
    assert(g.stationary_ms == 250);
    assert(four_forward_heartbeat(&g, 500));
    assert(!four_forward_arm(&g, 500));
    for (uint32_t t = 510; t <= 750; t += 10) assert(four_forward_tick(&g, raw, t));
    assert(!four_forward_arm(&g, 750)); /* old heartbeat */
    assert(four_forward_heartbeat(&g, 750));
    assert(!four_forward_arm(&g, 781)); /* stale sample */
    const uint32_t start = ready(&g, raw, 0);
    four_forward_stop(&g, 0, start);
    /* separate fresh init for an arm between fixed sampling ticks */
    assert(four_forward_init(&g, &c, raw, 0));
    for (uint32_t t = 10; t <= 500; t += 10) assert(four_forward_tick(&g, raw, t));
    assert(four_forward_heartbeat(&g, 500));
    assert(four_forward_arm(&g, 505));
    assert(four_forward_tick(&g, raw, 515)); /* arm aligns child/group timestamps */
    four_forward_stop(&g, 0, 515);
    zero(&g);
    assert(g.run_used && g.fault == 0U);
    for (uint32_t t = 525; t <= 1025; t += 10) assert(four_forward_tick(&g, raw, t));
    assert(four_forward_heartbeat(&g, 1025));
    assert(!four_forward_arm(&g, 1025));
}

static void test_forward_signs_and_stop(void)
{
    four_forward_group_t g;
    int32_t raw[4] = {0};
    uint32_t start = ready(&g, raw, 0);
    for (uint32_t dt = 10; dt <= 500; dt += 10) {
        if (dt % 50 == 0) assert(four_forward_heartbeat(&g, start + dt));
        assert(four_forward_tick(&g, raw, start + dt));
    }
    for (unsigned i = 0; i < 4; ++i) {
        assert(g.raw_duty[i] * g.loop[i].config.wheel[i].motor_sign > 0);
        assert(abs(g.raw_duty[i]) <= 100);
    }
    progress(raw, -1);
    assert(four_forward_tick(&g, raw, start + 510));
    assert(g.enabled && g.fault == 0U);
    for (unsigned i = 0; i < 4; ++i) {
        assert(g.counts[i] == raw[i]);
        assert(g.loop[i].wheel[i].measured.forward_cps == 100.0f);
    }
    four_forward_stop(&g, 0, start + 510);
    zero(&g);
}

static void test_timeout_and_latch(void)
{
    four_forward_group_t g;
    int32_t raw[4] = {0};
    const uint32_t start = ready(&g, raw, 0);
    for (uint32_t dt = 10; dt < 150; dt += 10)
        assert(four_forward_tick(&g, raw, start + dt));
    assert(!four_forward_tick(&g, raw, start + 150));
    assert(g.fault == FOUR_SPEED_FAULT_COMMAND);
    zero(&g);
    assert(!four_forward_heartbeat(&g, start + 151));
    four_forward_stop(&g, 0, start + 152);
    assert(g.fault == FOUR_SPEED_FAULT_COMMAND && g.run_used);
    assert(!four_forward_arm(&g, start + 1000));
    ready(&g, raw, 0);
    assert(!four_forward_heartbeat(&g, 650)); /* late heartbeat cannot revive */
    zero(&g);
}

static void test_each_feedback_fault(void)
{
    for (unsigned failed = 0; failed < 4; ++failed) {
        for (unsigned overspeed = 0; overspeed < 2; ++overspeed) {
            four_forward_group_t g;
            int32_t raw[4] = {0};
            ready(&g, raw, 0);
            progress(raw, -1);
            const int sign = g.loop[failed].config.wheel[failed].encoder_sign;
            raw[failed] = (overspeed ? 101 : -1) * sign;
            assert(!four_forward_tick(&g, raw, 510));
            assert(g.fault == FOUR_SPEED_FAULT_FEEDBACK);
            zero(&g); /* even previously processed healthy loops must be stopped */
        }
    }
}

static void test_each_stall(void)
{
    for (unsigned failed = 0; failed < 4; ++failed) {
        for (unsigned had_progress = 0; had_progress < 2; ++had_progress) {
            four_forward_group_t g;
            int32_t raw[4] = {0};
            const uint32_t start = ready(&g, raw, 0);
            const uint32_t deadline = had_progress ? 510U : 800U;
            for (uint32_t dt = 10; dt <= deadline; dt += 10) {
                progress(raw, dt == 10 && had_progress ? -1 : (int)failed);
                if (dt % 50 == 0) assert(four_forward_heartbeat(&g, start + dt));
                assert(four_forward_tick(&g, raw, start + dt) == (dt < deadline));
            }
            assert(g.fault == FOUR_SPEED_FAULT_STALL);
            zero(&g);
        }
    }
}

static void test_duration(void)
{
    four_forward_group_t g;
    int32_t raw[4] = {0};
    const uint32_t start = ready(&g, raw, 0);
    for (uint32_t dt = 10; dt < 5000; dt += 10) {
        progress(raw, -1);
        if (dt % 50 == 0) assert(four_forward_heartbeat(&g, start + dt));
        assert(four_forward_tick(&g, raw, start + dt));
    }
    assert(!four_forward_tick(&g, raw, start + 5000));
    assert(g.fault == FOUR_SPEED_FAULT_DURATION);
    zero(&g);
}

static void test_sample_timing_and_null(void)
{
    four_forward_group_t g;
    int32_t raw[4] = {0};
    ready(&g, raw, 0);
    assert(!four_forward_tick(&g, raw, 505));
    assert(g.enabled && g.fault == 0U);
    assert(!four_forward_tick(&g, raw, 531));
    assert(g.fault == FOUR_SPEED_FAULT_TIMING);
    zero(&g);
    ready(&g, raw, 0);
    assert(!four_forward_tick(&g, NULL, 510));
    assert(g.fault == FOUR_SPEED_FAULT_FEEDBACK);
    zero(&g);
    assert(!four_forward_tick(NULL, raw, 0));
    assert(!four_forward_arm(NULL, 0));
    assert(!four_forward_heartbeat(NULL, 0));
    four_forward_stop(NULL, 0, 0);
}

static void test_clock_and_count_wrap(void)
{
    four_forward_group_t g;
    int32_t raw[4] = {INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN};
    uint32_t start = ready(&g, raw, UINT32_MAX - 100U);
    raw[0] = raw[2] = INT32_MIN;
    raw[1] = raw[3] = INT32_MAX;
    assert(four_forward_tick(&g, raw, start + 10U));
    for (unsigned i = 0; i < 4; ++i)
        assert(g.loop[i].wheel[i].measured.forward_cps == 100.0f);
    four_forward_stop(&g, 0, start + 10U);
    zero(&g);
}

int main(void)
{
    test_profile_and_lock();
    test_settle_and_one_shot();
    test_forward_signs_and_stop();
    test_timeout_and_latch();
    test_each_feedback_fault();
    test_each_stall();
    test_duration();
    test_sample_timing_and_null();
    test_clock_and_count_wrap();
    puts("four-forward group: 9 offline test groups passed (not hardware)");
    return 0;
}
