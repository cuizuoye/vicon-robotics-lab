#include "four_wheel_speed.h"

#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>

/* 人工测试数据，不是MG310标定值或实机可用PI增益。 */
static four_speed_config_t test_config(void)
{
    four_speed_config_t c;
    four_speed_defaults(&c);
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        c.wheel[i].counts_per_rev = 1000.0f;
        c.wheel[i].calibration_confirmed = true;
        c.wheel[i].kp_permille_per_cps = 1.0f;
        c.wheel[i].ki_permille_per_count = 0.5f;
    }
    return c;
}

static void assert_zero(const four_speed_controller_t *c)
{
    assert(!c->enabled);
    for (unsigned i = 0; i < FOUR_SPEED_WHEELS; ++i) {
        assert(c->wheel[i].raw_duty_permille == 0);
        assert(c->wheel[i].requested_cps == 0.0f);
        assert(c->wheel[i].ramped_cps == 0.0f);
        assert(c->wheel[i].integral_permille == 0.0f);
    }
}

static void test_units(void)
{
    assert(FOUR_SPEED_FL == 0 && FOUR_SPEED_FR == 1 && FOUR_SPEED_RL == 2 &&
           FOUR_SPEED_RR == 3 && FOUR_SPEED_WHEELS == 4);
    four_speed_config_t config = test_config();
    four_speed_measurement_t m;
    float target;
    assert(four_speed_counts_to_measurement(&config.wheel[0], 10, 10, &m));
    assert(fabsf(m.forward_cps - 1000.0f) < 0.001f);
    assert(fabsf(m.rpm - 60.0f) < 0.001f);
    assert(fabsf(m.speed_m_s - 0.18849556f) < 0.00001f);
    assert(four_speed_m_s_to_cps(&config.wheel[0], m.speed_m_s, &target));
    assert(fabsf(target - 1000.0f) < 0.01f);
    assert(four_speed_counts_to_measurement(&config.wheel[1], -20, 20, &m));
    assert(m.forward_cps == 1000.0f && m.rpm == 60.0f);
    assert(four_speed_counts_to_measurement(&config.wheel[1], 20, 20, &m));
    assert(m.forward_cps == -1000.0f);
    assert(!four_speed_counts_to_measurement(&config.wheel[0], 10, 0, &m));
    assert(m.forward_cps == 0.0f && m.rpm == 0.0f && m.speed_m_s == 0.0f);
    config.wheel[0].counts_per_rev = 0.0f;
    assert(!four_speed_m_s_to_cps(&config.wheel[0], 1.0f, &target) && target == 0.0f);
    config.wheel[0].counts_per_rev = NAN;
    assert(!four_speed_counts_to_measurement(&config.wheel[0], 10, 10, &m));
    assert(!four_speed_counts_to_measurement(NULL, 10, 10, &m));
    assert(!four_speed_counts_to_measurement(&config.wheel[1], 10, 10, NULL));
    assert(!four_speed_m_s_to_cps(&config.wheel[1], INFINITY, &target));
}

static void test_config_lock(void)
{
    four_speed_config_t config;
    four_speed_defaults(&config);
    four_speed_controller_t c;
    int32_t raw[4] = {0};
    assert(!four_speed_init(&c, &config));
    assert(c.fault == FOUR_SPEED_FAULT_CONFIG);
    assert(!four_speed_arm(&c, 0, raw, 0));
    assert_zero(&c);
    config = test_config();
    config.wheel[2].calibration_confirmed = false;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.wheel[0].kp_permille_per_cps = 0;
    config.wheel[0].ki_permille_per_count = 0;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.wheel[0].encoder_sign = 0;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.wheel[0].ki_permille_per_count = NAN;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.duty_limit_permille = 200;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.duty_limit_permille = 95;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.max_sample_gap_ms = 31;
    assert(!four_speed_init(&c, &config));
    config = test_config();
    config.feedback_filter_ms = NAN;
    assert(!four_speed_init(&c, &config));
    assert(!four_speed_init(&c, NULL));
    assert(!four_speed_init(NULL, &config));
}

static void test_ramp_pi_and_signs(void)
{
    for (unsigned wheel = 0; wheel < FOUR_SPEED_WHEELS; ++wheel) {
        for (int direction = -1; direction <= 1; direction += 2) {
            four_speed_config_t config = test_config();
            four_speed_controller_t c;
            int32_t raw[4] = {0};
            assert(four_speed_init(&c, &config));
            assert(four_speed_arm(&c, (uint8_t)wheel, raw, 0));
            assert(four_speed_set_target(&c, (float)direction * 400.0f, 0));
            assert(!four_speed_tick(&c, raw, 5));
            assert(four_speed_tick(&c, raw, 10));
            assert(c.wheel[wheel].ramped_cps == (float)direction * 2.0f);
            assert(four_speed_tick(&c, raw, 30));
            assert(c.wheel[wheel].ramped_cps == (float)direction * 6.0f);
            const int expected_raw_sign = direction * config.wheel[wheel].motor_sign;
            assert(c.wheel[wheel].raw_duty_permille == 10 * expected_raw_sign);
            for (unsigned i = 0; i < 4; ++i)
                if (i != wheel) assert(c.wheel[i].raw_duty_permille == 0);
            raw[wheel] = 10 * direction * config.wheel[wheel].encoder_sign;
            assert(four_speed_tick(&c, raw, 40));
            assert(c.wheel[wheel].measured.forward_cps == (float)direction * 1000.0f);
            assert(c.wheel[wheel].raw_duty_permille == 0); /* 超速，不反向制动 */
            assert(four_speed_set_target(&c, 0, 40));
            assert(c.wheel[wheel].raw_duty_permille == 0);
            assert(c.wheel[wheel].integral_permille == 0);
            four_speed_stop(&c, 0);
            assert_zero(&c);
        }
    }
}

static void test_saturation(void)
{
    four_speed_config_t config = test_config();
    config.ramp_cps_per_s = 50000.0f;
    four_speed_controller_t c;
    int32_t raw[4] = {0};
    assert(four_speed_init(&c, &config));
    assert(four_speed_arm(&c, 0, raw, 0));
    assert(four_speed_set_target(&c, 500, 0));
    for (uint32_t t = 10; t <= 100; t += 10) {
        assert(four_speed_tick(&c, raw, t));
        assert(c.wheel[0].raw_duty_permille == 100);
        assert(c.wheel[0].integral_permille == 0); /* 饱和不堆积分 */
    }
    assert(four_speed_set_target(&c, 0, 100));
    assert(c.wheel[0].raw_duty_permille == 0);
    four_speed_stop(&c, 0);
    assert_zero(&c);
}

static void test_faults(void)
{
    four_speed_config_t config = test_config();
    four_speed_controller_t c;
    int32_t raw[4] = {0};
    assert(four_speed_init(&c, &config));
    assert(four_speed_arm(&c, 0, raw, 0));
    assert(four_speed_set_target(&c, 400, 0));
    for (uint32_t t = 10; t < 150; t += 10) assert(four_speed_tick(&c, raw, t));
    assert(!four_speed_tick(&c, raw, 150));
    assert(c.fault == FOUR_SPEED_FAULT_COMMAND);
    assert_zero(&c);
    assert(!four_speed_arm(&c, 0, raw, 151)); /* 锁存故障，不自动重启 */
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(four_speed_set_target(&c, 400, 0));
    assert(!four_speed_tick(&c, raw, 31) && c.fault == FOUR_SPEED_FAULT_TIMING);
    assert_zero(&c);
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(!four_speed_set_target(&c, NAN, 0));
    assert_zero(&c);
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(!four_speed_set_target(&c, 501, 0));
    assert_zero(&c);
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(four_speed_set_target(&c, 300, 0));
    assert(!four_speed_set_target(&c, -300, 10));
    assert_zero(&c);
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    raw[2] = 1;
    assert(!four_speed_tick(&c, raw, 10) && c.fault == FOUR_SPEED_FAULT_FEEDBACK);
    assert_zero(&c);
    raw[2] = 0;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    raw[0] = 101;
    assert(!four_speed_tick(&c, raw, 10) && c.fault == FOUR_SPEED_FAULT_FEEDBACK);
    assert_zero(&c);
    raw[0] = 0;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(!four_speed_tick(&c, NULL, 10));
    assert_zero(&c);
}

static void test_progress_and_duration(void)
{
    four_speed_config_t config = test_config();
    four_speed_controller_t c;
    int32_t raw[4] = {0};
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    for (uint32_t t = 10; t < 800; t += 10) {
        assert(four_speed_set_target(&c, 400, t));
        assert(four_speed_tick(&c, raw, t));
    }
    assert(four_speed_set_target(&c, 400, 800));
    raw[0] = 1; /* 到截止时间才出现计数，不能复活超时窗口 */
    assert(!four_speed_tick(&c, raw, 800) && c.fault == FOUR_SPEED_FAULT_STALL);
    assert_zero(&c);
    raw[0] = 0;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    raw[0] = 1;
    assert(four_speed_set_target(&c, 400, 10) && four_speed_tick(&c, raw, 10));
    for (uint32_t t = 20; t < 510; t += 10)
        assert(four_speed_set_target(&c, 400, t) && four_speed_tick(&c, raw, t));
    raw[0] = 2;
    assert(four_speed_set_target(&c, 400, 510));
    assert(!four_speed_tick(&c, raw, 510) && c.fault == FOUR_SPEED_FAULT_STALL);
    assert_zero(&c);
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    for (uint32_t t = 10; t < 5000; t += 10) {
        raw[0]++;
        assert(four_speed_set_target(&c, 400, t) && four_speed_tick(&c, raw, t));
    }
    assert(!four_speed_tick(&c, raw, 5000));
    assert(c.fault == FOUR_SPEED_FAULT_DURATION);
    assert_zero(&c);
}

static void test_wrap_and_numeric_guard(void)
{
    four_speed_config_t config = test_config();
    four_speed_controller_t c;
    int32_t raw[4] = {INT32_MAX, 0, 0, 0};
    assert(four_speed_init(&c, &config));
    assert(four_speed_arm(&c, 0, raw, UINT32_MAX - 4U));
    assert(four_speed_set_target(&c, 400, UINT32_MAX - 4U));
    raw[0] = INT32_MIN;
    assert(four_speed_tick(&c, raw, 5));
    assert(c.wheel[0].measured.forward_cps == 100.0f);
    assert(fabsf(c.wheel[0].filtered_cps - 100.0f / 6.0f) < 0.001f);
    raw[0] = INT32_MIN;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    raw[0] = INT32_MAX;
    assert(four_speed_tick(&c, raw, 10));
    assert(c.wheel[0].measured.forward_cps == -100.0f);
    config.wheel[0].kp_permille_per_cps = FLT_MAX;
    raw[0] = 0;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    assert(four_speed_set_target(&c, 400, 0));
    assert(!four_speed_tick(&c, raw, 10) && c.fault == FOUR_SPEED_FAULT_CONFIG);
    assert_zero(&c);
}

static void test_compare(void)
{
    uint32_t a, b;
    assert(four_speed_compare(0, &a, &b) && a == 0 && b == 0);
    assert(four_speed_compare(100, &a, &b) && a == 100 && b == 90);
    assert(four_speed_compare(-100, &a, &b) && a == 90 && b == 100);
    assert(four_speed_compare(10, &a, &b) && a == 100 && b == 99);
    assert(!four_speed_compare(101, &a, &b) && a == 0 && b == 0);
    assert(!four_speed_compare(5, &a, &b) && a == 0 && b == 0);
    assert(!four_speed_compare(0, NULL, &b));
}

static void test_synthetic_motor(void)
{
    /* 人工一阶模型+量化计数，未辨识真实电机，不作为悬空/落地验收。 */
    four_speed_config_t config = test_config();
    config.wheel[0].kp_permille_per_cps = 0.1f;
    config.wheel[0].ki_permille_per_count = 0.5f;
    four_speed_controller_t c;
    int32_t raw[4] = {0};
    float position = 0.0f;
    float actual_cps = 0.0f;
    float sum = 0.0f;
    unsigned samples = 0;
    assert(four_speed_init(&c, &config) && four_speed_arm(&c, 0, raw, 0));
    for (uint32_t t = 10; t < 5000; t += 10) {
        const float gain = t < 2500 ? 6.0f : 4.0f;
        actual_cps += 0.01f / 0.08f * (gain * c.wheel[0].raw_duty_permille - actual_cps);
        position += actual_cps * 0.01f;
        raw[0] = (int32_t)position;
        assert(four_speed_set_target(&c, 300.0f, t));
        assert(four_speed_tick(&c, raw, t));
        assert(c.wheel[0].raw_duty_permille >= 0 && c.wheel[0].raw_duty_permille <= 100);
        for (unsigned i = 1; i < 4; ++i) assert(c.wheel[i].raw_duty_permille == 0);
        if (t >= 4500) {
            sum += c.wheel[0].filtered_cps;
            samples++;
        }
    }
    const float mean = sum / (float)samples;
    assert(fabsf(mean - 300.0f) < 40.0f);
    printf("synthetic motor: target=300 cps, final-0.5s filtered mean=%.2f cps; not hardware\n", (double)mean);
    four_speed_stop(&c, 0);
    assert_zero(&c);
}

int main(void)
{
    test_units();
    test_config_lock();
    test_ramp_pi_and_signs();
    test_saturation();
    test_faults();
    test_progress_and_duration();
    test_wrap_and_numeric_guard();
    test_compare();
    test_synthetic_motor();
    puts("four-wheel speed: 9 test groups passed (offline only)");
    return 0;
}
