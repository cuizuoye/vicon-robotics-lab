#include "bench_dual_control.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void assert_stopped(const bench_dual_control_t *control)
{
    assert(!control->armed && control->mask == 0U);
    assert(control->duty_a == 0 && control->duty_b == 0);
}

static void arm(bench_dual_control_t *control, const char *command, uint32_t now)
{
    assert(bench_dual_accept(control, "hb", now) == NULL);
    assert(strncmp(bench_dual_accept(control, command, now + 30U), "OK,armed=", 9U) == 0);
}

static void test_boot_arm_and_single_channels(void)
{
    const char *arms[] = {"arm A A5", "arm B A5", "arm AB A5"};
    const char *duties[] = {"duty A 100", "duty B 100", "duty AB 100 -100"};
    for (size_t i = 0; i < 3U; ++i) {
        bench_dual_control_t c;
        bench_dual_init(&c);
        assert_stopped(&c);
        assert(c.fault_bits == 0U && !c.heartbeat_seen);
        arm(&c, arms[i], 100U);
        assert(c.armed && c.mask == i + 1U);
        assert(c.duty_a == 0 && c.duty_b == 0);
        assert(strncmp(bench_dual_accept(&c, duties[i], 160U), "OK,duty=", 8U) == 0);
        assert(c.pulse_used);
        assert(c.duty_a == (i == 1U ? 0 : 100));
        assert(c.duty_b == (i == 0U ? 0 : i == 1U ? 100 : -100));
        assert(strcmp(bench_dual_accept(&c, "stop", 170U), "OK,stopped_all\r\n") == 0);
        assert_stopped(&c);
        assert(!c.heartbeat_seen && c.fault_bits == 0U);
    }
}

static void test_reverse_from_stopped_only(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm A A5", 100U);
    assert(strcmp(bench_dual_accept(&c, "duty A -100", 160U),
                  "OK,duty=A:-100,B:0\r\n") == 0);
    assert(c.duty_a == -100 && c.duty_b == 0);
    assert(strcmp(bench_dual_accept(&c, "duty A 100", 180U),
                  "ERR,one_pulse_per_arm\r\n") == 0);
    assert_stopped(&c);
    assert(c.fault_bits & BENCH_DUAL_FAULT_PROTOCOL);
}

static void test_repeat_and_channel_switch_stop_both(void)
{
    const char *bad[] = {"duty AB 100 100", "duty AB -100 -100",
                         "duty A 100", "duty B 100", "arm A A5", "arm AB A5"};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        bench_dual_control_t c;
        bench_dual_init(&c);
        arm(&c, "arm AB A5", 100U);
        (void)bench_dual_accept(&c, "duty AB 100 100", 160U);
        assert(c.duty_a == 100 && c.duty_b == 100);
        assert(strncmp(bench_dual_accept(&c, bad[i], 180U), "ERR,", 4U) == 0);
        assert_stopped(&c);
        assert(c.fault_bits & BENCH_DUAL_FAULT_PROTOCOL);
    }
}

static void test_invalid_legacy_and_malformed_commands(void)
{
    const char *bad[] = {"arm A5", "duty 100", "duty A 200", "duty B 101",
                         "duty AB 100", "duty AB 100 100 extra", "duty A 0100",
                         "duty A 100 ", "arm C A5", "arm A A4", "arm A A5 extra",
                         "speed 900", "clear 5A", "zero", "brake", "hb extra", ""};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        bench_dual_control_t c;
        bench_dual_init(&c);
        arm(&c, "arm AB A5", 100U);
        (void)bench_dual_accept(&c, "duty AB 100 100", 160U);
        assert(strcmp(bench_dual_accept(&c, bad[i], 180U), "ERR,bad_command\r\n") == 0);
        assert_stopped(&c);
        assert(c.fault_bits & BENCH_DUAL_FAULT_PROTOCOL);
    }
    bench_dual_control_t c;
    bench_dual_init(&c);
    assert(strcmp(bench_dual_accept(&c, NULL, 0U), "ERR,bad_command\r\n") == 0);
    assert_stopped(&c);
}

static void test_arm_and_duty_preconditions(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    assert(strcmp(bench_dual_accept(&c, "arm A A5", 100U),
                  "ERR,heartbeat_required\r\n") == 0);
    assert_stopped(&c);
    bench_dual_init(&c);
    (void)bench_dual_accept(&c, "hb", 100U);
    assert(strncmp(bench_dual_accept(&c, "arm B A5", 601U), "ERR,", 4U) == 0);
    assert_stopped(&c);
    bench_dual_init(&c);
    assert(strncmp(bench_dual_accept(&c, "duty B 100", 100U), "ERR,", 4U) == 0);
    assert_stopped(&c);
    bench_dual_init(&c);
    arm(&c, "arm B A5", 100U);
    assert(strncmp(bench_dual_accept(&c, "duty A 100", 160U), "ERR,", 4U) == 0);
    assert_stopped(&c);
}

static void test_heartbeat_timeout_and_late_heartbeat(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm AB A5", 100U);
    (void)bench_dual_accept(&c, "duty AB 100 100", 160U);
    bench_dual_tick(&c, 600U);
    assert(c.armed);
    /* 新心跳必须先触发过期检查，不能复活动作。 */
    (void)bench_dual_accept(&c, "hb", 601U);
    assert_stopped(&c);
    assert(c.fault_bits & BENCH_DUAL_FAULT_TIMEOUT);
    assert(strncmp(bench_dual_accept(&c, "arm AB A5", 631U), "ERR,", 4U) == 0);
    assert_stopped(&c);
}

static void test_pulse_limit_cannot_be_extended_by_heartbeat(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm AB A5", 100U);
    (void)bench_dual_accept(&c, "duty AB 100 -100", 160U);
    for (uint32_t now = 200U; now <= 2300U; now += 100U) {
        (void)bench_dual_accept(&c, "hb", now);
        assert(c.armed);
    }
    bench_dual_tick(&c, 2359U);
    assert(c.armed);
    (void)bench_dual_accept(&c, "hb", 2360U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT);
    (void)bench_dual_accept(&c, "stop", 2400U);
    assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT);
}

static void test_unsigned_time_wrap(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    const uint32_t start = UINT32_MAX - 250U;
    arm(&c, "arm AB A5", start);
    (void)bench_dual_accept(&c, "duty AB 100 100", start + 60U);
    bench_dual_tick(&c, start + 500U);
    assert(c.armed);
    bench_dual_tick(&c, start + 501U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_TIMEOUT);
    bench_dual_init(&c);
    arm(&c, "arm AB A5", start);
    (void)bench_dual_accept(&c, "duty AB 100 100", start + 60U);
    for (uint32_t elapsed = 100U; elapsed < 2260U; elapsed += 100U) {
        (void)bench_dual_accept(&c, "hb", start + elapsed);
    }
    bench_dual_tick(&c, start + 2260U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT);
}

static void test_all_channel_fault_stop_and_rearm_boundary(void)
{
    const uint32_t faults[] = {BENCH_DUAL_FAULT_UART, BENCH_DUAL_FAULT_ESTOP};
    for (size_t i = 0; i < 2U; ++i) {
        bench_dual_control_t c;
        bench_dual_init(&c);
        arm(&c, "arm AB A5", 100U);
        (void)bench_dual_accept(&c, "duty AB -100 -100", 160U);
        if (i == 0U) {
            bench_dual_stop(&c, faults[i]);
        } else {
            assert(strcmp(bench_dual_accept(&c, "estop", 170U), "OK,estop_all\r\n") == 0);
        }
        assert_stopped(&c);
        (void)bench_dual_accept(&c, "stop", 180U);
        (void)bench_dual_accept(&c, "hb", 210U);
        assert(strncmp(bench_dual_accept(&c, "arm A A5", 240U), "ERR,", 4U) == 0);
        assert_stopped(&c);
        assert(c.fault_bits & faults[i]);
    }
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm A A5", 100U);
    (void)bench_dual_accept(&c, "stop", 150U);
    assert(strncmp(bench_dual_accept(&c, "arm B A5", 160U), "ERR,", 4U) == 0);
    assert_stopped(&c);
}

static void test_pwm_mapping_and_out_of_range(void)
{
    uint32_t a = 999U, b = 999U;
    assert(bench_dual_pwm_compare(0, &a, &b) && a == 0U && b == 0U);
    assert(bench_dual_pwm_compare(100, &a, &b) && a == 100U && b == 90U);
    assert(bench_dual_pwm_compare(-100, &a, &b) && a == 90U && b == 100U);
    assert(!bench_dual_pwm_compare(200, &a, &b) && a == 0U && b == 0U);
    assert(!bench_dual_pwm_compare(INT16_MIN, &a, &b) && a == 0U && b == 0U);
    assert(!bench_dual_pwm_compare(100, NULL, &b));
}

static void test_long_forward_observation_cap_and_heartbeat(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm AB A5 5s", 100U);
    assert(c.forward_observe_only && c.pulse_limit_ms == 5200U);
    assert(strcmp(bench_dual_accept(&c, "duty AB 100 -100", 160U),
                  "OK,duty=A:100,B:-100\r\n") == 0);
    for (uint32_t now = 200U; now <= 5300U; now += 100U) {
        (void)bench_dual_accept(&c, "hb", now);
        assert(c.armed && c.duty_a == 100 && c.duty_b == -100);
    }
    bench_dual_tick(&c, 5359U);
    assert(c.armed);
    (void)bench_dual_accept(&c, "hb", 5360U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT);

    bench_dual_init(&c);
    arm(&c, "arm AB A5 5s", 100U);
    (void)bench_dual_accept(&c, "duty AB 100 -100", 160U);
    bench_dual_tick(&c, 601U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_TIMEOUT);
}

static void test_long_arm_does_not_extend_other_modes_or_allow_other_directions(void)
{
    const char *wrong[] = {"duty AB 100 100", "duty AB -100 -100", "duty A 100", "duty B -100"};
    for (size_t i = 0; i < sizeof(wrong) / sizeof(wrong[0]); ++i) {
        bench_dual_control_t c;
        bench_dual_init(&c);
        arm(&c, "arm AB A5 5s", 100U);
        assert(strncmp(bench_dual_accept(&c, wrong[i], 160U), "ERR,", 4U) == 0);
        assert_stopped(&c);
        assert(c.pulse_limit_ms == 0U && !c.forward_observe_only);
    }
    bench_dual_control_t c;
    bench_dual_init(&c);
    arm(&c, "arm AB A5 5s", 100U);
    (void)bench_dual_accept(&c, "stop", 150U);
    arm(&c, "arm AB A5", 200U);
    assert(c.pulse_limit_ms == 2200U && !c.forward_observe_only);
    (void)bench_dual_accept(&c, "duty AB 100 100", 260U);
    for (uint32_t now = 300U; now <= 2400U; now += 100U) {
        (void)bench_dual_accept(&c, "hb", now);
    }
    bench_dual_tick(&c, 2460U);
    assert_stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT);
}

int main(void)
{
    test_boot_arm_and_single_channels();
    test_reverse_from_stopped_only();
    test_repeat_and_channel_switch_stop_both();
    test_invalid_legacy_and_malformed_commands();
    test_arm_and_duty_preconditions();
    test_heartbeat_timeout_and_late_heartbeat();
    test_pulse_limit_cannot_be_extended_by_heartbeat();
    test_unsigned_time_wrap();
    test_all_channel_fault_stop_and_rearm_boundary();
    test_pwm_mapping_and_out_of_range();
    test_long_forward_observation_cap_and_heartbeat();
    test_long_arm_does_not_extend_other_modes_or_allow_other_directions();
    puts("bench_dual_control: 12 scenario groups PASS (offline; no hardware)");
    return 0;
}
