#include "bench_dual_control.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if !defined(BENCH_FRONT_DEMO30)
#error "This test requires the independent demo30 profile"
#endif

static void stopped(const bench_dual_control_t *c)
{
    assert(!c->armed && c->mask == 0U && c->pulse_limit_ms == 0U);
    assert(c->duty_a == 0 && c->duty_b == 0);
}

static void start(bench_dual_control_t *c, uint32_t now)
{
    bench_dual_init(c);
    stopped(c);
    assert(!c->demo_used);
    assert(bench_dual_accept(c, "hb", now) == NULL);
    assert(strcmp(bench_dual_accept(c, "arm AB A5 30s", now + 30U),
                  "OK,armed=AB30\r\n") == 0);
    assert(c->armed && c->mask == 3U && c->pulse_limit_ms == 30200U);
    assert(c->duty_a == 0 && c->duty_b == 0 && !c->demo_used);
    assert(strcmp(bench_dual_accept(c, "duty AB 100 -100", now + 60U),
                  "OK,duty=A:100,B:-100\r\n") == 0);
    assert(c->demo_used && c->duty_a == 100 && c->duty_b == -100);
}

static void test_thirty_second_request_then_normal_stop(void)
{
    bench_dual_control_t c;
    start(&c, 100U);
    for (uint32_t now = 200U; now <= 30100U; now += 100U) {
        (void)bench_dual_accept(&c, "hb", now);
        assert(c.armed && c.duty_a == 100 && c.duty_b == -100);
    }
    assert(strcmp(bench_dual_accept(&c, "stop", 30160U), "OK,stopped_all\r\n") == 0);
    stopped(&c);
    assert(c.demo_used && c.fault_bits == 0U);
}

static void test_cap_even_with_heartbeat_and_time_wrap(void)
{
    const uint32_t starts[] = {100U, UINT32_MAX - 500U};
    for (size_t i = 0; i < 2U; ++i) {
        bench_dual_control_t c;
        const uint32_t now = starts[i];
        start(&c, now);
        for (uint32_t elapsed = 100U; elapsed <= 30200U; elapsed += 100U) {
            (void)bench_dual_accept(&c, "hb", now + elapsed);
            assert(c.armed);
        }
        bench_dual_tick(&c, now + 60U + 30199U);
        assert(c.armed);
        (void)bench_dual_accept(&c, "hb", now + 60U + 30200U);
        stopped(&c);
        assert(c.fault_bits == BENCH_DUAL_FAULT_PULSE_LIMIT && c.demo_used);
    }
}

static void test_heartbeat_loss_and_late_heartbeat(void)
{
    bench_dual_control_t c;
    start(&c, 100U);
    bench_dual_tick(&c, 600U);
    assert(c.armed);
    (void)bench_dual_accept(&c, "hb", 601U);
    stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_TIMEOUT);
}

static void test_no_second_demo_after_stop(void)
{
    bench_dual_control_t c;
    start(&c, 100U);
    (void)bench_dual_accept(&c, "stop", 200U);
    (void)bench_dual_accept(&c, "hb", 250U);
    assert(strcmp(bench_dual_accept(&c, "arm AB A5 30s", 280U),
                  "ERR,demo_already_used\r\n") == 0);
    stopped(&c);
    assert(c.demo_used && c.fault_bits == BENCH_DUAL_FAULT_PROTOCOL);
}

static void test_arm_without_motion_does_not_consume_demo(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    (void)bench_dual_accept(&c, "hb", 100U);
    (void)bench_dual_accept(&c, "arm AB A5 30s", 130U);
    (void)bench_dual_accept(&c, "stop", 160U);
    assert(!c.demo_used);
    (void)bench_dual_accept(&c, "hb", 200U);
    assert(strcmp(bench_dual_accept(&c, "arm AB A5 30s", 230U),
                  "OK,armed=AB30\r\n") == 0);
    assert(c.armed && c.duty_a == 0 && c.duty_b == 0);
}

static void test_other_directions_and_legacy_commands_stop(void)
{
    const char *bad[] = {"arm A A5", "arm B A5", "arm AB A5", "arm AB A5 5s",
        "arm AB A5 60s", "duty AB 100 100", "duty AB -100 100", "duty A 100",
        "duty B -100", "duty AB 200 -200", "clear 5A", "speed 800", "zero", "brake", ""};
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        bench_dual_control_t c;
        start(&c, 100U);
        assert(strncmp(bench_dual_accept(&c, bad[i], 180U), "ERR,", 4U) == 0);
        stopped(&c);
        assert(c.fault_bits & BENCH_DUAL_FAULT_PROTOCOL);
    }
}

static void test_repeat_and_faults_stop_both(void)
{
    bench_dual_control_t c;
    start(&c, 100U);
    assert(strcmp(bench_dual_accept(&c, "duty AB 100 -100", 180U),
                  "ERR,one_pulse_per_arm\r\n") == 0);
    stopped(&c);
    assert(c.fault_bits & BENCH_DUAL_FAULT_PROTOCOL);
    start(&c, 100U);
    (void)bench_dual_accept(&c, "estop", 180U);
    stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_ESTOP);
    start(&c, 100U);
    bench_dual_stop(&c, BENCH_DUAL_FAULT_UART);
    stopped(&c);
    (void)bench_dual_accept(&c, "stop", 200U);
    assert(c.fault_bits == BENCH_DUAL_FAULT_UART && c.demo_used);
}

static void test_motion_requires_arm_and_heartbeat(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    assert(strncmp(bench_dual_accept(&c, "duty AB 100 -100", 0U), "ERR,", 4U) == 0);
    stopped(&c);
    bench_dual_init(&c);
    assert(strcmp(bench_dual_accept(&c, "arm AB A5 30s", 0U),
                  "ERR,heartbeat_required\r\n") == 0);
    stopped(&c);
}

int main(void)
{
    test_thirty_second_request_then_normal_stop();
    test_cap_even_with_heartbeat_and_time_wrap();
    test_heartbeat_loss_and_late_heartbeat();
    test_no_second_demo_after_stop();
    test_arm_without_motion_does_not_consume_demo();
    test_other_directions_and_legacy_commands_stop();
    test_repeat_and_faults_stop_both();
    test_motion_requires_arm_and_heartbeat();
    puts("front demo30: 8 scenario groups passed (offline only)");
    return 0;
}
