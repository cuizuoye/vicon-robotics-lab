#include "bench_dual_control.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#ifndef BENCH_FRONT_ENCODERS_READONLY
#error "requires readonly profile"
#endif

static void stopped(const bench_dual_control_t *c)
{
    assert(!c->armed && c->mask == 0U && c->pulse_limit_ms == 0U);
    assert(c->duty_a == 0 && c->duty_b == 0);
}

int main(void)
{
    bench_dual_control_t c;
    bench_dual_init(&c);
    stopped(&c);
    assert(bench_dual_accept(&c, "status", 1U) == NULL);
    assert(strcmp(bench_dual_accept(&c, "stop", 2U), "OK,stopped_all\r\n") == 0);
    assert(c.fault_bits == 0U);
    const char *bad[] = {"hb", "arm A A5", "arm B A5", "arm AB A5", "arm AB A5 5s",
        "arm AB A5 30s", "duty A 100", "duty A -100", "duty B 100", "duty B -100",
        "duty AB 100 -100", "clear 5A", "zero", "speed 100", "brake", ""};
    for (size_t i = 0U; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        bench_dual_init(&c);
        assert(strcmp(bench_dual_accept(&c, bad[i], 10U), "ERR,readonly_command\r\n") == 0);
        stopped(&c);
        assert(c.fault_bits == BENCH_DUAL_FAULT_PROTOCOL);
        (void)bench_dual_accept(&c, "stop", 20U);
        stopped(&c);
        assert(c.fault_bits == BENCH_DUAL_FAULT_PROTOCOL);
    }
    bench_dual_init(&c);
    (void)bench_dual_accept(&c, "estop", 100U);
    stopped(&c);
    assert(c.fault_bits == BENCH_DUAL_FAULT_ESTOP);
    bench_dual_stop(&c, BENCH_DUAL_FAULT_UART);
    bench_dual_tick(&c, UINT32_MAX);
    stopped(&c);
    assert(c.fault_bits == (BENCH_DUAL_FAULT_ESTOP | BENCH_DUAL_FAULT_UART));
    bench_dual_init(&c);
    assert(strcmp(bench_dual_accept(&c, NULL, 1U), "ERR,bad_command\r\n") == 0);
    stopped(&c);
    puts("front encoder readonly: all commands stay disabled; faults latch (offline)");
    return 0;
}
