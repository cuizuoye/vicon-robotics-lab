#include "bench_four_control.h"
#include "encoder_accumulator.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static void stopped(const bench_four_t *c)
{
    assert(c->selected == 0U);
    for (unsigned i = 0; i < FOUR_WHEELS; ++i) assert(c->duty[i] == 0);
}

static void command(bench_four_t *c, const char *line, uint32_t now)
{
    const char *reply = bench_four_accept(c, line, now);
    assert(reply == NULL || strncmp(reply, "OK,", 3U) == 0);
}

int main(void)
{
    bench_four_t c;
    const char *arms[] = {"arm FL A5", "arm FR A5", "arm RL A5", "arm RR A5"};
    const char *duties[][2] = {{"duty FL 100", "duty FL -100"},
        {"duty FR 100", "duty FR -100"}, {"duty RL 100", "duty RL -100"},
        {"duty RR 100", "duty RR -100"}};
    for (unsigned i = 0; i < FOUR_WHEELS; ++i) {
        for (unsigned sign = 0; sign < 2U; ++sign) {
            bench_four_init(&c);
            stopped(&c);
            command(&c, "hb", 0U);
            command(&c, arms[i], 1U);
            for (unsigned j = 0; j < FOUR_WHEELS; ++j) assert(c.duty[j] == 0);
            command(&c, duties[i][sign], 2U);
            assert(c.selected == i + 1U && c.duty[i] == (sign == 0 ? 100 : -100));
            for (unsigned j = 0; j < FOUR_WHEELS; ++j) if (j != i) assert(c.duty[j] == 0);
            /* 重复/在线换向都全停且锁存，不允许补发PWM延长本轮。 */
            assert(strncmp(bench_four_accept(&c, duties[i][1U - sign], 3U), "ERR,", 4U) == 0);
            stopped(&c);
            assert(c.fault == FOUR_FAULT_PROTOCOL);
            command(&c, "stop", 4U);
            assert(c.fault == FOUR_FAULT_PROTOCOL);
        }
    }
    const char *invalid[] = {"arm ALL A5", "duty FL 200", "arm A A5", "clear 5A", "zero",
        "speed 100", "duty FR 100 ", "duty RL 0", "", "arm FL A5 30s", "duty FL 100 100"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        bench_four_init(&c);
        command(&c, "hb", 0U);
        command(&c, "arm FL A5", 1U);
        command(&c, "duty FL 100", 2U);
        assert(strncmp(bench_four_accept(&c, invalid[i], 3U), "ERR,", 4U) == 0);
        stopped(&c);
        assert(c.fault != 0U);
    }
    bench_four_init(&c);
    (void)bench_four_accept(&c, "arm FL A5", 0U);
    stopped(&c);
    assert(c.fault == FOUR_FAULT_PROTOCOL);
    bench_four_init(&c);
    command(&c, "hb", 0U);
    command(&c, "arm FR A5", 1U);
    (void)bench_four_accept(&c, "duty FL 100", 2U);
    stopped(&c);
    assert(c.fault == FOUR_FAULT_PROTOCOL);
    bench_four_init(&c);
    command(&c, "hb", UINT32_MAX - 4U);
    command(&c, "arm RL A5", UINT32_MAX - 3U);
    command(&c, "duty RL -100", UINT32_MAX - 2U);
    bench_four_tick(&c, 495U);
    assert(c.selected == 3U);
    command(&c, "hb", 496U); /* 迟到心跳不能复活已超时请求。 */
    stopped(&c);
    assert(c.fault == FOUR_FAULT_TIMEOUT);
    bench_four_init(&c);
    command(&c, "hb", 0U);
    command(&c, "arm RR A5", 1U);
    command(&c, "duty RR 100", 2U);
    for (uint32_t now = 100U; now <= 5200U; now += 100U) command(&c, "hb", now);
    bench_four_tick(&c, 5202U);
    stopped(&c);
    assert(c.fault == FOUR_FAULT_LIMIT);
    command(&c, "stop", 5300U);
    assert(c.fault == FOUR_FAULT_LIMIT);
    bench_four_init(&c);
    command(&c, "estop", 0U);
    assert(c.fault == FOUR_FAULT_ESTOP);
    bench_four_stop(&c, FOUR_FAULT_UART);
    assert(c.fault == (FOUR_FAULT_ESTOP | FOUR_FAULT_UART));
    bench_four_init(&c);
    (void)bench_four_accept(&c, NULL, 0U);
    stopped(&c);
    assert(c.fault == FOUR_FAULT_PROTOCOL);
    uint32_t a, b;
    assert(bench_four_compare(100, &a, &b) && a == 100U && b == 90U);
    assert(bench_four_compare(-100, &a, &b) && a == 90U && b == 100U);
    assert(bench_four_compare(0, &a, &b) && a == 0U && b == 0U);
    assert(!bench_four_compare(200, &a, &b) && a == 0U && b == 0U);
    assert(!bench_four_compare(100, NULL, &b));

    encoder_accumulator_t e[4];
    for (unsigned i = 0; i < 4U; ++i) encoder_accumulator_init(&e[i], i == 2 ? 32 : 16, 0U, 0U);
    encoder_accumulator_tick(&e[0], 10U, 9U);
    assert(encoder_accumulator_count(&e[0]) == 0);
    encoder_accumulator_tick(&e[0], 10U, 10U);
    encoder_accumulator_tick(&e[1], 65530U, 10U);
    encoder_accumulator_tick(&e[2], UINT32_MAX - 5U, 10U);
    assert(encoder_accumulator_count(&e[0]) == 10 && e[0].cps == 1000);
    assert(encoder_accumulator_count(&e[1]) == -6 && e[1].cps == -600);
    assert(encoder_accumulator_count(&e[2]) == -6 && e[2].cps == -600);
    assert(encoder_accumulator_count(&e[3]) == 0);
    encoder_accumulator_tick(&e[1], 4U, 20U);
    encoder_accumulator_tick(&e[2], 4U, 20U);
    assert(encoder_accumulator_count(&e[1]) == 4 && encoder_accumulator_count(&e[2]) == 4);
    e[0].total = INT32_MAX;
    encoder_accumulator_tick(&e[0], 11U, 20U);
    assert(encoder_accumulator_count(&e[0]) == INT32_MIN);
    encoder_accumulator_tick(&e[0], 10U, 30U);
    assert(encoder_accumulator_count(&e[0]) == INT32_MAX);
    encoder_accumulator_init(&e[3], 16U, 0U, UINT32_MAX - 4U);
    encoder_accumulator_tick(&e[3], 3U, 5U);
    assert(encoder_accumulator_count(&e[3]) == 3 && e[3].cps == 300);
    encoder_accumulator_init(&e[2], 32U, 0U, 0U);
    encoder_accumulator_tick(&e[2], INT32_MAX, 10U);
    assert(e[2].cps == INT32_MAX);
    puts("four bench: 8 directions, isolation, latching, timing/wrap, PWM mapping, encoder widths passed (offline)");
    return 0;
}
