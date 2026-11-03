#include "forward_bench_control.h"
#include "speed_uart_queue.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static void zero(const forward_bench_t *b)
{
    assert(!b->group.enabled);
    for (unsigned i = 0; i < 4; ++i) {
        assert(b->group.raw_duty[i] == 0);
        assert(b->group.loop[i].wheel[i].requested_cps == 0.0f);
    }
}

static void settled(forward_bench_t *b, int32_t raw[4])
{
    forward_bench_init(b, raw, 0);
    for (uint32_t t = 10; t <= 500; t += 10) assert(forward_bench_tick(b, raw, t));
}

static void test_schema_and_start(void)
{
    forward_bench_t b;
    int32_t raw[4] = {0};
    forward_bench_init(&b, raw, 0);
    zero(&b);
    assert(strstr(forward_bench_accept(&b, FORWARD_BENCH_RUN, 0), "not_settled"));
    settled(&b, raw);
    const char *bad[] = {"run FL 300 S5", "run F50 S30", "run F50 S5 ",
        "run F51 S5", "run F-50 S5", "arm A5", "duty 100", "clear 5A", "run F50 S5\n"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        assert(strncmp(forward_bench_accept(&b, bad[i], 500), "ERR,", 4) == 0);
        zero(&b);
        assert(!b.group.run_used);
    }
    assert(strstr(forward_bench_accept(&b, FORWARD_BENCH_RUN, 500), "no_fresh_heartbeat"));
    assert(strcmp(forward_bench_accept(&b, "hb", 500), "OK,hb_forward\r\n") == 0);
    assert(strcmp(forward_bench_accept(&b, FORWARD_BENCH_RUN, 500), "OK,running_forward\r\n") == 0);
    assert(b.group.enabled && b.group.run_used);
    assert(strstr(forward_bench_accept(&b, FORWARD_BENCH_RUN, 501), "busy_or_fault"));
    assert(forward_bench_accept(&b, "status", 501) == NULL);
}

static void test_real_telemetry_and_format(void)
{
    forward_bench_t b;
    int32_t raw[4] = {0};
    settled(&b, raw);
    assert(strncmp(forward_bench_accept(&b, "hb", 500), "OK,", 3) == 0);
    assert(strncmp(forward_bench_accept(&b, FORWARD_BENCH_RUN, 500), "OK,", 3) == 0);
    raw[0] = raw[2] = 1; raw[1] = raw[3] = -1;
    assert(forward_bench_tick(&b, raw, 510));
    for (unsigned i = 0; i < 4; ++i) {
        assert(b.counts[i] == raw[i]);
        assert(b.raw_cps[i] == (i % 2 ? -100 : 100));
        assert(b.rpm_milli[i] > 0 && b.speed_mm_s[i] > 0);
    }
    speed_bench_diagnostics_t d = {.ms=510, .ticks=51, .dt_min=10, .dt_max=10};
    for (unsigned i = 0; i < 4; ++i)
        assert(four_speed_compare(b.group.raw_duty[i], &d.compares[2*i], &d.compares[2*i+1]));
    char frame[SPEED_UART_FRAME_BYTES], small[20];
    assert(forward_bench_format(&b, &d, "TELSP", frame, sizeof(frame)));
    assert(strstr(frame, "fw=forward_pi_v1") && strstr(frame, "en=1,sel=5"));
    assert(strstr(frame, "enc_fr=-1") && strstr(frame, "enc_rr=-1"));
    assert(strstr(frame, "want_fl=281") && strstr(frame, "want_fr=278"));
    assert(strstr(frame, "want_rl=280") && strstr(frame, "want_rr=281"));
    assert(strlen(frame) < SPEED_UART_FRAME_BYTES);
    assert(!forward_bench_format(&b, &d, "TELSP", small, sizeof(small)));
    assert(!forward_bench_format(&b, &d, "TELSP", small, 0));
    forward_bench_accept(&b, "stop", 510);
    raw[3] = -2; /* stop后真实滑行计数仍可见，不用算法环的隔离适配数组。 */
    assert(forward_bench_tick(&b, raw, 520));
    assert(b.counts[3] == -2 && b.raw_cps[3] == -100);
    zero(&b);
}

static void test_faults_and_one_shot(void)
{
    forward_bench_t b;
    int32_t raw[4] = {0};
    settled(&b, raw);
    forward_bench_accept(&b, "hb", 500);
    forward_bench_accept(&b, FORWARD_BENCH_RUN, 500);
    for (uint32_t t = 510; t <= 650; t += 10) assert(forward_bench_tick(&b, raw, t));
    assert(b.group.fault == FOUR_SPEED_FAULT_COMMAND);
    forward_bench_accept(&b, "stop", 650);
    assert(b.group.fault == FOUR_SPEED_FAULT_COMMAND && b.group.run_used);
    zero(&b);
    settled(&b, raw);
    forward_bench_accept(&b, "hb", 500);
    forward_bench_accept(&b, FORWARD_BENCH_RUN, 500);
    forward_bench_accept(&b, "stop", 510);
    for (uint32_t t = 520; t <= 1020; t += 10) assert(forward_bench_tick(&b, raw, t));
    forward_bench_accept(&b, "hb", 1020);
    assert(strstr(forward_bench_accept(&b, FORWARD_BENCH_RUN, 1020), "one_shot_used"));
    zero(&b);
    settled(&b, raw);
    forward_bench_accept(&b, "estop", 500);
    assert(b.group.fault == SPEED_BENCH_FAULT_ESTOP);
    forward_bench_accept(&b, "stop", 501);
    assert(b.group.fault == SPEED_BENCH_FAULT_ESTOP);
    zero(&b);
    settled(&b, raw);
    assert(!forward_bench_tick(&b, NULL, 510));
    assert(b.group.fault == FOUR_SPEED_FAULT_FEEDBACK);
    zero(&b);
}

static void test_uart_queue_boundary(void)
{
    speed_uart_queue_t q;
    speed_queue_init(&q);
    const char *commands = "stop\nhb\nrun F50 S5\nstatus\n";
    for (const char *p = commands; *p; ++p) speed_queue_receive(&q, (uint8_t)*p);
    char line[SPEED_UART_LINE_BYTES];
    assert(speed_queue_take_line(&q, line) && strcmp(line, "stop") == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "hb") == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, FORWARD_BENCH_RUN) == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "status") == 0);
    assert(!q.fault);
    for (unsigned i = 0; i < SPEED_UART_LINE_BYTES + 1U; ++i) speed_queue_receive(&q, 'x');
    assert(q.fault);
    forward_bench_t b;
    int32_t raw[4] = {0};
    settled(&b, raw);
    forward_bench_accept(&b, "hb", 500);
    forward_bench_accept(&b, FORWARD_BENCH_RUN, 500);
    four_forward_stop(&b.group, SPEED_BENCH_FAULT_UART, 510);
    zero(&b);
    assert(b.group.fault == SPEED_BENCH_FAULT_UART && b.group.run_used);
}

int main(void)
{
    test_schema_and_start();
    test_real_telemetry_and_format();
    test_faults_and_one_shot();
    test_uart_queue_boundary();
    puts("forward bench: 4 offline control/telemetry/UART test groups passed");
    return 0;
}
