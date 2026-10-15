#include "drive_bench_control.h"
#include "speed_uart_queue.h"

#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const char *const commands[] = {"run F50 S5", "run L50 S5", "run Y200 S5", "run D25 S5"};
static const int signs[4][4] = {{1,1,1,1}, {-1,1,1,-1}, {-1,1,-1,1}, {0,1,1,0}};

static void settled(drive_bench_t *b, int32_t raw[4])
{
    drive_bench_init(b, raw, 0);
    for (uint32_t t = 10; t <= 500; t += 10) assert(drive_bench_tick(b, raw, t));
}

static void test_commands_and_modes(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    settled(&b, raw);
    const char *bad[] = {"arm A5", "clear 5A", "duty 10", "run L51 S5", "run Y200 S30",
        "run Y-200 S5", "run F50 S5 ", "run D25 S5\n", "cmd_vel 0 0.05 0"};
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        assert(strncmp(drive_bench_accept(&b, bad[i], 500), "ERR,", 4) == 0);
        assert(!b.group.enabled && !b.group.run_used);
    }
    assert(strstr(drive_bench_accept(&b, commands[0], 500), "no_fresh_heartbeat"));
    for (unsigned m = 0; m < 4; ++m) {
        settled(&b, raw);
        assert(strcmp(drive_bench_accept(&b, "hb", 500), "OK,hb_drive\r\n") == 0);
        assert(strcmp(drive_bench_accept(&b, commands[m], 500), "OK,running_drive\r\n") == 0);
        assert(b.mode == m + 1 && b.group.enabled && b.group.run_used);
        assert(strstr(drive_bench_accept(&b, commands[(m+1)%4], 501), "busy_or_fault"));
        assert(b.mode == m + 1);
        drive_bench_accept(&b, "stop", 501);
        assert(!b.mode && !b.group.enabled && b.group.run_used);
        assert(strstr(drive_bench_accept(&b, commands[m], 502), "one_shot_used"));
    }
    assert(drive_bench_accept(&b, "status", 502) == NULL);
}

static void test_real_channels_and_telemetry(void)
{
    drive_bench_t b; int32_t raw[4] = {0};
    char frame[SPEED_UART_FRAME_BYTES];
    for (unsigned m = 0; m < 4; ++m) {
        memset(raw, 0, sizeof(raw)); settled(&b, raw);
        drive_bench_accept(&b, "hb", 500); drive_bench_accept(&b, commands[m], 500);
        for (unsigned i = 0; i < 4; ++i) raw[i] = signs[m][i] * b.adopted.wheel[i].encoder_sign;
        assert(drive_bench_tick(&b, raw, 510));
        for (unsigned i = 0; i < 4; ++i) {
            assert(b.counts[i] == raw[i] && b.raw_cps[i] == raw[i] * 100);
            assert(b.speed_mm_s[i] * signs[m][i] >= 0);
        }
        speed_bench_diagnostics_t d = {.ms=510, .ticks=51, .dt_min=10, .dt_max=10};
        assert(four_drive_compares(&b.group, d.compares));
        assert(drive_bench_format(&b, &d, "TELSP", frame, sizeof(frame)));
        assert(strstr(frame, "fw=drive_pi_v1") && strstr(frame, "en=1,sel=5"));
        char expected[25]; snprintf(expected, sizeof(expected), ",mode=%u,", m + 1);
        assert(strstr(frame, expected));
        if (m == 1) assert(strstr(frame, "want_fl=-281") && strstr(frame, "vy=50"));
        if (m == 2) assert(strstr(frame, "wz=200") && strstr(frame, "want_rl=-163"));
        if (m == 3) assert(strstr(frame, "want_fl=0") && strstr(frame, "want_rr=0"));
        drive_bench_accept(&b, "stop", 510);
        raw[3]++;
        assert(drive_bench_tick(&b, raw, 520)); /* coast telemetry is real even after stop */
        assert(b.counts[3] == raw[3] && b.raw_cps[3] == 100);
        assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
        assert(strstr(frame, "mode=0,vx=0,vy=0,wz=0"));
    }
}

static void test_faults_buffer_and_null(void)
{
    drive_bench_t b; int32_t raw[4] = {0}; settled(&b, raw);
    drive_bench_accept(&b, "hb", 500); drive_bench_accept(&b, commands[1], 500);
    for (uint32_t t = 510; t <= 650; t += 10) assert(drive_bench_tick(&b, raw, t));
    assert(b.group.fault == FOUR_SPEED_FAULT_COMMAND && b.mode == 0);
    drive_bench_accept(&b, "stop", 650);
    assert(b.group.fault && b.group.run_used);
    settled(&b, raw); drive_bench_accept(&b, "estop", 500);
    assert(b.group.fault == SPEED_BENCH_FAULT_ESTOP);
    settled(&b, raw); assert(!drive_bench_tick(&b, NULL, 510));
    assert(b.group.fault == FOUR_SPEED_FAULT_FEEDBACK);
    speed_bench_diagnostics_t d = {.ms=UINT32_MAX, .ticks=UINT32_MAX, .dt_min=10, .dt_max=30};
    b.updates = UINT32_MAX;
    for (unsigned i = 0; i < 4; ++i) {
        b.counts[i] = INT32_MIN; b.raw_cps[i] = INT32_MIN;
        b.rpm_milli[i] = -10000000; b.speed_mm_s[i] = -1000000;
    }
    char frame[SPEED_UART_FRAME_BYTES], small[8];
    assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
    assert(strlen(frame) < SPEED_UART_FRAME_BYTES);
    assert(!drive_bench_format(&b, &d, "TELSP", small, sizeof(small)));
    assert(!drive_bench_format(&b, &d, "TELSP", frame, 0));
    assert(!drive_bench_format(NULL, &d, "TELSP", frame, sizeof(frame)));
    assert(!drive_bench_format(&b, NULL, "TELSP", frame, sizeof(frame)));
    assert(!drive_bench_format(&b, &d, NULL, frame, sizeof(frame)));
    assert(!drive_bench_format(&b, &d, "TELSP", NULL, sizeof(frame)));
    assert(!drive_bench_tick(NULL, raw, 0));
    drive_bench_init(NULL, raw, 0);
    assert(strstr(drive_bench_accept(NULL, "status", 0), "ERR,"));
    assert(strstr(drive_bench_accept(&b, NULL, 0), "ERR,"));
    /* 超长/满队列故障由复用的硬件主循环/IRQ变成全组UART停机。 */
    speed_uart_queue_t q; speed_queue_init(&q);
    for (unsigned i = 0; i < SPEED_UART_LINE_BYTES + 1; ++i) speed_queue_receive(&q, 'x');
    assert(q.fault);
    four_drive_stop(&b.group, SPEED_BENCH_FAULT_UART, 520);
    assert(b.group.fault & SPEED_BENCH_FAULT_UART);
}

int main(void)
{
    test_commands_and_modes(); test_real_channels_and_telemetry(); test_faults_buffer_and_null();
    puts("drive bench: 3 offline protocol/telemetry test groups passed (not hardware)");
    return 0;
}
