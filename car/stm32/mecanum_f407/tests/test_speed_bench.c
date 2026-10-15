#include "speed_bench_control.h"
#include "speed_uart_queue.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_readonly_and_cpr(void)
{
    speed_bench_t b;
    const int32_t raw[4] = {0};
    speed_bench_init(&b, true, raw, 0);
    assert(b.pi.fault == 0 && !b.pi.enabled);
    assert(fabsf(b.adopted.wheel[0].counts_per_rev - 1059.6667f) < 0.001f);
    assert(b.adopted.wheel[1].counts_per_rev == 1049.0f);
    assert(fabsf(b.adopted.wheel[2].counts_per_rev - 1055.6667f) < 0.001f);
    assert(b.adopted.wheel[3].counts_per_rev == 1061.0f);
    assert(strcmp(speed_bench_accept(&b, "hb", 0), "OK,hb_speed\r\n") == 0);
    assert(strcmp(speed_bench_accept(&b, "run FL 300 S5", 0), "ERR,read_only\r\n") == 0);
    assert(!b.pi.enabled);
    assert(strcmp(speed_bench_accept(&b, "arm FL A5", 0), "ERR,speed_command\r\n") == 0);
    assert(strcmp(speed_bench_accept(&b, "duty FL 100", 0), "ERR,speed_command\r\n") == 0);
    assert(speed_bench_accept(&b, "status", 0) == NULL);
    assert(strcmp(speed_bench_accept(&b, "stop", 0), "OK,stopped_speed\r\n") == 0);
}

static void test_protocol(void)
{
    const int32_t raw[4] = {0};
    speed_bench_t b;
    speed_bench_init(&b, false, raw, 0);
    assert(strcmp(speed_bench_accept(&b, "run FR 300 S5", 0), "ERR,not_settled\r\n") == 0);
    for (uint32_t t = 10; t <= 500; t += 10) assert(speed_bench_tick(&b, raw, t));
    assert(strcmp(speed_bench_accept(&b, "run FR 300 S5", 500), "ERR,no_fresh_heartbeat\r\n") == 0);
    const char *const invalid[] = {"run ", "run FL 0 S5", "run FL +300 S5", "run FF 300 S5",
        "run FL 301 S5", "run FL 300 S5x", "run FL 300 A5", "run FL -0 S5", "run FL 300 S5 "};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        const char *reply = speed_bench_accept(&b, invalid[i], 500);
        assert(reply != NULL && strncmp(reply, "ERR,", 4) == 0 && !b.pi.enabled);
    }
    (void)speed_bench_accept(&b, "hb", 500);
    assert(strcmp(speed_bench_accept(&b, "run FR 300 S5", 500), "OK,running_speed\r\n") == 0);
    assert(b.pi.enabled && b.pi.selected == 1);
    assert(strcmp(speed_bench_accept(&b, "run RR 300 S5", 500), "ERR,busy_or_fault\r\n") == 0);
    assert(speed_bench_tick(&b, raw, 510));
    assert(b.pi.wheel[0].raw_duty_permille == 0 && b.pi.wheel[2].raw_duty_permille == 0);
    (void)speed_bench_accept(&b, "estop", 511);
    assert(!b.pi.enabled && b.pi.fault == SPEED_BENCH_FAULT_ESTOP);
    (void)speed_bench_accept(&b, "stop", 512);
    assert(b.pi.fault == SPEED_BENCH_FAULT_ESTOP);
    assert(strcmp(speed_bench_accept(&b, "run FR 300 S5", 520), "ERR,busy_or_fault\r\n") == 0);
}

static void test_timeout_and_units(void)
{
    int32_t raw[4] = {0};
    speed_bench_t b;
    speed_bench_init(&b, false, raw, 0);
    for (uint32_t t = 10; t <= 500; t += 10) (void)speed_bench_tick(&b, raw, t);
    (void)speed_bench_accept(&b, "hb", 500);
    (void)speed_bench_accept(&b, "run FL 300 S5", 500);
    for (uint32_t t = 510; t < 650; t += 10) (void)speed_bench_tick(&b, raw, t);
    assert(strcmp(speed_bench_accept(&b, "hb", 650), "ERR,heartbeat_late\r\n") == 0);
    assert(!b.pi.enabled && b.pi.fault == FOUR_SPEED_FAULT_COMMAND);
    speed_bench_init(&b, true, raw, 0);
    raw[1] = -10;
    assert(speed_bench_tick(&b, raw, 10));
    assert(b.raw_cps[1] == -1000);
    assert(b.rpm_milli[1] > 57000 && b.rpm_milli[1] < 58000);
    assert(b.speed_mm_s[1] > 170 && b.speed_mm_s[1] < 190);
    assert(!b.pi.enabled);
    assert(speed_bench_tick(&b, raw, 41));
    assert(b.pi.fault == FOUR_SPEED_FAULT_TIMING);
}

static void feed(speed_uart_queue_t *q, const char *text)
{
    for (size_t i = 0; text[i]; ++i) speed_queue_receive(q, (uint8_t)text[i]);
}

static void test_rx(void)
{
    speed_uart_queue_t q;
    char line[SPEED_UART_LINE_BYTES];
    speed_queue_init(&q);
    feed(&q, "stop\r\nstatus\nhb\nrun FL 300 S5\n");
    assert(q.rx_count == 4 && !q.fault);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "stop") == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "status") == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "hb") == 0);
    assert(speed_queue_take_line(&q, line) && strcmp(line, "run FL 300 S5") == 0);
    assert(!speed_queue_take_line(&q, line));
    feed(&q, "1\n2\n3\n4\n5\n");
    assert(q.fault && q.rx_count == 4);
    speed_queue_init(&q);
    for (unsigned i = 0; i < 70; ++i) speed_queue_receive(&q, 'x');
    feed(&q, "run FL 300 S5\n");
    assert(q.fault && !speed_queue_take_line(&q, line));
    feed(&q, "status\n");
    assert(speed_queue_take_line(&q, line) && strcmp(line, "status") == 0);
    speed_queue_init(&q);
    speed_queue_receive(&q, 0);
    feed(&q, "run FL 300 S5\n");
    assert(q.fault && !speed_queue_take_line(&q, line));
}

static void test_tx(void)
{
    speed_uart_queue_t q;
    speed_queue_init(&q);
    uint16_t length;
    assert(speed_queue_peek_tx(&q, &length) == NULL && length == 0);
    for (unsigned i = 0; i < 4; ++i) assert(speed_queue_send(&q, "frame", 5));
    assert(!speed_queue_send(&q, "overflow", 8) && q.fault);
    assert(strcmp(speed_queue_peek_tx(&q, &length), "frame") == 0 && length == 5);
    speed_queue_finish_tx(&q);
    assert(speed_queue_send(&q, "next", 4));
    for (unsigned i = 0; i < 4; ++i) speed_queue_finish_tx(&q);
    assert(speed_queue_peek_tx(&q, &length) == NULL);
    speed_queue_init(&q);
    assert(!speed_queue_send(&q, NULL, 5));
    speed_queue_init(&q);
    assert(!speed_queue_send(&q, "x", SPEED_UART_FRAME_BYTES));
}

static void test_telemetry(void)
{
    speed_bench_t b;
    int32_t raw[4] = {INT32_MIN, INT32_MIN, INT32_MAX, INT32_MAX};
    speed_bench_init(&b, true, raw, 0);
    speed_bench_diagnostics_t d = {.ms = UINT32_MAX, .ticks = UINT32_MAX,
        .dt_min = 9, .dt_max = 11, .irq_us_max = 100, .gpio_low = true};
    char frame[SPEED_UART_FRAME_BYTES];
    assert(speed_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
    assert(strstr(frame, "fw=speed_ro_v1") != NULL);
    assert(strstr(frame, "gpio_low=1") != NULL);
    assert(strlen(frame) < SPEED_UART_FRAME_BYTES);
    assert(!speed_bench_format(&b, &d, "TELSP", frame, 50));
    speed_bench_init(&b, false, raw, 0);
    assert(speed_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
    assert(strstr(frame, "fw=speed_pi_v2") != NULL && strstr(frame, ",stable_ms=0,used=0") != NULL);
    printf("speed telemetry: worst-count frame fits %u-byte queue slot\n", SPEED_UART_FRAME_BYTES);
}

static void test_settling_one_shot_and_direction(void)
{
    int32_t raw[4] = {0};
    speed_bench_t b;
    speed_bench_init(&b, false, raw, 0);
    for (uint32_t t = 10; t <= 490; t += 10) (void)speed_bench_tick(&b, raw, t);
    (void)speed_bench_accept(&b, "hb", 490);
    assert(strcmp(speed_bench_accept(&b, "run FL 300 S5", 490), "ERR,not_settled\r\n") == 0);
    raw[0] = 1;
    (void)speed_bench_tick(&b, raw, 500);
    assert(b.stationary_ms == 0);
    for (uint32_t t = 510; t <= 1000; t += 10) (void)speed_bench_tick(&b, raw, t);
    (void)speed_bench_accept(&b, "hb", 1000);
    assert(strcmp(speed_bench_accept(&b, "run FL 300 S5", 1031), "ERR,not_settled\r\n") == 0);
    assert(strcmp(speed_bench_accept(&b, "run FL 300 S5", 1000), "OK,running_speed\r\n") == 0);
    (void)speed_bench_accept(&b, "stop", 1001);
    assert(b.stationary_ms == 0 && !b.pi.enabled && b.run_used);
    for (uint32_t t = 1010; t <= 1510; t += 10) (void)speed_bench_tick(&b, raw, t);
    (void)speed_bench_accept(&b, "hb", 1510);
    assert(strcmp(speed_bench_accept(&b, "run FL -300 S5", 1510), "ERR,one_shot_used\r\n") == 0);
    speed_bench_init(&b, false, raw, UINT32_MAX - 299U);
    for (unsigned i = 1; i <= 50; ++i)
        (void)speed_bench_tick(&b, raw, UINT32_MAX - 299U + i * 10U);
    assert(b.stationary_ms == 500);
    (void)speed_bench_accept(&b, "hb", 200);
    assert(strcmp(speed_bench_accept(&b, "run FL 300 S5", 200), "OK,running_speed\r\n") == 0);
    --raw[0];
    (void)speed_bench_tick(&b, raw, 210);
    assert(!b.pi.enabled && b.pi.fault == FOUR_SPEED_FAULT_FEEDBACK);
}

int main(void)
{
    test_readonly_and_cpr(); test_protocol(); test_timeout_and_units();
    test_rx(); test_tx(); test_telemetry(); test_settling_one_shot_and_direction();
    puts("speed bench: 7 test groups passed (offline only)");
    return 0;
}
