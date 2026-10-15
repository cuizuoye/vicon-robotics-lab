/* 新镜像真实C遥测格式与主机解析联检；不是动力学仿真。 */
#include "drive_bench_control.h"
#include "speed_uart_queue.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    const char *commands[] = {"run F50 S10", "run L50 S10", "run Y200 S10", "run D25 S10"};
    for (unsigned mode = 0; mode < 4; ++mode) {
        drive_bench_t b; int32_t raw[4] = {0};
        drive_bench_init(&b, raw, 0);
        for (uint32_t t = 10; t <= 500; t += 10) assert(drive_bench_tick(&b, raw, t));
        drive_bench_accept(&b, "hb", 500); drive_bench_accept(&b, commands[mode], 500);
        assert(drive_bench_tick(&b, raw, 510));
        speed_bench_diagnostics_t d = {.ms=510, .ticks=51, .dt_min=10, .dt_max=10};
        char frame[SPEED_UART_FRAME_BYTES];
        assert(four_drive_compares(&b.group, d.compares));
        assert(drive_bench_format(&b, &d, "TELSP", frame, sizeof(frame))); fputs(frame, stdout);
        drive_bench_accept(&b, "stop", 510);
        assert(four_drive_compares(&b.group, d.compares));
        assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame))); fputs(frame, stdout);
    }
    /* 后轮无进展时的原始原因/通道仍被保留。 */
    drive_bench_t b; int32_t raw[4] = {0};
    drive_bench_init(&b, raw, 0);
    for (uint32_t t = 10; t <= 500; t += 10) drive_bench_tick(&b, raw, t);
    drive_bench_accept(&b, "hb", 500); drive_bench_accept(&b, commands[1], 500);
    for (uint32_t t = 510; t <= 1300; t += 10) {
        raw[0]--; raw[1]--;
        if (t % 50 == 0) drive_bench_accept(&b, "hb", t);
        drive_bench_tick(&b, raw, t);
    }
    assert(b.group.fault == FOUR_SPEED_FAULT_STALL && b.group.fault_wheel == 3);
    speed_bench_diagnostics_t d = {.ms=1300, .ticks=130, .dt_min=10, .dt_max=10};
    assert(four_drive_compares(&b.group, d.compares));
    char frame[SPEED_UART_FRAME_BYTES];
    assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame))); fputs(frame, stdout);
    return 0;
}
