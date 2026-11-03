/* C真实格式化器→Python协议解析回归，不连接硬件。 */
#include "drive_bench_control.h"
#include "speed_uart_queue.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

int main(void)
{
    const char *commands[] = {"run F50 S5", "run L50 S5", "run Y200 S5", "run D25 S5"};
    for (unsigned mode = 0; mode < 4; ++mode) {
        drive_bench_t b;
        int32_t raw[4] = {0};
        drive_bench_init(&b, raw, 0);
        for (uint32_t t = 10; t <= 500; t += 10) assert(drive_bench_tick(&b, raw, t));
        drive_bench_accept(&b, "hb", 500);
        drive_bench_accept(&b, commands[mode], 500);
        for (uint32_t t = 510; t <= 1000; t += 10) {
            for (unsigned i = 0; i < 4; ++i) {
                const float want = b.group.plan.forward_cps[i];
                raw[i] += ((want > 0) - (want < 0)) * b.adopted.wheel[i].encoder_sign;
            }
            if (t % 50 == 0) drive_bench_accept(&b, "hb", t);
            assert(drive_bench_tick(&b, raw, t));
        }
        speed_bench_diagnostics_t d = {.ms=1000, .ticks=100, .dt_min=10, .dt_max=10};
        assert(four_drive_compares(&b.group, d.compares));
        char frame[SPEED_UART_FRAME_BYTES];
        assert(drive_bench_format(&b, &d, "TELSP", frame, sizeof(frame)));
        fputs(frame, stdout);
        drive_bench_accept(&b, "stop", 1000);
        assert(four_drive_compares(&b.group, d.compares));
        assert(drive_bench_format(&b, &d, "STATUSSP", frame, sizeof(frame)));
        fputs(frame, stdout);
    }
    return 0;
}
