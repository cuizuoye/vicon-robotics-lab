#ifndef SPEED_BENCH_CONTROL_H
#define SPEED_BENCH_CONTROL_H

#include "four_wheel_speed.h"
#include <stddef.h>

#define SPEED_BENCH_FAULT_UART (1U << 8)
#define SPEED_BENCH_FAULT_ESTOP (1U << 9)

typedef struct {
    four_speed_controller_t pi;
    four_speed_config_t adopted;
    bool read_only;
    bool heartbeat_seen;
    bool run_used; /* 首轮试验每次复位仅一次运行，不接受停机后的自动重启。 */
    uint32_t stationary_ms;
    uint32_t stopped_ms;
    uint32_t heartbeat_ms;
    uint32_t sampled_ms;
    uint32_t updates;
    int32_t counts[4];
    int32_t raw_cps[4];
    int32_t rpm_milli[4];
    int32_t speed_mm_s[4];
} speed_bench_t;

typedef struct {
    uint32_t ms, ticks, dt_min, dt_max, irq_us_max;
    uint32_t compares[8];
#ifdef RR_PWM_FINE
    uint32_t pwm_rr_arr, pwm_rr_psc;
#endif
    bool gpio_low;
    bool watchdog_reset;
} speed_bench_diagnostics_t;

void speed_bench_init(speed_bench_t *bench, bool read_only,
                       const int32_t counts[4], uint32_t now_ms);
const char *speed_bench_accept(speed_bench_t *bench, const char *line,
                              uint32_t now_ms);
bool speed_bench_tick(speed_bench_t *bench, const int32_t counts[4], uint32_t now_ms);
bool speed_bench_format(const speed_bench_t *bench, const speed_bench_diagnostics_t *diagnostics,
                       const char *prefix, char *out, size_t capacity);

#endif
