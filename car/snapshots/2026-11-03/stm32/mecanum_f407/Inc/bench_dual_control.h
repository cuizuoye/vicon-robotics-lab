#ifndef BENCH_DUAL_CONTROL_H
#define BENCH_DUAL_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 独立台架协议：不是MC-Link，且不接受旧单电机arm/duty命令。 */
#define BENCH_DUAL_HEARTBEAT_MS 500U
#define BENCH_DUAL_PULSE_LIMIT_MS 2200U
#define BENCH_DUAL_OBSERVE_LIMIT_MS 5200U
#if defined(BENCH_FRONT_DEMO30)
#define BENCH_FRONT_DEMO_LIMIT_MS 30200U
#endif
#define BENCH_DUAL_FAULT_TIMEOUT (1U << 0)
#define BENCH_DUAL_FAULT_UART (1U << 1)
#define BENCH_DUAL_FAULT_ESTOP (1U << 2)
#define BENCH_DUAL_FAULT_PROTOCOL (1U << 7)
#define BENCH_DUAL_FAULT_PULSE_LIMIT (1U << 8)

typedef struct {
    bool armed;
    bool heartbeat_seen;
    bool pulse_used;
    bool forward_observe_only;
    uint8_t mask;
    int16_t duty_a;
    int16_t duty_b;
    uint32_t last_heartbeat_ms;
    uint32_t pulse_started_ms;
    uint32_t pulse_limit_ms;
    uint32_t fault_bits;
#if defined(BENCH_FRONT_DEMO30)
    /* 演示每次上电只运行一次；stop不会恢复运行资格。 */
    bool demo_used;
#endif
} bench_dual_control_t;

void bench_dual_init(bench_dual_control_t *control);
void bench_dual_stop(bench_dual_control_t *control, uint32_t fault);
void bench_dual_tick(bench_dual_control_t *control, uint32_t now_ms);
const char *bench_dual_accept(bench_dual_control_t *control,
                            const char *line, uint32_t now_ms);
bool bench_dual_pwm_compare(int16_t duty, uint32_t *compare1, uint32_t *compare2);

#endif
