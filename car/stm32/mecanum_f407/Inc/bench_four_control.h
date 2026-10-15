#ifndef BENCH_FOUR_CONTROL_H
#define BENCH_FOUR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#define FOUR_WHEELS 4U
#define FOUR_HEARTBEAT_MS 500U
#define FOUR_PULSE_LIMIT_MS 5200U
#define FOUR_FAULT_TIMEOUT (1U << 0)
#define FOUR_FAULT_UART (1U << 1)
#define FOUR_FAULT_ESTOP (1U << 2)
#define FOUR_FAULT_PROTOCOL (1U << 7)
#define FOUR_FAULT_LIMIT (1U << 8)

/* 固定FL/FR/RL/RR，sel=0禁用，1..4单轮；绝不支持多轮同时使能。 */
typedef struct {
    bool heartbeat_seen;
    bool pulse_used;
    uint8_t selected;
    int16_t duty[FOUR_WHEELS];
    uint32_t heartbeat_ms;
    uint32_t pulse_ms;
    uint32_t fault;
} bench_four_t;

void bench_four_init(bench_four_t *c);
void bench_four_stop(bench_four_t *c, uint32_t fault);
void bench_four_tick(bench_four_t *c, uint32_t now);
const char *bench_four_accept(bench_four_t *c, const char *line, uint32_t now);
bool bench_four_compare(int16_t duty, uint32_t *first, uint32_t *second);

#endif
