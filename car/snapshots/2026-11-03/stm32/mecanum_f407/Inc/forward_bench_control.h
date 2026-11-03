#ifndef FORWARD_BENCH_CONTROL_H
#define FORWARD_BENCH_CONTROL_H

#include "four_forward_group.h"
#include "speed_bench_control.h" /* 仅复用诊断字段/故障位，不链接单轮状态机。 */

#define FORWARD_BENCH_RUN "run F50 S5"

typedef struct {
    four_forward_group_t group;
    four_speed_config_t adopted;
    uint32_t sampled_ms, updates;
    int32_t counts[4], raw_cps[4], rpm_milli[4], speed_mm_s[4];
} forward_bench_t;

void forward_bench_init(forward_bench_t *bench, const int32_t counts[4], uint32_t now_ms);
const char *forward_bench_accept(forward_bench_t *bench, const char *line, uint32_t now_ms);
bool forward_bench_tick(forward_bench_t *bench, const int32_t counts[4], uint32_t now_ms);
bool forward_bench_format(const forward_bench_t *bench,
                          const speed_bench_diagnostics_t *diagnostics,
                          const char *prefix, char *out, size_t capacity);

#endif
