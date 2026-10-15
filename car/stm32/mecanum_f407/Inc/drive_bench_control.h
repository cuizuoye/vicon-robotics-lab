#ifndef DRIVE_BENCH_CONTROL_H
#define DRIVE_BENCH_CONTROL_H

#include "four_drive_group.h"
#include "speed_bench_control.h"

enum { DRIVE_MODE_IDLE = 0, DRIVE_MODE_FORWARD, DRIVE_MODE_LEFT,
       DRIVE_MODE_YAW, DRIVE_MODE_DIAGONAL, DRIVE_MODE_RR_DIAGNOSTIC };

typedef struct {
    four_drive_group_t group;
    four_speed_config_t adopted;
    uint32_t sampled_ms, updates;
    int32_t counts[4], raw_cps[4], rpm_milli[4], speed_mm_s[4];
    uint8_t mode;
} drive_bench_t;

void drive_bench_init(drive_bench_t *b, const int32_t counts[4], uint32_t now);
const char *drive_bench_accept(drive_bench_t *b, const char *line, uint32_t now);
bool drive_bench_tick(drive_bench_t *b, const int32_t counts[4], uint32_t now);
bool drive_bench_format(const drive_bench_t *b, const speed_bench_diagnostics_t *d,
                        const char *prefix, char *out, size_t capacity);

#endif
