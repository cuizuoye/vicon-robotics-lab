#ifndef RR_DIAGNOSTIC_TRACE_H
#define RR_DIAGNOSTIC_TRACE_H
#include "drive_bench_control.h"

#define RR_TRACE_CAPACITY 1001U /* 10秒100Hz + 一帧停机；满时停机，不覆盖。 */
typedef struct {
    uint32_t ms, fault;
    int32_t enc, cps, fcps, target, integral_milli;
    int16_t duty;
    uint16_t p1, p2;
    uint8_t enabled, assist, other_outputs;
} rr_trace_row_t;
typedef struct {
    rr_trace_row_t rows[RR_TRACE_CAPACITY];
    uint16_t count;
    bool capturing, complete, overflow;
} rr_trace_t;

/* 仅ISR写入；完成后只读，dump期间不复制大数组/不重新使能。 */
bool rr_trace_capture(rr_trace_t *trace, const drive_bench_t *bench,
                      const uint32_t actual_compares[8], uint32_t now);
bool rr_trace_format(const rr_trace_t *trace, unsigned index, char *out, size_t capacity);
#endif
