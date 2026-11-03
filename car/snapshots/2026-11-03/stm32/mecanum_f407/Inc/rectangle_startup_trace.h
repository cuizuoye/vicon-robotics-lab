#ifndef RECTANGLE_STARTUP_TRACE_H
#define RECTANGLE_STARTUP_TRACE_H

#include "rectangle_demo.h"
#include <stddef.h>

/* 独立诊断镜像：首段最多5秒，不改变原矩形或旧台架的输出/保护。 */
#define RECT_TRACE_RUN_LIMIT_MS 5000U
#define RECT_TRACE_CAPACITY 600U
#define RECT_TRACE_POST_STOP_ROWS 20U
#define RECT_TRACE_MAGIC UINT32_C(0x52535431)
#define RECT_TRACE_FAULT_LIMIT (UINT32_C(1) << 24)
#define RECT_TRACE_FAULT_DATA (UINT32_C(1) << 25)

typedef struct {
    uint32_t ms;
    int32_t counts[4];
    float filtered_cps[4], ramped_cps[4], integral_permille[4];
    int16_t duty_permille[4];
    uint16_t compares[8];
    uint32_t fault;
    uint8_t state, enabled, key, assist_mask, fault_wheel, legs, before_state, reserved;
} rectangle_trace_row_t;

typedef struct {
    uint32_t magic, version, row_bytes, capacity, count, post_stop_rows;
    uint8_t capturing, complete, overflow, reserved;
    float target_cps[4];
    uint32_t started_ms, stopped_ms;
    rectangle_trace_row_t rows[RECT_TRACE_CAPACITY];
} rectangle_startup_trace_t;

_Static_assert(sizeof(rectangle_trace_row_t) == 104U, "trace row ABI changed");
_Static_assert(offsetof(rectangle_startup_trace_t, rows) == 52U, "trace header ABI changed");
_Static_assert(RECT_TRACE_CAPACITY > RECT_TRACE_RUN_LIMIT_MS / 10U +
               RECT_TRACE_POST_STOP_ROWS + 1U, "trace cannot hold bounded trial");

void rectangle_trace_init(rectangle_startup_trace_t *trace);
bool rectangle_startup_diag_tick(rectangle_demo_t *demo, const int32_t counts[4],
                                bool pressed, uint32_t now);
/* 仅ISR写入；停机后再记录约200ms滑行计数，随后冻结，不覆盖旧行。 */
bool rectangle_trace_capture(rectangle_startup_trace_t *trace, const rectangle_demo_t *demo,
                             const int32_t counts[4], const uint32_t actual_compares[8],
                             uint8_t before_state, bool pressed, uint32_t now);
#endif
