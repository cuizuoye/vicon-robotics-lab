#ifndef MC_LINK_TRACE_H
#define MC_LINK_TRACE_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define MC_TRACE_CAPACITY 128u
enum { MC_TRACE_RX=1, MC_TRACE_UART=2, MC_TRACE_TELEMETRY=3, MC_TRACE_LOOP=4 };
typedef struct {
    uint32_t kind, sequence, at_ms, duration_ms, detail;
} mc_trace_event_t;
/* Single writer: UDP task only. Never accessed from the UART RX task. */
typedef struct {
    mc_trace_event_t events[MC_TRACE_CAPACITY];
    uint32_t next, count;
    bool dumped;
} mc_link_trace_t;
static inline void mc_trace_push(mc_link_trace_t *s, uint32_t kind,
                                uint32_t sequence, uint32_t at_ms,
                                uint32_t duration_ms, uint32_t detail)
{
    s->events[s->next] = (mc_trace_event_t){kind,sequence,at_ms,duration_ms,detail};
    s->next = (s->next + 1u) % MC_TRACE_CAPACITY;
    if (s->count < MC_TRACE_CAPACITY) ++s->count;
}
static inline const mc_trace_event_t *mc_trace_get(const mc_link_trace_t *s, size_t i)
{
    if (i >= s->count) return NULL;
    return &s->events[(s->next + MC_TRACE_CAPACITY - s->count + i) % MC_TRACE_CAPACITY];
}
#endif
