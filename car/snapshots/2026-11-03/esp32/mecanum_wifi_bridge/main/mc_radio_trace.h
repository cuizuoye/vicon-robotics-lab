#ifndef MC_RADIO_TRACE_H
#define MC_RADIO_TRACE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MC_RADIO_TRACE_CAPACITY 128u
typedef struct {
    uint32_t callback_ms, hardware_us, sequence, retry, frame_length, tid;
} mc_radio_event_t;
/* Callers serialize writes/reset/freeze with their platform lock. After
 * freezing, the UDP task may read the ring without holding the Wi-Fi lock. */
typedef struct {
    mc_radio_event_t events[MC_RADIO_TRACE_CAPACITY];
    uint32_t next, count;
    bool frozen;
} mc_radio_trace_t;
static inline void mc_radio_push(mc_radio_trace_t *s, uint32_t callback_ms,
    uint32_t hardware_us, uint16_t sequence, bool retry, uint32_t length, uint32_t tid)
{
    if (s->frozen) return;
    s->events[s->next] = (mc_radio_event_t){callback_ms, hardware_us, sequence, retry, length, tid};
    s->next = (s->next + 1u) % MC_RADIO_TRACE_CAPACITY;
    if (s->count < MC_RADIO_TRACE_CAPACITY) ++s->count;
}
static inline const mc_radio_event_t *mc_radio_get(const mc_radio_trace_t *s, size_t i)
{
    if (i >= s->count) return NULL;
    return &s->events[(s->next + MC_RADIO_TRACE_CAPACITY - s->count + i) % MC_RADIO_TRACE_CAPACITY];
}
#endif
