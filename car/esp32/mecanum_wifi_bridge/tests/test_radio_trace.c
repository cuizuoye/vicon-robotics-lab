#include <assert.h>
#include <stdio.h>
#include "mc_radio_trace.h"
int main(void)
{
    mc_radio_trace_t s = {0};
    assert(mc_radio_get(&s, 0) == NULL);
    for (uint32_t i = 0; i < MC_RADIO_TRACE_CAPACITY + 20; ++i)
        mc_radio_push(&s, i, i * 1000, i, i & 1, 120, 6);
    assert(s.count == MC_RADIO_TRACE_CAPACITY);
    for (uint32_t i = 0; i < MC_RADIO_TRACE_CAPACITY; ++i) {
        const mc_radio_event_t *e = mc_radio_get(&s, i);
        assert(e->callback_ms == i + 20 && e->sequence == i + 20);
        assert(e->hardware_us == (i + 20) * 1000 && e->frame_length == 120);
        assert(e->tid == 6);
    }
    assert(mc_radio_get(&s, s.count) == NULL);
    s.frozen = true;
    uint32_t next = s.next;
    mc_radio_push(&s, 999, 999, 999, true, 999, 0);
    assert(s.next == next && mc_radio_get(&s, s.count-1)->sequence == 147);
    s = (mc_radio_trace_t){0};
    mc_radio_push(&s, UINT32_MAX, UINT32_MAX, 4095, false, 100, 255);
    mc_radio_push(&s, 2, 999, 0, true, 100, 6);
    assert(mc_radio_get(&s, 0)->hardware_us == UINT32_MAX);
    puts("PASS: radio ring wrap, freeze, reset, bounds, timestamp rollover storage");
}
