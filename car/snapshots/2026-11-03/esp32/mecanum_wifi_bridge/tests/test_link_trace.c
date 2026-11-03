#include <assert.h>
#include <stdio.h>
#include "mc_link_trace.h"
int main(void)
{
    mc_link_trace_t s={0};
    assert(mc_trace_get(&s,0)==NULL);
    for (uint32_t i=0;i<MC_TRACE_CAPACITY+36;++i) mc_trace_push(&s,MC_TRACE_RX,i,100+i,0,20);
    assert(s.count==MC_TRACE_CAPACITY && mc_trace_get(&s,MC_TRACE_CAPACITY)==NULL);
    for (uint32_t i=0;i<MC_TRACE_CAPACITY;++i) {
        const mc_trace_event_t *e=mc_trace_get(&s,i);
        assert(e->sequence==i+36 && e->at_ms==i+136 && e->kind==MC_TRACE_RX);
    }
    s=(mc_link_trace_t){0};
    mc_trace_push(&s,MC_TRACE_UART,7,UINT32_MAX,2,1);
    assert(mc_trace_get(&s,0)->at_ms==UINT32_MAX);
    puts("PASS: trace order, bounded wrap, empty/reset, timestamps");
}
