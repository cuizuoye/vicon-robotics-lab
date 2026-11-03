#include <assert.h>
#include <stdio.h>
#include "mc_uart_consumer.h"

int main(void)
{
    mc_frame_header_v1_t header = {0};
    const uint32_t boot_session = 0x53544d32u;
    const uint32_t pc_session = 0x12345678u;

    assert(!mc_uart_consumer_accepts(0u, NULL));
    assert(!mc_uart_consumer_accepts(pc_session, NULL));
    header.session_id = boot_session;
    /* Prolonged no-hotspot/no-host interval admits no frames to the queue. */
    for (unsigned i = 0; i < 100000u; ++i)
        assert(!mc_uart_consumer_accepts(0u, &header));
    assert(mc_uart_consumer_accepts(boot_session, &header));
    assert(!mc_uart_consumer_accepts(pc_session, &header));
    header.session_id = pc_session;
    header.message_type = MC_MSG_HELLO_ACK;
    assert(mc_uart_consumer_accepts(pc_session, &header));
    header.message_type = MC_MSG_WHEEL_TELEMETRY;
    assert(mc_uart_consumer_accepts(pc_session, &header));
    header.message_type = MC_MSG_LINK_STATUS;
    assert(mc_uart_consumer_accepts(pc_session, &header));
    /* Disconnect and a new session cannot reuse old queued telemetry. */
    assert(!mc_uart_consumer_accepts(0u, &header));
    assert(!mc_uart_consumer_accepts(pc_session + 1u, &header));
    header.session_id = 0u;
    assert(!mc_uart_consumer_accepts(0u, &header));
    assert(!mc_uart_consumer_accepts(pc_session, &header));
    puts("UART consumer session gate: PASS (100000 idle frames + session transitions)");
    return 0;
}
