#include "mc_uart_diagnostic.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t frame[MC_MAX_RAW_FRAME_SIZE];
static size_t length;
static void build(uint8_t type, uint16_t flags, uint32_t sequence,
                  uint32_t session, const void *payload, uint16_t size)
{
    assert(mc_build_frame(type, flags, sequence, session, 100, payload, size,
                          frame, sizeof(frame), &length));
}
static mc_readonly_status_t safe_status(void)
{
    const mc_readonly_status_t status = {.tag=0x314f5234, .ticks=100, .updates=100,
        .dt_min=10, .dt_max=10, .gpio_low=1};
    return status;
}
int main(void)
{
    assert(mc_protocol_self_test());
    mc_wheel_telemetry_v1_t wheel = {.sample_time_ms=1000, .state_flags=109};
    mc_readonly_status_t status = safe_status();
    build(MC_MSG_WHEEL_TELEMETRY, 0, 1, 9, &wheel, sizeof(wheel));
    assert(!mc_uart_ack_matches(frame, length, 9, 1)); /* telemetry before ACK */
    build(MC_MSG_HELLO_ACK, MC_FLAG_RESPONSE, 1, 9, NULL, 0);
    assert(mc_uart_ack_matches(frame, length, 9, 1));
    assert(!mc_uart_ack_matches(frame, length, 8, 1));
    assert(!mc_uart_ack_matches(frame, length, 9, 2));
    frame[length-1] ^= 1;
    assert(!mc_uart_ack_matches(frame, length, 9, 1));
    build(MC_MSG_HELLO_ACK, MC_FLAG_RESPONSE|MC_FLAG_ENABLED, 1, 9, NULL, 0);
    assert(!mc_uart_ack_matches(frame, length, 9, 1));
    build(MC_MSG_HELLO_ACK, MC_FLAG_RESPONSE, 1, 9, &wheel, 1);
    assert(!mc_uart_ack_matches(frame, length, 9, 1));

    mc_uart_diagnostic_t state = {0};
    assert(!mc_uart_diagnostic_safe(&state));
    build(MC_MSG_WHEEL_TELEMETRY, 0, 1, 9, &wheel, sizeof(wheel));
    assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
    assert(!mc_uart_diagnostic_safe(&state));
    build(MC_MSG_LINK_STATUS, 0, 2, 9, &status, sizeof(status));
    assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
    assert(mc_uart_diagnostic_safe(&state));
    assert(!mc_uart_diagnostic_accept(&state, frame, length, 9)); /* replay */
    assert(!mc_uart_diagnostic_safe(&state));

    for (unsigned issue=0; issue<10; ++issue) {
        memset(&state, 0, sizeof(state));
        wheel = (mc_wheel_telemetry_v1_t){.sample_time_ms=1000, .state_flags=109};
        status = safe_status();
        if (issue==0) wheel.state_flags |= 2u; /* unexpected wheel state bit */
        if (issue==1) wheel.fault_bits=1;
        if (issue==2) wheel.last_command_sequence=1;
        if (issue==3) wheel.wheel_rpm_x10[3]=1;
        if (issue==4) status.tag=0;
        if (issue==5) status.gpio_low=0;
        if (issue==6) status.watchdog_reset=1;
        if (issue==7) status.compares[7]=1;
        if (issue==8) status.dt_max=11;
        if (issue==9) status.updates=99;
        build(MC_MSG_WHEEL_TELEMETRY, 0, 1, 9, &wheel, sizeof(wheel));
        assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
        build(MC_MSG_LINK_STATUS, 0, 2, 9, &status, sizeof(status));
        assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
        assert(!mc_uart_diagnostic_safe(&state));
    }
    memset(&state, 0, sizeof(state));
    build(MC_MSG_LINK_STATUS, 0, 1, 8, &status, sizeof(status));
    assert(!mc_uart_diagnostic_accept(&state, frame, length, 9));
    build(MC_MSG_LINK_STATUS, 0, 1, 9, &status, 59);
    assert(!mc_uart_diagnostic_accept(&state, frame, length, 9));
    build(MC_MSG_CMD_VEL, 0, 1, 9, NULL, 0);
    assert(!mc_uart_diagnostic_accept(&state, frame, length, 9));
    assert(!mc_uart_diagnostic_accept(NULL, frame, length, 9));
    memset(&state, 0, sizeof(state));
    wheel = (mc_wheel_telemetry_v1_t){.sample_time_ms=1000, .state_flags=109};
    status = safe_status();
    build(MC_MSG_WHEEL_TELEMETRY, 0, UINT32_MAX, 9, &wheel, sizeof(wheel));
    assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
    build(MC_MSG_LINK_STATUS, 0, 0, 9, &status, sizeof(status));
    assert(mc_uart_diagnostic_accept(&state, frame, length, 9));
    assert(mc_uart_diagnostic_safe(&state));
    puts("PASS: ACK matching, telemetry ordering, CRC, session, replay/wrap and readonly safety checks");
    return 0;
}
