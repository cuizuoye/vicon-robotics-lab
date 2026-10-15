#include "mc_protocol.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

int main(void)
{
    static const uint8_t expected_cobs[] = {
        0x06, 0x4d, 0x43, 0x01, 0x10, 0x09, 0x02, 0x2a,
        0x01, 0x01, 0x08, 0x78, 0x56, 0x34, 0x12, 0x06,
        0x12, 0x0f, 0x02, 0x0a, 0x02, 0xfa, 0x06, 0x83,
        0xff, 0x20, 0x03, 0x96, 0x01, 0x01, 0x05, 0x83,
        0x5a, 0x4e, 0x9f,
    };
    const mc_cmd_vel_v1_t command = {
        .vx_mm_s = 250,
        .vy_mm_s = -125,
        .wz_mrad_s = 800,
        .timeout_ms = 150,
        .control_mode = 0,
        .reserved = 0,
    };
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];
    size_t raw_length = 0U;
    assert(mc_build_frame(
        MC_MSG_CMD_VEL, MC_FLAG_ACK_REQUIRED | MC_FLAG_ENABLED,
        42U, 0x12345678U, 987654U, &command, sizeof(command),
        raw, sizeof(raw), &raw_length));
    assert(mc_validate_frame(raw, raw_length, NULL));

    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE];
    size_t encoded_length = 0U;
    assert(mc_cobs_encode(
        raw, raw_length, encoded, sizeof(encoded), &encoded_length));
    assert(encoded_length == sizeof(expected_cobs));
    assert(memcmp(encoded, expected_cobs, sizeof(expected_cobs)) == 0);

    uint8_t decoded[MC_MAX_RAW_FRAME_SIZE];
    size_t decoded_length = 0U;
    assert(mc_cobs_decode(
        encoded, encoded_length, decoded, sizeof(decoded), &decoded_length));
    assert(decoded_length == raw_length);
    assert(memcmp(decoded, raw, raw_length) == 0);
    puts("mc_protocol_test: PASS");
    return 0;
}
