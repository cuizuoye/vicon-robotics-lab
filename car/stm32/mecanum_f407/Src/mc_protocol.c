#include "mc_protocol.h"

#include <string.h>


uint32_t mc_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t index = 0; index < length; ++index) {
        crc ^= data[index];
        for (unsigned bit = 0; bit < 8u; ++bit) {
            const uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1u) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}


bool mc_build_frame(
    uint8_t message_type,
    uint16_t flags,
    uint32_t sequence,
    uint32_t session_id,
    uint32_t sender_time_ms,
    const void *payload,
    uint16_t payload_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    const size_t frame_length = sizeof(mc_frame_header_v1_t) + payload_length + sizeof(uint32_t);
    if (output == NULL || output_length == NULL || payload_length > MC_MAX_PAYLOAD_SIZE
        || output_capacity < frame_length || (payload_length > 0u && payload == NULL)) {
        return false;
    }

    const mc_frame_header_v1_t header = {
        .magic = {MC_MAGIC_0, MC_MAGIC_1},
        .version = MC_PROTOCOL_VERSION,
        .message_type = message_type,
        .flags = flags,
        .sequence = sequence,
        .session_id = session_id,
        .sender_time_ms = sender_time_ms,
        .payload_length = payload_length,
    };
    memcpy(output, &header, sizeof(header));
    if (payload_length > 0u) {
        memcpy(output + sizeof(header), payload, payload_length);
    }
    const uint32_t crc = mc_crc32(output, sizeof(header) + payload_length);
    memcpy(output + sizeof(header) + payload_length, &crc, sizeof(crc));
    *output_length = frame_length;
    return true;
}


bool mc_validate_frame(
    const uint8_t *frame,
    size_t frame_length,
    mc_frame_header_v1_t *header_out)
{
    if (frame == NULL || frame_length < MC_FRAME_OVERHEAD) {
        return false;
    }
    mc_frame_header_v1_t header;
    memcpy(&header, frame, sizeof(header));
    if (header.magic[0] != MC_MAGIC_0 || header.magic[1] != MC_MAGIC_1
        || header.version != MC_PROTOCOL_VERSION
        || header.payload_length > MC_MAX_PAYLOAD_SIZE) {
        return false;
    }
    const size_t expected_length = sizeof(header) + header.payload_length + sizeof(uint32_t);
    if (frame_length != expected_length) {
        return false;
    }
    uint32_t received_crc;
    memcpy(&received_crc, frame + expected_length - sizeof(received_crc), sizeof(received_crc));
    if (received_crc != mc_crc32(frame, expected_length - sizeof(received_crc))) {
        return false;
    }
    if (header_out != NULL) {
        *header_out = header;
    }
    return true;
}


bool mc_sequence_is_newer(uint32_t sequence, uint32_t previous)
{
    const uint32_t difference = sequence - previous;
    return difference != 0u && difference < 0x80000000u;
}


bool mc_cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    if (input == NULL || output == NULL || output_length == NULL || output_capacity == 0u) {
        return false;
    }
    size_t read_index = 0u;
    size_t write_index = 1u;
    size_t code_index = 0u;
    uint8_t code = 1u;
    while (read_index < input_length) {
        if (input[read_index] == 0u) {
            if (code_index >= output_capacity) {
                return false;
            }
            output[code_index] = code;
            code_index = write_index;
            if (write_index >= output_capacity) {
                return false;
            }
            ++write_index;
            code = 1u;
            ++read_index;
        } else {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index++] = input[read_index++];
            ++code;
            if (code == 0xFFu) {
                output[code_index] = code;
                code_index = write_index;
                if (write_index >= output_capacity) {
                    return false;
                }
                ++write_index;
                code = 1u;
            }
        }
    }
    output[code_index] = code;
    *output_length = write_index;
    return true;
}


bool mc_cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length)
{
    if (input == NULL || input_length == 0u || output == NULL || output_length == NULL) {
        return false;
    }
    size_t read_index = 0u;
    size_t write_index = 0u;
    while (read_index < input_length) {
        const uint8_t code = input[read_index++];
        if (code == 0u || read_index + (size_t)(code - 1u) > input_length) {
            return false;
        }
        for (uint8_t offset = 1u; offset < code; ++offset) {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index++] = input[read_index++];
        }
        if (code < 0xFFu && read_index < input_length) {
            if (write_index >= output_capacity) {
                return false;
            }
            output[write_index++] = 0u;
        }
    }
    *output_length = write_index;
    return true;
}


bool mc_protocol_self_test(void)
{
    static const uint8_t expected_raw[] = {
        0x4d, 0x43, 0x01, 0x10, 0x09, 0x00, 0x2a, 0x00,
        0x00, 0x00, 0x78, 0x56, 0x34, 0x12, 0x06, 0x12,
        0x0f, 0x00, 0x0a, 0x00, 0xfa, 0x00, 0x83, 0xff,
        0x20, 0x03, 0x96, 0x00, 0x00, 0x00, 0x83, 0x5a,
        0x4e, 0x9f,
    };
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
    size_t raw_length = 0u;
    if (!mc_build_frame(MC_MSG_CMD_VEL, MC_FLAG_ACK_REQUIRED | MC_FLAG_ENABLED,
                        42u, 0x12345678u, 987654u, &command, sizeof(command),
                        raw, sizeof(raw), &raw_length)
        || raw_length != sizeof(expected_raw)
        || memcmp(raw, expected_raw, sizeof(expected_raw)) != 0
        || !mc_validate_frame(raw, raw_length, NULL)) {
        return false;
    }
    uint8_t cobs[MC_MAX_COBS_FRAME_SIZE];
    size_t cobs_length = 0u;
    if (!mc_cobs_encode(raw, raw_length, cobs, sizeof(cobs), &cobs_length)
        || cobs_length != sizeof(expected_cobs)
        || memcmp(cobs, expected_cobs, sizeof(expected_cobs)) != 0) {
        return false;
    }
    uint8_t decoded[MC_MAX_RAW_FRAME_SIZE];
    size_t decoded_length = 0u;
    return mc_cobs_decode(cobs, cobs_length, decoded, sizeof(decoded), &decoded_length)
        && decoded_length == raw_length
        && memcmp(decoded, raw, raw_length) == 0;
}

