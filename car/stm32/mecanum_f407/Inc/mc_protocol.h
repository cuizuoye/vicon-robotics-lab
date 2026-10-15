#ifndef MC_PROTOCOL_H
#define MC_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mecanum_protocol_v1.h"

#define MC_FRAME_OVERHEAD (sizeof(mc_frame_header_v1_t) + sizeof(uint32_t))
#define MC_MAX_RAW_FRAME_SIZE (MC_FRAME_OVERHEAD + MC_MAX_PAYLOAD_SIZE)
#define MC_MAX_COBS_FRAME_SIZE (MC_MAX_RAW_FRAME_SIZE + (MC_MAX_RAW_FRAME_SIZE / 254u) + 2u)

uint32_t mc_crc32(const uint8_t *data, size_t length);

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
    size_t *output_length);

bool mc_validate_frame(
    const uint8_t *frame,
    size_t frame_length,
    mc_frame_header_v1_t *header_out);

bool mc_sequence_is_newer(uint32_t sequence, uint32_t previous);

bool mc_cobs_encode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

bool mc_cobs_decode(
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_capacity,
    size_t *output_length);

bool mc_protocol_self_test(void);

#endif

