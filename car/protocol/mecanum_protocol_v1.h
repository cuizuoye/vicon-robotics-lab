#ifndef MECANUM_PROTOCOL_V1_H
#define MECANUM_PROTOCOL_V1_H

#include <stdint.h>

#define MC_PROTOCOL_VERSION 1u
#define MC_MAGIC_0 ((uint8_t)'M')
#define MC_MAGIC_1 ((uint8_t)'C')
#define MC_MAX_PAYLOAD_SIZE 256u
#define MC_UDP_COMMAND_PORT 46000u
#define MC_UDP_TELEMETRY_PORT 46001u
#define MC_UART_BAUD_RATE 921600u

typedef enum {
    MC_MSG_HELLO = 0x01,
    MC_MSG_HELLO_ACK = 0x02,
    MC_MSG_HEARTBEAT = 0x03,
    MC_MSG_CMD_VEL = 0x10,
    MC_MSG_CONTROL_ENABLE = 0x11,
    MC_MSG_ESTOP = 0x12,
    MC_MSG_ACK = 0x20,
    MC_MSG_WHEEL_TELEMETRY = 0x30,
    MC_MSG_LINK_STATUS = 0x31,
} mc_message_type_t;

typedef enum {
    MC_FLAG_ACK_REQUIRED = 1u << 0,
    MC_FLAG_RESPONSE = 1u << 1,
    MC_FLAG_FAULT = 1u << 2,
    MC_FLAG_ENABLED = 1u << 3,
} mc_frame_flags_t;

#if defined(__GNUC__)
#define MC_PACKED __attribute__((packed))
#else
#define MC_PACKED
#endif

/* All multi-byte wire fields are little-endian. CRC32 follows the payload. */
typedef struct MC_PACKED {
    uint8_t magic[2];
    uint8_t version;
    uint8_t message_type;
    uint16_t flags;
    uint32_t sequence;
    uint32_t session_id;
    uint32_t sender_time_ms;
    uint16_t payload_length;
} mc_frame_header_v1_t;

/* Linear velocity is mm/s; angular velocity is mrad/s. */
typedef struct MC_PACKED {
    int16_t vx_mm_s;
    int16_t vy_mm_s;
    int16_t wz_mrad_s;
    uint16_t timeout_ms;
    uint8_t control_mode;
    uint8_t reserved;
} mc_cmd_vel_v1_t;

typedef struct MC_PACKED {
    uint8_t enabled;
    uint8_t clear_latched_faults;
    uint16_t reserved;
} mc_control_enable_v1_t;

typedef struct MC_PACKED {
    uint32_t sample_time_ms;
    int16_t wheel_rpm_x10[4];
    int32_t encoder_counts[4];
    uint16_t battery_mv;
    uint16_t state_flags;
    uint32_t fault_bits;
    uint32_t last_command_sequence;
} mc_wheel_telemetry_v1_t;

_Static_assert(sizeof(mc_frame_header_v1_t) == 20u, "MC header size mismatch");
_Static_assert(sizeof(mc_cmd_vel_v1_t) == 10u, "MC CMD_VEL size mismatch");
_Static_assert(sizeof(mc_control_enable_v1_t) == 4u, "MC CONTROL_ENABLE size mismatch");
_Static_assert(sizeof(mc_wheel_telemetry_v1_t) == 40u, "MC telemetry size mismatch");

#endif

