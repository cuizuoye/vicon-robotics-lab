#ifndef FOUR_LINK_BENCH_H
#define FOUR_LINK_BENCH_H
#include "mc_protocol.h"
#include "drive_bench_control.h"

/* Default: fixed F50 one-shot bench. FOUR_LINK_MOTION selects a separate,
 * excluded-from-default-build 120s streaming image; never silently upgrade. */
enum { FOUR_LINK_EXPIRED=1U<<16, FOUR_LINK_REJECTED=1U<<17 };
enum { FOUR_LINK_SESSION=1, FOUR_LINK_ENABLED=2, FOUR_LINK_CALIBRATED=4,
       FOUR_LINK_ZERO=8, FOUR_LINK_TIMED_OUT=16, FOUR_LINK_FOUR=64,
       FOUR_LINK_BENCH=128, FOUR_LINK_PERMIT=256, FOUR_LINK_USED=512 };
#ifdef FOUR_LINK_GROUNDB
#define FOUR_LINK_BENCH_TAG UINT32_C(0x39535734) /* 4WS9 XY+yaw staged commissioning v5. */
#define FOUR_LINK_MOTION_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_GROUNDA)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x41475734) /* 4WGA: 0.10 m/s, 600 ms recovery demo */
#define FOUR_LINK_MOTION_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_GROUND9)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x39475734) /* 4WG9: safe explicit multi-run rearm */
#define FOUR_LINK_MOTION_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_GROUND8)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x38475734) /* 4WG8: 0.20 m/s large-course candidate */
#define FOUR_LINK_MOTION_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_GROUND7)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x37475734) /* 4WG7: per-wheel feedforward + PI */
#define FOUR_LINK_MOTION_LIMIT_MS FOUR_DRIVE_RUN_LIMIT_MS
#elif defined(FOUR_LINK_GROUND6)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x36475734) /* 4WG6: quiet cancellation wheels */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_GROUND5)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x35475734) /* 4WG5: streamed-ramp startup hold */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_GROUND4)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x34475734) /* 4WG4: recoverable idle lease */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_GROUND3)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x33475734) /* 4WG3: bumpless ground handoff */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_GROUND2)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x32475734) /* 4WG2: 30% start, 35% cap */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_GROUND)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x31475734) /* 4WG1: bounded ground commissioning */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#elif defined(FOUR_LINK_MOTION)
#define FOUR_LINK_BENCH_TAG UINT32_C(0x314d5734) /* 4WM1: not the fixed-forward bench */
#define FOUR_LINK_MOTION_LIMIT_MS 120000U
#else
#define FOUR_LINK_BENCH_TAG UINT32_C(0x31425734) /* 4WB1 */
#endif
typedef struct {
    mc_frame_header_v1_t header;
    union { mc_cmd_vel_v1_t velocity; mc_control_enable_v1_t control; } body;
} four_link_request_t;
typedef struct {
    uint32_t session_id, last_sequence, last_command_sequence, last_command_ms;
    uint32_t fault;
    uint32_t source_anchor_ms, local_anchor_ms, last_source_elapsed;
    bool tier_selected, bench_completed;
    uint8_t action_mode;
    uint32_t action_session_id;
    bool session_active, zero_seen, command_seen, expired, permitted;
#ifdef FOUR_LINK_MOTION
    uint32_t first_motion_ms;
    bool motion_started, transitioning;
#endif
} four_link_bench_t;
typedef struct MC_PACKED {
    uint32_t ms, dt_ms;
    int32_t raw_delta, measured_milli_cps, requested_milli_cps, effective_milli_cps;
    uint16_t reason, wheel;
    int16_t duty_permille;
    uint16_t opposite_counts;
} four_feedback_wire_t;
_Static_assert(sizeof(four_feedback_wire_t)==32,"candidate first feedback snapshot");
typedef struct MC_PACKED {
    uint32_t tag, ticks, updates, dt_min, dt_max, irq_us_max, started_ms, stationary_ms;
    uint32_t compares[8];
    int16_t raw_duty[4], target_cps[4], filtered_cps[4], body_milli[3];
    uint8_t startup_mask, fault_wheel;
    uint16_t watchdog_reset, permitted;
    four_feedback_wire_t first_feedback_fault;
    /* Versioned live output authority; latest request alone cannot validate
     * an old-direction deceleration during a legitimate reversal. */
    float effective_cps[4];
    int8_t granted_direction[4];
    uint8_t reversing_mask, output_authority_version;
    uint16_t output_authority_reserved;
    float shaped_body_si[3], vector_accel_limit, output_slew_limit;
    uint16_t stage_duty_limit;
    uint8_t stage_id, tier_selected, saturated_mask, hardware_released;
    uint8_t rejoining_mask, servo_reserved;
    float yaw_rate_limit, yaw_accel_limit, wheel_rpm_limit;
    uint32_t coupled_shape_revision;
} four_link_bench_status_t;
_Static_assert(sizeof(float)==4,"4WGC wire requires IEEE single precision");
_Static_assert(sizeof(four_link_bench_status_t)==200,"4WS9 v5 200-byte XY+yaw diagnostic");
enum { SERVO_LINK_SOURCE_AGE=1U<<18, SERVO_LINK_OFFLINE_LOCK=1U<<19 };

void four_link_bench_init(four_link_bench_t *link);
/* CRC/格式解析在主循环完成；不在执行器临界区做逐字节CRC。 */
bool four_link_bench_decode(const uint8_t *frame, size_t length, four_link_request_t *request);
/* 以下状态机只由100Hz采样ISR调用，返回true仅表示需要HELLO_ACK。 */
bool four_link_bench_process(four_link_bench_t *link, drive_bench_t *bench,
                             const four_link_request_t *request, uint32_t now);
void four_link_bench_deadline(four_link_bench_t *link, drive_bench_t *bench, uint32_t now);
bool four_link_bench_payloads(const four_link_bench_t *link, const drive_bench_t *bench,
    const speed_bench_diagnostics_t *diagnostics, mc_wheel_telemetry_v1_t *telemetry,
    four_link_bench_status_t *status);
#endif
