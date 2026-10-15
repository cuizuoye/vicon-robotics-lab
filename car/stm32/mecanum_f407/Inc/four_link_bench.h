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
#ifdef FOUR_LINK_MOTION
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
    bool session_active, zero_seen, command_seen, expired, permitted;
#ifdef FOUR_LINK_MOTION
    uint32_t first_motion_ms;
    bool motion_started, transitioning;
#endif
} four_link_bench_t;
typedef struct MC_PACKED {
    uint32_t tag, ticks, updates, dt_min, dt_max, irq_us_max, started_ms, stationary_ms;
    uint32_t compares[8];
    int16_t raw_duty[4], target_cps[4], filtered_cps[4], body_milli[3];
    uint8_t startup_mask, fault_wheel;
    uint16_t watchdog_reset, permitted;
} four_link_bench_status_t;
_Static_assert(sizeof(four_link_bench_status_t)==100,"four link bench status size");

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
