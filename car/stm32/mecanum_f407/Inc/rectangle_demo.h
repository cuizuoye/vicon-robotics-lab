#ifndef RECTANGLE_DEMO_H
#define RECTANGLE_DEMO_H

#include "four_drive_group.h"

enum {
    RECT_HOLD_MS = 1000U, RECT_COUNTDOWN_MS = 5000U,
    RECT_LEG_LIMIT_MS = 30000U, RECT_TOTAL_LIMIT_MS = 120000U,
    RECT_PAUSE_MS = 1000U, RECT_LEGS = 4U
};

typedef enum {
    RECT_BOOT_RELEASE, RECT_READY, RECT_HOLD, RECT_ARMED_RELEASE,
    RECT_COUNTDOWN, RECT_MOVE, RECT_PAUSE, RECT_COMPLETE,
    RECT_STOPPED, RECT_FAULT
} rectangle_state_t;

enum {
    RECT_FAULT_STOP = 1U << 16, RECT_FAULT_LEG_TIMEOUT = 1U << 17,
    RECT_FAULT_TOTAL_TIMEOUT = 1U << 18, RECT_FAULT_ODOMETRY = 1U << 19,
    RECT_FAULT_NOT_SETTLED = 1U << 20, RECT_FAULT_STATE = 1U << 21,
    RECT_FAULT_UART = 1U << 22, RECT_FAULT_WATCHDOG = 1U << 23
};

/* 自主调度使用轮编码器，不接收Vicon位姿，不是Vicon位置闭环。 */
typedef struct {
    four_drive_group_t group;
    four_speed_config_t calibration;
    mecanum_drive_geometry_t geometry;
    rectangle_state_t state;
    uint32_t fault, sampled_ms, phase_ms, sequence_ms, release_ms;
    int32_t counts[4];
    float progress_m, lateral_m, yaw_rad, total_yaw_rad;
    uint8_t completed_legs;
    bool used, release_seen;
} rectangle_demo_t;

bool rectangle_demo_init(rectangle_demo_t *demo, const int32_t counts[4],
                         uint32_t now, bool watchdog_reset);
/* 输入为“按键确实按下”，电平/引脚由HAL层映射；未释放的启动键不能自动启动。 */
bool rectangle_demo_tick(rectangle_demo_t *demo, const int32_t counts[4],
                         bool button_pressed, uint32_t now);
void rectangle_demo_stop(rectangle_demo_t *demo, uint32_t fault, uint32_t now);
bool rectangle_demo_compares(const rectangle_demo_t *demo, uint32_t compares[8]);

#endif
