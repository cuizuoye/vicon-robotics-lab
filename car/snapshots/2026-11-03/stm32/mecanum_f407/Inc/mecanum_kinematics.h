#ifndef MECANUM_KINEMATICS_H
#define MECANUM_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MECANUM_FRONT_LEFT = 0,
    MECANUM_FRONT_RIGHT = 1,
    MECANUM_REAR_LEFT = 2,
    MECANUM_REAR_RIGHT = 3,
    MECANUM_WHEEL_COUNT = 4,
} mecanum_wheel_index_t;

typedef struct {
    int32_t wheel_mm_s[MECANUM_WHEEL_COUNT];
    uint16_t scale_permille;
} mecanum_wheel_targets_t;

/*
 * X configuration, ROS body axes: +x forward, +y left, +yaw CCW.
 * Positive wheel speed drives that wheel toward chassis forward.
 * All outputs are proportionally scaled when one exceeds max_wheel_mm_s.
 */
bool mecanum_inverse_kinematics(
    int32_t vx_mm_s,
    int32_t vy_mm_s,
    int32_t wz_mrad_s,
    uint32_t half_length_mm,
    uint32_t half_width_mm,
    uint32_t max_wheel_mm_s,
    mecanum_wheel_targets_t *targets);

/* SI接口独立于旧整数接口；轮序FL/FR/RL/RR，角速度正值代表车体前进。 */
typedef struct {
    float wheel_radius_m;
    float half_length_m;
    float half_width_m;
} mecanum_geometry_si_t;

typedef struct {
    float vx_m_s;
    float vy_m_s;
    float wz_rad_s;
} mecanum_body_velocity_t;

typedef struct {
    float rad_s[MECANUM_WHEEL_COUNT];
    float scale;
} mecanum_angular_targets_t;

/* 纯计算，不使能电机；失败时输出归零。两半尺寸均须为正。 */
bool mecanum_inverse_si(const mecanum_geometry_si_t *geometry,
                        const mecanum_body_velocity_t *body,
                        float max_wheel_rad_s, mecanum_angular_targets_t *out);
bool mecanum_forward_si(const mecanum_geometry_si_t *geometry,
                        const float wheel_rad_s[MECANUM_WHEEL_COUNT],
                        mecanum_body_velocity_t *out);

#endif
