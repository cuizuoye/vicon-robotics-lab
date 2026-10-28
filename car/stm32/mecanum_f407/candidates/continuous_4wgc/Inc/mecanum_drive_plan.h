#ifndef MECANUM_DRIVE_PLAN_H
#define MECANUM_DRIVE_PLAN_H

#include "four_wheel_speed.h"
#include "mecanum_kinematics.h"

typedef struct {
    mecanum_geometry_si_t kinematics;
    bool dimensions_confirmed;
    bool x_layout_confirmed;
} mecanum_drive_geometry_t;

typedef struct {
    float wheel_m_s[4];
    float forward_cps[4];
    float expected_raw_encoder_cps[4];
    int8_t motor_direction_reference[4];
    mecanum_body_velocity_t applied_body;
    float common_scale;
} mecanum_drive_plan_t;

/*
 * 纯速度目标规划，绝不调用PWM/串口/使能。尺寸或X排列未确认时失败并全归零。
 * 上限由独立固件配置给出；4WG7及之前为50mm/s/300cps，4WG8为
 * 200mm/s/1200cps。不同CPR触发限速时仍全组同比例缩放。
 * forward_cps送PI时不乘motor_sign；PI最终PWM方向才使用该符号。
 */
bool mecanum_drive_plan(const mecanum_drive_geometry_t *geometry,
                        const four_speed_config_t *calibration,
                        const mecanum_body_velocity_t *requested,
                        float max_wheel_m_s, mecanum_drive_plan_t *out);

/* 编码器模型速度，不是Vicon真值/融合里程计；raw_cps按各轮符号归一化一次。 */
bool mecanum_drive_body_from_raw_cps(const mecanum_drive_geometry_t *geometry,
                                   const four_speed_config_t *calibration,
                                   const float raw_cps[4],
                                   mecanum_body_velocity_t *out);

#endif
