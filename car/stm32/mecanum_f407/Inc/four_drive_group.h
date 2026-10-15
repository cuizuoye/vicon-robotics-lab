#ifndef FOUR_DRIVE_GROUP_H
#define FOUR_DRIVE_GROUP_H

#include "mecanum_drive_plan.h"

/* 受限悬空组控制：启动时固定目标，单次额度，不是连续遥控/轨迹执行器。 */
typedef struct {
    four_speed_controller_t loop[4];
    mecanum_drive_geometry_t geometry;
    mecanum_drive_plan_t plan;
    int32_t counts[4];
    int16_t raw_duty[4];
    uint32_t fault, sampled_ms, stopped_ms, stationary_ms, heartbeat_ms, started_ms;
    bool heartbeat_seen, enabled, run_used;
#ifdef DRIVE_STARTUP_OBSERVE
    /* 仅独立观察镜像启用；其它目标结构/行为均不变。 */
    uint32_t startup_counts[4];
    uint8_t startup_mask, startup_done_mask, fault_wheel; /* 故障轮1..4；0未知。 */
#endif
} four_drive_group_t;

bool four_drive_init(four_drive_group_t *group, const mecanum_drive_geometry_t *geometry,
                     const four_speed_config_t *config, const int32_t counts[4], uint32_t now);
bool four_drive_heartbeat(four_drive_group_t *group, uint32_t now);
bool four_drive_arm(four_drive_group_t *group, const mecanum_body_velocity_t *requested,
                    uint32_t now);
#ifdef FOUR_LINK_MOTION
/* Separate motion image only: same-direction updates; reversal must settle first. */
bool four_drive_retarget(four_drive_group_t *group, const mecanum_drive_plan_t *plan,
                         uint32_t now);
#endif
#ifdef RR_DIAGNOSTIC
/* 独立镜像唯一固定单轮动作；不经过整车运动学，不接受任意速度。 */
bool four_drive_arm_rr_diagnostic(four_drive_group_t *group, uint32_t now);
#endif
bool four_drive_tick(four_drive_group_t *group, const int32_t counts[4], uint32_t now);
void four_drive_stop(four_drive_group_t *group, uint32_t fault, uint32_t now);
/* 批量校验方向/零目标/使能/CCR映射；失败输出全零，硬件层不可部分提交。 */
bool four_drive_compares(const four_drive_group_t *group, uint32_t compares[8]);

#endif
