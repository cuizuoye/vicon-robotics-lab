#ifndef FOUR_FORWARD_GROUP_H
#define FOUR_FORWARD_GROUP_H

#include "four_wheel_speed.h"

/* 仅供下一阶段悬空联动：固定50mm/s前进，不是通用底盘/轨迹接口。 */
#define FOUR_FORWARD_SPEED_M_S 0.050f

typedef struct {
    four_speed_controller_t loop[FOUR_SPEED_WHEELS];
    int32_t counts[FOUR_SPEED_WHEELS];
    float target_cps[FOUR_SPEED_WHEELS];
    int16_t raw_duty[FOUR_SPEED_WHEELS];
    uint32_t fault;
    uint32_t sampled_ms;
    uint32_t stopped_ms;
    uint32_t stationary_ms;
    uint32_t heartbeat_ms;
    bool heartbeat_seen;
    bool enabled;
    bool run_used;
} four_forward_group_t;

/* HAL-free、未接入任何已烧录镜像。初始化/心跳/采样均不自动使能。 */
bool four_forward_init(four_forward_group_t *group,
                       const four_speed_config_t *config,
                       const int32_t counts[FOUR_SPEED_WHEELS], uint32_t now_ms);
bool four_forward_heartbeat(four_forward_group_t *group, uint32_t now_ms);
bool four_forward_arm(four_forward_group_t *group, uint32_t now_ms);
bool four_forward_tick(four_forward_group_t *group,
                       const int32_t counts[FOUR_SPEED_WHEELS], uint32_t now_ms);
/* 停机清四轮积分和输出，但不清故障、不恢复单次额度。 */
void four_forward_stop(four_forward_group_t *group, uint32_t fault, uint32_t now_ms);

#endif
