#ifndef SERVO_POLICY_H
#define SERVO_POLICY_H
#include <stdbool.h>
#include <stdint.h>
enum { SERVO_STAGE_COUNT=7, SERVO_TIER_MAGIC=0x7900 };
typedef struct {
    float speed_m_s, accel_m_s2;
    int16_t duty_permille;
    float output_slew_permille_s;
    float yaw_rate_rad_s, yaw_accel_rad_s2;
} servo_stage_t;
bool servo_stage_get(unsigned index, servo_stage_t *out);
float servo_stage_target_rpm(unsigned index);
float servo_stage_feedback_rpm(unsigned index);
#endif
