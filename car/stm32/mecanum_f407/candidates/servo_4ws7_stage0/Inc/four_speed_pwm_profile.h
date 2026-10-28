#ifndef SERVO_4WS7_PROFILE_H
#define SERVO_4WS7_PROFILE_H
#ifndef SERVO_STAGE0_COMMISSIONING
#error Separate 4WS7 stage-0 commissioning target only.
#endif
#include "../../../Inc/four_speed_pwm_profile.h"
/* Absolute ceilings, NOT default running permissions or achieved performance. */
#undef FOUR_SPEED_DUTY_LIMIT_PERMILLE
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#undef FOUR_DRIVE_TARGET_LIMIT_CPS
#define FOUR_DRIVE_TARGET_LIMIT_CPS 1700.0f
#undef FOUR_DRIVE_MAX_LINEAR_M_S
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.200f
#undef FOUR_DRIVE_MAX_WHEEL_M_S
#define FOUR_DRIVE_MAX_WHEEL_M_S 0.300f
#undef FOUR_DRIVE_MAX_BODY_MM_S
#define FOUR_DRIVE_MAX_BODY_MM_S 200
#undef FOUR_DRIVE_MAX_YAW_RAD_S
#define FOUR_DRIVE_MAX_YAW_RAD_S 0.100f
#undef FOUR_DRIVE_MAX_YAW_MRAD_S
#define FOUR_DRIVE_MAX_YAW_MRAD_S 100

#define SERVO_TARGET_RPM 100.0f
#define SERVO_FEEDBACK_RPM 120.0f
/* Bound derived from 2m/s^2 vector acceleration plus .4rad/s^2 yaw,
 * 60mm wheels and maximum accepted CPR. Not an independent normal ramp. */
#define SERVO_WHEEL_SLEW_GUARD_CPS_S 967.0f
#define SERVO_FEEDBACK_FILTER_MS 20.0f
#define SERVO_SOURCE_LAG_LIMIT_MS 120U
#define SERVO_SOURCE_LEAD_LIMIT_MS 60U
/* Capable of stage 0 ONLY, not authority to run. This target is built but
 * NOT flashed by preparation tools. No native bypass/CLI release flag. */
#ifdef SERVO_EXECUTION_RELEASED
#error Do not set a release flag; use the bounded stage-0 target.
#endif
#define SERVO_EXECUTION_RELEASED 1
#ifdef SERVO_NATIVE_TEST
#error Native bypass forbidden in the stage-0 commissioning target.
#endif
#define SERVO_BENCH_LIMIT_MS 5000U
#define SERVO_BENCH_MAGIC 0x7080U
_Static_assert(FOUR_DRIVE_COMMAND_TIMEOUT_MS==600U,"lease unchanged");
_Static_assert(FOUR_DRIVE_RUN_LIMIT_MS==150000U,"absolute budget unchanged");
_Static_assert(FOUR_DRIVE_STARTUP_DUTY_PERMILLE==300,"breakaway not raised");
_Static_assert(FOUR_SPEED_PWM_COUNTS==100U,"10kHz PWM retained");
#endif
