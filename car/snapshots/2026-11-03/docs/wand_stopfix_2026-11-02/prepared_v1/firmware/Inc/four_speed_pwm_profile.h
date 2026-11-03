#ifndef SERVO_4WS7_PROFILE_H
#define SERVO_4WS7_PROFILE_H
#ifndef FOUR_SERVO_CANDIDATE
#error Independent OFFLINE 4WSC candidate only.
#endif
#include "../../../../../stm32/mecanum_f407/Inc/four_speed_pwm_profile.h"
/* Absolute ceilings, NOT default running permissions or achieved performance. */
#undef FOUR_SPEED_DUTY_LIMIT_PERMILLE
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 800
#undef FOUR_DRIVE_TARGET_LIMIT_CPS
#define FOUR_DRIVE_TARGET_LIMIT_CPS 7100.0f
#undef FOUR_DRIVE_MAX_LINEAR_M_S
#define FOUR_DRIVE_MAX_LINEAR_M_S 1.200f
#undef FOUR_DRIVE_MAX_WHEEL_M_S
#define FOUR_DRIVE_MAX_WHEEL_M_S 1.256636f
#undef FOUR_DRIVE_MAX_BODY_MM_S
#define FOUR_DRIVE_MAX_BODY_MM_S 1200
#undef FOUR_DRIVE_MAX_YAW_RAD_S
#define FOUR_DRIVE_MAX_YAW_RAD_S 2.000f
#undef FOUR_DRIVE_MAX_YAW_MRAD_S
#define FOUR_DRIVE_MAX_YAW_MRAD_S 2000

#undef FOUR_DRIVE_RUN_LIMIT_MS
#define FOUR_DRIVE_RUN_LIMIT_MS 630000U
#define SERVO_TARGET_RPM 400.0f
#define SERVO_FEEDBACK_RPM 440.0f
/* Absolute bound for new4m/s^2 plus12rad/s^2 yaw; old stages retain26000/33000,
 * 60mm wheels and maximum accepted CPR. Not an independent normal ramp. */
#define SERVO_BASE_WHEEL_SLEW_GUARD_CPS_S 26000.0f
#define SERVO_PREVIOUS_WHEEL_SLEW_GUARD_CPS_S 33000.0f
#define SERVO_WHEEL_SLEW_GUARD_CPS_S 43000.0f
#define SERVO_FEEDBACK_FILTER_MS 20.0f
#define SERVO_SOURCE_LAG_LIMIT_MS 120U
#define SERVO_SOURCE_LEAD_LIMIT_MS 60U
/* Separate staged commissioning image; no test bypass. Capability is not site approval. */
#ifndef SERVO_STAGED_COMMISSIONING
#error Independent staged commissioning target required.
#endif
#ifdef SERVO_NATIVE_TEST
#error Native execution bypass forbidden.
#endif
#define SERVO_EXECUTION_RELEASED 1
#define SERVO_BENCH_LIMIT_MS 5000U
#define SERVO_BENCH_MAGIC 0x7C80U
#include "servo_policy.h"
_Static_assert(FOUR_DRIVE_COMMAND_TIMEOUT_MS==600U,"lease unchanged");
_Static_assert(FOUR_DRIVE_RUN_LIMIT_MS==630000U,"600s host plus 30s independent margin");
_Static_assert(FOUR_DRIVE_STARTUP_DUTY_PERMILLE==300,"breakaway not raised");
_Static_assert(FOUR_SPEED_PWM_COUNTS==100U,"10kHz PWM retained");
#endif
