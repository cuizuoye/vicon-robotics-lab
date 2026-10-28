#ifndef SERVO_4WS7_PROFILE_H
#define SERVO_4WS7_PROFILE_H
#ifndef FOUR_SERVO_CANDIDATE
#error Independent OFFLINE 4WS7 candidate only.
#endif
#include "../../../Inc/four_speed_pwm_profile.h"
/* Absolute ceilings, NOT default running permissions or achieved performance. */
#undef FOUR_SPEED_DUTY_LIMIT_PERMILLE
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 800
#undef FOUR_DRIVE_TARGET_LIMIT_CPS
#define FOUR_DRIVE_TARGET_LIMIT_CPS 7100.0f
#undef FOUR_DRIVE_MAX_LINEAR_M_S
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.700f
#undef FOUR_DRIVE_MAX_WHEEL_M_S
#define FOUR_DRIVE_MAX_WHEEL_M_S 1.256636f
#undef FOUR_DRIVE_MAX_BODY_MM_S
#define FOUR_DRIVE_MAX_BODY_MM_S 700
#undef FOUR_DRIVE_MAX_YAW_RAD_S
#define FOUR_DRIVE_MAX_YAW_RAD_S 0.100f
#undef FOUR_DRIVE_MAX_YAW_MRAD_S
#define FOUR_DRIVE_MAX_YAW_MRAD_S 100

#define SERVO_TARGET_RPM 400.0f
#define SERVO_FEEDBACK_RPM 440.0f
/* Bound derived from 2m/s^2 vector acceleration plus .4rad/s^2 yaw,
 * 60mm wheels and maximum accepted CPR. Not an independent normal ramp. */
#define SERVO_WHEEL_SLEW_GUARD_CPS_S 16300.0f
#define SERVO_FEEDBACK_FILTER_MS 20.0f
#define SERVO_SOURCE_LAG_LIMIT_MS 120U
#define SERVO_SOURCE_LEAD_LIMIT_MS 60U
/* This OFFLINE artifact cannot be enabled on hardware. Promotion requires
 * separate review/build; native synthetic tests exercise the exact algorithms. */
#ifndef SERVO_EXECUTION_RELEASED
#define SERVO_EXECUTION_RELEASED 0
#endif
#if SERVO_EXECUTION_RELEASED != 0
#error Hardware execution is NOT released in this offline candidate.
#endif
#if defined(SERVO_NATIVE_TEST) && defined(__arm__)
#error Native test bypass must never enter an ARM image.
#endif
_Static_assert(FOUR_DRIVE_COMMAND_TIMEOUT_MS==600U,"lease unchanged");
_Static_assert(FOUR_DRIVE_RUN_LIMIT_MS==150000U,"absolute budget unchanged");
_Static_assert(FOUR_DRIVE_STARTUP_DUTY_PERMILLE==300,"breakaway not raised");
_Static_assert(FOUR_SPEED_PWM_COUNTS==100U,"10kHz PWM retained");
#endif
