#ifndef HIGH_SPEED_4WGD_PROFILE_H
#define HIGH_SPEED_4WGD_PROFILE_H
#ifndef FOUR_HIGHSPEED_CANDIDATE
#error Independent OFFLINE high-speed candidate only.
#endif
/* Import unchanged timing, startup, PWM frequency and quantisation policies. */
#include "../../../Inc/four_speed_pwm_profile.h"

#define FOUR_HIGH_NOLOAD_RPM 500.0f
#define FOUR_HIGH_TARGET_RPM 400.0f
/* Target cap, not a claim of achieved speed. 10ms encoder quantisation and
 * bounded overshoot have a separate 440rpm raw-feedback trip; 10000cps
 * corruption protection remains in the reused controller. */
#define FOUR_HIGH_FEEDBACK_RPM 440.0f

#undef FOUR_SPEED_DUTY_LIMIT_PERMILLE
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 800
#undef FOUR_DRIVE_TARGET_LIMIT_CPS
#define FOUR_DRIVE_TARGET_LIMIT_CPS 7100.0f
#undef FOUR_DRIVE_MAX_LINEAR_M_S
#define FOUR_DRIVE_MAX_LINEAR_M_S 1.256f
#undef FOUR_DRIVE_MAX_WHEEL_M_S
/* Conservative float inset below pi*0.060m*400rpm/60. */
#define FOUR_DRIVE_MAX_WHEEL_M_S 1.256636f
#undef FOUR_DRIVE_MAX_BODY_MM_S
#define FOUR_DRIVE_MAX_BODY_MM_S 1256

_Static_assert(FOUR_DRIVE_COMMAND_TIMEOUT_MS==600U,"watchdog must not be relaxed");
_Static_assert(FOUR_DRIVE_STARTUP_DUTY_PERMILLE==300,"breakaway power not raised");
_Static_assert(FOUR_SPEED_PWM_COUNTS==100U,"accepted PWM frequency/resolution retained");
#endif
