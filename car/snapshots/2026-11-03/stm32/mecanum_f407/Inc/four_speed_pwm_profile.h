#ifndef FOUR_SPEED_PWM_PROFILE_H
#define FOUR_SPEED_PWM_PROFILE_H

/* Ground commissioning is a separate motion image.  It raises only the
 * bounded PWM envelope; all bench and suspended-motion images stay at 10%. */
#if defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9) || \
    defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 8 requires the bounded streaming motion image.
#endif
/* 0.20 m/s candidate.  It keeps the accepted 4WG7 PWM envelope and
 * controller structure; only the explicit velocity/encoder range and run
 * budget are raised for the 5 m x 4 m course. */
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 0
#define FOUR_DRIVE_STARTUP_COUNTS 1U
#define FOUR_DRIVE_STARTUP_MS 500U
#define FOUR_DRIVE_MIN_TRACKABLE_CPS 20.0f
#define FOUR_DRIVE_START_THRESHOLD_CPS 100.0f
#define FOUR_DRIVE_FEEDFORWARD_CONTROL 1
#elif defined(FOUR_LINK_GROUND7)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 7 requires the bounded streaming motion image.
#endif
/* Feedforward/PID ground candidate.  30% is only a bounded breakaway ceiling;
 * normal tracking uses the per-wheel kS+kV model in four_wheel_speed.c. */
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 0
#define FOUR_DRIVE_STARTUP_COUNTS 1U
#define FOUR_DRIVE_STARTUP_MS 500U
#define FOUR_DRIVE_MIN_TRACKABLE_CPS 20.0f
#define FOUR_DRIVE_START_THRESHOLD_CPS 100.0f
#define FOUR_DRIVE_FEEDFORWARD_CONTROL 1
#elif defined(FOUR_LINK_GROUND6)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 6 requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 300
#define FOUR_DRIVE_STARTUP_COUNTS 16U
#define FOUR_DRIVE_STARTUP_MS 500U
/* Below one count per 50 ms at the measured CPR. Such targets arise from
 * mecanum cancellation and are not a controllable wheel-speed demand. */
#define FOUR_DRIVE_MIN_TRACKABLE_CPS 20.0f
/* Do not consume the one-shot motion allowance on the trajectory's start
 * waypoint or controller noise.  Once moving, ordinary deceleration remains
 * allowed below this threshold. */
#define FOUR_DRIVE_START_THRESHOLD_CPS 100.0f
#elif defined(FOUR_LINK_GROUND5)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 5 requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 300
#define FOUR_DRIVE_STARTUP_COUNTS 16U
#define FOUR_DRIVE_STARTUP_MS 500U
#elif defined(FOUR_LINK_GROUND4)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 4 requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 300
#define FOUR_DRIVE_STARTUP_COUNTS 16U
#define FOUR_DRIVE_STARTUP_MS 500U
#elif defined(FOUR_LINK_GROUND3)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 3 requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 300
#define FOUR_DRIVE_STARTUP_COUNTS 16U
#define FOUR_DRIVE_STARTUP_MS 500U
#elif defined(FOUR_LINK_GROUND2)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile 2 requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 350
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 300
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 0
#define FOUR_DRIVE_STARTUP_COUNTS 16U
#define FOUR_DRIVE_STARTUP_MS 500U
#elif defined(FOUR_LINK_GROUND)
#if !defined(FOUR_LINK_MOTION) || !defined(DRIVE_STARTUP_OBSERVE)
#error Ground profile requires the bounded streaming motion image.
#endif
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 200
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 150
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 0
#define FOUR_DRIVE_STARTUP_COUNTS 12U
#define FOUR_DRIVE_STARTUP_MS 400U
#else
#define FOUR_SPEED_DUTY_LIMIT_PERMILLE 100
#define FOUR_DRIVE_STARTUP_DUTY_PERMILLE 80
#define FOUR_DRIVE_STARTUP_HANDOFF_PERMILLE 0
#define FOUR_DRIVE_STARTUP_COUNTS 8U
#define FOUR_DRIVE_STARTUP_MS 200U
#endif

/* 仅隔离RR镜像提高目标；不开放整车或任意轮速命令，不提高10%输出上限。 */
#if defined(FOUR_LINK_GROUNDB)
/* Faster recovery demonstration profile: retain 4WGA's 600 ms link lease
 * and proven actuator envelope, while raising only the body-axis command
 * limit from 0.10 to 0.15 m/s. */
#define FOUR_DRIVE_TARGET_LIMIT_CPS 1200.0f
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.15f
#define FOUR_DRIVE_MAX_WHEEL_M_S 0.20f
#define FOUR_DRIVE_MAX_BODY_MM_S 150
#define FOUR_DRIVE_RUN_LIMIT_MS 150000U
#elif defined(FOUR_LINK_GROUNDA)
/* Vicon recovery demonstration profile: keep the proven 35% actuator
 * envelope, but cap each commanded body axis at 0.10 m/s.  The 600 ms lease
 * covers the measured 160--555 ms Wi-Fi receive stalls while bounding stale
 * travel to 6 cm at this profile's maximum translation speed. */
#define FOUR_DRIVE_TARGET_LIMIT_CPS 1200.0f
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.10f
#define FOUR_DRIVE_MAX_WHEEL_M_S 0.20f
#define FOUR_DRIVE_MAX_BODY_MM_S 100
#define FOUR_DRIVE_RUN_LIMIT_MS 150000U
#elif defined(FOUR_LINK_GROUND8) || defined(FOUR_LINK_GROUND9)
#define FOUR_DRIVE_TARGET_LIMIT_CPS 1200.0f
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.20f
#define FOUR_DRIVE_MAX_WHEEL_M_S 0.20f
#define FOUR_DRIVE_MAX_BODY_MM_S 200
#define FOUR_DRIVE_RUN_LIMIT_MS 150000U
#elif defined(RR_SPEED2)
#if !defined(RR_DIAGNOSTIC) || !defined(RR_PWM_FINE) || !defined(DRIVE_STARTUP_OBSERVE)
#error RR speed comparison requires the isolated fine-PWM diagnostic.
#endif
#define RR_DIAGNOSTIC_TARGET_CPS 328.0f
#define RR_DIAGNOSTIC_RUN_COMMAND "run RR328 S10"
#define FOUR_DRIVE_TARGET_LIMIT_CPS 328.0f
#else
#define RR_DIAGNOSTIC_TARGET_CPS 164.0f
#define RR_DIAGNOSTIC_RUN_COMMAND "run RR164 S10"
#define FOUR_DRIVE_TARGET_LIMIT_CPS 300.0f
#endif

#ifndef FOUR_DRIVE_MAX_LINEAR_M_S
#define FOUR_DRIVE_MAX_LINEAR_M_S 0.05f
#define FOUR_DRIVE_MAX_WHEEL_M_S 0.05f
#define FOUR_DRIVE_MAX_BODY_MM_S 50
#define FOUR_DRIVE_RUN_LIMIT_MS 120000U
#endif
#define FOUR_DRIVE_MAX_YAW_RAD_S 0.20f
#define FOUR_DRIVE_MAX_YAW_MRAD_S 200

#if defined(FOUR_LINK_GROUNDA) || defined(FOUR_LINK_GROUNDB)
#define FOUR_DRIVE_COMMAND_TIMEOUT_MS 600U
#else
#define FOUR_DRIVE_COMMAND_TIMEOUT_MS 150U
#endif

/* 仅RR隔离镜像细化占空比；旧镜像继续使用ARR=99、整1%输出。 */
#ifdef RR_PWM_FINE
#if !defined(RR_DIAGNOSTIC) || !defined(DRIVE_STARTUP_OBSERVE)
#error Fine PWM is restricted to the bounded RR-only diagnostic.
#endif
#define FOUR_SPEED_PWM_COUNTS 8400U
#define FOUR_SPEED_DUTY_QUANTUM 1
#define FOUR_SPEED_PWM_APB1_PSC 0U
#define FOUR_SPEED_PWM_APB2_PSC 1U
#else
#define FOUR_SPEED_PWM_COUNTS 100U
#define FOUR_SPEED_DUTY_QUANTUM 10
#define FOUR_SPEED_PWM_APB1_PSC 83U
#define FOUR_SPEED_PWM_APB2_PSC 167U
#endif

/* APB1定时器84MHz、APB2定时器168MHz，两个配置均为10kHz。 */
_Static_assert(84000000U / (FOUR_SPEED_PWM_APB1_PSC + 1U) /
               FOUR_SPEED_PWM_COUNTS == 10000U, "APB1 PWM frequency changed");
_Static_assert(168000000U / (FOUR_SPEED_PWM_APB2_PSC + 1U) /
               FOUR_SPEED_PWM_COUNTS == 10000U, "APB2 PWM frequency changed");

#endif
