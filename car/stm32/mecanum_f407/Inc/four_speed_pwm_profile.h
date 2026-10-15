#ifndef FOUR_SPEED_PWM_PROFILE_H
#define FOUR_SPEED_PWM_PROFILE_H

/* 仅隔离RR镜像提高目标；不开放整车或任意轮速命令，不提高10%输出上限。 */
#ifdef RR_SPEED2
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
