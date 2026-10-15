#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define MOTOR_PWM_FREQUENCY_HZ          10000U
#define MOTOR_PWM_PERIOD_COUNTS         100U
#define MOTOR_BRINGUP_MAX_PERMILLE      200
#define MOTOR_DIRECTION_DEADTIME_MS     5U

/* Conservative software-only PI defaults. Tune on the suspended chassis. */
#define SPEED_TARGET_MAX_CPS            5000
#define SPEED_KP_NUMERATOR              1
#define SPEED_KP_DENOMINATOR            10
#define SPEED_KI_NUMERATOR              1
#define SPEED_KI_DENOMINATOR            10000

/* Keep zero until one output-shaft revolution is measured on this motor. */
#ifndef MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV
#define MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV 0U
#endif
#define MOTOR_A_WHEEL_DIAMETER_MM        60U
#define MOTOR_LINK_TELEMETRY_PERIOD_MS   100U

#define COMMAND_UART_BAUDRATE           115200U
#define COMMAND_LINE_MAX_LENGTH         64U
#define COMMAND_HEARTBEAT_TIMEOUT_MS    500U
#define TELEMETRY_PERIOD_MS             100U
#define ENCODER_SAMPLE_PERIOD_MS        10U

#define ARM_KEY                         0xA5
#define CLEAR_FAULT_KEY                 0x5A

#define FAULT_NONE                      0U
#define FAULT_COMMAND_TIMEOUT           (1U << 0)
#define FAULT_UART_OVERFLOW             (1U << 1)
#define FAULT_ESTOP                     (1U << 2)
#define FAULT_CONFIG_UNCALIBRATED       (1U << 3)
#define FAULT_UNSUPPORTED_MOTION        (1U << 4)

#endif
