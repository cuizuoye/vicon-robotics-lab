#include "board.h"

#include "app_config.h"
#include "command.h"
#include "encoder_a.h"
#include "motor_a.h"
#include "speed_controller.h"
#include "uart_console.h"

#include <stdbool.h>
#include <stdint.h>

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart2;

static bool control_enabled;
static bool heartbeat_seen;
static uint32_t last_heartbeat_ms;
static uint32_t fault_bits;
static speed_controller_t speed_controller;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART2_UART_Init(void);
static void handle_command(const command_t *command, uint32_t now_ms);
static void publish_status(uint32_t now_ms, const char *prefix);
static void safety_stop(uint32_t fault);
static void status_led_tick(uint32_t now_ms);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_TIM9_Init();
    MX_USART2_UART_Init();

    control_enabled = false;
    heartbeat_seen = false;
    last_heartbeat_ms = 0U;
    fault_bits = FAULT_NONE;

    motor_a_init(&htim9);
    encoder_a_init(&htim3, HAL_GetTick());
    speed_controller_init(&speed_controller, HAL_GetTick());
    uart_console_init(&huart2);
    uart_console_write("READY,motor_a_safe_off\r\n");
    uart_console_write("INFO,send hb then arm A5; duty range -200..200\r\n");

    uint32_t last_telemetry_ms = HAL_GetTick();
    char line[COMMAND_LINE_MAX_LENGTH];

    while (1) {
        const uint32_t now_ms = HAL_GetTick();
        motor_a_tick(now_ms);
        encoder_a_tick(now_ms);

        int16_t closed_loop_duty = 0;
        if (control_enabled &&
            speed_controller_tick(
                &speed_controller,
                encoder_a_counts_per_second(),
                now_ms,
                &closed_loop_duty)) {
            motor_a_request(closed_loop_duty, now_ms);
        }

        if (uart_console_take_overflow()) {
            safety_stop(FAULT_UART_OVERFLOW);
            uart_console_write("ERR,uart_overflow\r\n");
        }

        if (uart_console_take_line(line, sizeof(line))) {
            command_t command;
            if (command_parse(line, &command)) {
                handle_command(&command, now_ms);
            } else {
                uart_console_write("ERR,bad_command\r\n");
            }
        }

        if (control_enabled &&
            (!heartbeat_seen ||
             (uint32_t)(now_ms - last_heartbeat_ms) > COMMAND_HEARTBEAT_TIMEOUT_MS)) {
            safety_stop(FAULT_COMMAND_TIMEOUT);
            uart_console_write("ERR,heartbeat_timeout\r\n");
        }

        if ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
            last_telemetry_ms = now_ms;
            publish_status(now_ms, "TEL");
        }
        status_led_tick(now_ms);
    }
}

static void handle_command(const command_t *command, uint32_t now_ms)
{
    switch (command->type) {
    case COMMAND_HELP:
        uart_console_write(
            "HELP,hb|arm A5|duty -200..200|speed cps|brake|stop|estop|clear 5A|zero|status\r\n");
        break;
    case COMMAND_STATUS:
        publish_status(now_ms, "STATUS");
        break;
    case COMMAND_HEARTBEAT:
        heartbeat_seen = true;
        last_heartbeat_ms = now_ms;
        break;
    case COMMAND_ARM:
        if (command->argument != ARM_KEY) {
            uart_console_write("ERR,bad_arm_key\r\n");
        } else if (fault_bits != FAULT_NONE) {
            uart_console_write("ERR,fault_latched\r\n");
        } else if (!heartbeat_seen ||
                   (uint32_t)(now_ms - last_heartbeat_ms) > COMMAND_HEARTBEAT_TIMEOUT_MS) {
            uart_console_write("ERR,heartbeat_required\r\n");
        } else {
            control_enabled = true;
            uart_console_write("OK,armed\r\n");
        }
        break;
    case COMMAND_DUTY:
        if (!control_enabled) {
            uart_console_write("ERR,not_armed\r\n");
        } else if (command->argument < -MOTOR_BRINGUP_MAX_PERMILLE ||
                   command->argument > MOTOR_BRINGUP_MAX_PERMILLE) {
            uart_console_write("ERR,duty_out_of_range\r\n");
        } else {
            speed_controller_disable(&speed_controller);
            motor_a_request((int16_t)command->argument, now_ms);
            uart_console_printf("OK,duty=%ld\r\n", (long)command->argument);
        }
        break;
    case COMMAND_SPEED:
        if (!control_enabled) {
            uart_console_write("ERR,not_armed\r\n");
        } else if (command->argument < -SPEED_TARGET_MAX_CPS ||
                   command->argument > SPEED_TARGET_MAX_CPS) {
            uart_console_write("ERR,speed_out_of_range\r\n");
        } else {
            speed_controller_set_target(
                &speed_controller, command->argument, now_ms);
            uart_console_printf("OK,speed_cps=%ld\r\n", (long)command->argument);
        }
        break;
    case COMMAND_BRAKE:
        if (!control_enabled) {
            uart_console_write("ERR,not_armed\r\n");
        } else {
            speed_controller_disable(&speed_controller);
            motor_a_brake();
            uart_console_write("OK,brake\r\n");
        }
        break;
    case COMMAND_STOP:
        speed_controller_disable(&speed_controller);
        motor_a_stop();
        control_enabled = false;
        uart_console_write("OK,stopped\r\n");
        break;
    case COMMAND_ESTOP:
        safety_stop(FAULT_ESTOP);
        uart_console_write("OK,estop_latched\r\n");
        break;
    case COMMAND_CLEAR:
        if (command->argument != CLEAR_FAULT_KEY) {
            uart_console_write("ERR,bad_clear_key\r\n");
        } else if (control_enabled || !motor_a_is_high_z()) {
            uart_console_write("ERR,stop_before_clear\r\n");
        } else {
            fault_bits = FAULT_NONE;
            heartbeat_seen = false;
            uart_console_write("OK,faults_cleared\r\n");
        }
        break;
    case COMMAND_ZERO_ENCODER:
        if (control_enabled || !motor_a_is_high_z()) {
            uart_console_write("ERR,stop_before_zero\r\n");
        } else {
            encoder_a_zero();
            uart_console_write("OK,encoder_zeroed\r\n");
        }
        break;
    case COMMAND_INVALID:
    default:
        uart_console_write("ERR,bad_command\r\n");
        break;
    }
}

static void publish_status(uint32_t now_ms, const char *prefix)
{
    uart_console_printf(
        "%s,ms=%lu,en=%u,hiz=%u,mode=%s,target_cps=%ld,duty=%d,enc=%ld,cps=%ld,fault=0x%08lx\r\n",
        prefix,
        (unsigned long)now_ms,
        control_enabled ? 1U : 0U,
        motor_a_is_high_z() ? 1U : 0U,
        speed_controller_enabled(&speed_controller) ? "speed" : "duty",
        (long)speed_controller_target(&speed_controller),
        (int)motor_a_command(),
        (long)encoder_a_count(),
        (long)encoder_a_counts_per_second(),
        (unsigned long)fault_bits);
}

static void safety_stop(uint32_t fault)
{
    speed_controller_disable(&speed_controller);
    motor_a_stop();
    control_enabled = false;
    fault_bits |= fault;
}

static void status_led_tick(uint32_t now_ms)
{
    static uint32_t previous_toggle_ms;
    const uint32_t period_ms = fault_bits != FAULT_NONE ? 100U : 500U;
    if (!control_enabled && fault_bits == FAULT_NONE) {
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
        return;
    }
    if ((uint32_t)(now_ms - previous_toggle_ms) >= period_ms) {
        previous_toggle_ms = now_ms;
        HAL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clocks = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 8U;
    oscillator.PLL.PLLN = 336U;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clocks.APB1CLKDivider = RCC_HCLK_DIV4;
    clocks.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    gpio.Pin = STATUS_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);
}

static void MX_TIM9_Init(void)
{
    TIM_OC_InitTypeDef pwm = {0};
    htim9.Instance = TIM9;
    htim9.Init.Prescaler = 167U;
    htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim9.Init.Period = MOTOR_PWM_PERIOD_COUNTS - 1U;
    htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
        Error_Handler();
    }
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim9);
}

static void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFFU;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 6U;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 6U;
    if (HAL_TIM_Encoder_Init(&htim3, &encoder) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = COMMAND_UART_BAUDRATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    TIM9->CCR1 = 0U;
    TIM9->CCR2 = 0U;
    TIM9->CCER = 0U;
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
