#include "board.h"

#include "app_config.h"
#include "encoder_a.h"
#include "mc_protocol.h"
#include "mc_uart_transport.h"
#include "mecanum_protocol_v1.h"
#include "motor_a.h"
#include "motor_link_control.h"
#include "speed_controller.h"

#include <limits.h>
#include <stdbool.h>
#include <stdint.h>

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart2;

static motor_link_control_t link_control;
static speed_controller_t speed_controller;
static uint32_t telemetry_sequence;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART2_UART_Init(void);
static void apply_control_state(uint32_t now_ms);
static void send_hello_ack(uint32_t now_ms);
static void send_telemetry(uint32_t now_ms);
static int16_t encoder_rpm_x10(void);
static void status_led_tick(uint32_t now_ms);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM3_Init();
    MX_TIM9_Init();
    MX_USART2_UART_Init();

    if (!mc_protocol_self_test()) {
        Error_Handler();
    }
    motor_a_init(&htim9);
    encoder_a_init(&htim3, HAL_GetTick());
    speed_controller_init(&speed_controller, HAL_GetTick());
    motor_link_control_init(
        &link_control, MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV != 0U);
    mc_uart_transport_init(&huart2);
    mc_uart_transport_enable_startup_line_error_guard();
    telemetry_sequence = 0U;

    uint32_t last_telemetry_ms = HAL_GetTick();
    uint8_t raw_frame[MC_MAX_RAW_FRAME_SIZE];
    while (1) {
        const uint32_t now_ms = HAL_GetTick();
        motor_a_tick(now_ms);
        encoder_a_tick(now_ms);

        if (mc_uart_transport_take_fault()) {
            motor_link_control_force_fault(
                &link_control, FAULT_UART_OVERFLOW);
            apply_control_state(now_ms);
        }

        size_t raw_length = 0U;
        if (mc_uart_transport_take_frame(
                raw_frame, sizeof(raw_frame), &raw_length)) {
            mc_frame_header_v1_t validated_header;
            if (mc_validate_frame(raw_frame,raw_length,&validated_header)) {
                mc_uart_transport_confirm_valid_frame();
            }
            const motor_link_event_t event = motor_link_control_accept(
                &link_control, raw_frame, raw_length, now_ms);
            if (event == MOTOR_LINK_EVENT_HELLO) {
                send_hello_ack(now_ms);
            }
            if (event != MOTOR_LINK_EVENT_NONE) {
                apply_control_state(now_ms);
            }
        }

        if (motor_link_control_tick(&link_control, now_ms)) {
            apply_control_state(now_ms);
        }

        int16_t closed_loop_duty = 0;
        if (link_control.output_enabled
            && speed_controller_tick(
                &speed_controller,
                encoder_a_counts_per_second(),
                now_ms,
                &closed_loop_duty)) {
            motor_a_request(closed_loop_duty, now_ms);
        }

        if ((uint32_t)(now_ms - last_telemetry_ms)
            >= MOTOR_LINK_TELEMETRY_PERIOD_MS) {
            last_telemetry_ms = now_ms;
            send_telemetry(now_ms);
        }
        status_led_tick(now_ms);
    }
}

static void apply_control_state(uint32_t now_ms)
{
    if (!link_control.output_enabled) {
        speed_controller_disable(&speed_controller);
        motor_a_stop();
        return;
    }
    const int32_t target_cps = motor_link_control_target_cps(&link_control);
    speed_controller_set_target(&speed_controller, target_cps, now_ms);
    if (target_cps == 0) {
        motor_a_brake();
    }
}

static void send_hello_ack(uint32_t now_ms)
{
    (void)mc_uart_transport_send_frame(
        MC_MSG_HELLO_ACK,
        MC_FLAG_RESPONSE,
        link_control.last_sequence,
        link_control.session_id,
        now_ms,
        NULL,
        0U);
}

static int16_t encoder_rpm_x10(void)
{
#if MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV == 0U
    return 0;
#else
    int64_t rpm_x10 =
        ((int64_t)encoder_a_counts_per_second() * 600)
        / MOTOR_A_ENCODER_COUNTS_PER_WHEEL_REV;
    if (rpm_x10 > INT16_MAX) {
        rpm_x10 = INT16_MAX;
    } else if (rpm_x10 < INT16_MIN) {
        rpm_x10 = INT16_MIN;
    }
    return (int16_t)rpm_x10;
#endif
}

static void send_telemetry(uint32_t now_ms)
{
    if (!link_control.session_active) {
        return;
    }
    mc_wheel_telemetry_v1_t telemetry = {0};
    telemetry.sample_time_ms = now_ms;
    telemetry.wheel_rpm_x10[0] = encoder_rpm_x10();
    telemetry.encoder_counts[0] = encoder_a_count();
    telemetry.battery_mv = 0U;
    telemetry.state_flags = motor_link_control_state_flags(&link_control);
    telemetry.fault_bits = link_control.fault_bits;
    telemetry.last_command_sequence = link_control.last_sequence;

    uint16_t flags = 0U;
    if (link_control.output_enabled) {
        flags |= MC_FLAG_ENABLED;
    }
    if (link_control.fault_bits != FAULT_NONE) {
        flags |= MC_FLAG_FAULT;
    }
    ++telemetry_sequence;
    (void)mc_uart_transport_send_frame(
        MC_MSG_WHEEL_TELEMETRY,
        flags,
        telemetry_sequence,
        link_control.session_id,
        now_ms,
        &telemetry,
        sizeof(telemetry));
}

static void status_led_tick(uint32_t now_ms)
{
    static uint32_t previous_toggle_ms;
    const uint32_t period_ms =
        link_control.fault_bits != FAULT_NONE ? 100U : 500U;
    if (!link_control.session_active) {
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

    clocks.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK
        | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_1) != HAL_OK
        || HAL_TIM_PWM_ConfigChannel(&htim9, &pwm, TIM_CHANNEL_2) != HAL_OK) {
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
    huart2.Init.BaudRate = MC_UART_BAUD_RATE;
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
