#include "board.h"

#include "app_config.h"
#include "bench_dual_control.h"
#include "encoder_a.h"
#if defined(BENCH_FRONT_ENCODERS_READONLY)
#include "encoder_b.h"
#endif
#include "uart_console.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(BENCH_FRONT_ENCODERS_READONLY)
#define BENCH_FIRMWARE_NAME "front_enc_ro_v1"
#define BENCH_BOOT_LINE "READY,front_encoder_readonly_v1\r\n"
#define BENCH_INFO_LINE "INFO,115200; GPIO low; TIM3/TIM4 raw counts only; no arm/duty/hb\r\n"
#elif defined(BENCH_FRONT_DEMO30)
#define BENCH_FIRMWARE_NAME "demo30_v1"
#define BENCH_BOOT_LINE "READY,front_demo30_safe_off_v1\r\n"
#define BENCH_INFO_LINE "INFO,115200; suspended front wheels; fixed A+/B- 10%; one 30s demo per boot\r\n"
#else
#define BENCH_FIRMWARE_NAME "dual_v2"
#define BENCH_BOOT_LINE "READY,dual_motor_safe_off_v2\r\n"
#define BENCH_INFO_LINE "INFO,115200; fixed 10%; short 2s; explicit A+/B- observe 5s\r\n"
#endif

_Static_assert(MOTOR_PWM_PERIOD_COUNTS == 100U, "bench PWM mapping requires period 100");
_Static_assert(COMMAND_UART_BAUDRATE == 115200U, "bench UART protocol is 115200");

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
#if defined(BENCH_FRONT_ENCODERS_READONLY)
TIM_HandleTypeDef htim4;
#endif
TIM_HandleTypeDef htim9;
UART_HandleTypeDef huart2;

static bench_dual_control_t control;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
#if !defined(BENCH_FRONT_ENCODERS_READONLY)
static void MX_PWM_Init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance);
#endif
static void MX_TIM3_Init(void);
#if defined(BENCH_FRONT_ENCODERS_READONLY)
static void MX_TIM4_Init(void);
#endif
static void MX_USART2_UART_Init(void);
static void apply_outputs(void);
static void publish_status(uint32_t now_ms, const char *prefix);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    bench_dual_init(&control);
#if !defined(BENCH_FRONT_ENCODERS_READONLY)
    MX_PWM_Init(&htim9, TIM9);
    MX_PWM_Init(&htim1, TIM1);
#endif
    MX_TIM3_Init();
#if defined(BENCH_FRONT_ENCODERS_READONLY)
    MX_TIM4_Init();
#endif
    MX_USART2_UART_Init();

    /* 两路CCR先为0，再启用定时器；启动和arm均不会发出转动请求。 */
    apply_outputs();
#if !defined(BENCH_FRONT_ENCODERS_READONLY)
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
#endif
    encoder_a_init(&htim3, HAL_GetTick());
#if defined(BENCH_FRONT_ENCODERS_READONLY)
    encoder_b_init(&htim4, HAL_GetTick());
#endif
    uart_console_init(&huart2);
    uart_console_write(BENCH_BOOT_LINE);
    uart_console_write(BENCH_INFO_LINE);

    uint32_t last_telemetry_ms = HAL_GetTick();
    char line[COMMAND_LINE_MAX_LENGTH];
    while (1) {
        uint32_t now_ms = HAL_GetTick();
        bench_dual_tick(&control, now_ms);
        apply_outputs();
        encoder_a_tick(now_ms);
#if defined(BENCH_FRONT_ENCODERS_READONLY)
        encoder_b_tick(now_ms);
#endif

        if (uart_console_take_overflow()) {
            bench_dual_stop(&control, BENCH_DUAL_FAULT_UART);
            apply_outputs();
            uart_console_write("ERR,uart_overflow\r\n");
        }
        if (uart_console_take_line(line, sizeof(line))) {
            now_ms = HAL_GetTick();
            const char *reply = bench_dual_accept(&control, line, now_ms);
            /* ACK发送前就写入两路停机/PWM状态；不让串口等待延迟stop。 */
            apply_outputs();
            if (reply != NULL) {
                uart_console_write(reply);
            }
            if (strcmp(line, "status") == 0) {
#if defined(BENCH_FRONT_ENCODERS_READONLY)
                publish_status(HAL_GetTick(), "STATUS3");
#else
                publish_status(HAL_GetTick(), "STATUS2");
#endif
            }
        }
        now_ms = HAL_GetTick();
        bench_dual_tick(&control, now_ms);
        apply_outputs();
        if ((uint32_t)(now_ms - last_telemetry_ms) >= TELEMETRY_PERIOD_MS) {
            last_telemetry_ms = now_ms;
#if defined(BENCH_FRONT_ENCODERS_READONLY)
            publish_status(now_ms, "TEL3");
#else
            publish_status(now_ms, "TEL2");
#endif
        }
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                         control.armed || control.fault_bits != 0U ?
                         GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static void apply_outputs(void)
{
#if defined(BENCH_FRONT_ENCODERS_READONLY)
    /* 不初始化电机定时器、不切AF；包括错误命令在内始终GPIO低。 */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11,
                     GPIO_PIN_RESET);
#else
    uint32_t a1, a2, b1, b2;
    if (!bench_dual_pwm_compare(control.duty_a, &a1, &a2) ||
        !bench_dual_pwm_compare(control.duty_b, &b1, &b2)) {
        Error_Handler();
    }
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, a1);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, a2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, b1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, b2);
#endif
}

static void publish_status(uint32_t now_ms, const char *prefix)
{
#if defined(BENCH_FRONT_ENCODERS_READONLY)
    /* 标定前只报原始计数/秒，不套减速比、不伪造RPM或CPR。 */
    uart_console_printf(
        "%s,fw=" BENCH_FIRMWARE_NAME ",ms=%lu,en=0,mask=0,a=0,b=0,az=1,bz=1,limit_ms=0,enc_a=%ld,cps_a=%ld,enc_b=%ld,cps_b=%ld,fault=0x%08lx\r\n",
        prefix, (unsigned long)now_ms,
        (long)encoder_a_count(), (long)encoder_a_counts_per_second(),
        (long)encoder_b_count(), (long)encoder_b_counts_per_second(),
        (unsigned long)control.fault_bits);
#else
    /* B尚未接编码器，不伪造B速度；cps_a仍只是计数/秒，不是RPM。 */
    uart_console_printf(
        "%s,fw=" BENCH_FIRMWARE_NAME ",ms=%lu,en=%u,mask=%u,a=%d,b=%d,az=%u,bz=%u,limit_ms=%lu,enc_a=%ld,cps_a=%ld,fault=0x%08lx\r\n",
        prefix, (unsigned long)now_ms, control.armed ? 1U : 0U,
        (unsigned int)control.mask, (int)control.duty_a, (int)control.duty_b,
        control.duty_a == 0 ? 1U : 0U, control.duty_b == 0 ? 1U : 0U,
        (unsigned long)control.pulse_limit_ms,
        (long)encoder_a_count(), (long)encoder_a_counts_per_second(),
        (unsigned long)control.fault_bits);
#endif
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
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    gpio.Pin = STATUS_LED_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(STATUS_LED_PORT, &gpio);

    /* 启动时先用GPIO把全部AIN/BIN拉低，再切到零CCR的定时器复用。 */
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11;
    HAL_GPIO_WritePin(GPIOE, gpio.Pin, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOE, &gpio);
}

#if !defined(BENCH_FRONT_ENCODERS_READONLY)
static void MX_PWM_Init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef pwm = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = 167U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = MOTOR_PWM_PERIOD_COUNTS - 1U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.RepetitionCounter = 0U;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) {
        Error_Handler();
    }
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(timer, &pwm, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(timer, &pwm, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (instance == TIM1) {
        TIM_BreakDeadTimeConfigTypeDef safety = {0};
        safety.OffStateRunMode = TIM_OSSR_DISABLE;
        safety.OffStateIDLEMode = TIM_OSSI_DISABLE;
        safety.LockLevel = TIM_LOCKLEVEL_OFF;
        safety.DeadTime = 0U;
        safety.BreakState = TIM_BREAK_DISABLE;
        safety.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        safety.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(timer, &safety) != HAL_OK) {
            Error_Handler();
        }
    }
    HAL_TIM_MspPostInit(timer);
}
#endif

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

#if defined(BENCH_FRONT_ENCODERS_READONLY)
static void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0U;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFFU;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 6U;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 6U;
    if (HAL_TIM_Encoder_Init(&htim4, &encoder) != HAL_OK) {
        Error_Handler();
    }
}
#endif

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
    /* 任何初始化/运行致命异常同时禁用两路，再将AIN/BIN切为低GPIO。 */
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM9->CCR1 = 0U;
    TIM9->CCR2 = 0U;
    TIM9->CCER = 0U;
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCER = 0U;
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11;
    HAL_GPIO_WritePin(GPIOE, gpio.Pin, GPIO_PIN_RESET);
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &gpio);
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
