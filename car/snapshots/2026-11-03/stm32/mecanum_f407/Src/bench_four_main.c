#include "board.h"
#include "app_config.h"
#include "bench_four_control.h"
#include "encoder_accumulator.h"
#include "uart_console.h"
#include <string.h>

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim8, htim9, htim12;
UART_HandleTypeDef huart2;
static bench_four_t control;
static encoder_accumulator_t enc[FOUR_WHEELS];
static TIM_HandleTypeDef *const encoder_timers[] = {&htim3, &htim4, &htim2, &htim8};
static TIM_HandleTypeDef *const motor_timers[] = {&htim9, &htim1, &htim1, &htim12};
static const uint32_t first_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_1};
static const uint32_t second_channels[] = {TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2};

static void clocks_init(void)
{
    RCC_OscInitTypeDef o = {0};
    RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    o.HSEState = RCC_HSE_ON;
    o.PLL.PLLState = RCC_PLL_ON;
    o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 8U;
    o.PLL.PLLN = 336U;
    o.PLL.PLLP = RCC_PLLP_DIV2;
    o.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4;
    c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
}

static void controls_gpio_low(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
    HAL_GPIO_WritePin(GPIOE, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &g);
}

static void pwm_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef pwm = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = instance == TIM12 ? 83U : 167U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 99U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.RepetitionCounter = 0U;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
    pwm.OCMode = TIM_OCMODE_PWM1;
    pwm.Pulse = 0U;
    pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
    pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    pwm.OCFastMode = TIM_OCFAST_DISABLE;
    pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
    pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    for (unsigned i = 0; i < (instance == TIM1 ? 4U : 2U); ++i)
        if (HAL_TIM_PWM_ConfigChannel(timer, &pwm, channels[i]) != HAL_OK) Error_Handler();
    if (instance == TIM1) {
        TIM_BreakDeadTimeConfigTypeDef safety = {0};
        safety.OffStateRunMode = TIM_OSSR_DISABLE;
        safety.OffStateIDLEMode = TIM_OSSI_DISABLE;
        safety.LockLevel = TIM_LOCKLEVEL_OFF;
        safety.BreakState = TIM_BREAK_DISABLE;
        safety.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
        safety.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
        if (HAL_TIMEx_ConfigBreakDeadTime(timer, &safety) != HAL_OK) Error_Handler();
    }
    HAL_TIM_MspPostInit(timer);
}

static void encoder_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint8_t bits)
{
    TIM_Encoder_InitTypeDef e = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = 0U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = bits == 32U ? UINT32_MAX : 0xFFFFU;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    e.EncoderMode = TIM_ENCODERMODE_TI12;
    e.IC1Polarity = TIM_ICPOLARITY_RISING;
    e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = TIM_ICPSC_DIV1;
    e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = 6U;
    e.IC2Filter = 6U;
    if (HAL_TIM_Encoder_Init(timer, &e) != HAL_OK) Error_Handler();
}

static void apply_outputs(void)
{
    for (unsigned i = 0; i < FOUR_WHEELS; ++i) {
        uint32_t first, second;
        if ((control.duty[i] != 0 && control.selected != i + 1U) ||
            !bench_four_compare(control.duty[i], &first, &second)) Error_Handler();
        __HAL_TIM_SET_COMPARE(motor_timers[i], first_channels[i], first);
        __HAL_TIM_SET_COMPARE(motor_timers[i], second_channels[i], second);
    }
}

static void publish(uint32_t now, const char *prefix)
{
    /* 读实际CCR，不把逻辑请求当示波器波形；计数未标定，不输出伪RPM。 */
    uart_console_printf(
        "%s,fw=four_bench_v1,ms=%lu,en=%u,sel=%u,limit_ms=%u,"
        "d_fl=%d,d_fr=%d,d_rl=%d,d_rr=%d,"
        "enc_fl=%ld,cps_fl=%ld,enc_fr=%ld,cps_fr=%ld,enc_rl=%ld,cps_rl=%ld,enc_rr=%ld,cps_rr=%ld,"
        "p_fl1=%lu,p_fl2=%lu,p_fr1=%lu,p_fr2=%lu,p_rl1=%lu,p_rl2=%lu,p_rr1=%lu,p_rr2=%lu,fault=0x%08lx\r\n",
        prefix, (unsigned long)now, control.selected != 0U ? 1U : 0U,
        (unsigned)control.selected, control.selected != 0U ? (unsigned)FOUR_PULSE_LIMIT_MS : 0U,
        (int)control.duty[0], (int)control.duty[1], (int)control.duty[2], (int)control.duty[3],
        (long)encoder_accumulator_count(&enc[0]), (long)enc[0].cps,
        (long)encoder_accumulator_count(&enc[1]), (long)enc[1].cps,
        (long)encoder_accumulator_count(&enc[2]), (long)enc[2].cps,
        (long)encoder_accumulator_count(&enc[3]), (long)enc[3].cps,
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim9, TIM_CHANNEL_1),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim9, TIM_CHANNEL_2),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim12, TIM_CHANNEL_1),
        (unsigned long)__HAL_TIM_GET_COMPARE(&htim12, TIM_CHANNEL_2), (unsigned long)control.fault);
}

int main(void)
{
    HAL_Init();
    /* 比时钟初始化更早拉低8根控制线；仍不代替断动力烧录的边界。 */
    controls_gpio_low();
    clocks_init();
    bench_four_init(&control);
    GPIO_InitTypeDef led = {0};
    led.Pin = STATUS_LED_PIN;
    led.Mode = GPIO_MODE_OUTPUT_PP;
    led.Pull = GPIO_NOPULL;
    HAL_GPIO_WritePin(STATUS_LED_PORT, led.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(STATUS_LED_PORT, &led);
    pwm_init(&htim9, TIM9);
    pwm_init(&htim1, TIM1);
    pwm_init(&htim12, TIM12);
    encoder_init(&htim3, TIM3, 16U);
    encoder_init(&htim4, TIM4, 16U);
    encoder_init(&htim2, TIM2, 32U);
    encoder_init(&htim8, TIM8, 16U);
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200U;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    apply_outputs();
    for (unsigned i = 0; i < FOUR_WHEELS; ++i) {
        if (HAL_TIM_PWM_Start(motor_timers[i], first_channels[i]) != HAL_OK ||
            HAL_TIM_PWM_Start(motor_timers[i], second_channels[i]) != HAL_OK) Error_Handler();
        __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
        if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
        encoder_accumulator_init(&enc[i], i == 2U ? 32U : 16U, 0U, HAL_GetTick());
    }
    uart_console_init(&huart2);
    uart_console_write("READY,four_bench_safe_off_v1\r\n");
    uart_console_write("INFO,115200; one FL/FR/RL/RR only; fixed +/-10%; host 5s; MCU cap 5.2s\r\n");
    uint32_t last_telemetry = HAL_GetTick();
    char line[COMMAND_LINE_MAX_LENGTH];
    while (1) {
        uint32_t now = HAL_GetTick();
        bench_four_tick(&control, now);
        apply_outputs();
        for (unsigned i = 0; i < FOUR_WHEELS; ++i)
            encoder_accumulator_tick(&enc[i], __HAL_TIM_GET_COUNTER(encoder_timers[i]), now);
        if (uart_console_take_overflow()) {
            bench_four_stop(&control, FOUR_FAULT_UART);
            apply_outputs();
            uart_console_write("ERR,uart_overflow\r\n");
        }
        if (uart_console_take_line(line, sizeof(line))) {
            const char *reply = bench_four_accept(&control, line, HAL_GetTick());
            apply_outputs();
            if (reply != NULL) uart_console_write(reply);
            if (strcmp(line, "status") == 0) publish(HAL_GetTick(), "STATUS4");
        }
        now = HAL_GetTick();
        bench_four_tick(&control, now);
        apply_outputs();
        if ((uint32_t)(now - last_telemetry) >= 100U) {
            last_telemetry = now;
            publish(now, "TEL4");
        }
        HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN,
                          control.selected != 0U || control.fault != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void Error_Handler(void)
{
    __disable_irq();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
    TIM9->CCR1 = TIM9->CCR2 = 0U;
    TIM12->CCR1 = TIM12->CCR2 = 0U;
    TIM1->CCER = TIM9->CCER = TIM12->CCER = 0U;
    controls_gpio_low();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
