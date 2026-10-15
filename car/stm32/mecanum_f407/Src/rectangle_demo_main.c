#include "board.h"
#include "encoder_accumulator.h"
#include "rectangle_demo.h"
#ifdef RECTANGLE_STARTUP_TRACE
#include "rectangle_startup_trace.h"
#define RECTANGLE_FIRMWARE "rect_start_trace_v1"
#else
#define RECTANGLE_FIRMWARE "rect_enc_v1"
#endif
#include "speed_uart_transport.h"
#include "stm32f4xx_ll_iwdg.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim6, htim8, htim9, htim12;
UART_HandleTypeDef huart2;
static rectangle_demo_t demo;
static encoder_accumulator_t enc[4];
static volatile bool stop_pending, uart_failed;
static uint32_t previous_irq_ms;
static const uint16_t control_e = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 |
    GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
static const uint16_t control_b = GPIO_PIN_14 | GPIO_PIN_15;
static TIM_HandleTypeDef *const encoder_timers[] = {&htim3, &htim4, &htim2, &htim8};
static TIM_HandleTypeDef *const motor_timers[] = {&htim9, &htim1, &htim1, &htim12};
static const uint32_t first_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_1};
static const uint32_t second_channels[] = {TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2};
typedef struct {
    uint32_t ms, ticks, fault, dt_min, dt_max, irq_us_max, compares[8];
    int32_t counts[4], progress_mm, lateral_mm, yaw_mrad, total_yaw_mrad;
    int16_t duty[4];
    uint8_t state, legs, used, key, watchdog_reset;
} rectangle_snapshot_t;
static rectangle_snapshot_t snapshot;
#ifdef RECTANGLE_STARTUP_TRACE
static rectangle_startup_trace_t startup_trace;
#endif

static uint32_t enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void leave(uint32_t p) { __set_PRIMASK(p); }

static void controls_low(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_PULLDOWN; g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = control_e; HAL_GPIO_WritePin(GPIOE, g.Pin, GPIO_PIN_RESET); HAL_GPIO_Init(GPIOE, &g);
    g.Pin = control_b; HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_RESET); HAL_GPIO_Init(GPIOB, &g);
}

static void clear_motor_registers(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE(); __HAL_RCC_TIM9_CLK_ENABLE(); __HAL_RCC_TIM12_CLK_ENABLE();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER = TIM9->CCER = TIM12->CCER = 0U;
    TIM1->CR1 = TIM9->CR1 = TIM12->CR1 = 0U;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
    TIM9->CCR1 = TIM9->CCR2 = TIM12->CCR1 = TIM12->CCR2 = 0U;
}

static void clocks_init(void)
{
    RCC_OscInitTypeDef o = {0}; RCC_ClkInitTypeDef c = {0};
    __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    o.OscillatorType = RCC_OSCILLATORTYPE_HSE; o.HSEState = RCC_HSE_ON;
    o.PLL.PLLState = RCC_PLL_ON; o.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    o.PLL.PLLM = 8U; o.PLL.PLLN = 336U; o.PLL.PLLP = RCC_PLLP_DIV2; o.PLL.PLLQ = 7U;
    if (HAL_RCC_OscConfig(&o) != HAL_OK) Error_Handler();
    c.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider = RCC_SYSCLK_DIV1;
    c.APB1CLKDivider = RCC_HCLK_DIV4; c.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&c, FLASH_LATENCY_5) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(SysTick_IRQn, 1U, 0U);
}

static void key_and_led_init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    /* 天空星官方KEY=PA0，按下为高；不是RST或BOOT，也不使用PA15编码器脚。 */
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &g);
    g.Pin = STATUS_LED_PIN; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_Init(STATUS_LED_PORT, &g);
}

static void pwm_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef p = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = instance == TIM12 ? FOUR_SPEED_PWM_APB1_PSC : FOUR_SPEED_PWM_APB2_PSC;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP; timer->Init.Period = FOUR_SPEED_PWM_COUNTS - 1U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
    p.OCMode = TIM_OCMODE_PWM1; p.OCPolarity = p.OCNPolarity = TIM_OCPOLARITY_HIGH;
    p.OCFastMode = TIM_OCFAST_DISABLE;
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    for (unsigned i = 0; i < (instance == TIM1 ? 4U : 2U); ++i)
        if (HAL_TIM_PWM_ConfigChannel(timer, &p, channels[i]) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(timer);
}

static void encoder_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint8_t bits)
{
    TIM_Encoder_InitTypeDef e = {0};
    timer->Instance = instance; timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = bits == 32U ? UINT32_MAX : 0xFFFFU;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    e.EncoderMode = TIM_ENCODERMODE_TI12;
    e.IC1Polarity = e.IC2Polarity = TIM_ICPOLARITY_RISING;
    e.IC1Selection = e.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    e.IC1Prescaler = e.IC2Prescaler = TIM_ICPSC_DIV1;
    e.IC1Filter = e.IC2Filter = 6U;
    if (HAL_TIM_Encoder_Init(timer, &e) != HAL_OK) Error_Handler();
}

static void apply_outputs(void)
{
    uint32_t cc[8];
    if (!rectangle_demo_compares(&demo, cc)) Error_Handler();
    /* 全组校验后才写八路CCR；失败直接GPIO LOW，不提交部分轮。 */
    for (unsigned i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COMPARE(motor_timers[i], first_channels[i], cc[2U * i]);
        __HAL_TIM_SET_COMPARE(motor_timers[i], second_channels[i], cc[2U * i + 1U]);
    }
    snapshot.compares[0] = TIM9->CCR1; snapshot.compares[1] = TIM9->CCR2;
    snapshot.compares[2] = TIM1->CCR1; snapshot.compares[3] = TIM1->CCR2;
    snapshot.compares[4] = TIM1->CCR3; snapshot.compares[5] = TIM1->CCR4;
    snapshot.compares[6] = TIM12->CCR1; snapshot.compares[7] = TIM12->CCR2;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance != TIM6) return;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3U, 0U); HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}
void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance != TIM6) return;
    const uint32_t started = DWT->CYCCNT, now = HAL_GetTick(), dt = now - previous_irq_ms;
    previous_irq_ms = now;
    if (dt < snapshot.dt_min) snapshot.dt_min = dt;
    if (dt > snapshot.dt_max) snapshot.dt_max = dt;
    ++snapshot.ticks; snapshot.ms = now;
    int32_t counts[4];
    for (unsigned i = 0; i < 4; ++i) {
        encoder_accumulator_tick(&enc[i], __HAL_TIM_GET_COUNTER(encoder_timers[i]), now);
        counts[i] = encoder_accumulator_count(&enc[i]);
    }
    const bool pressed = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET;
#ifdef RECTANGLE_STARTUP_TRACE
    const uint8_t before_state = (uint8_t)demo.state;
#endif
    if (stop_pending) rectangle_demo_stop(&demo, RECT_FAULT_STOP, now);
    if (uart_failed) rectangle_demo_stop(&demo, RECT_FAULT_UART, now);
#ifdef RECTANGLE_STARTUP_TRACE
    (void)rectangle_startup_diag_tick(&demo, counts, pressed, now);
#else
    (void)rectangle_demo_tick(&demo, counts, pressed, now);
#endif
    apply_outputs();
#ifdef RECTANGLE_STARTUP_TRACE
    if (!rectangle_trace_capture(&startup_trace, &demo, counts, snapshot.compares,
                                 before_state, pressed, now)) {
        rectangle_demo_stop(&demo, RECT_TRACE_FAULT_DATA, now);
        apply_outputs();
    }
#endif
    memcpy(snapshot.counts, counts, sizeof(counts));
    memcpy(snapshot.duty, demo.group.raw_duty, sizeof(snapshot.duty));
    snapshot.state = (uint8_t)demo.state; snapshot.legs = demo.completed_legs;
    snapshot.fault = demo.fault; snapshot.used = demo.used; snapshot.key = pressed;
    snapshot.progress_mm = isfinite(demo.progress_m) ? (int32_t)lroundf(demo.progress_m * 1000.0f) : 0;
    snapshot.lateral_mm = isfinite(demo.lateral_m) ? (int32_t)lroundf(demo.lateral_m * 1000.0f) : 0;
    snapshot.yaw_mrad = isfinite(demo.yaw_rad) ? (int32_t)lroundf(demo.yaw_rad * 1000.0f) : 0;
    snapshot.total_yaw_mrad = isfinite(demo.total_yaw_rad) ? (int32_t)lroundf(demo.total_yaw_rad * 1000.0f) : 0;
    /* KEY是软件停机，不是独立硬件急停；现场异常仍切断动力。 */
    bool led = false;
    if (demo.state == RECT_MOVE || demo.state == RECT_COMPLETE) led = true;
    else if (demo.state == RECT_READY) led = (now / 500U) % 2U != 0U;
    else if (demo.state == RECT_HOLD || demo.state == RECT_ARMED_RELEASE ||
             demo.state == RECT_COUNTDOWN || demo.state == RECT_PAUSE) led = (now / 100U) % 2U != 0U;
    else if (demo.state == RECT_FAULT || demo.state == RECT_STOPPED) led = (now / 50U) % 2U != 0U;
    HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, led ? GPIO_PIN_SET : GPIO_PIN_RESET);
    const uint32_t us = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U);
    if (us > snapshot.irq_us_max) snapshot.irq_us_max = us;
}

static void watchdog_init(void)
{
    __HAL_DBGMCU_UNFREEZE_IWDG();
    LL_IWDG_Enable(IWDG); LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64); LL_IWDG_SetReloadCounter(IWDG, 249U);
    const uint32_t start = HAL_GetTick();
    while (!LL_IWDG_IsReady(IWDG)) if (HAL_GetTick() - start > 100U) Error_Handler();
    LL_IWDG_ReloadCounter(IWDG); /* LSI标称约500ms，未宣称实测时间。 */
}

static void publish(void)
{
    const uint32_t p = enter(); const rectangle_snapshot_t s = snapshot; leave(p);
    char line[SPEED_UART_FRAME_BYTES];
    const int n = snprintf(line, sizeof(line),
        "RECT,fw=" RECTANGLE_FIRMWARE ",ms=%lu,state=%u,legs=%u,used=%u,key=%u,fault=0x%08lx,"
        "progress_mm=%ld,lateral_mm=%ld,yaw_mrad=%ld,total_yaw_mrad=%ld,ticks=%lu,dt_min=%lu,dt_max=%lu,irq_us_max=%lu,"
        "wdg_reset=%u,fl=%ld,fr=%ld,rl=%ld,rr=%ld,d_fl=%d,d_fr=%d,d_rl=%d,d_rr=%d,"
        "p0=%lu,p1=%lu,p2=%lu,p3=%lu,p4=%lu,p5=%lu,p6=%lu,p7=%lu\r\n",
        (unsigned long)s.ms, s.state, s.legs, s.used, s.key, (unsigned long)s.fault,
        (long)s.progress_mm, (long)s.lateral_mm, (long)s.yaw_mrad, (long)s.total_yaw_mrad,
        (unsigned long)s.ticks, (unsigned long)s.dt_min, (unsigned long)s.dt_max,
        (unsigned long)s.irq_us_max, s.watchdog_reset,
        (long)s.counts[0], (long)s.counts[1], (long)s.counts[2], (long)s.counts[3],
        s.duty[0], s.duty[1], s.duty[2], s.duty[3],
        (unsigned long)s.compares[0], (unsigned long)s.compares[1],
        (unsigned long)s.compares[2], (unsigned long)s.compares[3],
        (unsigned long)s.compares[4], (unsigned long)s.compares[5],
        (unsigned long)s.compares[6], (unsigned long)s.compares[7]);
    if (n <= 0 || (size_t)n >= sizeof(line) || !speed_uart_send(line)) uart_failed = true;
}

int main(void)
{
    HAL_Init(); controls_low(); clear_motor_registers(); clocks_init(); key_and_led_init();
    snapshot.dt_min = UINT32_MAX;
    snapshot.watchdog_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    const int32_t zero_counts[4] = {0};
    (void)rectangle_demo_init(&demo, zero_counts, HAL_GetTick(), snapshot.watchdog_reset);
#ifdef RECTANGLE_STARTUP_TRACE
    rectangle_trace_init(&startup_trace);
#endif
    encoder_init(&htim3, TIM3, 16U); encoder_init(&htim4, TIM4, 16U);
    encoder_init(&htim2, TIM2, 32U); encoder_init(&htim8, TIM8, 16U);
    for (unsigned i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
        if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
        encoder_accumulator_init(&enc[i], i == 2U ? 32U : 16U, 0U, HAL_GetTick());
    }
    pwm_init(&htim9, TIM9); pwm_init(&htim1, TIM1); pwm_init(&htim12, TIM12);
    apply_outputs();
    for (unsigned i = 0; i < 4; ++i)
        if (HAL_TIM_PWM_Start(motor_timers[i], first_channels[i]) != HAL_OK ||
            HAL_TIM_PWM_Start(motor_timers[i], second_channels[i]) != HAL_OK) Error_Handler();
    huart2.Instance = USART2; huart2.Init.BaudRate = 115200U;
    huart2.Init.WordLength = UART_WORDLENGTH_8B; huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE; huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE; huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U); speed_uart_init(&huart2); watchdog_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    htim6.Instance = TIM6; htim6.Init.Prescaler = 8399U; htim6.Init.Period = 99U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    previous_irq_ms = demo.sampled_ms = demo.group.sampled_ms = HAL_GetTick();
    for (unsigned i = 0; i < 4; ++i) enc[i].sampled_ms = previous_irq_ms;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
#ifdef RECTANGLE_STARTUP_TRACE
    (void)speed_uart_send("READY," RECTANGLE_FIRMWARE ",key=PA0_high,first_leg_only=1,max_run_ms=5000\r\n");
#else
    (void)speed_uart_send("READY," RECTANGLE_FIRMWARE ",key=PA0_high,side=1000mm,loops=1\r\n");
#endif
    uint32_t last_publish = HAL_GetTick(), last_refreshed = 0;
    char command[SPEED_UART_LINE_BYTES];
    while (1) {
        speed_uart_service();
        if (speed_uart_fault()) uart_failed = true;
        if (speed_uart_take_line(command)) {
            if (!strcmp(command, "stop") || !strcmp(command, "estop")) stop_pending = true;
            else if (!strcmp(command, "status")) publish();
            else (void)speed_uart_send("ERR,only_status_stop_estop_allowed\r\n");
        }
        const uint32_t now = HAL_GetTick();
        if (now - last_publish >= 200U) { last_publish = now; publish(); }
        const uint32_t p = enter();
        const uint32_t ticks = snapshot.ticks, ms = snapshot.ms; leave(p);
        /* 同时要求主循环和100Hz采样存活；电脑断开是预期，不依赖外部心跳。 */
        if (ticks != last_refreshed && now - ms <= 30U) {
            last_refreshed = ticks; LL_IWDG_ReloadCounter(IWDG);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq(); controls_low(); clear_motor_registers();
    while (1) {} /* 看门狗重启后锁存故障；必须排查后重新上电，不会自动续跑。 */
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; Error_Handler(); }
#endif
