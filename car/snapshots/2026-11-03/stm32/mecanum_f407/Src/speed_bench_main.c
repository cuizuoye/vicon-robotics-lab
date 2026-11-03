#include "board.h"
#include "encoder_accumulator.h"
#include "speed_bench_control.h"
#ifdef FOUR_LINK_READONLY
#include "four_link_readonly.h"
#include "mc_uart_transport.h"
#if !SPEED_BENCH_READONLY
#error Four-wheel MC-Link readonly must not initialize PWM.
#endif
static four_ro_control_t ro_link;
static volatile bool ro_uart_fault;
static uint32_t ro_tx_sequence;
#else
#include "speed_uart_transport.h"
#endif
#include "stm32f4xx_ll_iwdg.h"

#include <limits.h>
#include <string.h>

#ifndef SPEED_BENCH_READONLY
#error "Choose a separate read-only or suspended speed-bench target"
#endif

TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim6, htim8, htim9, htim12;
UART_HandleTypeDef huart2;
static speed_bench_t bench;
static encoder_accumulator_t enc[4];
static speed_bench_diagnostics_t diagnostics;
static uint32_t previous_irq_ms;
static const uint16_t control_e = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 |
    GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
static const uint16_t control_b = GPIO_PIN_14 | GPIO_PIN_15;
static TIM_HandleTypeDef *const encoder_timers[] = {&htim3, &htim4, &htim2, &htim8};
#if !SPEED_BENCH_READONLY
static TIM_HandleTypeDef *const motor_timers[] = {&htim9, &htim1, &htim1, &htim12};
static const uint32_t first_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_1};
static const uint32_t second_channels[] = {TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2};
#endif

static uint32_t enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void leave(uint32_t p) { __set_PRIMASK(p); }

static void controls_low(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin = control_e;
    HAL_GPIO_WritePin(GPIOE, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = control_b;
    HAL_GPIO_WritePin(GPIOB, g.Pin, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &g);
}

static void clear_motor_registers(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER = TIM9->CCER = TIM12->CCER = 0U;
    TIM1->CR1 = TIM9->CR1 = TIM12->CR1 = 0U;
    TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = TIM1->CCR4 = 0U;
    TIM9->CCR1 = TIM9->CCR2 = 0U;
    TIM12->CCR1 = TIM12->CCR2 = 0U;
}

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
    HAL_NVIC_SetPriority(SysTick_IRQn, 1U, 0U);
}

#if !SPEED_BENCH_READONLY
static void pwm_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef p = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = instance == TIM12 ? 83U : 167U;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = 99U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
    p.OCMode = TIM_OCMODE_PWM1;
    p.OCPolarity = TIM_OCPOLARITY_HIGH;
    p.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    p.OCFastMode = TIM_OCFAST_DISABLE;
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    for (unsigned i = 0; i < (instance == TIM1 ? 4U : 2U); ++i)
        if (HAL_TIM_PWM_ConfigChannel(timer, &p, channels[i]) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(timer);
}
#endif

static void encoder_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance, uint8_t bits)
{
    TIM_Encoder_InitTypeDef e = {0};
    timer->Instance = instance;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
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
    for (unsigned i = 0; i < 4; ++i) {
        uint32_t a = 0, b = 0;
#if !SPEED_BENCH_READONLY
        if (!four_speed_compare(bench.pi.wheel[i].raw_duty_permille, &a, &b) ||
            (bench.pi.wheel[i].raw_duty_permille != 0 &&
             (!bench.pi.enabled || bench.pi.selected != i))) Error_Handler();
        __HAL_TIM_SET_COMPARE(motor_timers[i], first_channels[i], a);
        __HAL_TIM_SET_COMPARE(motor_timers[i], second_channels[i], b);
#else
        /* 只读镜像未配置任何PWM复用；这里也独立禁止非零算法输出。 */
        if (bench.pi.enabled || bench.pi.wheel[i].raw_duty_permille != 0) Error_Handler();
#endif
        diagnostics.compares[2U * i] = a;
        diagnostics.compares[2U * i + 1U] = b;
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance != TIM6) return;
    __HAL_RCC_TIM6_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 3U, 0U);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void) { HAL_TIM_IRQHandler(&htim6); }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer->Instance != TIM6) return;
    const uint32_t started = DWT->CYCCNT;
    const uint32_t now = HAL_GetTick();
    const uint32_t dt = now - previous_irq_ms;
    previous_irq_ms = now;
    if (dt < diagnostics.dt_min) diagnostics.dt_min = dt;
    if (dt > diagnostics.dt_max) diagnostics.dt_max = dt;
    ++diagnostics.ticks;
    diagnostics.ms = now;
    int32_t counts[4];
    for (unsigned i = 0; i < 4; ++i) {
        encoder_accumulator_tick(&enc[i], __HAL_TIM_GET_COUNTER(encoder_timers[i]), now);
        counts[i] = encoder_accumulator_count(&enc[i]);
    }
#ifdef FOUR_LINK_READONLY
    if (ro_uart_fault) four_speed_stop(&bench.pi, SPEED_BENCH_FAULT_UART);
#else
    if (speed_uart_fault()) four_speed_stop(&bench.pi, SPEED_BENCH_FAULT_UART);
#endif
    (void)speed_bench_tick(&bench, counts, now);
#ifndef FOUR_LINK_READONLY
    if (speed_uart_fault()) four_speed_stop(&bench.pi, SPEED_BENCH_FAULT_UART);
#endif
    apply_outputs();
    const uint32_t duration = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U);
    if (duration > diagnostics.irq_us_max) diagnostics.irq_us_max = duration;
}

static void watchdog_init(void)
{
#if SPEED_BENCH_READONLY
    /* 只读始终GPIO低，可安全冻结调试看门狗，便于下一次断动力烧录。 */
    __HAL_DBGMCU_FREEZE_IWDG();
#else
    __HAL_DBGMCU_UNFREEZE_IWDG();
#endif
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
    LL_IWDG_SetReloadCounter(IWDG, 249U); /* LSI标称32kHz时约500ms，不是实测期限。 */
    const uint32_t start = HAL_GetTick();
    while (!LL_IWDG_IsReady(IWDG)) if (HAL_GetTick() - start > 100U) Error_Handler();
    LL_IWDG_ReloadCounter(IWDG);
}

static bool control_pins_low(void)
{
    const unsigned e_pins[] = {5, 6, 9, 11, 13, 14};
    for (unsigned i = 0; i < 6; ++i)
        if (((GPIOE->MODER >> (2U * e_pins[i])) & 3U) != 1U) return false;
    for (unsigned i = 14; i <= 15; ++i)
        if (((GPIOB->MODER >> (2U * i)) & 3U) != 1U) return false;
    return (GPIOE->ODR & control_e) == 0U && (GPIOB->ODR & control_b) == 0U;
}

static void publish(const char *prefix)
{
    speed_bench_t snapshot;
    speed_bench_diagnostics_t d;
    uint32_t p = enter();
    snapshot = bench;
    d = diagnostics;
    d.gpio_low = control_pins_low();
    /* 读取实际CCR，日志里的算法请求不替代定时器寄存器回报。 */
    d.compares[0] = TIM9->CCR1; d.compares[1] = TIM9->CCR2;
    d.compares[2] = TIM1->CCR1; d.compares[3] = TIM1->CCR2;
    d.compares[4] = TIM1->CCR3; d.compares[5] = TIM1->CCR4;
    d.compares[6] = TIM12->CCR1; d.compares[7] = TIM12->CCR2;
    leave(p);
#ifdef FOUR_LINK_READONLY
    (void)prefix;
    if (!ro_link.session_active) return;
    mc_wheel_telemetry_v1_t telemetry; four_ro_status_t status;
    if (!four_ro_payloads(&ro_link,&snapshot,&d,&telemetry,&status)) Error_Handler();
    const uint16_t flags=telemetry.fault_bits ? MC_FLAG_FAULT : 0;
    if (!mc_uart_transport_send_frame(MC_MSG_WHEEL_TELEMETRY,flags,++ro_tx_sequence,
            ro_link.session_id,d.ms,&telemetry,sizeof(telemetry)) ||
        !mc_uart_transport_send_frame(MC_MSG_LINK_STATUS,flags,++ro_tx_sequence,
            ro_link.session_id,d.ms,&status,sizeof(status))) {
        ro_uart_fault=true; ro_link.fault|=FOUR_RO_UART;
    }
#else
    char frame[SPEED_UART_FRAME_BYTES];
    if (!speed_bench_format(&snapshot, &d, prefix, frame, sizeof(frame)) || !speed_uart_send(frame)) {
        p = enter();
        four_speed_stop(&bench.pi, SPEED_BENCH_FAULT_UART);
        apply_outputs();
        leave(p);
    }
#endif
}

int main(void)
{
    HAL_Init();
    controls_low();
    clear_motor_registers();
    clocks_init();
    memset(&diagnostics, 0, sizeof(diagnostics));
    diagnostics.dt_min = UINT32_MAX;
    diagnostics.watchdog_reset = __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET;
    __HAL_RCC_CLEAR_RESET_FLAGS();
    const int32_t zero_counts[4] = {0};
    speed_bench_init(&bench, SPEED_BENCH_READONLY != 0, zero_counts, HAL_GetTick());
    if (diagnostics.watchdog_reset) four_speed_stop(&bench.pi, FOUR_SPEED_FAULT_TIMING);
    encoder_init(&htim3, TIM3, 16U); encoder_init(&htim4, TIM4, 16U);
    encoder_init(&htim2, TIM2, 32U); encoder_init(&htim8, TIM8, 16U);
    for (unsigned i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
        if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
        encoder_accumulator_init(&enc[i], i == 2U ? 32U : 16U, 0U, HAL_GetTick());
    }
#if !SPEED_BENCH_READONLY
    pwm_init(&htim9, TIM9); pwm_init(&htim1, TIM1); pwm_init(&htim12, TIM12);
    apply_outputs();
    for (unsigned i = 0; i < 4; ++i) {
        if (HAL_TIM_PWM_Start(motor_timers[i], first_channels[i]) != HAL_OK ||
            HAL_TIM_PWM_Start(motor_timers[i], second_channels[i]) != HAL_OK) Error_Handler();
    }
#endif
    huart2.Instance = USART2;
#ifdef FOUR_LINK_READONLY
    huart2.Init.BaudRate = MC_UART_BAUD_RATE;
#else
    huart2.Init.BaudRate = 115200U;
#endif
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U);
#ifdef FOUR_LINK_READONLY
    four_ro_init(&ro_link);
    if (!mc_protocol_self_test()) Error_Handler();
    mc_uart_transport_init(&huart2);
    mc_uart_transport_enable_startup_line_error_guard();
#else
    speed_uart_init(&huart2);
#endif
    watchdog_init();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
    htim6.Instance = TIM6;
    htim6.Init.Prescaler = 8399U; /* APB1 timer 84MHz /8400 /100 =100Hz。 */
    htim6.Init.Period = 99U;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) Error_Handler();
    __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
    previous_irq_ms = bench.sampled_ms = HAL_GetTick();
    for (unsigned i = 0; i < 4; ++i) enc[i].sampled_ms = previous_irq_ms;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
#ifndef FOUR_LINK_READONLY
    (void)speed_uart_send(SPEED_BENCH_READONLY ? "READY,speed_ro_v1\r\n" : "READY,speed_pi_v2\r\n");
#endif
    uint32_t last_telemetry = HAL_GetTick(), last_refreshed = 0;
#ifdef FOUR_LINK_READONLY
    uint8_t raw_frame[MC_MAX_RAW_FRAME_SIZE];
#else
    char line[SPEED_UART_LINE_BYTES];
#endif
    while (1) {
#ifdef FOUR_LINK_READONLY
        if (mc_uart_transport_take_fault()) {
            ro_uart_fault=true; ro_link.fault|=FOUR_RO_UART;
        }
        size_t raw_length=0;
        if (mc_uart_transport_take_frame(raw_frame,sizeof(raw_frame),&raw_length)) {
            mc_frame_header_v1_t validated_header;
            if (mc_validate_frame(raw_frame,raw_length,&validated_header))
                mc_uart_transport_confirm_valid_frame();
            const four_ro_event_t event=four_ro_accept(&ro_link,raw_frame,raw_length,HAL_GetTick());
            if (event==FOUR_RO_HELLO && !mc_uart_transport_send_frame(
                MC_MSG_HELLO_ACK,MC_FLAG_RESPONSE,ro_link.last_sequence,
                ro_link.session_id,HAL_GetTick(),NULL,0)) {
                ro_uart_fault=true; ro_link.fault|=FOUR_RO_UART;
            }
        }
        (void)four_ro_tick(&ro_link,HAL_GetTick());
#else
        speed_uart_service();
        if (speed_uart_take_line(line)) {
            const uint32_t p = enter();
            const char *reply = speed_bench_accept(&bench, line, HAL_GetTick());
            apply_outputs();
            leave(p);
            if (reply != NULL) (void)speed_uart_send(reply);
            if (strcmp(line, "status") == 0) publish("STATUSSP");
        }
#endif
        const uint32_t now = HAL_GetTick();
        if (now - last_telemetry >= 100U) {
            last_telemetry = now;
            publish("TELSP");
        }
        const uint32_t p = enter();
        const uint32_t ticks = diagnostics.ticks, sample_ms = diagnostics.ms;
        leave(p);
        /* 主循环及独立采样都必须推进；停IRQ不能靠主循环反复喂狗掩盖。 */
        if (ticks != last_refreshed && now - sample_ms <= 30U) {
            last_refreshed = ticks;
            LL_IWDG_ReloadCounter(IWDG);
        }
    }
}

void Error_Handler(void)
{
    __disable_irq();
    controls_low();
    clear_motor_registers();
    while (1) {} /* 已启动的IWDG可复位，但任何重新启动仍默认不使能。 */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { (void)file; (void)line; Error_Handler(); }
#endif
