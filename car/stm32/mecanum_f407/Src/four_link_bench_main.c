#include "board.h"
#include "encoder_accumulator.h"
#ifdef DRIVE_BENCH
#include "drive_bench_control.h"
/* 仅新目标替换算法/协议，复用同一已验证外设执行器；旧前进镜像不改行为。 */
#define forward_bench_t drive_bench_t
#define forward_bench_init drive_bench_init
#define forward_bench_tick drive_bench_tick
#define forward_bench_accept drive_bench_accept
#define forward_bench_format drive_bench_format
#define four_forward_stop four_drive_stop
#ifdef RR_SPEED2
#define BENCH_READY "READY,rr_diag_328_v1\r\n"
#elif defined(RR_PWM_FINE)
#define BENCH_READY "READY,rr_diag_pwm01_v1\r\n"
#elif defined(RR_DIAGNOSTIC)
#define BENCH_READY "READY,rr_diag_start8_v1\r\n"
#elif defined(DRIVE_STARTUP_OBSERVE)
#define BENCH_READY "READY,drive_pi_start8_v1\r\n"
#else
#define BENCH_READY "READY,drive_pi_v1\r\n"
#endif
#else
#include "forward_bench_control.h"
#define BENCH_READY "READY,forward_pi_v1\r\n"
#endif
#include "four_link_bench.h"
#include "mc_uart_transport.h"
#if !defined(DRIVE_BENCH) || !defined(DRIVE_STARTUP_OBSERVE) || defined(RR_DIAGNOSTIC)
#error Wireless group images require the bounded startup protections.
#endif
#ifdef RR_DIAGNOSTIC
#include "rr_diagnostic_trace.h"
#include <stdio.h>
static rr_trace_t rr_trace;
static bool trace_dumping;
static unsigned trace_dump_index;
#endif
#include "stm32f4xx_ll_iwdg.h"

#include <limits.h>
#include <string.h>


TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim6, htim8, htim9, htim12;
UART_HandleTypeDef huart2;
static forward_bench_t bench;
static encoder_accumulator_t enc[4];
static speed_bench_diagnostics_t diagnostics;
static uint32_t previous_irq_ms;
static four_link_bench_t wireless;
static volatile bool link_uart_fault, request_pending, ack_pending, cache_ready;
static four_link_request_t pending_request;
static mc_frame_header_v1_t ack_header;
static mc_wheel_telemetry_v1_t cached_telemetry;
static four_link_bench_status_t cached_status;
static uint32_t cached_session, tx_sequence;
static const uint16_t control_e = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_9 |
    GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
static const uint16_t control_b = GPIO_PIN_14 | GPIO_PIN_15;
static TIM_HandleTypeDef *const encoder_timers[] = {&htim3, &htim4, &htim2, &htim8};
static TIM_HandleTypeDef *const motor_timers[] = {&htim9, &htim1, &htim1, &htim12};
static const uint32_t first_channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_1, TIM_CHANNEL_3, TIM_CHANNEL_1};
static const uint32_t second_channels[] = {TIM_CHANNEL_2, TIM_CHANNEL_2, TIM_CHANNEL_4, TIM_CHANNEL_2};

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

static void pwm_init(TIM_HandleTypeDef *timer, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef p = {0};
    timer->Instance = instance;
    timer->Init.Prescaler = instance == TIM12 ? FOUR_SPEED_PWM_APB1_PSC : FOUR_SPEED_PWM_APB2_PSC;
    timer->Init.CounterMode = TIM_COUNTERMODE_UP;
    timer->Init.Period = FOUR_SPEED_PWM_COUNTS - 1U;
    timer->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(timer) != HAL_OK) Error_Handler();
#ifdef RR_PWM_FINE
    if (instance->ARR != FOUR_SPEED_PWM_COUNTS - 1U ||
        instance->PSC != timer->Init.Prescaler) Error_Handler();
#endif
    p.OCMode = TIM_OCMODE_PWM1;
    p.OCPolarity = TIM_OCPOLARITY_HIGH;
    p.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    p.OCFastMode = TIM_OCFAST_DISABLE;
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    for (unsigned i = 0; i < (instance == TIM1 ? 4U : 2U); ++i)
        if (HAL_TIM_PWM_ConfigChannel(timer, &p, channels[i]) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(timer);
}

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
    uint32_t compares[8] = {0};
#ifdef DRIVE_BENCH
    if (!four_drive_compares(&bench.group, compares)) Error_Handler();
#else
    const int signs[] = {1, -1, 1, -1};
    for (unsigned i = 0; i < 4; ++i) {
        if (!four_speed_compare(bench.group.raw_duty[i], &compares[2U * i], &compares[2U * i + 1U]) ||
            (bench.group.raw_duty[i] != 0 && !bench.group.enabled) ||
            bench.group.raw_duty[i] * signs[i] < 0) Error_Handler();
    }
#endif
    /* 先校验四路，再在IRQ/临界区应用八路CCR；不能边算边写单轮PWM。 */
    for (unsigned i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COMPARE(motor_timers[i], first_channels[i], compares[2U * i]);
        __HAL_TIM_SET_COMPARE(motor_timers[i], second_channels[i], compares[2U * i + 1U]);
    }
    memcpy(diagnostics.compares, compares, sizeof(compares));
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
    /* 执行器/会话只由100Hz ISR拥有；UART ISR优先级更高，可抢占浮点控制计算。 */
    if (link_uart_fault) {
        wireless.fault |= SPEED_BENCH_FAULT_UART;
        wireless.permitted = false;
        four_drive_stop(&bench.group, SPEED_BENCH_FAULT_UART, now);
    }
    four_link_bench_deadline(&wireless, &bench, now);
    if (request_pending) {
        const four_link_request_t request = pending_request;
        request_pending = false;
        if (four_link_bench_process(&wireless, &bench, &request, now)) {
            ack_header = request.header;
            ack_pending = true;
        }
    }
    (void)forward_bench_tick(&bench, counts, now);
    four_link_bench_deadline(&wireless, &bench, now);
    apply_outputs();
    diagnostics.compares[0] = TIM9->CCR1; diagnostics.compares[1] = TIM9->CCR2;
    diagnostics.compares[2] = TIM1->CCR1; diagnostics.compares[3] = TIM1->CCR2;
    diagnostics.compares[4] = TIM1->CCR3; diagnostics.compares[5] = TIM1->CCR4;
    diagnostics.compares[6] = TIM12->CCR1; diagnostics.compares[7] = TIM12->CCR2;
    if (!four_link_bench_payloads(&wireless, &bench, &diagnostics,
                                 &cached_telemetry, &cached_status)) Error_Handler();
    cached_session = wireless.session_active ? wireless.session_id : 0;
    cache_ready = true;
#ifdef RR_DIAGNOSTIC
    /* 记录实际寄存器，不在ISR内格式化/发送。停机后冻结一次记录。 */
    const uint32_t cc[8] = {TIM9->CCR1, TIM9->CCR2, TIM1->CCR1, TIM1->CCR2,
                            TIM1->CCR3, TIM1->CCR4, TIM12->CCR1, TIM12->CCR2};
    if (!rr_trace_capture(&rr_trace, &bench, cc, now)) {
        four_drive_stop(&bench.group, FOUR_SPEED_FAULT_FEEDBACK, now);
        apply_outputs();
    }
#endif
    const uint32_t duration = (DWT->CYCCNT - started) / (SystemCoreClock / 1000000U);
    if (duration > diagnostics.irq_us_max) diagnostics.irq_us_max = duration;
    cached_status.irq_us_max = diagnostics.irq_us_max;
}

static void watchdog_init(void)
{
    __HAL_DBGMCU_UNFREEZE_IWDG();
    LL_IWDG_Enable(IWDG);
    LL_IWDG_EnableWriteAccess(IWDG);
    LL_IWDG_SetPrescaler(IWDG, LL_IWDG_PRESCALER_64);
    LL_IWDG_SetReloadCounter(IWDG, 249U); /* LSI标称32kHz时约500ms，不是实测期限。 */
    const uint32_t start = HAL_GetTick();
    while (!LL_IWDG_IsReady(IWDG)) if (HAL_GetTick() - start > 100U) Error_Handler();
    LL_IWDG_ReloadCounter(IWDG);
}

static void publish(const char *prefix)
{
    (void)prefix;
    mc_wheel_telemetry_v1_t telemetry;
    four_link_bench_status_t status;
    const uint32_t p=enter();
    const bool ready=cache_ready;
    const uint32_t session=cached_session;
    telemetry=cached_telemetry; status=cached_status;
    leave(p); /* 仅复制140字节缓存；CRC/浮点计算不屏蔽接收IRQ。 */
    if (!ready || !session) return;
    const uint16_t flags=(telemetry.fault_bits ? MC_FLAG_FAULT : 0) |
                         ((telemetry.state_flags & FOUR_LINK_ENABLED) ? MC_FLAG_ENABLED : 0);
    if (!mc_uart_transport_send_frame(MC_MSG_WHEEL_TELEMETRY,flags,++tx_sequence,
           session,telemetry.sample_time_ms,&telemetry,sizeof(telemetry)) ||
        !mc_uart_transport_send_frame(MC_MSG_LINK_STATUS,flags,++tx_sequence,
           session,telemetry.sample_time_ms,&status,sizeof(status))) link_uart_fault=true;
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
    forward_bench_init(&bench, zero_counts, HAL_GetTick());
    four_link_bench_init(&wireless);
    if (diagnostics.watchdog_reset) four_forward_stop(&bench.group, FOUR_SPEED_FAULT_TIMING, HAL_GetTick());
    encoder_init(&htim3, TIM3, 16U); encoder_init(&htim4, TIM4, 16U);
    encoder_init(&htim2, TIM2, 32U); encoder_init(&htim8, TIM8, 16U);
    for (unsigned i = 0; i < 4; ++i) {
        __HAL_TIM_SET_COUNTER(encoder_timers[i], 0U);
        if (HAL_TIM_Encoder_Start(encoder_timers[i], TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
        encoder_accumulator_init(&enc[i], i == 2U ? 32U : 16U, 0U, HAL_GetTick());
    }
    pwm_init(&htim9, TIM9); pwm_init(&htim1, TIM1); pwm_init(&htim12, TIM12);
    apply_outputs();
    for (unsigned i = 0; i < 4; ++i) {
        if (HAL_TIM_PWM_Start(motor_timers[i], first_channels[i]) != HAL_OK ||
            HAL_TIM_PWM_Start(motor_timers[i], second_channels[i]) != HAL_OK) Error_Handler();
    }
    huart2.Instance = USART2;
    huart2.Init.BaudRate = MC_UART_BAUD_RATE;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(USART2_IRQn, 2U, 0U);
    if (!mc_protocol_self_test()) Error_Handler();
    mc_uart_transport_init(&huart2);
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
    previous_irq_ms = bench.sampled_ms = bench.group.sampled_ms = HAL_GetTick();
    for (unsigned i = 0; i < 4; ++i) enc[i].sampled_ms = previous_irq_ms;
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
    /* 默认零PWM；新握手+静止零速+显式门控后才接受对应镜像的速度范围。
     * 默认镜像仍只接受固定F50；新流式镜像必须显式选择独立构建目标。 */
    uint32_t last_telemetry = HAL_GetTick(), last_refreshed = 0;
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];
    while (1) {
        if (mc_uart_transport_take_fault()) link_uart_fault=true;
        if (!request_pending) {
            size_t length=0;
            if (mc_uart_transport_take_frame(raw,sizeof(raw),&length)) {
                four_link_request_t decoded;
                if (four_link_bench_decode(raw,length,&decoded)) {
                    const uint32_t p=enter();
                    pending_request=decoded; request_pending=true;
                    leave(p);
                }
            }
        }
        if (ack_pending) {
            const uint32_t p=enter();
            const mc_frame_header_v1_t ack=ack_header;
            ack_pending=false;
            leave(p);
            if (!mc_uart_transport_send_frame(MC_MSG_HELLO_ACK,MC_FLAG_RESPONSE,
                ack.sequence,ack.session_id,HAL_GetTick(),NULL,0)) link_uart_fault=true;
        }
        const uint32_t now = HAL_GetTick();
#ifdef RR_DIAGNOSTIC
        if (trace_dumping && speed_uart_tx_idle()) {
            char frame[256];
            if (trace_dump_index < rr_trace.count) {
                if (rr_trace_format(&rr_trace, trace_dump_index, frame, sizeof(frame))) {
                    if (speed_uart_send(frame)) ++trace_dump_index;
                } else {
                    (void)speed_uart_send("ERR,trace_format\r\n");
                    trace_dumping = false;
                }
            } else {
                (void)snprintf(frame, sizeof(frame), "RREND,n=%u,overflow=%u\r\n",
                    (unsigned)rr_trace.count, rr_trace.overflow ? 1U : 0U);
                (void)speed_uart_send(frame);
                trace_dumping = false;
            }
        }
        /* 导出只发生在停轮后，期间暂停重复遥测以避免占满115200带宽。 */
        if (!trace_dumping && now - last_telemetry >= 100U) {
#else
        if (now - last_telemetry >= 100U) {
#endif
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


