#include "encoder_b.h"

#include "app_config.h"
#include "board.h"

/* B使用独立TIM4与状态，不改动已验证的A/TIM3实现。 */
static TIM_HandleTypeDef *encoder_timer;
static uint16_t previous_counter;
static uint32_t previous_sample_ms;
static int32_t total_count;
static int32_t counts_per_second;

void encoder_b_init(TIM_HandleTypeDef *timer, uint32_t now_ms)
{
    encoder_timer = timer;
    __HAL_TIM_SET_COUNTER(encoder_timer, 0U);
    if (HAL_TIM_Encoder_Start(encoder_timer, TIM_CHANNEL_ALL) != HAL_OK) {
        Error_Handler();
    }
    previous_counter = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
    previous_sample_ms = now_ms;
    total_count = 0;
    counts_per_second = 0;
}

void encoder_b_tick(uint32_t now_ms)
{
    const uint32_t elapsed_ms = now_ms - previous_sample_ms;
    if (elapsed_ms < ENCODER_SAMPLE_PERIOD_MS) {
        return;
    }
    const uint16_t counter = (uint16_t)__HAL_TIM_GET_COUNTER(encoder_timer);
    const int16_t delta = (int16_t)(counter - previous_counter);
    total_count += delta;
    counts_per_second = (int32_t)(((int64_t)delta * 1000) / elapsed_ms);
    previous_counter = counter;
    previous_sample_ms = now_ms;
}

void encoder_b_zero(void)
{
    __HAL_TIM_SET_COUNTER(encoder_timer, 0U);
    previous_counter = 0U;
    total_count = 0;
    counts_per_second = 0;
}

int32_t encoder_b_count(void) { return total_count; }
int32_t encoder_b_counts_per_second(void) { return counts_per_second; }
