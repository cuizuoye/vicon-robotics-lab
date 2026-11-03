#include "encoder_a.h"
#include "encoder_b.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>

int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels)
{
    assert(timer != NULL && channels == TIM_CHANNEL_ALL);
    return HAL_OK;
}
void Error_Handler(void) { assert(!"unexpected encoder start failure"); }

int main(void)
{
    TIM_HandleTypeDef a = {0}, b = {0};
    encoder_a_init(&a, 0U);
    encoder_b_init(&b, 0U);
    a.counter = 10U;
    encoder_a_tick(9U);
    assert(encoder_a_count() == 0);
    encoder_a_tick(10U);
    encoder_b_tick(10U);
    assert(encoder_a_count() == 10 && encoder_a_counts_per_second() == 1000);
    assert(encoder_b_count() == 0 && encoder_b_counts_per_second() == 0);
    b.counter = 65530U;
    encoder_b_tick(20U);
    assert(encoder_b_count() == -6 && encoder_b_counts_per_second() == -600);
    b.counter = 4U;
    encoder_b_tick(30U);
    assert(encoder_b_count() == 4 && encoder_b_counts_per_second() == 1000);
    b.counter = 65534U;
    encoder_b_tick(40U);
    assert(encoder_b_count() == -2 && encoder_b_counts_per_second() == -600);
    assert(encoder_a_count() == 10);
    encoder_b_zero();
    assert(b.counter == 0U && encoder_b_count() == 0 && encoder_a_count() == 10);
    encoder_a_zero();
    assert(encoder_a_count() == 0 && encoder_b_count() == 0);
    encoder_a_init(&a, UINT32_MAX - 4U);
    encoder_b_init(&b, UINT32_MAX - 4U);
    a.counter = 3U;
    b.counter = 65533U;
    encoder_a_tick(5U);
    encoder_b_tick(5U);
    assert(encoder_a_count() == 3 && encoder_a_counts_per_second() == 300);
    assert(encoder_b_count() == -3 && encoder_b_counts_per_second() == -300);
    encoder_a_tick(15U);
    encoder_b_tick(15U);
    assert(encoder_a_counts_per_second() == 0 && encoder_b_counts_per_second() == 0);
    puts("encoder pair: independent state, direction, 16-bit counter and ms wrap passed (offline)");
    return 0;
}
