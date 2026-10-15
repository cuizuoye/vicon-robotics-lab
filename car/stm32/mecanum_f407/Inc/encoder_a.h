#ifndef ENCODER_A_H
#define ENCODER_A_H

#include "stm32f4xx_hal.h"

#include <stdint.h>

void encoder_a_init(TIM_HandleTypeDef *timer, uint32_t now_ms);
void encoder_a_tick(uint32_t now_ms);
void encoder_a_zero(void);
int32_t encoder_a_count(void);
int32_t encoder_a_counts_per_second(void);

#endif

