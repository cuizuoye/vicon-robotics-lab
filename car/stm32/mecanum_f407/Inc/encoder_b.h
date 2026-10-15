#ifndef ENCODER_B_H
#define ENCODER_B_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

void encoder_b_init(TIM_HandleTypeDef *timer, uint32_t now_ms);
void encoder_b_tick(uint32_t now_ms);
void encoder_b_zero(void);
int32_t encoder_b_count(void);
int32_t encoder_b_counts_per_second(void);

#endif
