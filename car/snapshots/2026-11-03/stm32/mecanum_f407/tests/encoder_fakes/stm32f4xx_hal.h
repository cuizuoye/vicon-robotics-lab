#ifndef ENCODER_FAKE_HAL_H
#define ENCODER_FAKE_HAL_H
#include <stdint.h>
typedef struct { uint32_t counter; } TIM_HandleTypeDef;
#define TIM_CHANNEL_ALL 0xffffU
#define HAL_OK 0
#define __HAL_TIM_SET_COUNTER(timer, value) ((timer)->counter = (value))
#define __HAL_TIM_GET_COUNTER(timer) ((timer)->counter)
int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channels);
#endif
