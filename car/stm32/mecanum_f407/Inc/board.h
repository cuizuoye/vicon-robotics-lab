#ifndef BOARD_H
#define BOARD_H

#include "stm32f4xx_hal.h"

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim9;
extern UART_HandleTypeDef huart2;

#define STATUS_LED_PORT GPIOB
#define STATUS_LED_PIN  GPIO_PIN_2

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer);
void Error_Handler(void);

#endif

