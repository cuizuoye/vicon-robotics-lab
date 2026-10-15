#ifndef UART_FAKE_HAL_H
#define UART_FAKE_HAL_H
#include <stdint.h>
typedef struct { int id; } UART_HandleTypeDef;
#define HAL_OK 0
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
int HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
#endif
