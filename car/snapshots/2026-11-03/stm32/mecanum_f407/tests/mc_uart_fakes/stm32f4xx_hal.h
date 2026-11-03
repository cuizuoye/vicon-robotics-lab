#ifndef MC_UART_FAKE_HAL_H
#define MC_UART_FAKE_HAL_H
#include <stdint.h>
typedef struct { uint32_t SR; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; uint32_t ErrorCode; } UART_HandleTypeDef;
#define HAL_OK 0
#define HAL_BUSY 2
#define HAL_UART_ERROR_NONE 0U
#define HAL_UART_ERROR_PE 1U
#define HAL_UART_ERROR_NE 2U
#define HAL_UART_ERROR_FE 4U
#define HAL_UART_ERROR_ORE 8U
#define __disable_irq() ((void)0)
#define __enable_irq() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(uart) ((void)(uart))
int HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_UART_Transmit(UART_HandleTypeDef *, const uint8_t *, uint16_t, uint32_t);
uint32_t HAL_GetTick(void);
#endif
