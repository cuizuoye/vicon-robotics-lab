#ifndef UART_CONSOLE_H
#define UART_CONSOLE_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>

void uart_console_init(UART_HandleTypeDef *uart);
bool uart_console_take_line(char *line, size_t capacity);
bool uart_console_take_overflow(void);
void uart_console_write(const char *text);
void uart_console_printf(const char *format, ...);

#endif

