#ifndef SPEED_UART_TRANSPORT_H
#define SPEED_UART_TRANSPORT_H

#include "board.h"
#include "speed_uart_queue.h"

void speed_uart_init(UART_HandleTypeDef *uart);
bool speed_uart_take_line(char out[SPEED_UART_LINE_BYTES]);
bool speed_uart_send(const char *text);
void speed_uart_service(void);
bool speed_uart_fault(void);
#ifdef RR_DIAGNOSTIC
/* 停机导出每次只排一个帧，不能用发送失败来探测队列空间。 */
bool speed_uart_tx_idle(void);
#endif

#endif
