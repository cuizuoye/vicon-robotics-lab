#ifndef SPEED_UART_QUEUE_H
#define SPEED_UART_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SPEED_UART_RX_SLOTS 4U
#define SPEED_UART_LINE_BYTES 64U
#define SPEED_UART_TX_SLOTS 4U
#ifndef SPEED_UART_FRAME_BYTES
#define SPEED_UART_FRAME_BYTES 896U
#endif

/* 队列的跨ISR操作由硬件层保存/恢复PRIMASK，纯算法测试不接硬件。 */
typedef struct {
    char working[SPEED_UART_LINE_BYTES];
    char lines[SPEED_UART_RX_SLOTS][SPEED_UART_LINE_BYTES];
    uint16_t working_length;
    uint8_t rx_head, rx_tail, rx_count;
    bool drop_line;
    bool fault;
    char frames[SPEED_UART_TX_SLOTS][SPEED_UART_FRAME_BYTES];
    uint16_t lengths[SPEED_UART_TX_SLOTS];
    uint8_t tx_head, tx_tail, tx_count;
} speed_uart_queue_t;

void speed_queue_init(speed_uart_queue_t *q);
void speed_queue_receive(speed_uart_queue_t *q, uint8_t byte);
bool speed_queue_take_line(speed_uart_queue_t *q, char out[SPEED_UART_LINE_BYTES]);
bool speed_queue_send(speed_uart_queue_t *q, const char *text, size_t length);
const char *speed_queue_peek_tx(const speed_uart_queue_t *q, uint16_t *length);
void speed_queue_finish_tx(speed_uart_queue_t *q);

#endif
