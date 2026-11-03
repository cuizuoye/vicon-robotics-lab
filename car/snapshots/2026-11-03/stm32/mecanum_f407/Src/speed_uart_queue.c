#include "speed_uart_queue.h"

#include <string.h>

void speed_queue_init(speed_uart_queue_t *q) { memset(q, 0, sizeof(*q)); }

void speed_queue_receive(speed_uart_queue_t *q, uint8_t byte)
{
    if (byte == '\r' || byte == '\n') {
        if (!q->drop_line && q->working_length != 0U) {
            if (q->rx_count >= SPEED_UART_RX_SLOTS) q->fault = true;
            else {
                memcpy(q->lines[q->rx_tail], q->working, q->working_length);
                q->lines[q->rx_tail][q->working_length] = '\0';
                q->rx_tail = (uint8_t)((q->rx_tail + 1U) % SPEED_UART_RX_SLOTS);
                ++q->rx_count;
            }
        }
        q->working_length = 0U;
        q->drop_line = false;
    } else if (q->drop_line) {
        return;
    } else if (byte < 32U || byte > 126U || q->working_length >= SPEED_UART_LINE_BYTES - 1U) {
        /* 非ASCII/过长帧整行丢弃，不将尾部重新解释成有效运动命令。 */
        q->fault = true;
        q->drop_line = true;
        q->working_length = 0U;
    } else {
        q->working[q->working_length++] = (char)byte;
    }
}

bool speed_queue_take_line(speed_uart_queue_t *q, char out[SPEED_UART_LINE_BYTES])
{
    if (q->rx_count == 0U) return false;
    memcpy(out, q->lines[q->rx_head], SPEED_UART_LINE_BYTES);
    q->rx_head = (uint8_t)((q->rx_head + 1U) % SPEED_UART_RX_SLOTS);
    --q->rx_count;
    return true;
}

bool speed_queue_send(speed_uart_queue_t *q, const char *text, size_t length)
{
    if (text == NULL || length == 0U || length >= SPEED_UART_FRAME_BYTES ||
        q->tx_count >= SPEED_UART_TX_SLOTS) {
        q->fault = true;
        return false;
    }
    memcpy(q->frames[q->tx_tail], text, length);
    q->frames[q->tx_tail][length] = '\0';
    q->lengths[q->tx_tail] = (uint16_t)length;
    q->tx_tail = (uint8_t)((q->tx_tail + 1U) % SPEED_UART_TX_SLOTS);
    ++q->tx_count;
    return true;
}

const char *speed_queue_peek_tx(const speed_uart_queue_t *q, uint16_t *length)
{
    if (q->tx_count == 0U) { *length = 0U; return NULL; }
    *length = q->lengths[q->tx_head];
    return q->frames[q->tx_head];
}

void speed_queue_finish_tx(speed_uart_queue_t *q)
{
    if (q->tx_count == 0U) { q->fault = true; return; }
    q->tx_head = (uint8_t)((q->tx_head + 1U) % SPEED_UART_TX_SLOTS);
    --q->tx_count;
}
