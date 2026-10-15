#include "speed_uart_transport.h"

#include <string.h>

static speed_uart_queue_t queue;
static UART_HandleTypeDef *console;
static uint8_t rx_byte;
static bool sending;

static uint32_t enter(void) { uint32_t p = __get_PRIMASK(); __disable_irq(); return p; }
static void leave(uint32_t p) { __set_PRIMASK(p); }

void speed_uart_init(UART_HandleTypeDef *uart)
{
    console = uart;
    speed_queue_init(&queue);
    sending = false;
    if (HAL_UART_Receive_IT(console, &rx_byte, 1U) != HAL_OK) Error_Handler();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != console) return;
    const uint32_t p = enter();
    speed_queue_receive(&queue, rx_byte);
    if (HAL_UART_Receive_IT(console, &rx_byte, 1U) != HAL_OK) queue.fault = true;
    leave(p);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != console) return;
    const uint32_t p = enter();
    speed_queue_finish_tx(&queue);
    sending = false;
    leave(p);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != console) return;
    const uint32_t p = enter();
    queue.fault = true; /* 锁存，不自动恢复运动。 */
    __HAL_UART_CLEAR_OREFLAG(console);
    (void)HAL_UART_Receive_IT(console, &rx_byte, 1U);
    leave(p);
}

bool speed_uart_take_line(char out[SPEED_UART_LINE_BYTES])
{
    const uint32_t p = enter();
    const bool result = speed_queue_take_line(&queue, out);
    leave(p);
    return result;
}

bool speed_uart_send(const char *text)
{
    const size_t length = text == NULL ? 0U : strlen(text);
    const uint32_t p = enter();
    const bool result = speed_queue_send(&queue, text, length);
    leave(p);
    return result;
}

void speed_uart_service(void)
{
    const uint32_t p = enter();
    if (!sending) {
        uint16_t length;
        const char *frame = speed_queue_peek_tx(&queue, &length);
        if (frame != NULL) {
            sending = true;
            if (HAL_UART_Transmit_IT(console, (const uint8_t *)frame, length) != HAL_OK) {
                queue.fault = true;
                sending = false;
            }
        }
    }
    leave(p);
}

bool speed_uart_fault(void)
{
    const uint32_t p = enter();
    const bool fault = queue.fault;
    leave(p);
    return fault;
}

#ifdef RR_DIAGNOSTIC
bool speed_uart_tx_idle(void)
{
    const uint32_t p = enter();
    const bool idle = queue.tx_count == 0U && !sending;
    leave(p);
    return idle;
}
#endif
