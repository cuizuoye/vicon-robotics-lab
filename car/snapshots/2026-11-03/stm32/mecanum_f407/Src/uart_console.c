#include "uart_console.h"

#include "app_config.h"
#include "board.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *console_uart;
static uint8_t rx_byte;
static char working_line[COMMAND_LINE_MAX_LENGTH];
#if defined(BENCH_FOUR_WHEELS)
#define FOUR_RX_LINES 4U
static char queued_lines[FOUR_RX_LINES][COMMAND_LINE_MAX_LENGTH];
static volatile unsigned queue_head, queue_tail, queue_count;
static volatile size_t working_length;
#else
static char ready_line[COMMAND_LINE_MAX_LENGTH];
static volatile size_t working_length;
static volatile bool line_ready;
#endif
static volatile bool overflowed;

void uart_console_init(UART_HandleTypeDef *uart)
{
    console_uart = uart;
    working_length = 0U;
#if defined(BENCH_FOUR_WHEELS)
    queue_head = queue_tail = queue_count = 0U;
#else
    line_ready = false;
#endif
    overflowed = false;
    if (HAL_UART_Receive_IT(console_uart, &rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != console_uart) {
        return;
    }

    const char character = (char)rx_byte;
    if (character == '\n' || character == '\r') {
        if (working_length > 0U) {
#if defined(BENCH_FOUR_WHEELS)
            /* 长四路遥测发送期间ISR可暂存4条；满则锁存故障，不静默覆盖。 */
            if (queue_count < FOUR_RX_LINES) {
                memcpy(queued_lines[queue_tail], working_line, working_length);
                queued_lines[queue_tail][working_length] = '\0';
                queue_tail = (queue_tail + 1U) % FOUR_RX_LINES;
                ++queue_count;
            } else {
                overflowed = true;
            }
#else
            if (!line_ready) {
                memcpy(ready_line, working_line, working_length);
                ready_line[working_length] = '\0';
                line_ready = true;
            } else {
                overflowed = true;
            }
#endif
            working_length = 0U;
        }
    } else if (working_length < COMMAND_LINE_MAX_LENGTH - 1U) {
        working_line[working_length++] = character;
    } else {
        working_length = 0U;
        overflowed = true;
    }

    if (HAL_UART_Receive_IT(console_uart, &rx_byte, 1U) != HAL_OK) {
        overflowed = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != console_uart) {
        return;
    }
    overflowed = true;
    __HAL_UART_CLEAR_OREFLAG(console_uart);
    (void)HAL_UART_Receive_IT(console_uart, &rx_byte, 1U);
}

bool uart_console_take_line(char *line, size_t capacity)
{
#if defined(BENCH_FOUR_WHEELS)
    if (queue_count == 0U || line == NULL || capacity == 0U) return false;
    __disable_irq();
    const size_t length = strnlen(queued_lines[queue_head], COMMAND_LINE_MAX_LENGTH);
    const size_t copied = length < capacity - 1U ? length : capacity - 1U;
    memcpy(line, queued_lines[queue_head], copied);
    line[copied] = '\0';
    queue_head = (queue_head + 1U) % FOUR_RX_LINES;
    --queue_count;
    __enable_irq();
    return true;
#else
    if (!line_ready || line == NULL || capacity == 0U) {
        return false;
    }

    __disable_irq();
    const size_t length = strnlen(ready_line, sizeof(ready_line));
    const size_t copied = length < capacity - 1U ? length : capacity - 1U;
    memcpy(line, ready_line, copied);
    line[copied] = '\0';
    line_ready = false;
    __enable_irq();
    return true;
#endif
}

bool uart_console_take_overflow(void)
{
    __disable_irq();
    const bool result = overflowed;
    overflowed = false;
    __enable_irq();
    return result;
}

void uart_console_write(const char *text)
{
    if (text == NULL) {
        return;
    }
    (void)HAL_UART_Transmit(
        console_uart, (const uint8_t *)text, (uint16_t)strlen(text), 50U);
}

void uart_console_printf(const char *format, ...)
{
#if defined(BENCH_FOUR_WHEELS)
    /* 四编码器与8个实际CCR；仅新镜像扩容，旧镜像不变。 */
    char buffer[512];
#elif defined(BENCH_FRONT_ENCODERS_READONLY)
    /* 双编码器极值遥测超过160B；仅新目标扩容，原镜像保持不变。 */
    char buffer[224];
#else
    char buffer[160];
#endif
    va_list arguments;
    va_start(arguments, format);
    const int length = vsnprintf(buffer, sizeof(buffer), format, arguments);
    va_end(arguments);
    if (length <= 0) {
        return;
    }
    buffer[sizeof(buffer) - 1U] = '\0';
    uart_console_write(buffer);
}
