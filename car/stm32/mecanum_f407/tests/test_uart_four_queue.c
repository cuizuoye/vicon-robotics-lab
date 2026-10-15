#include "uart_console.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t *rx;
static char sent[600];
int HAL_UART_Receive_IT(UART_HandleTypeDef *uart, uint8_t *buffer, uint16_t length)
{
    assert(uart && length == 1U);
    rx = buffer;
    return HAL_OK;
}
int HAL_UART_Transmit(UART_HandleTypeDef *uart, const uint8_t *buffer, uint16_t length, uint32_t timeout)
{
    assert(uart && timeout == 50U && length < sizeof(sent));
    memcpy(sent, buffer, length);
    sent[length] = '\0';
    return HAL_OK;
}
void Error_Handler(void) { assert(!"unexpected HAL error"); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart);

static void feed(UART_HandleTypeDef *uart, const char *text)
{
    while (*text) {
        *rx = (uint8_t)*text++;
        HAL_UART_RxCpltCallback(uart);
    }
}
int main(void)
{
    UART_HandleTypeDef uart = {1};
    char line[64];
    uart_console_init(&uart);
    assert(!uart_console_take_line(line, sizeof(line)));
    feed(&uart, "hb\r\narm FL A5\nstatus\nstop\n");
    assert(!uart_console_take_overflow());
    const char *expected[] = {"hb", "arm FL A5", "status", "stop"};
    for (unsigned i = 0; i < 4U; ++i) {
        assert(uart_console_take_line(line, sizeof(line)));
        assert(strcmp(line, expected[i]) == 0);
    }
    assert(!uart_console_take_line(line, sizeof(line)));
    feed(&uart, "1\n2\n3\n4\n5\n");
    assert(uart_console_take_overflow());
    assert(!uart_console_take_overflow());
    assert(!uart_console_take_line(NULL, 10U));
    assert(!uart_console_take_line(line, 0U));
    for (unsigned i = 1U; i <= 4U; ++i) {
        assert(uart_console_take_line(line, sizeof(line)));
        assert(line[0] == (char)('0' + i));
    }
    feed(&uart, "1234567890123456789012345678901234567890123456789012345678901234\n");
    assert(uart_console_take_overflow());
    assert(!uart_console_take_line(line, sizeof(line)));
    feed(&uart, "status\n");
    assert(uart_console_take_line(line, sizeof(line)) && strcmp(line, "status") == 0);
    HAL_UART_ErrorCallback(&uart);
    assert(uart_console_take_overflow());
    uart_console_printf("%s:%d", "test", 42);
    assert(strcmp(sent, "test:42") == 0);
    puts("four UART: real ISR FIFO order, CRLF, bounded overflow and recovery passed (offline)");
    return 0;
}
