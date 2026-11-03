#include "mc_link_test.h"

#include "board.h"
#include "mc_protocol.h"

#include <stddef.h>
#include <string.h>

static UART_HandleTypeDef *link_uart;
static uint8_t rx_byte;
static uint8_t working_packet[MC_MAX_COBS_FRAME_SIZE];
static uint8_t ready_packet[MC_MAX_COBS_FRAME_SIZE];
static volatile size_t working_length;
static volatile size_t ready_length;
static volatile bool packet_ready;
static volatile bool receive_fault;

void mc_link_test_init(UART_HandleTypeDef *uart)
{
    link_uart = uart;
    working_length = 0U;
    ready_length = 0U;
    packet_ready = false;
    receive_fault = false;
    if (HAL_UART_Receive_IT(link_uart, &rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != link_uart) {
        return;
    }

    if (rx_byte == 0U) {
        if (working_length > 0U) {
            if (!packet_ready) {
                memcpy(ready_packet, working_packet, working_length);
                ready_length = working_length;
                packet_ready = true;
            } else {
                receive_fault = true;
            }
            working_length = 0U;
        }
    } else if (working_length < sizeof(working_packet)) {
        working_packet[working_length++] = rx_byte;
    } else {
        working_length = 0U;
        receive_fault = true;
    }

    if (HAL_UART_Receive_IT(link_uart, &rx_byte, 1U) != HAL_OK) {
        receive_fault = true;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != link_uart) {
        return;
    }
    receive_fault = true;
    working_length = 0U;
    __HAL_UART_CLEAR_OREFLAG(link_uart);
    (void)HAL_UART_Receive_IT(link_uart, &rx_byte, 1U);
}

static bool take_packet(uint8_t *packet, size_t *length)
{
    if (!packet_ready) {
        return false;
    }
    __disable_irq();
    const size_t copied = ready_length;
    memcpy(packet, ready_packet, copied);
    packet_ready = false;
    ready_length = 0U;
    __enable_irq();
    *length = copied;
    return true;
}

static bool send_hello_ack(const mc_frame_header_v1_t *hello, uint32_t now_ms)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];
    size_t raw_length = 0U;
    if (!mc_build_frame(
            MC_MSG_HELLO_ACK, MC_FLAG_RESPONSE, hello->sequence,
            hello->session_id, now_ms, NULL, 0U,
            raw, sizeof(raw), &raw_length)) {
        return false;
    }

    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE + 1U];
    size_t encoded_length = 0U;
    if (!mc_cobs_encode(
            raw, raw_length, encoded, MC_MAX_COBS_FRAME_SIZE,
            &encoded_length)) {
        return false;
    }
    encoded[encoded_length++] = 0U;
    return HAL_UART_Transmit(
               link_uart, encoded, (uint16_t)encoded_length, 50U) == HAL_OK;
}

bool mc_link_test_poll(uint32_t now_ms)
{
    if (receive_fault) {
        __disable_irq();
        receive_fault = false;
        __enable_irq();
    }

    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE];
    size_t encoded_length = 0U;
    if (!take_packet(encoded, &encoded_length)) {
        return false;
    }

    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];
    size_t raw_length = 0U;
    mc_frame_header_v1_t header;
    if (!mc_cobs_decode(
            encoded, encoded_length, raw, sizeof(raw), &raw_length)
        || !mc_validate_frame(raw, raw_length, &header)
        || header.message_type != MC_MSG_HELLO
        || header.payload_length != 0U
        || header.session_id == 0U) {
        return false;
    }
    return send_hello_ack(&header, now_ms);
}
