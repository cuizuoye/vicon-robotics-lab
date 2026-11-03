#include "mc_uart_transport.h"

#include "board.h"
#include "mc_protocol.h"

#include <string.h>

#define RX_PACKET_QUEUE_CAPACITY 8U

static UART_HandleTypeDef *transport_uart;
static uint8_t rx_byte;
static uint8_t working_packet[MC_MAX_COBS_FRAME_SIZE];
static uint8_t ready_packets[RX_PACKET_QUEUE_CAPACITY][MC_MAX_COBS_FRAME_SIZE];
static size_t ready_lengths[RX_PACKET_QUEUE_CAPACITY];
static volatile size_t working_length;
static volatile uint8_t queue_head;
static volatile uint8_t queue_tail;
static volatile uint8_t queue_count;
static volatile bool receive_fault;
static volatile bool discard_until_delimiter;
static volatile bool startup_line_error_guard;

static bool tolerate_startup_line_error(uint32_t error_code)
{
    const uint32_t tolerated =
        HAL_UART_ERROR_PE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE;
    return startup_line_error_guard && error_code != HAL_UART_ERROR_NONE &&
           (error_code & ~tolerated) == 0U;
}

#ifdef MC_UART_DIAGNOSTICS
volatile mc_uart_diagnostics_t mc_uart_diagnostics;
void mc_uart_diagnostics_capture_irq(UART_HandleTypeDef *uart)
{
    if (uart != transport_uart) return;
    const uint32_t sr = uart->Instance->SR;
    /* PE/FE/NE/ORE occupy SR bits 0..3 on STM32F407. Capture before HAL
       reads DR or the receive-complete callback rearms and clears ErrorCode. */
    if (sr & 15U) {
        ++mc_uart_diagnostics.irq_error_count;
        mc_uart_diagnostics.irq_error_sr = sr;
        mc_uart_diagnostics.irq_error_ms = HAL_GetTick();
    }
}
#endif

void mc_uart_transport_init(UART_HandleTypeDef *uart)
{
    transport_uart = uart;
    working_length = 0U;
    queue_head = 0U;
    queue_tail = 0U;
    queue_count = 0U;
    receive_fault = false;
    discard_until_delimiter = false;
    startup_line_error_guard = false;
#ifdef MC_UART_DIAGNOSTICS
    memset((void *)&mc_uart_diagnostics, 0, sizeof(mc_uart_diagnostics));
    mc_uart_diagnostics.tag = UINT32_C(0x31445255); /* URD1 */
#endif
    if (HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U) != HAL_OK) {
        Error_Handler();
    }
}

void mc_uart_transport_enable_startup_line_error_guard(void)
{
    startup_line_error_guard = true;
}

void mc_uart_transport_confirm_valid_frame(void)
{
    startup_line_error_guard = false;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != transport_uart) {
        return;
    }
#ifdef MC_UART_DIAGNOSTICS
    ++mc_uart_diagnostics.rx_bytes;
#endif
    /* HAL can call RxCplt before ErrorCallback in the same FE/NE/PE IRQ.
       Rearming here clears ErrorCode and causes a second, BUSY rearm in the
       error callback. Never admit the errored byte; let that callback own
       abort/rearm while retaining the fault notification. */
    if (uart->ErrorCode != HAL_UART_ERROR_NONE) {
        if (!tolerate_startup_line_error(uart->ErrorCode)) receive_fault = true;
        working_length = 0U;
        discard_until_delimiter = true;
        return;
    }
    if (discard_until_delimiter) {
        if (rx_byte == 0U) discard_until_delimiter = false;
    } else if (rx_byte == 0U) {
        if (working_length > 0U) {
            if (queue_count < RX_PACKET_QUEUE_CAPACITY) {
                memcpy(ready_packets[queue_head], working_packet, working_length);
                ready_lengths[queue_head] = working_length;
                queue_head = (uint8_t)(
                    (queue_head + 1U) % RX_PACKET_QUEUE_CAPACITY);
                ++queue_count;
#ifdef MC_UART_DIAGNOSTICS
                ++mc_uart_diagnostics.queued_frames;
#endif
            } else {
                receive_fault = true;
#ifdef MC_UART_DIAGNOSTICS
                ++mc_uart_diagnostics.queue_overflows;
#endif
            }
            working_length = 0U;
        }
    } else if (working_length < sizeof(working_packet)) {
        working_packet[working_length++] = rx_byte;
    } else {
        working_length = 0U;
        discard_until_delimiter = true;
        receive_fault = true;
#ifdef MC_UART_DIAGNOSTICS
        ++mc_uart_diagnostics.packet_overflows;
#endif
    }
    if (HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U) != HAL_OK) {
        receive_fault = true;
#ifdef MC_UART_DIAGNOSTICS
        ++mc_uart_diagnostics.rearm_failures;
#endif
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart != transport_uart) {
        return;
    }
#ifdef MC_UART_DIAGNOSTICS
    ++mc_uart_diagnostics.error_callbacks;
    mc_uart_diagnostics.last_error_code = uart->ErrorCode;
    mc_uart_diagnostics.last_sr = uart->Instance->SR;
    mc_uart_diagnostics.last_error_ms = HAL_GetTick();
#endif
    const bool tolerated = tolerate_startup_line_error(uart->ErrorCode);
    if (!tolerated) receive_fault = true;
    working_length = 0U;
    discard_until_delimiter = true;
    /* Drop pre-error queued commands as well as the damaged partial packet.
       This is receive recovery only: no application fault is cleared. */
    queue_head = queue_tail = queue_count = 0U;
    if (HAL_UART_AbortReceive(transport_uart) != HAL_OK) {
        receive_fault = true;
#ifdef MC_UART_DIAGNOSTICS
        ++mc_uart_diagnostics.rearm_failures;
#endif
        return;
    }
    __HAL_UART_CLEAR_OREFLAG(transport_uart);
#ifdef MC_UART_DIAGNOSTICS
    if (HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U) != HAL_OK) {
        receive_fault = true;
        ++mc_uart_diagnostics.rearm_failures;
    }
#else
    if (HAL_UART_Receive_IT(transport_uart, &rx_byte, 1U) != HAL_OK)
        receive_fault = true;
#endif
}

bool mc_uart_transport_take_frame(
    uint8_t *raw, size_t capacity, size_t *raw_length)
{
    if (raw == NULL || raw_length == NULL) {
        return false;
    }
    uint8_t encoded[MC_MAX_COBS_FRAME_SIZE];
    size_t encoded_length;
    __disable_irq();
    if (queue_count == 0U) {
        __enable_irq();
        return false;
    }
    encoded_length = ready_lengths[queue_tail];
    memcpy(encoded, ready_packets[queue_tail], encoded_length);
    queue_tail = (uint8_t)(
        (queue_tail + 1U) % RX_PACKET_QUEUE_CAPACITY);
    --queue_count;
    __enable_irq();
#ifdef MC_UART_DIAGNOSTICS
    const bool valid = mc_cobs_decode(encoded, encoded_length, raw, capacity, raw_length);
    if (!valid) ++mc_uart_diagnostics.invalid_cobs;
    return valid;
#else
    return mc_cobs_decode(encoded, encoded_length, raw, capacity, raw_length);
#endif
}

bool mc_uart_transport_take_fault(void)
{
    __disable_irq();
    const bool fault = receive_fault;
    receive_fault = false;
    __enable_irq();
    return fault;
}

bool mc_uart_transport_send_frame(
    uint8_t message_type,
    uint16_t flags,
    uint32_t sequence,
    uint32_t session_id,
    uint32_t sender_time_ms,
    const void *payload,
    uint16_t payload_length)
{
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];
    size_t raw_length = 0U;
    if (!mc_build_frame(
            message_type, flags, sequence, session_id, sender_time_ms,
            payload, payload_length, raw, sizeof(raw), &raw_length)) {
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
#ifdef MC_UART_DIAGNOSTICS
    const bool valid = HAL_UART_Transmit(transport_uart, encoded, (uint16_t)encoded_length, 50U) == HAL_OK;
    if (!valid) ++mc_uart_diagnostics.tx_failures;
    return valid;
#else
    return HAL_UART_Transmit(transport_uart, encoded, (uint16_t)encoded_length, 50U) == HAL_OK;
#endif
}
