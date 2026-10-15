#include "mc_uart_transport.h"
#include "mc_protocol.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t *rx;
static int rearm_result, tx_result;
static unsigned rearms, aborts;
static bool receiving;
int HAL_UART_Receive_IT(UART_HandleTypeDef *u, uint8_t *p, uint16_t n)
{
    assert(u && n==1); ++rearms;
    if (receiving) return HAL_BUSY;
    if (rearm_result) return rearm_result;
    rx=p; receiving=true; u->ErrorCode=HAL_UART_ERROR_NONE; return HAL_OK;
}
int HAL_UART_AbortReceive(UART_HandleTypeDef *u)
{ assert(u); ++aborts; receiving=false; return HAL_OK; }
int HAL_UART_Transmit(UART_HandleTypeDef *u, const uint8_t *p, uint16_t n, uint32_t ms)
{ assert(u && p && n && ms==50); return tx_result; }
uint32_t HAL_GetTick(void) { return 1234; }
void Error_Handler(void) { assert(!"unexpected Error_Handler"); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
static void feed(UART_HandleTypeDef *u, uint8_t b)
{ assert(receiving); receiving=false; *rx=b; HAL_UART_RxCpltCallback(u); }
int main(void)
{
    USART_TypeDef peripheral={.SR=0x42};
    UART_HandleTypeDef u={.Instance=&peripheral,.ErrorCode=0};
    mc_uart_transport_init(&u);
    assert(mc_uart_diagnostics.tag==0x31445255 && !mc_uart_transport_take_fault());
    mc_uart_diagnostics_capture_irq(&u);
    assert(mc_uart_diagnostics.irq_error_count==1 && mc_uart_diagnostics.irq_error_sr==0x42);
    assert(mc_uart_diagnostics.irq_error_ms==1234);
    u.ErrorCode=HAL_UART_ERROR_FE;
    const unsigned before_error_rearms=rearms;
    feed(&u,1); /* emulate HAL's RxCplt-before-ErrorCallback order */
    assert(rearms==before_error_rearms && u.ErrorCode==HAL_UART_ERROR_FE);
    HAL_UART_ErrorCallback(&u);
    assert(aborts==1 && rearms==before_error_rearms+1);
    assert(mc_uart_diagnostics.error_callbacks==1 && mc_uart_diagnostics.last_error_code==4);
    assert(mc_uart_diagnostics.last_sr==0x42 && mc_uart_diagnostics.last_error_ms==1234);
    assert(mc_uart_transport_take_fault()); /* instrumentation must preserve latch */
    assert(!mc_uart_transport_take_fault());
    feed(&u,1); feed(&u,0); /* discard damaged frame tail, synchronize */
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE], out[MC_MAX_RAW_FRAME_SIZE], encoded[MC_MAX_COBS_FRAME_SIZE];
    size_t n=0, c=0, decoded=0;
    assert(mc_build_frame(MC_MSG_HELLO,0,1,7,0,NULL,0,raw,sizeof(raw),&n));
    assert(mc_cobs_encode(raw,n,encoded,sizeof(encoded),&c));
    for (unsigned packet=0;packet<9;++packet) {
        for (size_t i=0;i<c;++i) feed(&u,encoded[i]);
        feed(&u,0);
    }
    assert(mc_uart_diagnostics.queued_frames==8 && mc_uart_diagnostics.queue_overflows==1);
    assert(mc_uart_transport_take_fault());
    for (unsigned i=0;i<8;++i) {
        assert(mc_uart_transport_take_frame(out,sizeof(out),&decoded));
        assert(decoded==n && memcmp(out,raw,n)==0);
    }
    assert(!mc_uart_transport_take_frame(out,sizeof(out),&decoded));
    /* FE with no RXNE: a transfer is still BUSY. Abort/rearm must restore it,
       preserve the fault and drop a command queued before the error. */
    for (size_t i=0;i<c;++i) feed(&u,encoded[i]);
    feed(&u,0);
    u.ErrorCode=HAL_UART_ERROR_FE;
    const unsigned before_no_rxne=rearms;
    HAL_UART_ErrorCallback(&u);
    assert(receiving && rearms==before_no_rxne+1 && aborts==2);
    assert(mc_uart_diagnostics.last_error_code==HAL_UART_ERROR_FE);
    assert(mc_uart_diagnostics.rearm_failures==0 && mc_uart_transport_take_fault());
    assert(!mc_uart_transport_take_frame(out,sizeof(out),&decoded));
    /* Even a valid-looking suffix is dropped until the next delimiter. */
    for (size_t i=0;i<c;++i) feed(&u,encoded[i]);
    feed(&u,0);
    assert(!mc_uart_transport_take_frame(out,sizeof(out),&decoded));
    for (size_t i=0;i<c;++i) feed(&u,encoded[i]);
    feed(&u,0);
    assert(mc_uart_transport_take_frame(out,sizeof(out),&decoded));
    assert(decoded==n && memcmp(out,raw,n)==0);
    assert(!mc_uart_transport_take_fault());
    feed(&u,255); feed(&u,0);
    assert(!mc_uart_transport_take_frame(out,sizeof(out),&decoded));
    assert(mc_uart_diagnostics.invalid_cobs==1);
    for (size_t i=0;i<=MC_MAX_COBS_FRAME_SIZE;++i) feed(&u,1);
    assert(mc_uart_diagnostics.packet_overflows==1 && mc_uart_transport_take_fault());
    rearm_result=1;
    feed(&u,0);
    assert(mc_uart_diagnostics.rearm_failures==1 && mc_uart_transport_take_fault());
    tx_result=1;
    assert(!mc_uart_transport_send_frame(MC_MSG_HELLO_ACK,MC_FLAG_RESPONSE,1,7,0,NULL,0));
    assert(mc_uart_diagnostics.tx_failures==1);
    puts("PASS: FE callback ordering, BUSY receive recovery, damaged-frame resync, stale queue discard and fault guards");
    return 0;
}
