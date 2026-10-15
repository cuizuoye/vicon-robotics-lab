#include "board.h"
#ifdef MC_UART_DIAGNOSTICS
#include "mc_uart_transport.h"
#endif

void NMI_Handler(void) { Error_Handler(); }
void HardFault_Handler(void) { Error_Handler(); }
void MemManage_Handler(void) { Error_Handler(); }
void BusFault_Handler(void) { Error_Handler(); }
void UsageFault_Handler(void) { Error_Handler(); }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

void USART2_IRQHandler(void)
{
#ifdef MC_UART_DIAGNOSTICS
    mc_uart_diagnostics_capture_irq(&huart2);
#endif
    HAL_UART_IRQHandler(&huart2);
}

