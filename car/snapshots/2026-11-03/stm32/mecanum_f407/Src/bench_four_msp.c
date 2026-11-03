#include "board.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM1) { __HAL_RCC_TIM1_CLK_ENABLE(); }
    else if (timer->Instance == TIM9) { __HAL_RCC_TIM9_CLK_ENABLE(); }
    else if (timer->Instance == TIM12) { __HAL_RCC_TIM12_CLK_ENABLE(); }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLDOWN;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    if (timer->Instance == TIM9) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
        g.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        g.Alternate = GPIO_AF3_TIM9;
        HAL_GPIO_Init(GPIOE, &g);
    } else if (timer->Instance == TIM1) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
        g.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
        g.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOE, &g);
    } else if (timer->Instance == TIM12) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        g.Pin = GPIO_PIN_14 | GPIO_PIN_15;
        g.Alternate = GPIO_AF9_TIM12;
        HAL_GPIO_Init(GPIOB, &g);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *timer)
{
    GPIO_InitTypeDef g = {0};
    /* FT数字输入，禁用内部上下拉；不用模拟模式。PB3/PA15占用原JTAG，仅保留SWD。 */
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    __HAL_RCC_GPIOB_CLK_ENABLE();
    if (timer->Instance == TIM3 || timer->Instance == TIM4) {
        if (timer->Instance == TIM3) { __HAL_RCC_TIM3_CLK_ENABLE(); }
        else { __HAL_RCC_TIM4_CLK_ENABLE(); }
        g.Pin = timer->Instance == TIM3 ? GPIO_PIN_4 | GPIO_PIN_5 : GPIO_PIN_6 | GPIO_PIN_7;
        g.Alternate = timer->Instance == TIM3 ? GPIO_AF2_TIM3 : GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &g);
    } else if (timer->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        g.Alternate = GPIO_AF1_TIM2;
        g.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIOA, &g);
        g.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &g);
    } else if (timer->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        g.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &g);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance != USART2) return;
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Pin = GPIO_PIN_5 | GPIO_PIN_6;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOD, &g);
    HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}
