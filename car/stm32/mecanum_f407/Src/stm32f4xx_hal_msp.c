#include "board.h"

void HAL_MspInit(void)
{
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM9) {
        __HAL_RCC_TIM9_CLK_ENABLE();
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM3) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer)
{
    if (timer->Instance == TIM9) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_GPIOE_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF3_TIM9;
        HAL_GPIO_Init(GPIOE, &gpio);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
    if (uart->Instance == USART2) {
        GPIO_InitTypeDef gpio = {0};
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &gpio);

        HAL_NVIC_SetPriority(USART2_IRQn, 5U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

