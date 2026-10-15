#ifndef MC_LINK_TEST_H
#define MC_LINK_TEST_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

void mc_link_test_init(UART_HandleTypeDef *uart);
bool mc_link_test_poll(uint32_t now_ms);

#endif
