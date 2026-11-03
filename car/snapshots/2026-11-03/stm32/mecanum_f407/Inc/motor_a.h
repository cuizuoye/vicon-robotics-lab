#ifndef MOTOR_A_H
#define MOTOR_A_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

void motor_a_init(TIM_HandleTypeDef *timer);
void motor_a_request(int16_t duty_permille, uint32_t now_ms);
void motor_a_tick(uint32_t now_ms);
void motor_a_brake(void);
void motor_a_stop(void);
int16_t motor_a_command(void);
bool motor_a_is_high_z(void);

#endif
