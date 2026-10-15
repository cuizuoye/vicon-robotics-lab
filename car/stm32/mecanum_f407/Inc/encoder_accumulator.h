#ifndef ENCODER_ACCUMULATOR_H
#define ENCODER_ACCUMULATOR_H

#include <stdint.h>

/* 纯计数算法；TIM2为32位，其余编码器定时器为16位。 */
typedef struct {
    uint32_t previous;
    uint32_t sampled_ms;
    uint32_t total;
    int32_t cps;
    uint8_t bits;
} encoder_accumulator_t;

void encoder_accumulator_init(encoder_accumulator_t *e, uint8_t bits,
                              uint32_t counter, uint32_t now);
void encoder_accumulator_tick(encoder_accumulator_t *e, uint32_t counter, uint32_t now);
int32_t encoder_accumulator_count(const encoder_accumulator_t *e);

#endif
