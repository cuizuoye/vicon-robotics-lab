#include "encoder_accumulator.h"
#include <limits.h>

static int32_t signed_count(uint32_t value)
{
    return value <= INT32_MAX ? (int32_t)value : (int32_t)((int64_t)value - 4294967296LL);
}

void encoder_accumulator_init(encoder_accumulator_t *e, uint8_t bits,
                              uint32_t counter, uint32_t now)
{
    e->bits = bits == 32U ? 32U : 16U;
    e->previous = counter;
    e->sampled_ms = now;
    e->total = 0U;
    e->cps = 0;
}

void encoder_accumulator_tick(encoder_accumulator_t *e, uint32_t counter, uint32_t now)
{
    uint32_t elapsed = now - e->sampled_ms;
    if (elapsed < 10U) return;
    const uint64_t modulus = 1ULL << e->bits;
    int64_t delta = (counter - e->previous) & (uint32_t)(modulus - 1ULL);
    if ((uint64_t)delta >= modulus / 2ULL) delta -= (int64_t)modulus;
    e->total += (uint32_t)delta; /* unsigned累加定义回绕，避免有符号溢出。 */
    const int64_t rate = delta * 1000LL / elapsed;
    e->cps = rate > INT32_MAX ? INT32_MAX : rate < INT32_MIN ? INT32_MIN : (int32_t)rate;
    e->previous = counter;
    e->sampled_ms = now;
}

int32_t encoder_accumulator_count(const encoder_accumulator_t *e)
{
    return signed_count(e->total);
}
