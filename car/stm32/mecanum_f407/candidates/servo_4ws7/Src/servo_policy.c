#include "servo_policy.h"
#include <stddef.h>
#include <string.h>
bool servo_stage_get(unsigned index, servo_stage_t *out)
{
    static const servo_stage_t stages[SERVO_STAGE_COUNT]={
        {.20f,.08f,350,1200}, {.35f,.20f,450,1600},
        {.50f,.40f,600,2000}, {.70f,.40f,700,2000},
        {.70f,.80f,800,2500}, {.70f,1.20f,800,3000},
        {.70f,2.00f,800,4000}};
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (index>=SERVO_STAGE_COUNT) return false;
    *out=stages[index];
    return true;
}
