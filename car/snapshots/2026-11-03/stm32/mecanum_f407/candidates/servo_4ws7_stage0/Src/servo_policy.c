#include "servo_policy.h"
#include <string.h>
bool servo_stage_get(unsigned index, servo_stage_t *out)
{
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (index!=0) return false;
    *out=(servo_stage_t){.20f,.08f,350,1200};
    return true;
}
