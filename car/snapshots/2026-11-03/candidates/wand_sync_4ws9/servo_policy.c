#include "servo_policy.h"
#include <stddef.h>
#include <string.h>
bool servo_stage_get(unsigned index,servo_stage_t *out)
{
    static const servo_stage_t stages[SERVO_STAGE_COUNT]={
        {.20f,.08f,350,1200,.20f,.80f},{.35f,.20f,450,1600,.30f,1.f},
        {.50f,.40f,600,2000,.60f,2.f},{.70f,.40f,700,2000,1.f,4.f},
        {.70f,.80f,800,2500,1.5f,6.f},{.70f,1.20f,800,3000,2.f,8.f},
        {.70f,2.f,800,4000,2.f,12.f}};
    if(!out)return false;
    memset(out,0,sizeof(*out));
    if(index>=SERVO_STAGE_COUNT)return false;
    *out=stages[index];return true;
}
float servo_stage_target_rpm(unsigned i){return i==0?100.f:i==1?180.f:i==2?250.f:400.f;}
float servo_stage_feedback_rpm(unsigned i){return i==0?120.f:i==1?210.f:i==2?280.f:440.f;}
