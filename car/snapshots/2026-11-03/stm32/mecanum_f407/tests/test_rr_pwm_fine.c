#include "four_wheel_speed.h"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    assert(FOUR_SPEED_PWM_COUNTS == 8400U && FOUR_SPEED_DUTY_QUANTUM == 1);
    for (int d=-100; d<=100; ++d) {
        uint32_t a=1, b=1;
        assert(four_speed_compare((int16_t)d,&a,&b));
        if (!d) { assert(!a && !b); continue; }
        const unsigned mag=(unsigned)(d<0 ? -d : d), ticks=8400*mag/1000;
        assert(a==(d>0 ? 8400 : 8400-ticks));
        assert(b==(d<0 ? 8400 : 8400-ticks));
        assert(ticks>0 && ticks<=840 && ticks*1000<=mag*8400);
        assert(mag*8400-ticks*1000<1000); /* <1 tick舍入误差。 */
    }
    for (int d=-110; d<=110; ++d) {
        if (d>=-100 && d<=100) continue;
        uint32_t a=1,b=1; assert(!four_speed_compare(d,&a,&b) && !a && !b);
    }
    four_speed_config_t cfg; four_speed_defaults(&cfg);
    for(unsigned i=0;i<4;++i) {
        cfg.wheel[i].counts_per_rev=1061;
        cfg.wheel[i].calibration_confirmed=true;
        cfg.wheel[i].kp_permille_per_cps=1;
        cfg.wheel[i].ki_permille_per_count=.5f;
    }
    four_speed_controller_t c; int32_t raw[4]={0};
    assert(four_speed_init(&c,&cfg) && four_speed_arm(&c,3,raw,0));
    assert(four_speed_set_target(&c,164,0) && four_speed_tick(&c,raw,10));
    assert(c.wheel[3].raw_duty_permille==-2); /* 0.2%不再量化为0或1%。 */
    for(unsigned i=0;i<3;++i) assert(!c.wheel[i].raw_duty_permille);
    four_speed_stop(&c,FOUR_SPEED_FAULT_COMMAND);
    for(unsigned i=0;i<4;++i) assert(!c.wheel[i].raw_duty_permille);
    puts("RR fine PWM exhaustive mapping/quantization/zero-stop passed");
}
