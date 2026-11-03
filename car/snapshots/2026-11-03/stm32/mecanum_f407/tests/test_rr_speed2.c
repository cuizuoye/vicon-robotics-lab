#include "drive_bench_control.h"
#include <assert.h>
#include <string.h>

int main(void)
{
    drive_bench_t b; int32_t raw[4]={0}; drive_bench_init(&b,raw,0);
    assert(b.adopted.target_limit_cps==328 && b.adopted.duty_limit_permille==100);
    assert(b.adopted.run_limit_ms==10000 && b.adopted.command_timeout_ms==150);
    assert(b.adopted.ramp_cps_per_s==800);
    const char *bad[]={"run RR164 S10","run RR492 S10","run RL328 S10",
        "run RR328 S30","run RR329 S10","run RR328 S10 ","run F50 S10"};
    for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);++i) {
        assert(strstr(drive_bench_accept(&b,bad[i],0),"ERR,"));
        assert(!b.group.enabled && !b.group.run_used);
    }
    four_speed_config_t corrupted=b.adopted; corrupted.target_limit_cps=329;
    four_drive_group_t g;
    assert(!four_drive_init(&g,&b.group.geometry,&corrupted,raw,0));
    for(unsigned i=0;i<4;++i) {
        assert(b.adopted.wheel[i].kp_permille_per_cps==0.1f);
        assert(b.adopted.wheel[i].ki_permille_per_count==0.5f);
    }
    return 0;
}
