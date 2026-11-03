/* Real MCU algorithms, synthetic encoders only. No hardware qualification. */
#define main inherited_controller_main
#include "test_controller.c"
#undef main

static void new_target_ramp_and_guards(const char *folder)
{
    reset_at(7,0);servo_stage_t s;assert(servo_stage_get(7,&s));
    assert(s.speed_m_s==.70f && s.accel_m_s2==4.f);
    assert(s.yaw_rate_rad_s==.10f && s.yaw_accel_rad_s2==.40f);
    assert(s.duty_permille==800 && s.output_slew_permille_s==4000);
    for(unsigned i=0;i<4;i++) {
        const float bound=(sqrtf(2)*s.accel_m_s2+.146f*s.yaw_accel_rad_s2)*
            b.adopted.wheel[i].counts_per_rev/(3.14159265358979323846f*.060f);
        assert(bound<33000.f && b.group.loop[i].config.ramp_cps_per_s==33000.f);
    }
    command[0]=700;uint32_t ramp=0;
    for(unsigned i=0;i<50;i++) {
        tick(true,10);assert(!b.group.fault && l.permitted);
        if(b.group.shaped_body.vx_m_s>=.699999f) {ramp=now-l.first_motion_ms;break;}
    }
    assert(ramp>=170 && ramp<=190);
    printf("STAGE7_TARGET_RAMP_MS=%u (target only, not physical response)\n",ramp);
    run_for(700,0,0,1000);
    char path[1024];assert(snprintf(path,sizeof(path),"%s/stage7_active.bin",folder)>0);
    save_status(path);
    for(unsigned stage=0;stage<7;stage++) {
        reset_at(stage,0);
        for(unsigned i=0;i<4;i++)assert(b.group.loop[i].config.ramp_cps_per_s==26000.f);
    }
    /* Corruption may not silently widen an old tier's defensive bound. */
    reset_at(0,0);run_for(100,0,0,1000);
    b.group.loop[0].config.ramp_cps_per_s=33000.f;
    uint32_t compares[8];assert(!four_drive_compares(&b.group,compares));
    for(unsigned i=0;i<8;i++)assert(!compares[i]);
    reset_at(7,0);run_for(300,0,0,1000);
    b.group.loop[0].config.ramp_cps_per_s=26000.f;
    assert(!four_drive_compares(&b.group,compares));
    for(unsigned i=0;i<8;i++)assert(!compares[i]);
}

static void continuous_axes_reversals_zero_and_stop(void)
{
    reset_at(7,0);
    const int axes[][3]={{700,0,0},{0,700,0},{-700,0,0},{0,-700,0},
        {494,494,0},{-494,494,0},{-494,-494,0},{494,-494,0}};
    for(unsigned i=0;i<8;i++)run_for(axes[i][0],axes[i][1],axes[i][2],1800);
    for(unsigned i=0;i<2000;i++) {
        const float t=i*.010f;
        command[0]=(int)(450*cosf(t*1.5f));command[1]=(int)(450*sinf(t*1.5f));
        command[2]=(int)(90*sinf(t));
        tick(true,10);assert(!b.group.fault && l.permitted);
    }
    command[0]=command[1]=command[2]=0;velocity(true);
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
    const uint32_t began=l.first_motion_ms;
    run_for(300,0,0,1000);assert(l.first_motion_ms==began);
    velocity(false);assert(!l.permitted && !b.group.enabled);check_outputs();
}

static void policy_and_faults(void)
{
    reset_at(7,0);run_for(300,0,0,500);
    now+=600;four_link_bench_deadline(&l,&b,now);
    assert(b.group.fault && !l.permitted);check_outputs();
    const uint32_t fault=b.group.fault;
    velocity(true);assert((b.group.fault&fault)==fault && !l.permitted && !b.group.enabled);
    reset_at(7,0);command[0]=701;velocity(true);assert(b.group.fault && !l.permitted);
    reset_at(7,0);command[0]=command[1]=495;velocity(true);assert(b.group.fault && !l.permitted);
    reset_at(7,0);command[2]=101;velocity(true);assert(b.group.fault && !l.permitted);
    /* Do not lower the inherited mechanical start threshold to make .1rad/s
     * look successful. This experimental tier is XY-tuned, not yaw-qualified. */
    reset_at(7,0);command[2]=100;
    for(unsigned i=0;i<100;i++) {tick(true,10);assert(!b.group.fault && !b.group.enabled);}
    for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
    /* Previous-image tier/bench words must not acquire any new permission. */
    for(unsigned i=0;i<8;i++) {
        const uint16_t old_words[]={0x7800,0x7900,0x7A00,0x7880,0x7980,0x7A80};
        for(unsigned j=0;j<6;j++) {
            mc_control_enable_v1_t e={.reserved=(uint16_t)(old_words[j]+i)};
            uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t request;
            assert(mc_build_frame(MC_MSG_CONTROL_ENABLE,0,1,1,1,&e,sizeof(e),raw,sizeof(raw),&used));
            assert(!four_link_bench_decode(raw,used,&request));
        }
    }
    reset_at(7,UINT32_MAX-1500U);run_for(400,0,0,3000);
    assert(!b.group.fault && l.permitted);
}

int main(int argc,char **argv)
{
    assert(argc==2);
    new_target_ramp_and_guards(argv[1]);
    continuous_axes_reversals_zero_and_stop();policy_and_faults();
    puts("PASS: stage7 target acceleration, eight directions, mixed circle, measured reversal, zero, lease, corruption, old words, timer wrap; synthetic only");
    return 0;
}
