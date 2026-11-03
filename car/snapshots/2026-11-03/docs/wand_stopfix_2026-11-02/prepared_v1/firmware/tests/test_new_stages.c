/* Actual MCU control/protocol, synthetic encoders. NOT hardware proof. */
#define main inherited_controller_main
#include "test_controller.c"
#undef main

static void high_stages(const char *folder)
{
    for(unsigned stage=8;stage<=9;stage++) {
        servo_stage_t p;assert(servo_stage_get(stage,&p));reset_at(stage,0);
        assert(p.accel_m_s2==4.f && p.yaw_rate_rad_s==2.f && p.yaw_accel_rad_s2==12.f);
        for(unsigned j=0;j<4;j++) {
            const float bound=(sqrtf(2)*p.accel_m_s2+.146f*p.yaw_accel_rad_s2)*
                b.adopted.wheel[j].counts_per_rev/(3.14159265358979323846f*.060f);
            assert(bound<43000.f && b.group.loop[j].config.ramp_cps_per_s==43000.f);
        }
        unsigned target_ms=0;
        for(unsigned i=0;i<80;i++) {
            command[0]=(int)lroundf(p.speed_m_s*1000.f);tick(true,10);
            assert(!b.group.fault && l.permitted);
            if(b.group.shaped_body.vx_m_s>=p.speed_m_s-.0001f){target_ms=now-l.first_motion_ms;break;}
        }
        assert(target_ms && target_ms<=(unsigned)ceilf(p.speed_m_s/4.f*1000.f)+20);
        printf("STAGE%u_TARGET_RAMP_MS=%u; not physical acceleration\n",stage,target_ms);
        char name[1024];assert(snprintf(name,sizeof(name),"%s/high_stage%u.bin",folder,stage)>0);
        save_status(name);
        for(unsigned i=0;i<2500;i++) {
            float t=i*.01f;
            command[0]=(int)(.65f*p.speed_m_s*1000.f*cosf(1.1f*t));
            command[1]=(int)(.65f*p.speed_m_s*1000.f*sinf(1.1f*t));
            command[2]=(int)(1.8f*1000.f*sinf(.7f*t));
            tick(true,10);assert(!b.group.fault && l.permitted);
            for(unsigned j=0;j<4;j++)assert(fabsf(b.group.plan.forward_cps[j])*60.f/
                b.adopted.wheel[j].counts_per_rev<=servo_stage_target_rpm(stage)+.01f);
        }
        const uint32_t origin=l.first_motion_ms;
        command[0]=command[1]=command[2]=0;velocity(true);check_outputs();
        for(unsigned j=0;j<4;j++)assert(!b.group.raw_duty[j]);
        run_for(300,0,1000,1000);assert(l.first_motion_ms==origin);
        velocity(false);assert(!l.permitted && !b.group.enabled);check_outputs();
        reset_at(stage,0);command[0]=(int)lroundf(p.speed_m_s*1000.f)+1;
        velocity(true);assert(b.group.fault && !l.permitted);
        reset_at(stage,0);run_for(300,0,1000,500);now+=600;
        four_link_bench_deadline(&l,&b,now);assert(b.group.fault && !l.permitted);check_outputs();
        reset_at(stage,0);run_for(300,0,1000,500);
        b.group.loop[0].config.ramp_cps_per_s=33000.f;
        uint32_t cc[8];assert(!four_drive_compares(&b.group,cc));
        for(unsigned j=0;j<8;j++)assert(!cc[j]);
    }
    const uint16_t old[]={0x7B00,0x7B80,0x7A00,0x7A80};
    for(unsigned i=0;i<4;i++)for(unsigned j=0;j<8;j++) {
        mc_control_enable_v1_t e={.enabled=1,.reserved=(uint16_t)(old[i]+j)};
        uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t n;four_link_request_t request;
        assert(mc_build_frame(MC_MSG_CONTROL_ENABLE,0,1,1,1,&e,sizeof(e),raw,sizeof(raw),&n));
        assert(!four_link_bench_decode(raw,n,&request));
    }
}
int main(int argc,char **argv)
{
    assert(argc==2);high_stages(argv[1]);
    puts("PASS: new XY+yaw tiers, continuous 25s circles/reversals, SI ramp, wheel budget, zero, lease, stage corruption and old-word rejection; synthetic only");
    return 0;
}
