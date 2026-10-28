/* Reuse the ACTUAL protocol/controller test harness, no actuation bypass. */
#define main inherited_controller_main
#include "test_controller.c"
#undef main

static void angular_tiers(const char *folder)
{
    for(unsigned stage=0;stage<SERVO_STAGE_COUNT;stage++) {
        servo_stage_t s;assert(servo_stage_get(stage,&s));reset_at(stage,0);
        command[2]=(int)lroundf(s.yaw_rate_rad_s*1000.f);
        unsigned ramp=0;
        for(unsigned i=0;i<1000;i++) {
            tick(true,10);assert(!b.group.fault && l.permitted);
            if(fabsf(b.group.shaped_body.wz_rad_s-s.yaw_rate_rad_s)<.00001f) {
                ramp=now-l.first_motion_ms;break;
            }
        }
        assert(ramp && ramp<=(unsigned)ceilf(s.yaw_rate_rad_s/s.yaw_accel_rad_s2*1000.f)+20);
        printf("ANGULAR_RAMP stage=%u rate=%.3f accel=%.3f target_ms=%u\n",
            stage,(double)s.yaw_rate_rad_s,(double)s.yaw_accel_rad_s2,ramp);
        run_for(0,0,command[2],1000);
        char path[1024];assert(snprintf(path,sizeof(path),"%s/rotation_stage%u.bin",folder,stage)>0);
        save_status(path);
        run_for(0,0,-command[2],1600);
        assert(b.group.shaped_body.wz_rad_s<-.95f*s.yaw_rate_rad_s);
        for(unsigned i=0;i<4;i++) {
            assert(!b.group.loop[i].reversing);
            assert(fabsf(b.group.loop[i].wheel[i].ramped_cps)*60.f/b.adopted.wheel[i].counts_per_rev
                <=servo_stage_target_rpm(stage)+.001f);
        }
        const uint32_t began=l.first_motion_ms;
        command[2]=0;velocity(true);check_outputs();
        for(unsigned i=0;i<4;i++)assert(!b.group.raw_duty[i]);
        assert(l.first_motion_ms==began);
        /* The next-higher tier's yaw is rejected, NOT silently clamped. */
        reset_at(stage,0);command[2]=(int)lroundf(s.yaw_rate_rad_s*1000.f)+1;
        velocity(true);assert(b.group.fault && !l.permitted);check_outputs();
    }
}

static void mixed_motion_and_continuous_reversals(void)
{
    for(unsigned stage=0;stage<SERVO_STAGE_COUNT;stage++) {
        servo_stage_t s;assert(servo_stage_get(stage,&s));reset_at(stage,0);
        for(unsigned i=0;i<1200;i++) {
            const float a=i*.010f;
            command[0]=(int)(.65f*s.speed_m_s*1000.f*cosf(a));
            command[1]=(int)(.65f*s.speed_m_s*1000.f*sinf(a));
            command[2]=(int)(.90f*s.yaw_rate_rad_s*1000.f*sinf(a*1.5f));
            tick(true,10);assert(!b.group.fault && l.permitted);
            for(unsigned j=0;j<4;j++)assert(fabsf(b.group.plan.forward_cps[j])*60.f/
                b.adopted.wheel[j].counts_per_rev<=servo_stage_target_rpm(stage)+.01f);
        }
    }
    reset_at(6,0);run_for(300,300,1500,1000);
    for(unsigned i=0;i<10;i++) {
        run_for(i%2?300:-300,i%2?300:-300,i%2?1500:-1500,1000);
    }
}

static void old_mode_words_never_select_new_image(void)
{
    reset_at(0,0);
    /* Fresh unpermitted handshake: old magic is invalid even with CRC/session. */
    const mc_control_enable_v1_t stop={0};frame(MC_MSG_CONTROL_ENABLE,0,&stop,sizeof(stop),0);
    session++;frame(MC_MSG_HELLO,0,NULL,0,0);
    const mc_control_enable_v1_t old={.reserved=0x7800};
    uint8_t raw[MC_MAX_RAW_FRAME_SIZE];size_t used;four_link_request_t request;
    assert(mc_build_frame(MC_MSG_CONTROL_ENABLE,0,++seq,session,now,&old,sizeof(old),raw,sizeof(raw),&used));
    assert(!four_link_bench_decode(raw,used,&request));
    assert(!l.permitted);
}

int main(int argc,char **argv)
{
    assert(argc==2);
    angular_tiers(argv[1]);mixed_motion_and_continuous_reversals();old_mode_words_never_select_new_image();
    puts("PASS: 4WSA actual C yaw tiers / combined motion / neutral / old-mode rejection; synthetic encoders only");
    return 0;
}
