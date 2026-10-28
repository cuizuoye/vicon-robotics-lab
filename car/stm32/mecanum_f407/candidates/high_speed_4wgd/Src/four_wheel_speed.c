/* Reuse the accepted continuous reversal/zero/stall/PI implementation without
 * editing it. Only independent RPM target/feedback guards are added here. */
#define four_speed_set_target four_speed_set_target_4wgc_impl
#define four_speed_tick four_speed_tick_4wgc_impl
#include "../../continuous_4wgc/Src/four_wheel_speed.c"
#undef four_speed_set_target
#undef four_speed_tick

bool four_speed_set_target(four_speed_controller_t *c, float target, uint32_t now)
{
    if (!c) return false;
    if (c->selected>=FOUR_SPEED_WHEELS) {
        four_speed_stop(c,FOUR_SPEED_FAULT_CONFIG);
        return false;
    }
    const float limit=c->config.wheel[c->selected].counts_per_rev*FOUR_HIGH_TARGET_RPM/60.0f;
    if (!isfinite(target) || !isfinite(limit) || fabsf(target)>limit) {
        four_speed_stop(c,FOUR_SPEED_FAULT_COMMAND);
        return false;
    }
    return four_speed_set_target_4wgc_impl(c,target,now);
}

bool four_speed_tick(four_speed_controller_t *c, const int32_t raw[4], uint32_t now)
{
    if (!c) return false;
    const uint32_t dt=now-c->last_sample_ms;
    const unsigned selected=c->selected;
    const int32_t delta=(raw && selected<4) ?
        delta_i32(raw[selected],c->wheel[selected].previous_count) : 0;
    const bool updated=four_speed_tick_4wgc_impl(c,raw,now);
    if (updated && c->enabled && selected<4 &&
        fabsf(c->wheel[selected].measured.rpm)>FOUR_HIGH_FEEDBACK_RPM) {
        return feedback_fault(c,FOUR_FEEDBACK_OVERSPEED,now,dt,delta,
                              c->wheel[selected].measured.forward_cps);
    }
    return updated;
}
