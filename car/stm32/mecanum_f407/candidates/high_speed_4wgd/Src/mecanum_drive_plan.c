/* Homogeneous rescaling reuses the proven planner, including common wheel
 * scaling, CPR conversion and layout. No per-wheel clipping or copied IK. */
#define mecanum_drive_plan mecanum_drive_plan_4wgc_impl
#include "../../../Src/mecanum_drive_plan.c"
#undef mecanum_drive_plan

bool mecanum_drive_plan(const mecanum_drive_geometry_t *g, const four_speed_config_t *c,
                       const mecanum_body_velocity_t *body, float wheel_limit,
                       mecanum_drive_plan_t *out)
{
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (!body || !c || !isfinite(body->vx_m_s) || !isfinite(body->vy_m_s) ||
        !isfinite(body->wz_rad_s) || fabsf(body->vx_m_s)>FOUR_DRIVE_MAX_LINEAR_M_S ||
        fabsf(body->vy_m_s)>FOUR_DRIVE_MAX_LINEAR_M_S ||
        fabsf(body->wz_rad_s)>FOUR_DRIVE_MAX_YAW_RAD_S ||
        !isfinite(wheel_limit) || wheel_limit<=0 || wheel_limit>FOUR_DRIVE_MAX_WHEEL_M_S)
        return false;
    const float scale=fmaxf(1.0f,fmaxf(fabsf(body->vx_m_s),fabsf(body->vy_m_s)));
    const mecanum_body_velocity_t reduced={body->vx_m_s/scale,body->vy_m_s/scale,body->wz_rad_s/scale};
    four_speed_config_t reduced_config=*c;
    reduced_config.target_limit_cps/=scale;
    mecanum_drive_plan_t p;
    if (!mecanum_drive_plan_4wgc_impl(g,&reduced_config,&reduced,wheel_limit/scale,&p)) return false;
    for (unsigned i=0;i<4;i++) {
        p.wheel_m_s[i]*=scale; p.forward_cps[i]*=scale;
        p.expected_raw_encoder_cps[i]*=scale;
        if (fabsf(p.wheel_m_s[i])>wheel_limit ||
            fabsf(p.forward_cps[i])>c->wheel[i].counts_per_rev*FOUR_HIGH_TARGET_RPM/60.0f)
            return false;
    }
    p.applied_body.vx_m_s*=scale; p.applied_body.vy_m_s*=scale; p.applied_body.wz_rad_s*=scale;
    *out=p;
    return true;
}
