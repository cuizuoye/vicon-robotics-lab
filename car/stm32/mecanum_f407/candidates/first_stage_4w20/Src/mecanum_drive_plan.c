/* Reuse the same common-scale mecanum planner and continuous wheel control.
 * A diagonal must not acquire sqrt(2) times the first-stage permission. */
#define mecanum_drive_plan mecanum_drive_plan_first_stage_impl
#include "../../../Src/mecanum_drive_plan.c"
#undef mecanum_drive_plan

bool mecanum_drive_plan(const mecanum_drive_geometry_t *g,const four_speed_config_t *c,
                       const mecanum_body_velocity_t *body,float limit,mecanum_drive_plan_t *out)
{
    if (!out) return false;
    memset(out,0,sizeof(*out));
    if (!body || !isfinite(body->vx_m_s) || !isfinite(body->vy_m_s) ||
        !isfinite(body->wz_rad_s) ||
        hypotf(body->vx_m_s,body->vy_m_s)>FOUR_DRIVE_MAX_LINEAR_M_S ||
        fabsf(body->wz_rad_s)>FOUR_DRIVE_MAX_YAW_RAD_S) return false;
    return mecanum_drive_plan_first_stage_impl(g,c,body,limit,out);
}
