"""Independent XY+yaw pilot; existing MCU/host motion envelope is NOT raised."""
import math
from .core import Pose

INPUT_MODE = 'SPACE_TOGGLE_120S_V1'
POLICY = dict(operator_input_mode=INPUT_MODE, stage=4, max_active_s=120.0,
    engagements_per_process=1, spatial_policy='SIGNED_XY_YAW_4WS8_PILOT_V1',
    nominal_goal_forward_m=[-1.5,.5], nominal_goal_left_m=[-.4,.5],
    arena_buffer_m=.50, boundary_margin_m=.20, rod_reference_margin_m=.50,
    yaw_tracking_limit_rad=.65, yaw_rate_limit_radps=.10, yaw_accel_limit_radps2=.40,
    translation_scale=1., rotation_scale=1., reference_point='Sign5',
    origin_capture='FIRST_VALID_NEUTRAL_ACTIVE_V3',
    source_binding='SAME_RAW_SIGN_AND_FIVE_MARKERS_DIGEST_AND_CAR_STAMP_V1')


def active_limits(monitor, car, wand, goal):
    if not monitor.config.goal_inside(goal): return 'PILOT_GOAL_FENCE'
    if not monitor.config.inside(car): return 'PILOT_FOOTPRINT_FENCE'
    # The old .20rad deviation from the START heading was a translation-only
    # guard. Rotation uses independent goal-tracking/continuity checks instead,
    # not removal of angle protection. Continuous-turn error is checked in cores.
    if abs(math.atan2(math.sin(goal.yaw-car.yaw), math.cos(goal.yaw-car.yaw)))>.65:
        return 'ROTATION_TRACKING_ERROR'
    return ''


def metrics(monitor, inputs):
    car=inputs.poses.get('car',{}).get('pose');wand=inputs.poses.get('wand',{}).get('pose')
    def distance(a,b):
        return math.hypot(a.x-b.x,a.y-b.y) if isinstance(a,Pose) and isinstance(b,Pose) else None
    return dict(operator_input_mode=INPUT_MODE,spatial_policy=POLICY['spatial_policy'],
        hand_xy_m=distance(wand,monitor.wand),car_xy_m=distance(car,monitor.car),
        car_path_m=monitor.path,nominal_goal_forward_m=POLICY['nominal_goal_forward_m'],
        nominal_goal_left_m=POLICY['nominal_goal_left_m'],rotation_enabled=True,
        yaw_rate_limit_radps=.10,yaw_accel_limit_radps2=.40)
