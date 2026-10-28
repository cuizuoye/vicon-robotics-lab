"""Independent XY+yaw pilot; existing MCU/host motion envelope is NOT raised."""
import math
from .core import Pose

INPUT_MODE = 'SPACE_TOGGLE_120S_V1'
POLICY = dict(operator_input_mode=INPUT_MODE,stage=4,max_active_s=120.,
    engagements_per_process=1,spatial_policy='TRACKING_ONLY_NO_FIXED_FENCE_V1',
    fixed_fence_enabled=False,target_radius_limit_enabled=False,
    yaw_tracking_limit_rad=.65,yaw_rate_limit_radps=.10,yaw_accel_limit_radps2=.40,
    translation_scale=1.,rotation_scale=1.,reference_point='Sign5',
    origin_capture='FIRST_VALID_NEUTRAL_ACTIVE_V3',
    source_binding='SAME_RAW_SIGN_AND_FIVE_MARKERS_DIGEST_AND_CAR_STAMP_V1')


def active_limits(monitor, car, wand, goal):
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
        car_path_m=monitor.path,fixed_fence_enabled=False,target_radius_limit_enabled=False,rotation_enabled=True,
        yaw_rate_limit_radps=.10,yaw_accel_limit_radps2=.40)
