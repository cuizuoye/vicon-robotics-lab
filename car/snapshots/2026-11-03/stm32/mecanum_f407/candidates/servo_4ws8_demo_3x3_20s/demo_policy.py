"""Fixed-arena demonstration limits, independent in supervisor and final gate.

No I/O. A hand displacement is not a path budget. Raw Sign is still checked by
the unchanged source/Inputs validators; the existing mapping projects its target
into the fixed 3x3m region before differentiation. Do not impose another small
radial, sideways or cumulative-path limit on that already bounded target.
"""
import math

POLICY = dict(stage=4, max_active_s=20.0, engagements_per_process=1,
              spatial_policy='FIXED_3X3_GOAL_AND_FOOTPRINT_FENCE_V1',
              nominal_goal_forward_m=[0., 3.], nominal_goal_left_m=[0., 3.],
              arena_buffer_m=.50, boundary_margin_m=.20,
              rod_reference_margin_m=.50, max_heading_rad=.20,
              origin_capture='FIRST_VALID_NEUTRAL_ACTIVE_V3')


def active_limits(monitor, car, wand, goal):
    # Positions have already passed the original finite/fresh/jump checks.
    # The base final gate also checks these limits; these are independent
    # checks in both add-on monitors, not permission to bypass the base gate.
    if not monitor.config.goal_inside(goal):
        return 'PILOT_GOAL_FENCE'
    if not monitor.config.inside(car):
        return 'PILOT_FOOTPRINT_FENCE'
    dyaw = abs(math.atan2(math.sin(car.yaw-monitor.car.yaw),
                          math.cos(car.yaw-monitor.car.yaw)))
    if dyaw > POLICY['max_heading_rad']:
        return 'PILOT_HEADING_LIMIT'
    return ''


def metrics(monitor, inputs):
    car = inputs.poses.get('car', {}).get('pose')
    wand = inputs.poses.get('wand', {}).get('pose')
    distance = lambda a,b: (math.hypot(a.x-b.x, a.y-b.y)
        if a is not None and b is not None and a.valid() and b.valid() else None)
    return dict(spatial_policy=POLICY['spatial_policy'],
        hand_xy_m=distance(wand, monitor.wand),
        car_xy_m=distance(car, monitor.car), car_path_m=monitor.path,
        nominal_goal_forward_m=POLICY['nominal_goal_forward_m'],
        nominal_goal_left_m=POLICY['nominal_goal_left_m'])
