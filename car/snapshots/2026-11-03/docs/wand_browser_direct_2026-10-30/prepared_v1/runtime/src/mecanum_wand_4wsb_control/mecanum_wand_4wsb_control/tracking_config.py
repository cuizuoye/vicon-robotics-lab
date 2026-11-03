"""Explicit spatial-policy change, NOT a fictitiously enlarged arena.

Inherited arena numbers are pinned historical metadata used to validate the
unchanged guard set. They are never motion limits in this policy. The unused
legacy radius remains finite for source compatibility; a separate validated
boolean disables only its application in the follower.
"""
from dataclasses import dataclass, fields, replace
from .core import Pose
from .rotation_core import RotationSafetyConfig

SPATIAL_POLICY = 'TRACKING_ONLY_NO_FIXED_FENCE_V1'


@dataclass(frozen=True)
class TrackingSafetyConfig(RotationSafetyConfig):
    def __post_init__(self):
        if self.goal_boundary_policy != 'tracking':
            raise ValueError('explicit tracking-only spatial policy required')
        historical = {f.name: getattr(self, f.name) for f in fields(RotationSafetyConfig)}
        historical['goal_boundary_policy'] = 'clamp'
        # Validate all timing, speed, acceleration, lift, jump, footprint and
        # calibration fields against the preserved parent, not relaxed defaults.
        RotationSafetyConfig(**historical)

    def inside(self, pose):
        return isinstance(pose, Pose) and pose.valid()

    def goal_inside(self, pose):
        return self.inside(pose)

    def project_goal(self, goal, origin, radius):
        if not self.inside(goal) or not self.inside(origin):
            raise ValueError('nonfinite tracking target or origin')
        return goal

    def project_reference(self, goal, origin, radius):
        return self.project_goal(goal, origin, radius)

    def follower_config(self):
        # Use the same accepted continuous controller and all dynamic guards.
        historical = {f.name: getattr(self, f.name) for f in fields(RotationSafetyConfig)}
        historical['goal_boundary_policy'] = 'clamp'
        return replace(RotationSafetyConfig(**historical).follower_config(),
                       workspace_radius_enabled=False)

    def center_bounds(self, yaw):
        raise ValueError('tracking-only mode has no fixed center bounds')

    def goal_clearance_radius(self, pose):
        raise ValueError('tracking-only mode has no spatial radius budget')

    def goal_polygon(self): return ()
    def fence_polygon(self): return ()
    def stop_center_polygon(self, yaw): return ()
    def rod_reference_polygon(self): return ()
