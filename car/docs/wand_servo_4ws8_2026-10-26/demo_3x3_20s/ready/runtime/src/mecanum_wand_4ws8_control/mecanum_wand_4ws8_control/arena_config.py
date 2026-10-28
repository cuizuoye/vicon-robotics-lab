"""Strict 3x3/0.50m extension of the accepted SafetyConfig, not a new gate."""
from dataclasses import dataclass
import math
from .safety_core import SafetyConfig
from .arena_geometry import FixedArena


@dataclass(frozen=True)
class ArenaSafetyConfig(SafetyConfig):
    # arena_ prefix keeps signed coordinates out of base positive-only checks.
    arena_origin_x_m: float=.6437034797013303
    arena_origin_y_m: float=-1.0528685411524479
    arena_heading_rad: float=1.526260405304856
    arena_forward_max_m: float=3.
    arena_left_max_m: float=3.
    arena_buffer_m: float=.50

    def geometry(self):
        return FixedArena(self.arena_origin_x_m,self.arena_origin_y_m,self.arena_heading_rad,
            self.half_length_m,self.half_width_m,self.arena_buffer_m,self.boundary_margin_m,
            self.arena_forward_max_m,self.arena_left_max_m)

    def __post_init__(self):
        super().__post_init__()
        if (self.arena_forward_max_m!=3. or self.arena_left_max_m!=3.
                or self.arena_buffer_m!=.50 or self.boundary_margin_m!=.20
                or self.max_goal_radius_m!=4.75
                or self.half_length_m!=.095 or self.half_width_m!=.105):
            raise ValueError('only reviewed 3x3/0.50m geometry and 190x210mm body supported')
        expected=self.geometry().bbox(self.geometry().physical_bounds())
        actual=(self.arena_x_min,self.arena_x_max,self.arena_y_min,self.arena_y_max)
        if any(abs(a-b)>1e-8 for a,b in zip(actual,expected)):
            raise ValueError('arena display bounds do not match fixed oriented geometry')

    def inside(self,pose):return self.geometry().inside(pose)
    def goal_inside(self,pose):return self.geometry().goal_inside(pose)
    def center_bounds(self,yaw):return self.geometry().bbox(self.geometry().center_bounds(yaw))
    def goal_clearance_radius(self,pose):return self.geometry().goal_clearance_radius(pose)
    def goal_polygon(self):return self.geometry().polygon(self.geometry().goal_bounds())
    def fence_polygon(self):return self.geometry().polygon(self.geometry().physical_bounds())
    def stop_center_polygon(self,yaw):return self.geometry().polygon(self.geometry().center_bounds(yaw))
    def rod_reference_polygon(self):
        m=.50
        return self.geometry().polygon((-m,3.+m,-m,3.+m))
    def project_goal(self,goal,origin,radius):return self.geometry().project(goal,origin,radius)
    def project_reference(self,goal,origin,radius):
        # A car placed a few cm behind the saved corner must be able to slew
        # inward from its ACTUAL pose, without a fabricated reference jump.
        return self.geometry().project(goal,origin,radius,reference=True)
