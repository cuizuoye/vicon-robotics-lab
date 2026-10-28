"""Exact confirmed signed subregion, not inferred full Vicon coverage."""
from dataclasses import dataclass
from .safety_core import SafetyConfig
from .arena_geometry import FixedArena


@dataclass(frozen=True)
class ArenaSafetyConfig(SafetyConfig):
    arena_origin_x_m: float=.6437034797013303
    arena_origin_y_m: float=-1.0528685411524479
    arena_heading_rad: float=1.526260405304856
    arena_forward_min_m: float=-1.5
    arena_forward_max_m: float=.5
    arena_left_min_m: float=-.4
    arena_left_max_m: float=.5
    arena_buffer_m: float=.50

    def geometry(self):
        return FixedArena(self.arena_origin_x_m,self.arena_origin_y_m,self.arena_heading_rad,
            self.half_length_m,self.half_width_m,self.arena_buffer_m,self.boundary_margin_m,
            self.arena_forward_max_m,self.arena_left_max_m,
            self.arena_forward_min_m,self.arena_left_min_m)

    def __post_init__(self):
        super().__post_init__()
        exact=dict(arena_forward_min_m=-1.5,arena_forward_max_m=.5,
            arena_left_min_m=-.4,arena_left_max_m=.5,arena_buffer_m=.50,
            boundary_margin_m=.20,max_goal_radius_m=4.75,
            half_length_m=.095,half_width_m=.105)
        if any(getattr(self,k)!=v for k,v in exact.items()):
            raise ValueError('only confirmed forward[-1.5,.5]/left[-.4,.5] and preserved margins supported')
        expected=self.geometry().bbox(self.geometry().physical_bounds())
        actual=(self.arena_x_min,self.arena_x_max,self.arena_y_min,self.arena_y_max)
        if any(abs(a-b)>1e-8 for a,b in zip(actual,expected)):
            raise ValueError('display bounds differ from independently checked signed geometry')

    def inside(self,pose):return self.geometry().inside(pose)
    def goal_inside(self,pose):return self.geometry().goal_inside(pose)
    def center_bounds(self,yaw):return self.geometry().bbox(self.geometry().center_bounds(yaw))
    def goal_clearance_radius(self,pose):return self.geometry().goal_clearance_radius(pose)
    def goal_polygon(self):return self.geometry().polygon(self.geometry().goal_bounds())
    def fence_polygon(self):return self.geometry().polygon(self.geometry().physical_bounds())
    def stop_center_polygon(self,yaw):return self.geometry().polygon(self.geometry().center_bounds(yaw))
    def rod_reference_polygon(self):
        m=.50
        return self.geometry().polygon((self.arena_forward_min_m-m,self.arena_forward_max_m+m,
            self.arena_left_min_m-m,self.arena_left_max_m+m))
    def project_goal(self,goal,origin,radius):return self.geometry().project(goal,origin,radius)
    def project_reference(self,goal,origin,radius):
        return self.geometry().project(goal,origin,radius,reference=True)
