"""Fixed, oriented car-center goal region and independent footprint fence.

Pure geometry. No ROS/network/hardware. Coordinates are metres. The 0.50m
buffer starts beyond the reference-heading body outline at each goal edge;
the retained 0.20m additional margin triggers before its physical outer edge.
"""
from dataclasses import dataclass
import math
from .core import Pose


def rectangle(xmin,xmax,ymin,ymax):
    return ((xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax))


@dataclass(frozen=True)
class FixedArena:
    origin_x: float
    origin_y: float
    heading: float
    half_length: float=.095
    half_width: float=.105
    buffer: float=.50
    margin: float=.20
    forward_max: float=3.
    left_max: float=3.

    def __post_init__(self):
        if not all(type(v) in (int,float) and math.isfinite(v) for v in vars(self).values()):
            raise ValueError('arena values must be finite numbers')
        if (min(self.half_length,self.half_width,self.forward_max,self.left_max)<=0
                or self.margin<.20 or self.buffer<.50 or self.margin>=self.buffer):
            raise ValueError('invalid footprint, buffer or preserved margin')

    def world(self,forward,left):
        c,s=math.cos(self.heading),math.sin(self.heading)
        return self.origin_x+c*forward-s*left,self.origin_y+s*forward+c*left

    def local(self,x,y):
        c,s=math.cos(self.heading),math.sin(self.heading)
        dx,dy=x-self.origin_x,y-self.origin_y
        return c*dx+s*dy,-s*dx+c*dy

    def goal_bounds(self):return (0.,self.forward_max,0.,self.left_max)

    def physical_bounds(self):
        return (-self.half_length-self.buffer,self.forward_max+self.half_length+self.buffer,
                -self.half_width-self.buffer,self.left_max+self.half_width+self.buffer)

    def center_bounds(self,yaw):
        c,s=abs(math.cos(yaw-self.heading)),abs(math.sin(yaw-self.heading))
        ex=c*self.half_length+s*self.half_width+self.margin
        ey=s*self.half_length+c*self.half_width+self.margin
        xmin,xmax,ymin,ymax=self.physical_bounds()
        return xmin+ex,xmax-ex,ymin+ey,ymax-ey

    def polygon(self,bounds):return tuple(self.world(x,y) for x,y in rectangle(*bounds))

    def bbox(self,bounds):
        p=self.polygon(bounds)
        return min(x for x,y in p),max(x for x,y in p),min(y for x,y in p),max(y for x,y in p)

    def inside(self,pose):
        if not pose.valid():return False
        x,y=self.local(pose.x,pose.y);xmin,xmax,ymin,ymax=self.center_bounds(pose.yaw)
        return xmin-1e-10<=x<=xmax+1e-10 and ymin-1e-10<=y<=ymax+1e-10

    def goal_inside(self,pose):
        if not pose.valid():return False
        x,y=self.local(pose.x,pose.y)
        return -1e-10<=x<=self.forward_max+1e-10 and -1e-10<=y<=self.left_max+1e-10

    def project(self,goal,origin,radius,*,reference=False):
        if not goal.valid() or not origin.valid() or not math.isfinite(radius) or radius<=0:
            raise ValueError('invalid arena projection input')
        x,y=self.local(goal.x,goal.y)
        bounds=self.center_bounds(origin.yaw) if reference else self.goal_bounds()
        xmin,xmax,ymin,ymax=bounds
        x=max(xmin+1e-8,min(xmax-1e-8,x));y=max(ymin+1e-8,min(ymax-1e-8,y))
        wx,wy=self.world(x,y);dx,dy=wx-origin.x,wy-origin.y
        d=math.hypot(dx,dy);ratio=min(1.,(radius-1e-8)/d) if d else 1.
        projected=Pose(origin.x+ratio*dx,origin.y+ratio*dy,origin.z,origin.yaw)
        if not reference and not self.goal_inside(projected):
            # Never return a buffer goal because a too-small radial budget
            # interpolated toward an origin outside the nominal goal region.
            raise ValueError('radial budget cannot reach the fixed goal region')
        return projected

    def goal_clearance_radius(self,pose):
        x,y=self.local(pose.x,pose.y)
        return max(0.,min(x,self.forward_max-x,y,self.left_max-y))
