"""SIM ONLY relative XY+yaw servo; no ROS/network/firmware interfaces."""
from collections import deque
from dataclasses import dataclass,replace
import math

from bootstrap import ACCEPTED
from mecanum_wand_4ws8_control.core import Pose,Velocity,RelativeMapping,clamp,wrap_angle
from mecanum_wand_4ws8_control.continuous_servo import ContinuousSession
from mecanum_wand_4ws8_control.safety_core import Inputs,Supervisor,FinalGate,ZERO
from mecanum_vicon_control.mecanum_kinematics import body_to_wheel_angular
from preview_core import rotation,vector

RATES=(.10,.50,1.0,1.5,2.0,2.5)
ACCELS=(.40,1.0,2.0,4.0,6.0)
WHEEL_RADIUS=.030
HALF_WHEELBASE=.061
HALF_TRACK=.085
MAX_RPM=400.
ANGULAR_LEAD=.50
MAX_ANGULAR_TRACKING_ERROR=.65


def reference_pose(position,quaternion,definition):
    """Sign5 point + the HUMAN-accepted local forward axis, not raw Sign yaw."""
    if definition.get('bar')!=['Sign1','Sign5','Sign2'] or definition.get('junction')!='Sign5':
        raise ValueError('the accepted Sign5 crossbar reference is required')
    r=rotation(quaternion);forward=r@vector(definition['forward_local'])
    if math.hypot(forward[0],forward[1])<.35:raise ValueError('horizontal heading degeneracy')
    p=vector(position)+r@vector(definition['local_points']['Sign5'])
    return Pose(*p,math.atan2(forward[1],forward[0]))


@dataclass(frozen=True)
class SimRotationLimits:
    max_yaw_rate_radps:float
    max_yaw_accel_radps2:float
    def __post_init__(self):
        if (type(self.max_yaw_rate_radps) not in (int,float) or self.max_yaw_rate_radps not in RATES
                or type(self.max_yaw_accel_radps2) not in (int,float) or self.max_yaw_accel_radps2 not in ACCELS):
            raise ValueError('only enumerated software rotation experiments allowed')
    def __getattr__(self,key):
        if key.startswith('_'):raise AttributeError(key)
        return getattr(ACCEPTED,key)
    def follower_config(self):
        return replace(ACCEPTED.follower_config(),max_yaw_rate_radps=self.max_yaw_rate_radps,
            max_yaw_accel_radps2=self.max_yaw_accel_radps2)
    def project_goal(self,goal,origin,radius):
        p=ACCEPTED.project_goal(goal,origin,radius)
        return Pose(p.x,p.y,p.z,goal.yaw)
    def project_reference(self,goal,origin,radius):
        # Project against the ACTUAL requested orientation, not the original
        # fixed-heading outline. Keep the same physical fence and margins.
        g=self.geometry();x,y=g.local(goal.x,goal.y);xmin,xmax,ymin,ymax=g.center_bounds(goal.yaw)
        x=max(xmin+1e-8,min(xmax-1e-8,x));y=max(ymin+1e-8,min(ymax-1e-8,y))
        wx,wy=g.world(x,y);dx,dy=wx-origin.x,wy-origin.y;d=math.hypot(dx,dy)
        ratio=min(1.,(radius-1e-8)/d) if d else 1.
        return Pose(origin.x+ratio*dx,origin.y+ratio*dy,origin.z,goal.yaw)
    @property
    def name(self):return f'w{self.max_yaw_rate_radps:.2f}_a{self.max_yaw_accel_radps2:.2f}'


class RotationMapping:
    def __init__(self,mapping):
        self.base=mapping;self.car_origin=mapping.car_origin;self.wand_origin=mapping.wand_origin
        self.scale=mapping.scale;self.turn=0.
    def local_displacement(self,wand):return self.base.local_displacement(wand)
    def goal_from_local(self,forward,left):
        p=self.base.goal_from_local(forward,left)
        return Pose(p.x,p.y,p.z,self.car_origin.yaw+self.turn)
    def goal(self,wand):return self.goal_from_local(*self.local_displacement(wand))


def wheel_rpm(command):
    return tuple(v*60/(2*math.pi) for v in body_to_wheel_angular(command.vx,command.vy,command.wz,
        WHEEL_RADIUS,HALF_WHEELBASE,HALF_TRACK).as_tuple())


def coupled_shape(desired,previous,dt,c):
    if dt<=0 or not all(math.isfinite(v) for v in (dt,desired.vx,desired.vy,desired.wz)):
        raise ValueError('finite command and positive control interval required')
    peak=max(abs(v) for v in wheel_rpm(desired));speed=math.hypot(desired.vx,desired.vy)
    scale=min(1.,MAX_RPM/peak if peak else 1.,c.max_speed_mps/speed if speed else 1.,
        c.max_yaw_rate_radps/abs(desired.wz) if desired.wz else 1.)
    target=Velocity(desired.vx*scale,desired.vy*scale,desired.wz*scale)
    dx,dy,dw=target.vx-previous.vx,target.vy-previous.vy,target.wz-previous.wz
    delta=math.hypot(dx,dy)
    # ONE effective body-command acceleration limit. Convex interpolation
    # also preserves the common wheel-speed envelope (no per-wheel clipping).
    alpha=min(1.,c.max_accel_mps2*dt/delta if delta else 1.,
        c.max_yaw_accel_radps2*dt/abs(dw) if dw else 1.)
    return Velocity(previous.vx+alpha*dx,previous.vy+alpha*dy,previous.wz+alpha*dw)


class RotationSession(ContinuousSession):
    def __init__(self,*a,feedforward=True,**kw):
        self.rotation_active=False;self.yaw_samples=deque();self.use_yaw_feedforward=feedforward
        self.last_raw_yaw=None;self.yaw_receipt=-math.inf;self.car_continuous=0.;self.last_raw_car=None
        self.yaw_limited=False;self.desired_xy=(0.,0.)
        self.yaw_progress_origin=0.;self.yaw_progress_elapsed=0.
        super().__init__(*a,**kw)
    def engage(self,now):
        self.rotation_active=False;result=super().engage(now)
        if result[0]:
            self.mapping=RotationMapping(self.mapping)
            self.last_raw_yaw=self.mapping.wand_origin.yaw
            self.last_raw_car=self.car_continuous=self.mapping.car_origin.yaw
            self.yaw_progress_origin=self.car_continuous;self.yaw_progress_elapsed=0.
            self.yaw_samples.clear();self.yaw_samples.append((self.source_stamps['wand'],0.))
            self.yaw_receipt=self.samples['wand'][1];self.rotation_active=True
        return result
    def observe(self,source,pose,stamp,*,source_stamp=None):
        accepted=super().observe(source,pose,stamp,source_stamp=source_stamp)
        if accepted and source=='car' and self.rotation_active:
            self.car_continuous+=wrap_angle(pose.yaw-self.last_raw_car);self.last_raw_car=pose.yaw
        return accepted
    def record_target(self,wand,source_stamp,received):
        super().record_target(wand,source_stamp,received)
        if not self.rotation_active:return
        self.mapping.turn+=wrap_angle(wand.yaw-self.last_raw_yaw);self.last_raw_yaw=wand.yaw
        h=self.yaw_samples
        if h and source_stamp-h[-1][0]>.060:h.clear()
        h.append((source_stamp,self.mapping.turn))
        while len(h)>2 and source_stamp-h[0][0]>.040+1e-9:h.popleft()
        while len(h)>16:h.popleft()
        self.yaw_receipt=received
    def target_yaw_velocity(self):
        h=self.yaw_samples
        if (not self.use_yaw_feedforward or len(h)<2 or self.last_tick-self.yaw_receipt>.060
                or h[-1][0]-h[0][0]<.005 or max(v for _,v in h)-min(v for _,v in h)<.001):return 0.
        times=[t-h[0][0] for t,_ in h];mean=sum(times)/len(times)
        denominator=sum((t-mean)**2 for t in times)
        velocity=sum((t-mean)*(v-h[0][1]) for t,(_,v) in zip(times,h))/denominator
        return clamp(velocity,-self.config.max_yaw_rate_radps,self.config.max_yaw_rate_radps)
    def advance_reference(self,dt,car):
        super().advance_reference(dt,car)
        step=clamp(self.mapped_goal.yaw-self.goal.yaw,
            -self.config.max_yaw_rate_radps*dt,self.config.max_yaw_rate_radps*dt)
        angle=self.goal.yaw+step
        self.yaw_limited=abs(angle-self.car_continuous)>ANGULAR_LEAD
        if not self.yaw_limited:self.goal=replace(self.goal,yaw=angle)
    def translation_target(self,bx,by,feedforward):
        self.desired_xy=super().translation_target(bx,by,feedforward)
        return self.desired_xy
    def tick(self,now):
        previous=self.command;dt=now-self.last_tick if self.last_tick is not None else .02
        self.desired_xy=(0.,0.);output=super().tick(now)
        if self.state!='ACTIVE':return output
        error=self.goal.yaw-self.car_continuous
        ff=0. if self.yaw_limited else self.target_yaw_velocity()
        desired_w=(0. if abs(error)<=self.config.yaw_tolerance_rad and abs(ff)<.005 else
            clamp(ff+2.*error,-self.config.max_yaw_rate_radps,self.config.max_yaw_rate_radps))
        desired=Velocity(*self.desired_xy,desired_w)
        # Inherited XY controller/reference are reused; its provisional shaping
        # result is NOT fed through another ramp. Shape the captured unshaped
        # XY target and new yaw together exactly once here.
        self.command=coupled_shape(desired,previous,dt,self.config)
        if abs(self.command.wz)>.05 and abs(error)>.05:
            if abs(self.car_continuous-self.yaw_progress_origin)>.025:
                self.yaw_progress_origin=self.car_continuous;self.yaw_progress_elapsed=0.
            else:self.yaw_progress_elapsed+=dt
            if self.yaw_progress_elapsed>=3.:
                self._fault('ROTATION_NO_PROGRESS');return self.command
        else:self.yaw_progress_origin=self.car_continuous;self.yaw_progress_elapsed=0.
        return self.command


class RotationInputs(Inputs):
    def __init__(self,c):super().__init__(c);self.car_continuous=self.wand_continuous=None
    def observe_pose(self,source,pose,source_ros,steady,ros,roll=0.,pitch=0.):
        old=self.poses.get(source)
        if source=='wand' and old and 0<source_ros-old['source_ros']<=self.c.pose_timeout_s:
            delta=abs(wrap_angle(pose.yaw-old['pose'].yaw));dt=source_ros-old['source_ros']
            if delta>min(.60,8.*dt+.020):
                self.event('WAND_ANGLE_JUMP');self.pose_errors[source]='WAND_ANGLE_JUMP';self.poses.pop(source,None)
                return False
        accepted=super().observe_pose(source,pose,source_ros,steady,ros,roll,pitch)
        if accepted and source=='car':
            self.car_continuous=(pose.yaw if old is None or self.car_continuous is None else
                self.car_continuous+wrap_angle(pose.yaw-old['pose'].yaw))
        if accepted and source=='wand':
            self.wand_continuous=(pose.yaw if old is None or self.wand_continuous is None else
                self.wand_continuous+wrap_angle(pose.yaw-old['pose'].yaw))
        return accepted


class RotationSupervisor(Supervisor):
    def __init__(self,c,feedforward=True):
        super().__init__(c);self.inputs=RotationInputs(c);self.used=False
        self.follower=RotationSession(c.follower_config(),c.project_goal,c.project_reference,feedforward=feedforward)
    def request_start(self,steady,ros):
        if self.used:return False,'SIM_ROTATION_ONE_ENGAGEMENT_ONLY'
        return super().request_start(steady,ros)
    def request_recover(self,steady,ros):
        if self.used:return False,'New explicitly reviewed SIM session required after this trial'
        return super().request_recover(steady,ros)
    def tick(self,steady,ros):
        frame=super().tick(steady,ros)
        if self.state=='ACTIVE':
            self.used=True
            if abs(self.follower.goal.yaw-self.follower.car_continuous)>MAX_ANGULAR_TRACKING_ERROR:
                self.request_stop(steady,'ROTATION_TRACKING_ERROR',fault=True);return self.frame(steady,ros)
        return frame


class RotationFinalGate(FinalGate):
    def __init__(self,c):
        super().__init__(c);self.inputs=RotationInputs(c);self.angle_epoch='';self.car_anchor=None;self.angle_started=None
        self.wand_anchor=None
    def reference_valid(self,frame):
        if not super().reference_valid(frame):return False
        if self.reference_sample is None:
            return abs(wrap_angle(frame.goal.yaw-self.inputs.poses['car']['pose'].yaw))<=.025
        goal,stamp=self.reference_sample;dt=frame.issued_steady-stamp
        if dt<0 or abs(frame.goal.yaw-goal.yaw)>self.c.max_yaw_rate_radps*dt+1e-7:return False
        # This virtual harness supplies the same timestamped samples to BOTH
        # cores. Independently reconstruct intent, not just a permitted rate.
        # A real ROS migration must carry/bind that selected source identity;
        # this pure test is not evidence that such a wire adapter is ready.
        if self.car_anchor is None or self.wand_anchor is None:return False
        car0,yaw0=self.car_anchor
        intended=yaw0+self.inputs.wand_continuous-self.wand_anchor
        measured=yaw0+self.inputs.car_continuous-car0
        candidate=goal.yaw+clamp(intended-goal.yaw,-self.c.max_yaw_rate_radps*dt,self.c.max_yaw_rate_radps*dt)
        expected=goal.yaw if abs(candidate-measured)>ANGULAR_LEAD else candidate
        return abs(frame.goal.yaw-expected)<=1e-6
    def tick(self,steady,ros):
        output=super().tick(steady,ros);f=self.frame
        if f and f.enabled and f.state=='ACTIVE' and not self.fault and not self.hard_fault:
            if self.angle_epoch!=f.epoch:
                if self.angle_epoch:
                    self.trip('ROTATION_ALREADY_USED');return ZERO
                if output!=ZERO or f.command!=ZERO:
                    self.trip('ROTATION_INITIAL_REFERENCE_NOT_NEUTRAL');return ZERO
                self.angle_epoch=f.epoch;self.angle_started=steady
                self.car_anchor=(self.inputs.car_continuous,f.goal.yaw)
                self.wand_anchor=self.inputs.wand_continuous
            if steady-self.angle_started>=120.:
                self.trip('ROTATION_TIME_LIMIT_120S');return ZERO
            if max(abs(v) for v in wheel_rpm(f.command))>MAX_RPM+1e-6:
                self.trip('ROTATION_WHEEL_SPEED_LIMIT');return ZERO
            origin,target=self.car_anchor
            measured=target+self.inputs.car_continuous-origin
            if abs(f.goal.yaw-measured)>MAX_ANGULAR_TRACKING_ERROR:
                self.trip('ROTATION_TRACKING_ERROR');return ZERO
        return output
