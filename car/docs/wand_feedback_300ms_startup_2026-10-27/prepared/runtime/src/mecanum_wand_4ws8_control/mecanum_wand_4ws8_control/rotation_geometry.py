"""Pure 5-point geometry and relative heading preview. NO actuation API."""
from dataclasses import dataclass
from itertools import combinations
import math
import numpy as np

IDS=tuple(f'Sign{i}' for i in range(1,6))
REQUIRED=('sign',*IDS)
CAR_YAW_OFFSET=2.356194490192345


def vector(value,size=3):
    a=np.asarray(value,dtype=float)
    if a.shape!=(size,) or not np.isfinite(a).all():raise ValueError('坐标不是有限数值')
    return a


def rotation(q):
    x,y,z,w=vector(q,4);norm=math.sqrt(x*x+y*y+z*z+w*w)
    if not .99<=norm<=1.01:raise ValueError('四元数模长异常')
    x,y,z,w=(v/norm for v in (x,y,z,w))
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def wrap(angle):return math.atan2(math.sin(angle),math.cos(angle))


@dataclass(frozen=True)
class TShape:
    bar:tuple
    stem:tuple
    local_points:dict
    forward:tuple
    width:float
    residual:float

    def summary(self):
        return dict(bar=list(self.bar),junction=self.bar[1],stem=list(self.stem),
            forward_local=list(self.forward),width_m=self.width,line_residual_m=self.residual,
            local_points=self.local_points)


def identify_t(points):
    """Propose ALL plausible T layouts. Never choose an ambiguous layout."""
    if set(points)!=set(IDS):raise ValueError('需要固定编号的全部五点')
    p={k:vector(v) for k,v in points.items()}
    if any(np.linalg.norm(p[a]-p[b])<.02 for a,b in combinations(IDS,2)):
        raise ValueError('标记点重合或间距过小')
    choices=[]
    for triple in combinations(IDS,3):
        a=np.array([p[k] for k in triple]);center=a.mean(axis=0)
        _,_,vh=np.linalg.svd(a-center);axis=vh[0]
        order=sorted(triple,key=lambda k:float(np.dot(p[k]-center,axis)))
        if order[0]>order[-1]:order.reverse() # Fixed IDs, not current image left/right.
        width=float(np.linalg.norm(p[order[-1]]-p[order[0]]))
        residual=float(max(np.linalg.norm(v-center-np.dot(v-center,axis)*axis) for v in a))
        if width<.08 or residual>.004:continue
        middle=p[order[1]];other=tuple(k for k in IDS if k not in triple)
        d=[p[k]-middle for k in other]
        perpendicular=[v-np.dot(v,axis)*axis for v in d]
        lengths=[float(np.linalg.norm(v)) for v in perpendicular]
        if min(lengths)<.04 or max(lengths)>1.2 or max(lengths)/min(lengths)<1.3:continue
        if np.dot(perpendicular[0],perpendicular[1])/math.prod(lengths)<.97:continue
        stem_line=p[other[1]]-p[other[0]]
        if abs(np.dot(stem_line,axis))/np.linalg.norm(stem_line)>.20:continue
        if max(abs(np.dot(v,axis)) for v in d)>.020:continue
        forward=-sum(perpendicular)/2;forward/=np.linalg.norm(forward)
        choices.append(TShape(tuple(order),other,{k:p[k].tolist() for k in IDS},
            tuple(float(v) for v in forward),width,residual))
    return choices


@dataclass(frozen=True)
class Sample:
    position:tuple
    quaternion:tuple|None
    stamp:float
    received:float


class PreviewModel:
    """Coherent-frame geometry; loss/jump invalidates zero, never auto-resumes."""
    def __init__(self):
        self.latest={};self.frames={};self.history=[];self.candidates=[]
        self.definition=None;self.reference=None;self.last_angle=None;self.last_stamp=None
        self.turn=0.;self.valid=False;self.reason='等待Sign及五个点'
        self.packet=None;self.current_heading=None;self.source_ok=False
        self.last_processed=None;self.car_origin=None;self.fault=''
        self.last_ros=None;self.accepted=0;self.rejected=0;self.reference_count=0
        self.last_calibration_stamp=None;self.has_ever_valid=False

    def invalidate(self,reason):
        if self.reference is not None:self.fault=reason+'；恢复后须手动重新记录零角'
        self.valid=False;self.reason=reason;self.reference=None
        self.last_angle=self.last_stamp=None;self.turn=0.;self.car_origin=None

    def receive(self,name,position,quaternion,stamp,received,frame='map'):
        try:
            if name not in ('car',*REQUIRED) or frame!='map':raise ValueError('来源名称或坐标系不符')
            p=vector(position);q=None
            if name in ('sign','car'):rotation(quaternion);q=tuple(quaternion)
            if not all(math.isfinite(v) for v in (stamp,received)) or stamp<=0:raise ValueError('时间戳无效')
            old=self.latest.get(name)
            if old is not None and stamp<=old.stamp:return False
            sample=Sample(tuple(p),q,stamp,received);self.latest[name]=sample
            if name!='car':
                self.frames.setdefault(stamp,{})[name]=sample
                while len(self.frames)>32:del self.frames[min(self.frames)]
            self.accepted+=1;return True
        except (ValueError,TypeError):
            self.latest.pop(name,None);self.frames.clear();self.history.clear();self.candidates=[]
            self.rejected+=1;self.invalidate('无效输入：'+name);return False

    @staticmethod
    def fresh(sample,now,ros):
        return 0<=now-sample.received<=.20 and -.005<=ros-sample.stamp<=.20

    def update(self,now,ros):
        if self.last_ros is not None and ros<self.last_ros-.005:
            self.frames.clear();self.latest.clear();self.history.clear();self.candidates=[]
            self.last_calibration_stamp=None
            self.invalidate('ROS时钟回退');self.last_ros=ros;return
        self.last_ros=ros
        if not self.source_ok:
            self.history.clear();self.candidates=[]
            self.invalidate('等待唯一、已核验的Vicon发布源');return
        complete=[(stamp,frame) for stamp,frame in self.frames.items() if all(k in frame for k in REQUIRED)]
        if not complete:self.invalidate('等待同一源时间戳的Sign与五点，不能混用不同帧');return
        stamp,packet=max(complete,key=lambda x:x[0])
        if any(not self.fresh(v,now,ros) for v in packet.values()):
            self.history.clear();self.candidates=[];self.invalidate('Sign或标记点遮挡/过期');return
        # Source age, not a repeated GUI draw, determines freshness.
        self.packet=packet
        sign=packet['sign'];r=rotation(sign.quaternion);origin=vector(sign.position)
        local={k:(r.T@(vector(packet[k].position)-origin)).tolist() for k in IDS}
        if stamp!=self.last_calibration_stamp:
            self.history.append((stamp,local));self.last_calibration_stamp=stamp
            self.history=[v for v in self.history if stamp-v[0]<=.8]
        stable=(len(self.history)>=12 and stamp-self.history[0][0]>=.45)
        if stable:
            stack=np.array([[h[1][k] for k in IDS] for h in self.history])
            stable=float(np.max(np.std(stack,axis=0)))<=.003
            if stable:
                median=np.median(stack,axis=0)
                try:self.candidates=identify_t({k:median[i] for i,k in enumerate(IDS)})
                except ValueError:self.candidates=[]
        if self.definition is None:
            self.valid=False
            self.reason=('识别到候选，确认横杆三点后才可记录零角' if stable and self.candidates
                else '等待稳定、可唯一核对的T形几何；不猜测点号')
            return
        error=max(float(np.linalg.norm(vector(local[k])-vector(self.definition.local_points[k]))) for k in IDS)
        if error>.008:
            self.history.clear();self.invalidate('点号/刚体姿态不一致，拒绝可能的翻转');return
        forward=r@vector(self.definition.forward)
        if np.linalg.norm(forward[:2])<.35:
            self.invalidate('正方向接近竖直，水平航向不可可靠定义');return
        heading=math.atan2(forward[1],forward[0]);self.current_heading=heading
        self.valid=True;self.has_ever_valid=True;self.reason='方向有效；只读，无电机输出'
        if self.reference is not None and stamp!=self.last_stamp:
            dt=stamp-self.last_stamp;delta=wrap(heading-self.last_angle)
            if not 0<dt<=.20 or abs(delta)>min(.60,8.*dt+.015):
                self.invalidate('角度跳变或角速度异常；拒绝180度翻转');return
            self.turn+=delta;self.last_angle=heading;self.last_stamp=stamp
        self.last_processed=stamp

    def confirm_geometry(self,now,ros,index=0):
        self.update(now,ros)
        if not self.candidates or self.packet is None:raise ValueError('尚无稳定T形候选')
        if not self.source_ok or any(not self.fresh(v,now,ros) for v in self.packet.values()):
            raise ValueError('点位或来源不再有效；不能确认旧候选')
        if type(index) is not int or not 0<=index<len(self.candidates):raise ValueError('未明确选择候选')
        self.definition=self.candidates[index];self.reference=None;self.fault=''
        self.last_angle=self.last_stamp=None;self.car_origin=None;self.turn=0.
        return self.definition

    def zero(self,now,ros):
        self.update(now,ros)
        car=self.latest.get('car')
        if not self.valid or self.current_heading is None:raise ValueError(self.reason)
        if car is None or not self.fresh(car,now,ros):raise ValueError('记录相对车头需要新鲜car01位姿')
        r=rotation(car.quaternion)
        if math.hypot(r[0,0],r[1,0])<.5:raise ValueError('car01航向无效')
        self.car_origin=wrap(math.atan2(r[1,0],r[0,0])+CAR_YAW_OFFSET)
        self.reference=self.last_angle=self.current_heading
        self.last_stamp=self.packet['sign'].stamp;self.turn=0.;self.fault='';self.reference_count+=1

    def clear_zero(self):self.invalidate('已暂停/清除零角；不会自动恢复')

    def report(self):
        return dict(kind='SIGN_ROTATION_READONLY',valid=self.valid,reason=self.reason,fault=self.fault,
            bar=self.definition.summary() if self.definition else None,
            candidates=[v.summary() for v in self.candidates],
            heading_deg=math.degrees(self.current_heading) if self.valid else None,
            relative_deg=math.degrees(self.turn) if self.reference is not None and self.valid else None,
            target_car_heading_deg=math.degrees(self.car_origin+self.turn)
                if self.car_origin is not None and self.reference is not None and self.valid else None,
            references=self.reference_count,accepted=self.accepted,rejected=self.rejected,
            actuator_outputs=False,car_commanded=False)
