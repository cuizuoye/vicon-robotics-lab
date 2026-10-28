"""Same-frame raw source envelope. Pure validation; no ROS or hardware I/O."""
from collections import OrderedDict
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path
from .core import Pose
from .rotation_geometry import REQUIRED, rotation, vector

GEOMETRY=Path(__file__).with_name('rotation_geometry.json')


def definition():
    d=json.loads(GEOMETRY.read_text())
    if d['bar']!=['Sign1','Sign5','Sign2'] or d['junction']!='Sign5':raise ValueError('SIGN_DEFINITION')
    return d


def canonical(value):
    return json.dumps(value,sort_keys=True,allow_nan=False,separators=(',',':'))


@dataclass(frozen=True)
class Raw:
    name:str
    position:tuple
    quaternion:tuple|None
    stamp:float
    received:float


@dataclass(frozen=True)
class Packet:
    samples:tuple
    @property
    def digest(self):return hashlib.sha256(canonical([asdict(s) for s in self.samples]).encode()).hexdigest()
    @property
    def stamp(self):return self.samples[0].stamp
    @property
    def received(self):return min(s.received for s in self.samples)
    def payload(self):return dict(samples=[asdict(s) for s in self.samples],digest=self.digest)
    def pose(self,now,ros):
        if tuple(s.name for s in self.samples)!=REQUIRED:raise ValueError('SIGN_PACKET_NAMES')
        for s in self.samples:
            if (s.stamp!=self.stamp or type(s.stamp) not in (int,float) or type(s.received) not in (int,float)
                    or not math.isfinite(s.stamp) or not math.isfinite(s.received) or s.stamp<=0
                    or not 0<=now-s.received<=.20 or not -.005<=ros-s.stamp<=.20):
                raise ValueError('SIGN_PACKET_AGE_OR_MIXED_FRAME')
            vector(s.position)
        d=definition();s=self.samples[0];r=rotation(s.quaternion);o=vector(s.position)
        for v in self.samples[1:]:
            if v.quaternion is not None or math.dist(r.T@(vector(v.position)-o),d['local_points'][v.name])>.008:
                raise ValueError('SIGN_GEOMETRY_OR_ATTITUDE_FLIP')
        forward=r@vector(d['forward_local'])
        if math.hypot(forward[0],forward[1])<.35:raise ValueError('SIGN_HORIZONTAL_DEGENERACY')
        p=o+r@vector(d['local_points']['Sign5'])
        return Pose(*map(float,p),math.atan2(forward[1],forward[0]))


def decode_packet(value,now,ros):
    if not isinstance(value,dict) or set(value)!={'samples','digest'} or len(value['samples'])!=6:
        raise ValueError('SIGN_PACKET_SCHEMA')
    samples=[]
    for s in value['samples']:
        if set(s)!={'name','position','quaternion','stamp','received'}:raise ValueError('SIGN_SAMPLE_SCHEMA')
        samples.append(Raw(s['name'],tuple(s['position']),None if s['quaternion'] is None else tuple(s['quaternion']),s['stamp'],s['received']))
    p=Packet(tuple(samples));p.pose(now,ros)
    if p.digest!=value['digest']:raise ValueError('SIGN_PACKET_DIGEST')
    return p


class PacketCache:
    """Independent per-consumer bounded history; replay cannot refresh an age."""
    def __init__(self):self.packets=OrderedDict();self.last_stamp=-math.inf
    def add(self,p,now,ros):
        p.pose(now,ros)
        if p.stamp<=self.last_stamp:return False
        self.packets[p.digest]=p;self.last_stamp=p.stamp
        while len(self.packets)>64:self.packets.popitem(last=False)
        return True
    def latest(self,now,ros):
        if not self.packets:return None
        p=next(reversed(self.packets.values()));p.pose(now,ros);return p
    def exact(self,key,stamp,now,ros):
        p=self.packets.get(key)
        if p is None:return None
        if p.stamp!=stamp:raise ValueError('SIGN_SELECTED_STAMP')
        p.pose(now,ros);return p
