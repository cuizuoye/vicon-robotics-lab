"""Pinned rebuilt Sign definition; actual IDs retained, no source restamping.

Process-local SIM adapter. Reuses every original packet geometry/age check,
then relocates the control point to the newly numbered physical T junction.
"""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import math
import numpy as np

FILE=Path(__file__).with_name('sign_geometry.json')
SHA='00b68b558cccf6555a15db9da2c5db923d78f60003bf6411b44da760f3e5e2bf'
_installed=False

def definition():
    from mecanum_wand_4wsc_control.rotation_geometry import identify_t
    raw=FILE.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=SHA:raise ValueError('SIM12 Sign definition changed; review required')
    value=json.loads(raw)
    if value['bar']!=['Sign1','Sign4','Sign3'] or value['junction']!='Sign4' or value['stem']!=['Sign2','Sign5']:
        raise ValueError('rebuilt Sign identity mismatch')
    candidates=identify_t(value['local_points'])
    if len(candidates)!=1 or candidates[0].bar!=tuple(value['bar']):raise ValueError('ambiguous physical T shape')
    if np.linalg.norm(np.array(candidates[0].forward)-value['forward_local'])>1e-9:raise ValueError('forward must point away from stem')
    return value

def install_geometry():
    global _installed
    if _installed:return
    from mecanum_wand_4wsc_control import rotation_source as source
    from mecanum_wand_4wsc_control.rotation_geometry import rotation,vector
    from mecanum_wand_4wsc_control.core import Pose
    data=definition();original=source.Packet.pose
    source.definition=lambda:data
    def pose(packet,now,ros):
        accepted=original(packet,now,ros)  # Original 8mm, age, frame, attitude checks.
        s=packet.samples[0];p=vector(s.position)+rotation(s.quaternion)@vector(data['local_points'][data['junction']])
        return Pose(*map(float,p),accepted.yaw)
    source.Packet.pose=pose
    _installed=True

def synthetic_packet(x,y,yaw,now,ros,z=1.3):
    from mecanum_wand_4wsc_control.rotation_geometry import REQUIRED,rotation
    from mecanum_wand_4wsc_control.rotation_source import Raw
    from mecanum_wand_4wsc_control.cached_packet import CachedPacket
    data=definition();q=(0.,0.,math.sin(yaw/2),math.cos(yaw/2));r=rotation(q)
    origin=np.array([x,y,z])-r@np.array(data['local_points'][data['junction']])
    return CachedPacket((Raw('sign',tuple(origin),q,ros,now),*(
        Raw(k,tuple(origin+r@np.array(data['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))
