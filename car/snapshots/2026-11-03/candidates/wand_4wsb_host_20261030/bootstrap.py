"""Import only the independent prepared overlay, never install into old workspace."""
import os
from pathlib import Path
import sys
from prepare import ROOT, CONTROL, HARDWARE

OUT = Path(os.environ.get('WAND_4WSB_HOST',str(ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9'))).resolve()
for package in (CONTROL,HARDWARE):
    sys.path.insert(0,str(OUT/'runtime/src'/package))
sys.path.insert(0,str(ROOT/'src/mecanum_vicon_control'))


def config(stage=0):
    import json
    from mecanum_wand_4wsb_control.tracking_config import TrackingSafetyConfig
    path = OUT/'runtime/src'/HARDWARE/'config'/f'stage{stage}.NOT_APPROVED.json'
    return TrackingSafetyConfig(**json.loads(path.read_text())['safety'])


def synthetic_packet(x,y,yaw,now,ros,z=1.2):
    import math
    import numpy as np
    from mecanum_wand_4wsb_control.rotation_source import Raw,definition
    from mecanum_wand_4wsb_control.cached_packet import CachedPacket
    from mecanum_wand_4wsb_control.rotation_geometry import REQUIRED,rotation
    d = definition(); q = (0.,0.,math.sin(yaw/2),math.cos(yaw/2))
    r = rotation(q)
    origin = np.array([x,y,z])-r@np.array(d['local_points']['Sign5'])
    return CachedPacket((Raw('sign',tuple(origin),q,ros,now),*(
        Raw(k,tuple(origin+r@np.array(d['local_points'][k])),None,ros,now) for k in REQUIRED[1:])))
