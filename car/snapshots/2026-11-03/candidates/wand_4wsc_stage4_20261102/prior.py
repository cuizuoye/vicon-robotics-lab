"""Pinned stage0 physical observation permits a short next-tier experiment only.

No prior thermal/current/fail-safe result is promoted into a measured approval.
File-only validation executes before actuator construction.
"""
import hashlib
import json
from pathlib import Path

PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_ground_2026-11-02/STAGE0_REVIEW_AND_PROMOTION_REQUEST.json')
DIGEST='0b7f6e433a8cec8bbcb26e5f57effa287b5567209654642c30771c6549f44d17'
IMAGE='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'

def verify():
    def need(value,reason):
        if not value:raise ValueError('prior stage0 physical observation: '+reason)
    raw=PATH.read_bytes();need(hashlib.sha256(raw).hexdigest()==DIGEST,'review changed')
    r=json.loads(raw)
    need(r.get('identity')=='4WSC' and r.get('image_sha256')==IMAGE and r.get('stage')==0
        and r.get('purpose')=='sign' and r.get('physical_trial') is True
        and r.get('initial_ground_observation_passed') is True
        and r.get('user_report','').startswith('演示正常'),'current-image observed manual trial required')
    need(r.get('qualified_long_demo') is False and r.get('physical_stop_envelope_verified') is False
        and r.get('includes_600ms_link_loss_test') is False
        and r.get('electrical_thermal_verified') is False,'do not relabel unknown qualifications')
    parts={}
    for key in ('raw_result','measurements'):
        data=Path(r[key]).read_bytes()
        need(hashlib.sha256(data).hexdigest()==r[key+'_sha256'],'raw evidence changed')
        parts[key]=json.loads(data)
    result=parts['raw_result'];stop=result['stop_evidence'];m=parts['measurements']
    need(result.get('mode')=='hardware' and result.get('hardware_socket_opened') is True
        and result.get('physical_motion_started') is True and result.get('authority_ever_acquired') is True
        and result.get('reason')=='CLIENT_STOP_BUTTON'
        and stop.get('stop_confirmed') is True and stop.get('gate_output_zero') is True
        and stop.get('actual_hardware_trial') is True,'normal operator stop with fresh stop evidence required')
    t=stop['transitions'];start=next(v['t'] for v in t if v['state']=='ACTIVE')
    end=next(v['t'] for v in t if v['state']=='STOPPING' and v['t']>=start)
    need(20.<=end-start<=60.5,'meaningful stage0 trial required')
    c=json.loads((Path(r['raw_result']).parent/'CONTRACT.json').read_text())
    need(c.get('identity')=='4WSC' and c.get('stage')==0 and c.get('mode')=='hardware'
        and c.get('limits')==dict(speed=.2,accel=.08,yaw=.2,yaw_accel=.8),'exact physical stage0 contract required')
    need(m.get('physical_trial') is True and m.get('stop_confirmed') is True
        and m.get('stop_observation_contiguous') is True and m.get('fresh_pose_samples',0)>100
        and m.get('qualified_long_demo') is False and m.get('physical_stop_envelope_verified') is False,
        'contiguous normal stop observation, not a full high-tier envelope')
    return r
