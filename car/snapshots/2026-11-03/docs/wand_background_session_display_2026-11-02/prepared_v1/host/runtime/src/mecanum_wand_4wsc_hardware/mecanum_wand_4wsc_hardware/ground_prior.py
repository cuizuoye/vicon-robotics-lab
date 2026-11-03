"""Pinned operator-confirmed stage6 physical trial permits a short 600s duration validation.

This does not convert normal stop observations into an emergency envelope or
invent current and thermal data. Validation is file-only before hardware IO.
"""
import hashlib
import json
from pathlib import Path

PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage6_2026-11-02/OPERATOR_REVIEW.json')
DIGEST='a7411c740824cc380af790b4978bc01970f482bf97cfce27e8a41b39517bf2fc'
IMAGE='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'

def verify():
    def need(value,reason):
        if not value:raise ValueError('prior stage6 physical observation: '+reason)
    raw=PATH.read_bytes();need(hashlib.sha256(raw).hexdigest()==DIGEST,'operator review changed')
    r=json.loads(raw)
    need(r.get('identity')=='4WSC' and r.get('image_sha256')==IMAGE and r.get('stage')==6
        and r.get('purpose')=='sign' and r.get('physical_trial') is True
        and r.get('backend_trial_passed') is True and r.get('operator_observation_reviewed') is True
        and r.get('action_and_normal_timer_stop_observation_passed') is True
        and r.get('operator_feedback')=='演示完成，也正常，后续更高加速度暂时不需要了，我认为现在的加速度已经足够，先拓展到600s。',
        'current-image operator-confirmed stage6 trial required')
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
        and result.get('passed') is True and result.get('completed') is True
        and result.get('reason')=='STAGE6_GROUND60_TIME_COMPLETE'
        and stop.get('stop_confirmed') is True and stop.get('gate_output_zero') is True
        and stop.get('actual_hardware_trial') is True,'full stage6 trial and fresh stop evidence required')
    t=stop['transitions'];start=next(v['t'] for v in t if v['state']=='ACTIVE')
    end=next(v['t'] for v in t if v['state']=='STOPPING' and v['t']>=start)
    need(59.7<=end-start<=60.5,'complete stage6 short trial required')
    c=json.loads((Path(r['raw_result']).parent/'CONTRACT.json').read_text())
    need(c.get('identity')=='4WSC' and c.get('stage')==6 and c.get('mode')=='hardware'
        and c.get('limits')==dict(speed=.7,accel=2.,yaw=2.,yaw_accel=12.)
        and c.get('max_active_s')==60.,'exact physical stage6 contract required')
    need(m.get('physical_trial') is True and m.get('stop_confirmed') is True
        and m.get('stop_observation_contiguous') is True and m.get('fresh_pose_samples',0)>100
        and m.get('qualified_long_demo') is False and m.get('physical_stop_envelope_verified') is False,
        'contiguous normal stop observation, not a full high-tier envelope')
    return r
