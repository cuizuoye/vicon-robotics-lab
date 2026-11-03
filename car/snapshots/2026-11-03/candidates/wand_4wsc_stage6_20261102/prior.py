"""Pinned operator-confirmed stage4 physical trial permits a short stage6 test.

This does not convert normal stop observations into an emergency envelope or
invent current and thermal data. Validation is file-only before hardware IO.
"""
import hashlib
import json
from pathlib import Path

PATH=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_4wsc_stage4_2026-11-02/OPERATOR_REVIEW.json')
DIGEST='e95a1ca732bb8d8f9079508a6e05fd342a647c152a68b3d322cb0b7efd3a3e33'
IMAGE='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'

def verify():
    def need(value,reason):
        if not value:raise ValueError('prior stage4 physical observation: '+reason)
    raw=PATH.read_bytes();need(hashlib.sha256(raw).hexdigest()==DIGEST,'operator review changed')
    r=json.loads(raw)
    need(r.get('identity')=='4WSC' and r.get('image_sha256')==IMAGE and r.get('stage')==4
        and r.get('purpose')=='sign' and r.get('physical_trial') is True
        and r.get('backend_trial_passed') is True and r.get('operator_observation_reviewed') is True
        and r.get('action_and_normal_stop_observation_passed') is True
        and r.get('operator_feedback')=='高速档演示通过，操作正常，可以继续做下一步了',
        'current-image operator-confirmed stage4 trial required')
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
        and result.get('reason')=='STAGE4_GROUND60_TIME_COMPLETE'
        and stop.get('stop_confirmed') is True and stop.get('gate_output_zero') is True
        and stop.get('actual_hardware_trial') is True,'full stage4 trial and fresh stop evidence required')
    t=stop['transitions'];start=next(v['t'] for v in t if v['state']=='ACTIVE')
    end=next(v['t'] for v in t if v['state']=='STOPPING' and v['t']>=start)
    need(59.7<=end-start<=60.5,'complete stage4 short trial required')
    c=json.loads((Path(r['raw_result']).parent/'CONTRACT.json').read_text())
    need(c.get('identity')=='4WSC' and c.get('stage')==4 and c.get('mode')=='hardware'
        and c.get('limits')==dict(speed=.7,accel=.8,yaw=1.5,yaw_accel=6.)
        and c.get('max_active_s')==60.,'exact physical stage4 contract required')
    need(m.get('physical_trial') is True and m.get('stop_confirmed') is True
        and m.get('stop_observation_contiguous') is True and m.get('fresh_pose_samples',0)>100
        and m.get('qualified_long_demo') is False and m.get('physical_stop_envelope_verified') is False,
        'contiguous normal stop observation, not a full high-tier envelope')
    return r
