import hashlib
import json
from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import trs_static_fusion_policy as p
from trs_static_fusion_bench import load_backup
from mavlink_readonly_probe import decode_parameter
import usb_combined_fusion_policy as old


def encoder():
    return m.MAVLink(None, srcSystem=250, srcComponent=191)


def gates():
    return dict(route='usb', safe=True, authorized=True, backup_verified=True,
                locked_static=True, receive_time_verified=True)


@pytest.mark.parametrize('name,value', [('EKF2_EV_CTRL',11),('EKF2_EV_DELAY',65.)])
def test_two_independent_typed_writes(name,value):
    msg=p.parameter_message(encoder(),name,value)
    assert decode_parameter(msg)==value and p.allowed(msg,**gates())
    for key in ('safe','authorized','backup_verified','locked_static','receive_time_verified'):
        assert not p.allowed(msg,**{**gates(),key:False})
    assert not p.allowed(msg,**{**gates(),'route':'trs'})
    msg.target_system=2
    assert not p.allowed(msg,**gates())


@pytest.mark.parametrize('name', ['EKF2_EV_CTRL','EKF2_EV_DELAY'])
def test_restore_independent_of_source_clock_and_trs(name):
    msg=p.parameter_message(encoder(),name,0)
    gate=dict(route='usb',authorized=True,backup_verified=True,restore_safe=True)
    assert p.allowed(msg,**gate)
    for key in ('authorized','backup_verified','restore_safe'):
        assert not p.allowed(msg,**{**gate,key:False})


@pytest.mark.parametrize('name,value',[('EKF2_EV_CTRL',True),('EKF2_EV_CTRL',15),
    ('EKF2_EV_CTRL',11.),('EKF2_EV_DELAY',True),('EKF2_EV_DELAY',60),('EKF2_HGT_REF',2)])
def test_other_writes_rejected(name,value):
    with pytest.raises(ValueError):
        p.parameter_message(encoder(),name,value)


def test_old_gate_does_not_accept_new_delay_profile():
    msg=p.parameter_message(encoder(),'EKF2_EV_DELAY',65.)
    assert not old.allowed(msg,authorized=True,safe=True,backup_verified=True,clock_verified=True)
    msg=p.parameter_message(encoder(),'EKF2_EV_CTRL',11)
    assert not old.allowed(msg,authorized=True,safe=True,backup_verified=True)


def test_static_pose_only_trs_no_usb_or_motion():
    msg,*_=p.compact.message(encoder(),123,[1,2,3],[1,0,0,0],0)
    gate={**gates(),'route':'trs'}
    assert p.allowed(msg,**gate)
    assert not p.allowed(msg,**gates())
    assert not p.allowed(msg,**{**gate,'locked_static':False})
    for command in (400,176,21,22,185,241,246,511):
        msg=encoder().command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
        assert not p.allowed(msg,**gate)
    assert not p.allowed(encoder().timesync_encode(123,456),**gate)


def test_static_body_and_receive_mode_boundaries():
    assert p.stationary([0,0,0],[1,0,0,0],[0,0,0],[1,0,0,0])
    assert not p.stationary([.02,0,0],[1,0,0,0],[0,0,0],[1,0,0,0])
    assert not p.stationary([0,0,0],[0,0,0,0],[0,0,0],[1,0,0,0])
    assert not p.stationary([float('nan'),0,0],[1,0,0,0],[0,0,0],[1,0,0,0])
    assert p.receive_time_mode({'timestamp':10002,'timestamp_sample':10000})
    assert not p.receive_time_mode({'timestamp':10002,'timestamp_sample':10003})
    assert not p.receive_time_mode({'timestamp':10002,'timestamp_sample':8000})


def test_fixed_shell_usb_only():
    payload=p.COMMANDS['selector']
    msg=encoder().serial_control_encode(10,2,0,0,len(payload),payload.ljust(70,b'\0'))
    assert p.allowed(msg,**gates())
    assert not p.allowed(msg,**{**gates(),'route':'trs'})
    bad=b'commander arm\n'
    msg=encoder().serial_control_encode(10,2,0,0,len(bad),bad.ljust(70,b'\0'))
    assert not p.allowed(msg,**gates())


def test_backup_validates_indices_hash_and_baseline(tmp_path):
    names=list(p.READ_PARAMS)+[f'P{i}' for i in range(926-len(p.READ_PARAMS))]
    values={n:0 for n in names}
    values['EKF2_BARO_CTRL']=1
    record={'complete':True,'received_count':926,'expected_count':926,'missing_indices':[],
            'firmware':{'version':'1.14.3'},'system_id':1,'component_id':1,
            'records':[{'name':n,'value':values[n],'index':i} for i,n in enumerate(names)]}
    def save():
        raw=json.dumps(record).encode()
        (tmp_path/'parameters_verified.json').write_bytes(raw)
        (tmp_path/'parameters_manifest.json').write_text(json.dumps({'sha256':{
            'parameters_verified.json':hashlib.sha256(raw).hexdigest()}}))
    save()
    assert load_backup(tmp_path)['EKF2_EV_DELAY']==0
    record['records'][-1]['index']=0
    save()
    with pytest.raises(ValueError): load_backup(tmp_path)
    record['records'][-1]['index']=925
    record['records'][names.index('EKF2_EV_DELAY')]['value']=65
    save()
    with pytest.raises(ValueError): load_backup(tmp_path)
