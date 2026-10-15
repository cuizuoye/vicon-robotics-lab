import json
from pathlib import Path
import sys
import time

import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'ros2_ws/src/uav_coord_bridge'))
import trs_vision_reference_policy as p
import trs_vision_reference_permit as permit
from trs_static_fusion_bench import load_backup
from compact_vision import message as pose_message


def enc():
    return m.MAVLink(None,srcSystem=250,srcComponent=190)


GATES=dict(route='usb',safe=True,authorized=True,backup_verified=True,
           locked_static=True,receive_time_verified=True,restore_safe=True)


def baseline():
    r=dict.fromkeys(p.READ_PARAMS,0)
    r.update(EKF2_HGT_REF=3,EKF2_EV_CTRL=2,EKF2_BARO_CTRL=1,COM_RC_IN_MODE=4,COM_OBL_RC_ACT=4)
    return r


def test_scoped_changes_and_baseline_restore():
    for name,values in [('EKF2_EV_CTRL',(2,11)),('EKF2_EV_DELAY',(0,65))]:
        for value in values:
            msg=p.parameter_message(enc(),name,value)
            assert p.allowed(msg,**GATES)
            assert not p.allowed(msg,**{**GATES,'route':'trs'})
        assert p.allowed(p.parameter_message(enc(),name,values[0]),**{**GATES,'safe':False,'locked_static':False,'receive_time_verified':False})
        assert not p.allowed(p.parameter_message(enc(),name,values[1]),**{**GATES,'receive_time_verified':False})
    for name,value in [('EKF2_EV_CTRL',0),('EKF2_EV_CTRL',3),('EKF2_HGT_REF',3)]:
        with pytest.raises(ValueError): p.parameter_message(enc(),name,value)


def test_reference_and_pose_scope():
    b=baseline()
    assert p.scope_ok(b,b)
    assert p.scope_ok({**b,'EKF2_EV_CTRL':11,'EKF2_EV_DELAY':65},b)
    for key,value in [('EKF2_HGT_REF',2),('EKF2_RNG_CTRL',1),('COM_RC_IN_MODE',0),('EKF2_EV_CTRL',0)]:
        assert not p.scope_ok({**b,key:value},b)
    msg,_,_=pose_message(enc(),123,[0,0,0],[1,0,0,0],0)
    assert p.allowed(msg,**{**GATES,'route':'trs'})
    assert not p.allowed(msg,**GATES)
    assert not p.allowed(msg,**{**GATES,'route':'trs','locked_static':False})


def test_restore_only_rejects_enables_reference_writes_and_commands():
    from trs_vision_reference_restore import allowed
    assert allowed(p.parameter_message(enc(),'EKF2_EV_CTRL',2),safe=True,locked=True)
    assert not allowed(p.parameter_message(enc(),'EKF2_EV_CTRL',2),safe=True,locked=False)
    assert not allowed(p.parameter_message(enc(),'EKF2_EV_CTRL',11),safe=True,locked=True)
    assert not allowed(p.parameter_message(enc(),'EKF2_EV_DELAY',0),safe=True,locked=True)
    assert not allowed(enc().command_long_encode(1,1,400,0,1,0,0,0,0,0,0),safe=True,locked=True)


@pytest.mark.parametrize('command',[176,185,241,246,400,511])
def test_no_control_commands(command):
    msg=enc().command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
    assert not p.allowed(msg,**GATES)


def fixture(tmp_path,monkeypatch):
    monkeypatch.setattr(permit,'TOOLS',tmp_path)
    (tmp_path/'known_incidents').mkdir()
    for name,value in [('micoair743_v1143_shell_crash.json',{'block_active_benches':True}),
                        ('physical_safety_hold.json',{'block_props_off_benches':True})]:
        (tmp_path/'known_incidents'/name).write_text(json.dumps(value))
    for name in ('trs_static_fusion_bench.py','trs_vision_reference_policy.py','trs_static_fusion_policy.py',
        'vision_reference_xyz_policy.py','compact_vision.py','usb_combined_fusion_policy.py','usb_fusion_policy.py',
        'disarmed_offboard_policy.py','heading','board'):
        (tmp_path/name).touch()
    acceptance=tmp_path/'acceptance.json'
    acceptance.write_text(json.dumps(dict(vision_reference_static_xyz_verified=True,rollback_verified=True,
                                         error=None,identity={'uid2':permit.UID})))
    backup=tmp_path/'backup';backup.mkdir()
    values=baseline()
    rows=[{'name':k,'index':i,'type':6,'value':v} for i,(k,v) in enumerate(values.items())]
    rows += [{'name':f'FIXTURE_{i}','index':i,'type':6,'value':0} for i in range(len(rows),926)]
    f=backup/'parameters_verified.json'
    f.write_text(json.dumps(dict(complete=True,expected_count=926,received_count=926,missing_indices=[],
                                system_id=1,component_id=1,firmware={'version':'1.14.3'},records=rows)))
    (backup/'parameters_manifest.json').write_text(json.dumps({'sha256':{'parameters_verified.json':permit.digest(f)}}))
    output=tmp_path/'output.json'
    r={**permit.fields(output,backup,tmp_path/'heading',tmp_path/'board',acceptance),
       'issued_unix':time.time()-1,'expires_unix':time.time()+600,
       **{k:permit.digest(path) for k,path in permit.paths(backup,tmp_path/'heading',tmp_path/'board',acceptance).items()}}
    file=tmp_path/'permit.json';file.write_text(json.dumps(r))
    return file,backup,output,r


def test_permit_consumed_once_and_baseline_supported(tmp_path,monkeypatch):
    file,backup,output,_=fixture(tmp_path,monkeypatch)
    assert load_backup(backup,vision_reference_static=True)['EKF2_EV_CTRL']==2
    permit.consume(file,backup,output,tmp_path/'heading',tmp_path/'board')
    with pytest.raises(FileExistsError): permit.consume(file,backup,output,tmp_path/'heading',tmp_path/'board')
    assert json.loads((tmp_path/'known_incidents/physical_safety_hold.json').read_text())['block_props_off_benches'] is True


@pytest.mark.parametrize('key,value',[('height_reference',2),('scope','flight'),('user_authorized',False),
    ('max_fusion_seconds',60),('allowed_parameter_values',{'EKF2_EV_CTRL':[0,11]}),('runner_sha256','bad')])
def test_permit_rejects_wrong_scope(tmp_path,monkeypatch,key,value):
    file,backup,output,r=fixture(tmp_path,monkeypatch)
    file.write_text(json.dumps({**r,key:value}))
    with pytest.raises(ValueError):permit.consume(file,backup,output,tmp_path/'heading',tmp_path/'board')
    assert not Path(str(file)+'.consumed.json').exists()


def test_ground_check_requires_distinct_permit(tmp_path,monkeypatch):
    file,backup,output,r=fixture(tmp_path,monkeypatch)
    with pytest.raises(ValueError):
        permit.consume(file,backup,output,tmp_path/'heading',tmp_path/'board',ground_check=True)
    assert not Path(str(file)+'.consumed.json').exists()


def test_typed_baseline_restore_regression():
    # Backup records use float values, while integer encoders intentionally reject floats.
    b=baseline();b['EKF2_EV_CTRL']=2.0
    assert p.allowed(p.parameter_message(enc(),'EKF2_EV_CTRL',int(b['EKF2_EV_CTRL'])),**GATES)
