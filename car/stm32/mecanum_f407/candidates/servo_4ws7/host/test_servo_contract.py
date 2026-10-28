import ast
import json
import os
from pathlib import Path
import struct
import zlib
import sys
import pytest
from servo_contract import (AUTHORITY,HEADER,SERVO,SIZE,STAGES,TAG,decode_status,
                            decode_status_frame,tier_payload)


@pytest.fixture
def active():
    return bytearray((Path(os.environ['SERVO_FIXTURES'])/'active.bin').read_bytes())


def decode(data):
    return decode_status(data,synthetic_native_fixture=True)


def test_real_c_fixture_and_non_live_default(active):
    s=decode(active)
    assert s['stage']==6 and s['permitted'] and any(s['raw_duty'])
    assert not s['hardware_released']
    with pytest.raises(ValueError,match='no hardware'):decode_status(active)


def test_real_c_neutral_fixture():
    data=(Path(os.environ['SERVO_FIXTURES'])/'reversing.bin').read_bytes()
    s=decode(data)
    assert s['reversing_mask']==15 and not any(s['raw_duty'])


@pytest.mark.parametrize('stage',range(7))
def test_explicit_stage_command(stage):
    assert tier_payload(stage)==struct.pack('<BBH',0,0,0x7000+stage)
    assert tier_payload(stage,enable=True)==struct.pack('<BBH',1,0,0x7000+stage)


@pytest.mark.parametrize('stage',[-1,7,True,1.0,'6',None])
def test_invalid_stage_not_implicitly_coerced(stage):
    with pytest.raises(ValueError):tier_payload(stage)


@pytest.mark.parametrize('size',[100,132,156,183,185,256])
def test_legacy_or_wrong_size_rejected(active,size):
    data=(active+bytearray(300))[:size]
    with pytest.raises(ValueError):decode(data)


@pytest.mark.parametrize('offset,fmt,value',[
    (0,'I',int.from_bytes(b'4W20','little')),(0,'I',int.from_bytes(b'4WGD','little')),
    (152,'B',16),(153,'B',1),(154,'H',1),
    (156,'f',float('nan')),(160,'f',float('inf')),(164,'f',.2),
    (168,'f',1.9),(172,'f',1200),(176,'H',350),(178,'B',7),
    (179,'B',0),(180,'B',16),(181,'B',1),(182,'B',16),(183,'B',1),
    (64,'h',810),(64,'h',123),(72,'h',7100),(94,'B',16),(95,'B',5),
    (96,'H',1),(98,'H',0),(100,'I',123),(132,'f',float('nan')),
    (88,'h',200),(72,'h',100),
    (148,'b',0),(32,'I',99)])
def test_tampered_sources_authority_or_limits_rejected(active,offset,fmt,value):
    struct.pack_into('<'+fmt,active,offset,value)
    with pytest.raises(ValueError):decode(active)


def test_crc_and_frame_identity(active):
    header=HEADER.pack(b'MC',1,0x31,2,1,17,1000,SIZE)
    frame=header+active
    frame+=struct.pack('<I',zlib.crc32(frame))
    assert decode_status_frame(frame,synthetic_native_fixture=True)['tag']=='4WS7'
    bad=bytearray(frame);bad[-1]^=1
    with pytest.raises(ValueError,match='CRC'):decode_status_frame(bad,synthetic_native_fixture=True)


def test_no_hardware_modules_or_executable_hardware_contract():
    here=Path(__file__).parent
    tree=ast.parse((here/'servo_contract.py').read_text())
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    imports += [a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
    assert not any(x and x.split('.')[0] in ('socket','serial','rclpy','subprocess') for x in imports)
    assert SIZE==184 and TAG==int.from_bytes(b'4WS7','little')
    assert STAGES[-1].speed==.7 and STAGES[-1].accel==2


def test_original_rectangle_decoder_still_rejects_new_identity(active):
    workspace=Path(__file__).resolve().parents[5]
    sys.path.insert(0,str(workspace/'src/mecanum_vicon_control'))
    from mecanum_vicon_control.motion_link_status import decode_motion_status
    from mecanum_vicon_control.transport_protocol import ProtocolError
    with pytest.raises(ProtocolError):decode_motion_status(active)


def test_draft_contract_retains_geometry_and_unknown_hardware_evidence():
    spec=json.loads((Path(__file__).parents[1]/'offline_contract.json').read_text())
    assert spec['mapping_scale']==1 and spec['goal_area_m']==[3,3]
    assert spec['buffer_each_side_m']==.5
    assert not spec['hardware_released'] and not spec['flash_authorized']
    assert not spec['live_launcher_implemented'] and not spec['thermal_acceptance_passed']
    assert spec['measured_loaded_speed_mps'] is None
    assert spec['measured_motor_rms_a']==[None]*4
    for draft,stage in zip(spec['stages'],STAGES):
        assert (draft['speed_mps'],draft['accel_mps2'],draft['pwm_limit_permille'])==(stage.speed,stage.accel,stage.duty)
        assert not draft['hardware_approved']
