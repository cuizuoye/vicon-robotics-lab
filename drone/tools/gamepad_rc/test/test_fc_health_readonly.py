from pathlib import Path
import sys
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from fc_health_readonly import allowed,COMMANDS,PARAMS


def test_health_fixed_reads_only_no_fault_reset_or_param_writes():
    enc=m.MAVLink(None)
    for p in COMMANDS.values():
        msg=enc.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0'))
        assert allowed(msg,True) and not allowed(msg,False)
    for p in (b'reboot\n',b'hardfault_log fault\n',b'hardfault_log check\n',b'commander arm\n',b'param set EKF2_EV_CTRL 11\n'):
        assert not allowed(enc.serial_control_encode(10,2,0,0,len(p),p.ljust(70,b'\0')),True)
    assert not allowed(enc.param_set_encode(1,1,b'EKF2_EV_CTRL',0,6),True)
    assert not allowed(enc.serial_control_encode(10,0,0,0,0,bytes(70)),True)


def test_height_reference_is_readonly_not_implicitly_reset_to_baro():
    enc=m.MAVLink(None)
    assert 'EKF2_HGT_REF' in PARAMS
    for name in PARAMS:
        msg=enc.param_request_read_encode(1,1,name.encode(),-1)
        assert allowed(msg,True) and not allowed(msg,False)
    assert not allowed(enc.param_set_encode(1,1,b'EKF2_HGT_REF',0,6),True)


def test_sensor_profile_contains_only_fixed_single_sample_listeners():
    sensors=[payload for key,payload in COMMANDS.items() if key.startswith('sensor_')]
    assert len(sensors)==6
    assert all(p.startswith(b'listener ') and p.endswith(b' -n 1\n') for p in sensors)


def test_health_report_is_fixed_readonly_snapshot():
    assert COMMANDS['health_report'] == b'listener health_report -n 1\n'
