from pathlib import Path
import sys
import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import trs_message_interval_trial as p


def test_only_fixed_streams_and_exact_candidate_or_restore_values():
    e=m.MAVLink(None)
    for msgid,candidate in p.CANDIDATES.items():
        for value in (candidate,1_000_000):
            msg=p.interval_message(e,msgid,value)
            assert p.allowed(msg,safe=True,locked=True,original=1_000_000)
            assert not p.allowed(msg,safe=True,original=1_000_000)
            assert not p.allowed(msg,locked=True,original=1_000_000)
        assert not p.allowed(p.interval_message(e,msgid,123),safe=True,locked=True,original=1_000_000)
        assert p.allowed(p.interval_message(e,msgid,-1),safe=True,locked=True,original=0)
    assert p.allowed(p.interval_message(e,0),safe=True)
    assert not p.allowed(p.interval_message(e,0,500000),safe=True,locked=True,original=1000000)
    with pytest.raises(ValueError):p.interval_message(e,999)


@pytest.mark.parametrize('command',[176,185,21,22,241,246,400])
def test_no_flight_stop_unlock_or_reboot(command):
    msg=m.MAVLink(None).command_long_encode(1,1,command,0,1,0,0,0,0,0,0)
    assert not p.allowed(msg,safe=True,locked=True,original=1000000)


def test_stream_command_cannot_change_global_link_rate_or_target():
    e=m.MAVLink(None)
    for target,param3 in [(1,99999),(2,0)]:
        msg=e.command_long_encode(target,1,511,0,32,100000,param3,0,0,0,0)
        assert not p.allowed(msg,safe=True,locked=True,original=1000000)


def test_measure_actual_rate_and_gap():
    assert p.rate_stats([])['rate_hz'] is None
    r=p.rate_stats([0,.1,.2])
    assert r['rate_hz']==10 and r['max_arrival_gap_ms']==100
