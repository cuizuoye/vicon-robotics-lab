"""No hardware/network/ROS startup: UAPI parser and recorded-frame analysis."""
import json
from pathlib import Path
import shutil
import socket
import struct
from types import SimpleNamespace as NS
import pytest
import wifi_readonly as w
import trace_return_path as t


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import rclpy
    def denied(*a,**k):pytest.fail('offline test attempted network/socket/ROS I/O')
    monkeypatch.setattr(socket,'socket',denied);monkeypatch.setattr(rclpy,'init',denied)


@pytest.fixture
def c(tmp_path):return w.constants(tmp_path)


@pytest.mark.parametrize('data',[b'',b'x',b'abc',b'x'*15])
def test_attribute_roundtrip_and_nested_flag(data):
    assert w.attrs(w.attr(22|0x8000,data))=={22:data}


@pytest.mark.parametrize('data',[b'x',b'abc',struct.pack('=HH',3,1),struct.pack('=HH',20,1)+b'a'])
def test_malformed_attributes_are_not_silently_accepted(data):
    with pytest.raises(ValueError):w.attrs(data)


@pytest.mark.parametrize('family,command',[(55,1),(55,2),(55,3),(55,250),(16,1),(16,2),(88,17)])
def test_final_netlink_gate_forbids_all_mutations_before_send(c,family,command):
    s=w.ReadOnlyWifi.__new__(w.ReadOnlyWifi);s.c=c;s.family=55;s.sequence=0
    s.sock=NS(sendto=lambda *a:pytest.fail('rejected request reached kernel'))
    with pytest.raises(ValueError):s.request(family,command)


def test_kernel_header_constants_do_not_invent_protocol_numbers(c):
    assert c['abi_source']=='local_linux_uapi'
    assert c['NL80211_CMD_GET_STATION']>0
    assert c['NL80211_PS_DISABLED']!=c['NL80211_PS_ENABLED']
    assert len({c[k] for k in w.GETS})==3


def test_signed_dbm_and_counter_directions_decode(c):
    s=w.ReadOnlyWifi.__new__(w.ReadOnlyWifi);s.c=c
    nested=w.attr(c['NL80211_STA_INFO_SIGNAL'],struct.pack('=b',-68))+w.attr(
        c['NL80211_STA_INFO_TX_RETRIES'],struct.pack('=I',12))+w.attr(
        c['NL80211_STA_INFO_RX_DROP_MISC'],struct.pack('=Q',3))
    r=s.station({c['NL80211_ATTR_MAC']:bytes.fromhex('98c377ad5db9'),c['NL80211_ATTR_STA_INFO']:nested})
    assert r==dict(mac='02:00:00:00:00:00',signal=-68,tx_retries=12,rx_drop_misc=3)


def test_200ms_zero_only_guard_reused_without_adding_control_or_retry():
    t.diagnostic.offline_check()
    assert t.diagnostic.MAX_WINDOW_S==60.
    sent=[];s=t.diagnostic.PermanentZeroSocket(NS(sendto=lambda *a:sent.append(a)))
    s.stop()
    with pytest.raises(OSError):s.sendto(b'any',t.diagnostic.ESP)
    assert not sent


def test_pairing_replays_real_previous_trip_not_a_new_hardware_test(tmp_path):
    source=t.ROOT/'docs/wand_feedback_guard_2026-10-27/unarmed_zero_q6smwiri'
    for name in ('kernel_receive.json','RESULT.json','native_clock.jsonl','outgoing.json'):
        shutil.copyfile(source/name,tmp_path/name)
    r=t.analyse(tmp_path,[])
    assert not r['protocol_window_completed']
    assert r['source_ages_at_stop_ms']['LINK_STATUS']>200
    assert r['queue_at_stop']['available'] is False
    assert r['physical_component_root_cause_proven'] is False
    rows=json.loads((tmp_path/'PAIRS.json').read_text())
    # Same native tick 1623591: wheel before stop, diagnostic after stop.
    target=next(v for v in rows if v['source_ms']==1623591)
    assert target['diagnostic_minus_wheel_kernel_ms']==pytest.approx(85.359835)
    assert not target['wheel_after_tx_stop'] and target['diagnostic_after_tx_stop']
    assert r['protection_ms']==200 and not r['motion_started']


def test_sampler_does_not_turn_get_failure_into_healthy_measurement():
    sampler=t.Sampler({})
    # Use the explicitly denied socket constructor, proving failure is recorded
    # without fallback to another interface, configuration or privileged tool.
    class Broken:
        def snapshot(self):raise OSError('GET denied')
    original=t.system_snapshot
    try:
        t.system_snapshot=lambda:dict(test_only=True)
        sampler.collect(Broken())
    finally:t.system_snapshot=original
    assert sampler.latest['wifi_error']=='GET denied'
    assert 'wifi' not in sampler.latest
