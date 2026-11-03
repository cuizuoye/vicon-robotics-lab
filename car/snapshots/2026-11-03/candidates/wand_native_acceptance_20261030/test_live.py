"""File-only adapter tests. No ROS context, receiver or device is started."""
from types import SimpleNamespace
import os
import pytest
import live_sign as live


def test_domain0_is_confined_to_spawned_reader(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','191')
    seen=[]
    monkeypatch.setattr(live,'passive_child',lambda *a:seen.append((os.environ['ROS_DOMAIN_ID'],a)))
    # This function is the child process entry, not called in the real parent.
    live.domain0_reader_child('channel','stop','ready')
    assert seen==[('0',('channel','stop','ready'))]


def test_only_car_is_substituted_never_sign_stamp_or_conflicts():
    packet=dict(stamp=10.,received=11.,samples=['raw-marker-data'])
    actual_car=dict(pose=dict(x=99.,y=88.,yaw=1.),stamp=12.)
    original=dict(packet=packet,car=actual_car,issued=11.5,conflicts=['raw tracking lost'],rejected={'geometry':1})
    adapter=live.SoftwareCarReader(SimpleNamespace(poll=lambda:original),
        lambda:dict(x=.2,y=.3,z=.1,yaw=.4),lambda:13.)
    value=adapter.poll()
    assert value['car']['pose']['x']==.2 and value['car']['stamp']==13.
    assert adapter.last_car_observed is actual_car and original['car'] is actual_car
    assert value['packet'] is packet and packet['stamp']==10. and packet['received']==11.
    assert value['issued']==11.5 and value['conflicts']==['raw tracking lost']
    assert value['rejected']==original['rejected']


def test_absent_raw_source_is_not_fabricated_by_software_car():
    def never():raise AssertionError('must not create synthetic Sign')
    adapter=live.SoftwareCarReader(SimpleNamespace(poll=lambda:None),never,never)
    assert adapter.poll() is None


@pytest.mark.parametrize('duration',[0.,-1.,float('nan'),float('inf'),61.])
def test_invalid_probe_duration_fails_before_poll(duration):
    reader=SimpleNamespace(poll=lambda:pytest.fail('must reject before poll'))
    with pytest.raises(ValueError):live.qualify_reader(reader,duration)


def test_private_runtime_cannot_accept_real_domain_or_public_namespace(monkeypatch):
    from split_runtime import runtime
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE','LOCALHOST')
    monkeypatch.setenv('ROS_DOMAIN_ID','0')
    with pytest.raises(ValueError):runtime('/host4wsb_0123456789',7)
    monkeypatch.setenv('ROS_DOMAIN_ID','191')
    with pytest.raises(ValueError):runtime('/real_car',7)
    assert runtime('/host4wsb_0123456789',7).software_only is True
