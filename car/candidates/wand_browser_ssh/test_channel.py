"""No ROS init/socket: challenge, intent and original-clock regressions."""
import math
from types import SimpleNamespace as NS
import pytest
from channel import Channel

def opened(readonly=True):
    c=Channel(readonly);assert c.open('owner',10.);c.tick(10.);return c
def pulse(c,now=10.02,issued=10.,**kwargs):
    challenge=c.challenge(issued)
    d=dict(type='pulse',nonce=challenge['nonce'] if challenge else 'stopped',sequence=c.last_sequence+1,
        focused=True,visible=True,space=False,ui_gap_ms=20.,events=[])
    d.update(kwargs);return c.receive('owner',d,now)

def test_original_time_not_receipt_is_authority_boundary():
    c=opened();assert pulse(c,10.08)
    assert c.evidence(10.099)['issued']==10.
    assert c.evidence(10.101) is None
    c.tick(10.09);c.tick(10.101)
    assert c.fault=='BROWSER_PROOF_TIMEOUT' and not pulse(c,10.11,10.11)

@pytest.mark.parametrize('field,value',[('sequence',True),('sequence',-1),('focused',False),
    ('visible',False),('space',1),('ui_gap_ms',math.nan),('ui_gap_ms',101.),('ui_gap_ms',-1.),
    ('events',[{'kind':'arbitrary'}]),('events',[{'kind':'press','extra':True}]),
    ('events',[{'kind':'press'}]*9),('nonce',[]),('type','enable')])
def test_bad_message_revokes(field,value):
    c=opened();assert not pulse(c,**{field:value});assert c.stopped and c.evidence(10.02) is None

def test_challenge_not_reusable_or_retimestamped():
    c=opened();nonce=c.challenge(10.)['nonce'];assert pulse(c,nonce=nonce)
    assert not pulse(c,10.03,10.03,nonce=nonce) and c.stopped

def test_late_reply_rejected():
    c=opened();assert not pulse(c,10.101);assert c.stopped

def test_sequence_must_progress():
    c=opened();assert pulse(c);assert not pulse(c,10.03,10.03,sequence=0)

def test_server265ms_gap_not_hidden_by_queued_fresh_proof():
    c=opened();assert pulse(c);c.tick(10.02);c.tick(10.285)
    assert c.fault=='SERVER_CONTROL_LOOP_GAP' and not pulse(c,10.285,10.285)

def test_client265ms_gap_revokes():
    c=opened();assert not pulse(c,ui_gap_ms=265.) and c.stopped

def test_readonly_keys_count_without_actions_or_authority():
    c=opened();assert pulse(c,events=[{'kind':'press'},{'kind':'release'}])
    assert not c.events and c.key_events=={'press':1,'release':1} and c.readonly

def test_readonly_has_no_setpoint_or_hardware_action():
    c=opened();assert not pulse(c,events=[dict(kind='setpoint',x=0.,y=0.,yaw=0.,visible=True)])

def test_second_connection_cannot_replace_owner_even_after_fault():
    c=opened();assert not c.open('second',10.01)
    assert not c.receive('second',{},10.02) and not c.stopped
    c.disconnect('owner');assert not c.open('third',10.03) and c.stopped

@pytest.mark.parametrize('kind',['stop','estop'])
def test_stop_has_priority_over_queued_start(kind):
    c=opened(False);assert not pulse(c,events=[{'kind':'press'},{'kind':'release'},{'kind':kind}])
    assert c.stopped and not c.events

def test_stale_stop_is_always_safe():
    c=opened(False);c.receive('owner',dict(type='stop',reason='ESC'),50.)
    assert c.fault=='CLIENT_ESC' and c.stopped

def test_nonce_storage_and_event_queue_bounded():
    c=opened(False)
    for n in range(100):c.challenge(10.+n*.001)
    assert len(c.nonces)<=32
    for n in range(8):assert pulse(c,10.1+n*.001,10.1+n*.001,events=[{'kind':'press'}]*8)
    assert len(c.events)==64
    assert not pulse(c,10.11,10.11,events=[{'kind':'press'}]) and not c.events

@pytest.mark.parametrize('x,y,yaw',[(math.inf,0.,0.),(6.,0.,0.),(0.,0.,4.),('1',0.,0.)])
def test_software_setpoint_finite(x,y,yaw):
    c=opened(False);assert not pulse(c,events=[dict(kind='setpoint',x=x,y=y,yaw=yaw,visible=True)])

def test_software_inputs_preserve_event_order():
    c=opened(False);actions=[{'kind':'press'},{'kind':'release'},dict(kind='setpoint',x=.3,y=-.2,yaw=.4,visible=True)]
    assert pulse(c,events=actions) and list(c.events)==actions

def test_no_first_proof_times_out_without_authority():
    c=opened();c.tick(12.1);assert c.fault=='INITIAL_BROWSER_PROOF_TIMEOUT'

def test_held_lease_requires_new_original_time_evidence():
    # Actual adapter implementation, fake publishers only; no ROS initialized.
    from browser_operator import BrowserOperatorNode
    c=opened(False);assert pulse(c)
    published=[];n=BrowserOperatorNode.__new__(BrowserOperatorNode)
    n.channel=c;n.last_published_generation=-1;n.sequence=0;n.owner='software'
    n.model=NS(held=True,tick=lambda _:None)
    def end(*_):n.model.held=False
    n.model.end_trial=end;n.lease_pub=NS(publish=lambda m:published.append(__import__('json').loads(m.data)))
    n.clocks=lambda:(10.03,100.03);n.heartbeat();n.clocks=lambda:(10.06,100.06);n.heartbeat()
    assert len(published)==1 and published[0]['issued_steady']==10. and published[0]['issued_ros']==100.
    n.clocks=lambda:(10.101,100.101);n.heartbeat()
    assert not n.model.held and not published[-1]['held']
