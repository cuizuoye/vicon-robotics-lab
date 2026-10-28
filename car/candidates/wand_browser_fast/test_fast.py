"""Freshness, ingress arbitration and priority regressions. No ROS init."""
import queue
import heapq
import math
import pytest
from types import SimpleNamespace as NS
from control_channel import Channel
from channel import Channel as OldChannel
from mailbox import Mailbox

def c():
    v=Channel();v.open('one',10.);v.tick(10.);return v
def wire(v,issued,sequence=1):
    return dict(type='pulse',nonce=v.challenge(issued)['nonce'],sequence=sequence,
        focused=True,visible=True,space=False,ui_gap_ms=20,events=[])

def test_already_queued_fresh_reply_is_checked_before_old_proof_expiry():
    v=c();v.receive('one',wire(v,10.),10.02);v.begin_tick(10.06);v.finish_tick(10.06)
    d=wire(v,10.09,2);v.begin_tick(10.102)
    assert v.receive('one',d,10.103)
    v.finish_tick(10.104);assert not v.fault and v.evidence(10.104)['issued']==10.09

def test_ingress_does_not_relax_challenge100ms_or_retimestamp():
    v=c();v.receive('one',wire(v,10.),10.02);v.begin_tick(10.06);v.finish_tick(10.06)
    d=wire(v,10.);v.begin_tick(10.11)
    assert not v.receive('one',dict(d,sequence=2),10.111)
    assert v.fault=='CHALLENGE_EXPIRED_OR_REPLAYED' and v.evidence(10.111) is None

def test_fresh_queued_input_never_masks_actual265ms_task_block():
    v=c();v.receive('one',wire(v,10.),10.02);v.tick(10.02);d=wire(v,10.27,2)
    v.begin_tick(10.285);assert v.fault=='SERVER_CONTROL_LOOP_GAP'
    assert not v.receive('one',d,10.286)

def test_no_input_still_expires_at100ms():
    v=c();v.receive('one',wire(v,10.),10.02);v.tick(10.07);v.tick(10.101)
    assert v.fault=='BROWSER_PROOF_TIMEOUT'

def test_new_nonce_cannot_restore_a_failed_session():
    v=c();d=wire(v,10.09,2);v.receive('one',wire(v,10.),10.02);v.tick(10.07);v.tick(10.101)
    assert not v.receive('one',d,10.102) and v.stopped

def test_urgent_estop_needs_no_nonce_but_never_enables():
    v=c();v.receive('one',dict(type='stop',reason='ESTOP'),12.)
    assert v.fault=='OPERATOR_ESTOP' and v.stopped and not v.events

def test_priority_mailbox_has_only_one_pending_view():
    m=Mailbox(2)
    for n in range(1000):m.put_nowait(dict(kind='state',sequence=n))
    m.put_nowait(dict(kind='challenge',nonce='original'))
    assert m.get()['nonce']=='original';m.task_done()
    assert m.get()['sequence']==999;m.task_done()
    assert m.max_pending_views==1 and m.coalesced==999 and m.unfinished_tasks==0

def test_critical_actions_are_never_coalesced_or_silently_dropped():
    m=Mailbox(2);m.put_nowait(dict(kind='close'));m.put_nowait(dict(kind='challenge'))
    with pytest.raises(queue.Full):m.put_nowait(dict(kind='ready'))
    assert [m.get()['kind'],m.get()['kind']]==['close','challenge']

def test_terminal_result_overtakes_pending_challenge_and_view_even_when_full():
    m=Mailbox(1);m.put_nowait(dict(kind='challenge'));m.put_nowait(dict(kind='state'))
    m.put_nowait(dict(kind='result',passed=False));assert m.get()==dict(kind='result',passed=False)
    m.task_done();assert not m.critical and m.view is None and m.unfinished_tasks==0

def simulated_link(fast,oneway=.040):
    v=Channel() if fast else OldChannel();v.open('one',0.);v.tick(0.)
    arrivals=[];seq=0;period=10 if fast else 20
    for ms in range(5,2001,5):
        now=ms/1000
        if ms%period==0:
            d=wire(v,now,0)
            receipt=now+oneway
            # Old sender waits for its next20ms interval, phase shifted5ms.
            if not fast:receipt=.005+math.ceil((receipt-.005)/.020-1e-8)*.020
            seq+=1;d['sequence']=seq;heapq.heappush(arrivals,(receipt+oneway,seq,d))
        if fast:v.begin_tick(now)
        else:v.tick(now)
        while arrivals and arrivals[0][0]<=now+1e-9:
            _,_,d=heapq.heappop(arrivals);v.receive('one',d,now)
        if fast:v.finish_tick(now)
        if v.stopped:break
    return dict(fault=v.fault,proofs=v.proofs,elapsed=now,max_age=v.max_proof_age)

def test_80ms_roundtrip_comparison_preserves100ms_and_removes_interval_wait():
    old,new=simulated_link(False),simulated_link(True)
    assert old['fault']=='BROWSER_PROOF_TIMEOUT'
    assert not new['fault'] and new['elapsed']==2. and new['max_age']<=.1

@pytest.mark.parametrize('fast',[False,True])
def test_real_over100ms_network_delay_is_still_rejected(fast):
    r=simulated_link(fast,oneway=.060)
    assert r['fault']=='CHALLENGE_EXPIRED_OR_REPLAYED' and r['proofs']==0

def test_operator_coalesces_idle_intent_but_release_edge_is_immediate():
    from browser_operator import BrowserOperatorNode
    import json
    v=Channel(False);v.open('one',10.);v.tick(10.);v.receive('one',wire(v,10.),10.005)
    n=BrowserOperatorNode.__new__(BrowserOperatorNode);n.channel=v
    n.last_published_generation=-1;n.sequence=0;n.owner='test-only';out=[]
    n.model=NS(held=False,tick=lambda _:None)
    def end(*_):n.model.held=False
    n.model.end_trial=end;n.lease_pub=NS(publish=lambda m:out.append(json.loads(m.data)))
    for t in (10.005,10.010,10.015,10.020):n.clocks=lambda t=t:(t,t+100);n.heartbeat()
    assert len(out)==1 and not out[0]['held']
    n.model.held=True;n.clocks=lambda:(10.021,110.021);n.heartbeat()
    assert len(out)==2 and out[-1]['held'] and out[-1]['issued_steady']==10.
    n.model.held=False;n.clocks=lambda:(10.022,110.022);n.heartbeat()
    assert len(out)==3 and not out[-1]['held'] #1ms edge, no20ms wait.

def test_proof_expiration_revokes_even_inside_publication_rate_window():
    from browser_operator import BrowserOperatorNode
    import json
    v=Channel(False);v.open('one',10.);v.tick(10.);v.receive('one',wire(v,10.),10.005)
    n=BrowserOperatorNode.__new__(BrowserOperatorNode);n.channel=v;n.owner='test';n.sequence=0;n.last_published_generation=-1;out=[]
    n.model=NS(held=True,tick=lambda _:None)
    def end(*_):n.model.held=False
    n.model.end_trial=end;n.lease_pub=NS(publish=lambda m:out.append(json.loads(m.data)))
    n.clocks=lambda:(10.099,110.099);n.heartbeat();assert out[-1]['held']
    n.clocks=lambda:(10.101,110.101);n.heartbeat()
    assert not n.model.held and len(out)==2 and not out[-1]['held']
