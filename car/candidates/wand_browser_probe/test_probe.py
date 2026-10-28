"""The observer never resets or supplies authority to the strict channel."""
import ast
import math
from pathlib import Path
import pytest
from probe import Probe

def opened():
    p=Probe();assert p.open('one',10.);p.tick(10.);return p
def message(p,issued=10.,seq=1,**kw):
    d=dict(type='pulse',nonce=p.challenge(issued)['nonce'],sequence=seq,
        focused=True,visible=True,space=False,ui_gap_ms=20.,events=[]);d.update(kw);return d
def test_first_strict_failure_is_latched_forever_but_observation_continues():
    p=opened();p.receive('one',message(p),10.02);old=p.strict.proof
    p.tick(10.07);p.tick(10.102);assert p.strict.fault=='BROWSER_PROOF_TIMEOUT'
    p.receive('one',message(p,10.11,2),10.30)
    p.receive('one',message(p,10.31,3),10.32)
    s=p.summary(10.33)
    assert not p.closed and not s['strict_passed'] and s['post_failure_samples']==2
    assert p.strict.proof==old and p.strict.proofs==1 and p.strict.evidence(10.32) is None
    assert not s['authority_ever_acquired'] and s['timings']['roundtrip_s']['max']>.18
    assert s['first_failure']['reason']=='BROWSER_PROOF_TIMEOUT'

def test_client_pause_is_measured_not_disguised_by_success_maxima():
    p=opened();p.receive('one',message(p,ui_gap_ms=265.),10.02)
    assert p.strict.fault=='CLIENT_LOOP_GAP' and not p.closed
    assert p.summary(10.03)['max_client_gap_s']==.265

def test_diagnostic_echo_never_creates_or_renews_any_strict_proof():
    p=opened();d=message(p);p.receive('one',dict(type='diagnostic_echo',nonce=d['nonce']),10.25)
    assert p.strict.proof is None and p.strict.started is None and p.strict.proofs==0
    assert p.first is None and p.summary(10.26)['timings']['echo_roundtrip_s']['max']==.25
    p.receive('one',d,10.26);assert p.strict.fault=='CHALLENGE_EXPIRED_OR_REPLAYED'

@pytest.mark.parametrize('field',['focused','visible'])
def test_no_authority_when_focus_or_visibility_invalid(field):
    p=opened();p.receive('one',message(p,**{field:False}),10.02)
    assert p.strict.fault=='WINDOW_NOT_FOREGROUND' and not p.closed
    p.receive('one',message(p,10.03,2),10.04)
    assert p.strict.fault=='WINDOW_NOT_FOREGROUND' and p.strict.evidence(10.04) is None

def test_transport_stages_are_recorded_on_same_host_monotonic_clock():
    p=opened();p.receive('one',message(p),10.04,node_received=10.025,reader_received=10.03,node_gap=.012)
    s=p.samples[-1]
    assert s['issue_to_node_receive_s']==pytest.approx(.025)
    assert s['node_to_reader_s']==pytest.approx(.005)
    assert s['reader_to_main_s']==pytest.approx(.01) and not s['authority']

def test_stale_and_repeated_nonce_cannot_control_or_continue_invalid_stream():
    p=opened();d=message(p);p.receive('one',d,10.02);p.receive('one',dict(d,sequence=2),10.03)
    assert p.closed and 'NONCE' in p.end_reason

@pytest.mark.parametrize('bad',[{'kind':'start'},{'kind':'setpoint','x':.1}, {'kind':'estop'}])
def test_observer_has_no_control_actions(bad):
    p=opened();p.receive('one',message(p,events=[bad]),10.02)
    assert p.closed and 'READONLY_ACTION' in p.end_reason

@pytest.mark.parametrize('bad',[float('nan'),-1.,float('inf'),'20'])
def test_telemetry_is_finite_and_bounded(bad):
    p=opened();p.receive('one',message(p,ui_gap_ms=bad),10.02);assert p.closed

def test_second_owner_and_reconnect_cannot_rearm():
    p=opened();assert not p.open('two',10.01)
    p.receive('two',{},10.02);assert not p.closed
    p.disconnect('one',10.03);assert p.closed and not p.open('two',10.04)

def test_original_validator_will_still_fail265ms_server_gap():
    p=opened();p.receive('one',message(p),10.02);p.tick(10.02);p.tick(10.285)
    assert p.strict.fault=='SERVER_CONTROL_LOOP_GAP'
    p.receive('one',message(p,10.29,2),10.30)
    assert not p.closed and not p.strict.evidence(10.30)

def test_observer_sources_do_not_import_ros_or_actuator_modules():
    allowed={'collections','math','secrets','channel','argparse','json','os','pathlib','queue','select','threading','time','probe'}
    for name in ('probe.py','worker.py'):
        tree=ast.parse((Path(__file__).parent/name).read_text())
        imports=[]
        for n in ast.walk(tree):
            if isinstance(n,ast.Import):imports.extend(a.name.split('.')[0] for a in n.names)
            elif isinstance(n,ast.ImportFrom):imports.append(n.module.split('.')[0])
        assert set(imports)<=allowed
