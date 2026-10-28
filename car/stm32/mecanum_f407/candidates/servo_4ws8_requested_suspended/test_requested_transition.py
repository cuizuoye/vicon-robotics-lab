"""Explicit human-requested suspended transition; network forbidden."""
import hashlib
import json
import os
from pathlib import Path
import sys
import pytest

RUNTIME=Path(os.environ['STAGED_GENERATED'])
for package in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):sys.path.insert(0,str(RUNTIME/package))
from mecanum_wand_4ws8_hardware.authorization import verify_trial_review
from mecanum_wand_4ws8_hardware.requested_transition import TRANSITION
HERE=Path(__file__).resolve().parent
BASE=Path('/home/PRIMARY_USER/vicon_mecanum_ws/docs/wand_servo_4ws8_2026-10-23/ready/stage0_suspended_yjp1gyc2/bench/SESSION_RESULT.json')
SHA='de87e3fb01df19af5bae75d479cef03376340bd486154ce4e0d75a2c285993ab'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def review():
    return dict(kind='BOUNDED_TRIAL_NOT_QUALIFICATION',identity='4WS8',image_sha256=SHA,authorized_stage=2,
        purpose='suspended',human_authorized=True,authorized_transition=TRANSITION,
        user_request_evidence=str(HERE/'USER_REQUEST.json'),user_request_evidence_sha256=sha(HERE/'USER_REQUEST.json'),
        previous_stage=0,previous_stage_action_and_stop_passed=True,previous_stage_evidence=str(BASE),
        previous_stage_evidence_sha256=sha(BASE))

@pytest.fixture(autouse=True)
def forbid_io(monkeypatch):
    import socket
    def no(*a,**k):raise AssertionError('offline transition test must not open network')
    monkeypatch.setattr(socket,'socket',no)

def test_explicit_suspended_0_to_2_is_real_evidence_not_forged_stage1():
    r=review();assert json.loads(BASE.read_text())['stage']==0
    verify_trial_review(r,2,'suspended',SHA)
    assert r['previous_stage']==0

@pytest.mark.parametrize('stage,purpose',[(2,'sign'),(1,'suspended'),(3,'suspended'),(4,'suspended'),(6,'suspended')])
def test_exception_cannot_authorize_ground_or_other_skips(stage,purpose):
    r=review();r.update(authorized_stage=stage,purpose=purpose)
    with pytest.raises(ValueError):verify_trial_review(r,stage,purpose,SHA)

@pytest.mark.parametrize('field,value',[
    ('authorized_transition',None),('authorized_transition','ALLOW_ALL'),('human_authorized',False),
    ('previous_stage',1),('previous_stage_evidence_sha256','wrong'),('user_request_evidence_sha256','wrong'),
    ('image_sha256','wrong'),('purpose','sign')])
def test_missing_or_mutated_authority_is_rejected(field,value):
    r=review();r[field]=value
    with pytest.raises(ValueError):verify_trial_review(r,2,'suspended',SHA)

@pytest.mark.parametrize('field,value',[
    ('human_authorized',False),('entry','sign'),('maximum_action_s',10),('requested_stages',[2,3,4,6]),
    ('manual_hold_required',False),('ground_motion_authorized',True),('automatic_retry_authorized',True),
    ('identity','4WS7'),('image_sha256','wrong'),('direct_user_request','')])
def test_broadened_request_document_is_rejected(tmp_path,field,value):
    a=json.loads((HERE/'USER_REQUEST.json').read_text());a[field]=value
    p=tmp_path/'synthetic_modified_request.json';p.write_text(json.dumps(a))
    r=review();r.update(user_request_evidence=str(p),user_request_evidence_sha256=sha(p))
    with pytest.raises(ValueError):verify_trial_review(r,2,'suspended',SHA)
