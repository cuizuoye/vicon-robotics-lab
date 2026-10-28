"""One explicit unloaded 0->2 transition; no ground/other-stage exception."""
import hashlib
import json
from pathlib import Path

TRANSITION='USER_REQUESTED_SUSPENDED_0_TO_2_SINGLE_5S'
SCOPE='4WS8_SUSPENDED_STAGES_2_3_4_SINGLE_5S'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def predecessor(review,stage,purpose,image_sha):
    transition=review.get('authorized_transition')
    if transition is None:return stage-1
    if transition!=TRANSITION or stage!=2 or purpose!='suspended':
        raise ValueError('transition exception is suspended stage2 only')
    path=Path(review['user_request_evidence']);a=json.loads(path.read_text())
    if (sha(path)!=review.get('user_request_evidence_sha256') or
        a.get('scope')!=SCOPE or a.get('identity')!='4WS8' or a.get('image_sha256')!=image_sha or
        a.get('human_authorized') is not True or a.get('entry')!='suspended' or
        a.get('requested_stages')!=[2,3,4] or a.get('maximum_action_s')!=5 or
        a.get('manual_hold_required') is not True or a.get('ground_motion_authorized') is not False or
        a.get('automatic_retry_authorized') is not False or
        not isinstance(a.get('direct_user_request'),str) or not a['direct_user_request'].strip()):
        raise ValueError('explicit requested suspended sequence evidence mismatch')
    previous_path=Path(review['previous_stage_evidence']);p=json.loads(previous_path.read_text())
    if (p.get('stage')!=0 or p.get('purpose')!='suspended' or p.get('identity')!='4WS8' or
        p.get('image_sha256')!=image_sha or p.get('physical_trial') is not True or
        p.get('action_and_stop_passed') is not True or p.get('reviewed_by_human') is not True or
        sha(previous_path.parent/'raw_link.jsonl')!=p.get('raw_log_sha256') or
        sha(previous_path.parent/'events.json')!=p.get('events_sha256')):
        raise ValueError('stage0 completed physical trial with preserved raw evidence required')
    return 0
