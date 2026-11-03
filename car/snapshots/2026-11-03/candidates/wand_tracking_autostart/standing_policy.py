"""A user-declared default is not a sensor or a fresh human confirmation."""
import hashlib
import json
from pathlib import Path

POLICY='USER_STANDING_SITE_PRECONDITIONS_V1'
REQUEST=Path(__REQUEST_PATH__)
REQUEST_SHA=__REQUEST_SHA__
ASSUMED=['car_placed_stably','intended_paths_and_stopping_clearance_clear',
         'immediate_manual_power_cut_available']


def preconditions_record():
    raw=REQUEST.read_bytes()
    if hashlib.sha256(raw).hexdigest()!=REQUEST_SHA:
        raise ValueError('standing user instruction changed; rebuild/review required')
    r=json.loads(raw)
    expected=dict(policy=POLICY,manual_checkboxes_required=False,
        site_conditions_machine_verified=False,assumed_preconditions=ASSUMED,
        automatic_open_after_machine_preflight=True,automatic_prepare_allowed=True,
        automatic_motion_allowed=False,fresh_space_click_required=True,
        max_active_s=120.,firmware_change=False,motion_and_fault_guards_change=False)
    if any(r.get(k)!=v for k,v in expected.items()):
        raise ValueError('exact user-declared default scope required; no automatic motion')
    return dict(policy=POLICY,basis='EXPLICIT_USER_STANDING_INSTRUCTION',
        user_request=str(REQUEST),user_request_sha256=REQUEST_SHA,
        assumed_preconditions=ASSUMED.copy(),machine_verified=False,
        fresh_human_confirmation_recorded=False)


def verify_preconditions(profile):
    if profile.get('site_preconditions')!=preconditions_record():
        raise ValueError('user-declared defaults required; never invent current site checks')
    if profile.get('tracking_confirmations') or any(k in profile.get('staged_runtime_approval',{})
        for k in ('clear_and_power_cut_ready_now','single_click_mode_acknowledged_now')):
        raise ValueError('do not mislabel defaults as fresh human confirmations')
    return True
