"""File-only 4WSC image/backup/trial/site admission, before any hardware IO."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import time
from .stages import get_stage

KIND='4WSC_V8_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION'
IMAGE_SHA='c02b27867b1c92f3284e00c340775b7a5afe89985393e647cf7c0e34b90f81c0'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def require(value,reason):
    if not value:raise ValueError(reason)

def verify_flash_record(record_path,build_manifest=None):
    from .deployment_record import verify_flash_record as verify
    return verify(record_path,build_manifest)


def verify_trial_review(review,stage,purpose,firmware_sha=IMAGE_SHA):
    s=get_stage(stage)
    require(isinstance(review,dict) and review.get('kind')==KIND and review.get('identity')=='4WSC'
        and review.get('image_sha256')==firmware_sha and type(review.get('authorized_stage')) is int
        and review['authorized_stage']==stage and review.get('purpose')==purpose
        and review.get('human_authorized') is True,'explicit current-image/stage/purpose human review required')
    if stage:
        p=Path(review['previous_stage_evidence']);e=json.loads(p.read_text())
        require(sha(p)==review.get('previous_stage_evidence_sha256') and e.get('identity')=='4WSC'
            and e.get('image_sha256')==firmware_sha and e.get('stage')==stage-1
            and e.get('purpose')==purpose and e.get('physical_trial') is True
            and e.get('action_and_stop_passed') is True and e.get('reviewed_by_human') is True,
            'cannot promote a new-image stage using simulation or old firmware trials')
    if purpose=='sign':
        d=review.get('measured_stop_envelope_m')
        require(type(d) in (int,float) and math.isfinite(d) and 0<=d<=.50
            and review.get('stop_envelope_includes_600ms_fallback_and_footprint') is True
            and review.get('measured_stop_test_speed_mps',0)>=s.speed
            and review.get('measured_stop_test_yaw_rate_radps',0)>=s.yaw_rate
            and review.get('electrical_thermal_trial_reviewed') is True,
            'current-tier measured braking/electrical/thermal review required before ground following')
    return True

def endpoint(profile, *, rehearsal=False):
    h=profile['hardware'];ip=ipaddress.IPv4Address(h['esp_ip']);iface=h['network_interface']
    require(isinstance(iface,str) and 0<len(iface)<=15 and all(v.isalnum() or v in '_-.' for v in iface),'explicit network interface required')
    ports=tuple(h.get(k,d) for k,d in (('command_port',46000),('telemetry_port',46001)))
    require(all(type(p) is int and 1024<=p<=65535 for p in ports) and ports[0]!=ports[1],'distinct explicit UDP ports required')
    if rehearsal:
        require(str(ip)=='127.0.0.1' and iface=='lo' and not any(p in (46000,46001) for p in ports), 'rehearsal must stay on dedicated loopback ports')
    else:
        require(ip.is_private and not ip.is_loopback and not ip.is_unspecified and ports==(46000,46001),'reviewed real-car private endpoint required')
    return str(ip),iface,*ports

def verify_runtime_profile(profile,purpose,now=None):
    from mecanum_wand_4wsc_control.tracking_config import TrackingSafetyConfig
    require(purpose in ('sign','suspended') and profile.get('purpose')==purpose,'fixed reviewed purpose required')
    stage=profile['stage'];s=get_stage(stage);c=TrackingSafetyConfig(**profile['safety']);hw=profile['hardware']
    endpoint(profile)
    require(c.stage==stage and profile.get('mapping_scale')==1. and profile.get('identity')=='4WSC'
        and profile.get('arena_revision')=='tracking_only_600s_4wsc_v8'
        and profile.get('operator_input_mode')=='SPACE_TOGGLE_600S_V1'
        and hw.get('required_motion_tag')=='4WSC' and hw.get('output_authority_version')==8
        and hw.get('diagnostic_payload_bytes')==208 and hw.get('wire_timeout_ms')==600
        and hw.get('pwm_cap_permille')==s.duty and hw.get('motor_target_rpm_cap')==s.target_rpm
        and hw.get('wheel_slew_guard_cps_s')==s.wheel_slew_guard,'exact 4WSC stage/safety policy required')
    approval=profile.get('staged_runtime_approval',{});issued=approval.get('issued_utc_s');now=time.time() if now is None else now
    require(type(issued) in (int,float) and math.isfinite(issued) and 0<=now-issued<=600
        and approval.get('purpose')==purpose and approval.get('clear_and_power_cut_ready_now') is True
        and (approval.get('single_click_mode_acknowledged_now') is True if purpose=='sign' else approval.get('four_wheels_secured_clear_now') is True),
        'fresh site approval required; historical facts/draft are not authority')
    r=verify_flash_record(approval.get('flash_record'))
    verify_trial_review(profile.get('trial_review'),stage,purpose,r['image_sha256'])
    require(profile.get('confirmations',{}).get('wiring_secured') is True,'wiring condition must be confirmed')
    return True
