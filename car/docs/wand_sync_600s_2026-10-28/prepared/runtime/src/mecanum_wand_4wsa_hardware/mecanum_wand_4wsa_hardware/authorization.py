"""New-image evidence only. Historical 4WS8 evidence grants ZERO authority."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import time
from .stages import get_stage

KIND='4WSA_V6_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def verify_flash_record(record_path,build_manifest):
    if not record_path:raise ValueError('4WSA_FULL_BACKUP_AND_READBACK_REQUIRED')
    m=json.loads(Path(build_manifest).read_text());r=json.loads(Path(record_path).read_text());f=m['firmware']
    if (m.get('offline_passed') is not True or f['identity']!='4WSA' or f['authority_version']!=6
            or f['diagnostic_payload_bytes']!=204 or sha(f['path'])!=f['sha256']
            or r.get('identity')!='4WSA' or r.get('image_sha256')!=f['sha256']
            or r.get('flashed') is not True or r.get('independent_readback_verified') is not True):
        raise ValueError('New 4WSA image/readback identity required; old image cannot qualify')
    backup=Path(r['pre_flash_full_backup_path']);read=Path(r['full_readback_path']);image=Path(f['path'])
    if (backup.stat().st_size!=1048576 or sha(backup)!=r['pre_flash_full_backup_sha256']
            or read.stat().st_size!=1048576 or sha(read)!=r['full_readback_sha256']
            or read.read_bytes()[:image.stat().st_size]!=image.read_bytes()):
        raise ValueError('Complete backup/readback hashes or image bytes mismatch')
    return r

def verify_trial_review(review,stage,purpose,firmware_sha):
    limits=get_stage(stage)
    if (not isinstance(review,dict) or review.get('kind')!=KIND or review.get('identity')!='4WSA'
            or review.get('image_sha256')!=firmware_sha or review.get('authorized_stage')!=stage
            or review.get('purpose')!=purpose or review.get('human_authorized') is not True):
        raise ValueError('New image and stage-specific human trial review required')
    if stage:
        p=Path(review['previous_stage_evidence']);e=json.loads(p.read_text())
        if (sha(p)!=review.get('previous_stage_evidence_sha256') or e.get('identity')!='4WSA'
                or e.get('image_sha256')!=firmware_sha or e.get('stage')!=stage-1
                or e.get('purpose')!=purpose or e.get('physical_trial') is not True
                or e.get('action_and_stop_passed') is not True or e.get('reviewed_by_human') is not True):
            raise ValueError('Cannot skip physical commissioning tiers or reuse old-image results')
    if purpose=='sign':
        # First ground admission still needs measured stopping/electrical
        # evidence for THIS tier; the offline plant cannot manufacture it.
        d=review.get('measured_stop_envelope_m')
        if (type(d) not in (int,float) or not math.isfinite(d) or not 0<=d<=.50
                or review.get('stop_envelope_includes_600ms_fallback_and_footprint') is not True
                or review.get('measured_stop_test_speed_mps',0)<limits.speed
                or review.get('measured_stop_test_yaw_rate_radps',0)<limits.yaw_rate
                or review.get('electrical_thermal_trial_reviewed') is not True):
            raise ValueError('Current-tier measured translation/rotation stop and electrical review required')
    return True

def verify_runtime_profile(profile,purpose,now=None):
    if purpose not in ('sign','suspended'):raise ValueError('Fixed trial purpose required')
    from mecanum_wand_4wsa_control.tracking_config import TrackingSafetyConfig as RotationSafetyConfig
    c=RotationSafetyConfig(**profile['safety']);s=get_stage(profile['stage']);hw=profile['hardware']
    endpoint=ipaddress.IPv4Address(hw.get('esp_ip',''))
    interface=hw.get('network_interface')
    if (not endpoint.is_private or endpoint.is_loopback or endpoint.is_unspecified
            or not isinstance(interface,str) or not interface or len(interface)>15
            or not all(v.isalnum() or v in '_-.' for v in interface)):
        raise ValueError('Explicit private endpoint and reviewed network interface required')
    if (c.stage!=profile['stage'] or profile.get('mapping_scale')!=1.
            or profile.get('operator_input_mode')!='SPACE_TOGGLE_600S_V1'
            or profile.get('arena_revision')!='tracking_only_600s_4wsa_v6'
            or hw.get('required_motion_tag')!='4WSA' or hw.get('output_authority_version')!=6
            or hw.get('diagnostic_payload_bytes')!=204 or hw.get('wire_timeout_ms')!=600
            or hw.get('pwm_cap_permille')!=s.duty or hw.get('motor_target_rpm_cap')!=s.target_rpm):
        raise ValueError('Exact 4WSA tier/identity/safety geometry required')
    a=profile.get('staged_runtime_approval',{});now=time.time() if now is None else now
    if (a.get('purpose')!=purpose or type(a.get('issued_utc_s')) not in (int,float)
            or not 0<=now-a['issued_utc_s']<=600 or a.get('clear_and_power_cut_ready_now') is not True
            or (purpose=='sign' and a.get('single_click_mode_acknowledged_now') is not True)
            or (purpose=='suspended' and a.get('four_wheels_secured_clear_now') is not True)):
        raise ValueError('Fresh on-site approval required; offline profile is NOT approval')
    record=verify_flash_record(a.get('flash_record'),a.get('build_manifest'))
    verify_trial_review(profile.get('trial_review'),c.stage,purpose,record['image_sha256'])
    return True
