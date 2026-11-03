"""File-only admission. No historical image, SIM result or draft grants motion."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import time
from .stages import get_stage

KIND = '4WSB_V7_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION'


def sha(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def verify_flash_record(record_path, build_manifest):
    if not record_path or not build_manifest:
        raise ValueError('4WSB_FRESH_FULL_BACKUP_AND_READBACK_REQUIRED')
    m = json.loads(Path(build_manifest).read_text())
    r = json.loads(Path(record_path).read_text())
    f = m['firmware']
    if (m.get('host_offline_passed') is not True or m.get('identity') != '4WSB'
            or f['authority_version'] != 7 or f['diagnostic_payload_bytes'] != 208
            or sha(f['path']) != f['sha256'] or r.get('identity') != '4WSB'
            or r.get('image_sha256') != f['sha256'] or r.get('flashed') is not True
            or r.get('independent_readback_verified') is not True):
        raise ValueError('4WSB exact image, passed host and readback required')
    before = Path(r['pre_flash_full_backup_path']).read_bytes()
    after = Path(r['full_readback_path']).read_bytes()
    image = Path(f['path']).read_bytes()
    if (len(before) != 1048576 or len(after) != 1048576
            or hashlib.sha256(before).hexdigest() != r['pre_flash_full_backup_sha256']
            or hashlib.sha256(after).hexdigest() != r['full_readback_sha256']
            or after[:len(image)] != image or after[len(image):65536] != b'\xff'*(65536-len(image))
            or after[65536:] != before[65536:]
            or r.get('two_independent_backups_equal') is not True
            or r.get('option_bytes_unchanged') is not True
            or r.get('core_halted_after_programming') is not True):
        raise ValueError('full backup/readback, tail, erased padding or option-byte proof failed')
    if not isinstance(r.get('target_uid'), str) or not r['target_uid']:
        raise ValueError('fresh identified target UID required')
    return r


def verify_trial_review(review, stage, purpose, firmware_sha):
    s = get_stage(stage)
    if (not isinstance(review, dict) or review.get('kind') != KIND
            or review.get('identity') != '4WSB' or review.get('image_sha256') != firmware_sha
            or type(review.get('authorized_stage')) is not int or review['authorized_stage'] != stage
            or review.get('purpose') != purpose or review.get('human_authorized') is not True):
        raise ValueError('current-image, explicit-stage human review required')
    if stage:
        p = Path(review['previous_stage_evidence']); e = json.loads(p.read_text())
        if (sha(p) != review.get('previous_stage_evidence_sha256') or e.get('identity') != '4WSB'
                or e.get('image_sha256') != firmware_sha or e.get('stage') != stage-1
                or e.get('purpose') != purpose or e.get('physical_trial') is not True
                or e.get('action_and_stop_passed') is not True or e.get('reviewed_by_human') is not True):
            raise ValueError('cannot skip actual new-image commissioning stages')
    if purpose == 'sign':
        d = review.get('measured_stop_envelope_m')
        if (type(d) not in (int,float) or not math.isfinite(d) or not 0 <= d <= .50
                or review.get('stop_envelope_includes_600ms_fallback_and_footprint') is not True
                or review.get('measured_stop_test_speed_mps',0) < s.speed
                or review.get('measured_stop_test_yaw_rate_radps',0) < s.yaw_rate
                or review.get('electrical_thermal_trial_reviewed') is not True):
            raise ValueError('current-tier physical braking/electrical/thermal review required')
    return True


def verify_runtime_profile(profile, purpose, now=None):
    if purpose not in ('sign','suspended'): raise ValueError('fixed purpose required')
    from mecanum_wand_4wsb_control.tracking_config import TrackingSafetyConfig
    stage = profile['stage']; s = get_stage(stage)
    c = TrackingSafetyConfig(**profile['safety']); hw = profile['hardware']
    ip = ipaddress.IPv4Address(hw['esp_ip'])
    iface = hw.get('network_interface')
    if (not ip.is_private or ip.is_loopback or ip.is_unspecified
            or not isinstance(iface,str) or not 0 < len(iface) <= 15
            or not all(v.isalnum() or v in '_-.' for v in iface)):
        raise ValueError('explicit reviewed network endpoint required')
    if (c.stage != stage or profile.get('mapping_scale') != 1.
            or profile.get('operator_input_mode') != 'SPACE_TOGGLE_600S_V1'
            or profile.get('arena_revision') != 'tracking_only_600s_4wsb_v7'
            or hw.get('required_motion_tag') != '4WSB' or hw.get('output_authority_version') != 7
            or hw.get('diagnostic_payload_bytes') != 208 or hw.get('wire_timeout_ms') != 600
            or hw.get('pwm_cap_permille') != s.duty or hw.get('motor_target_rpm_cap') != s.target_rpm
            or hw.get('wheel_slew_guard_cps_s') != s.wheel_slew_guard):
        raise ValueError('exact 4WSB stage/identity/safety contract required')
    a = profile.get('staged_runtime_approval',{})
    now = time.time() if now is None else now
    issued = a.get('issued_utc_s')
    if (a.get('purpose') != purpose or type(issued) not in (int,float)
            or not math.isfinite(issued) or not 0 <= now-issued <= 600
            or a.get('clear_and_power_cut_ready_now') is not True
            or (purpose == 'sign' and a.get('single_click_mode_acknowledged_now') is not True)
            or (purpose == 'suspended' and a.get('four_wheels_secured_clear_now') is not True)):
        raise ValueError('fresh site approval required; draft is NOT approval')
    r = verify_flash_record(a.get('flash_record'),a.get('build_manifest'))
    verify_trial_review(profile.get('trial_review'),stage,purpose,r['image_sha256'])
    return True
