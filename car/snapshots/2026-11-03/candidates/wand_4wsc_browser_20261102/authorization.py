"""File-only 4WSC image/backup/trial/site admission, before any hardware IO."""
import hashlib
import ipaddress
import json
import math
from pathlib import Path
import time
from .stages import get_stage

KIND='4WSC_V8_600S_CONTROLLED_TRIAL_NOT_QUALIFICATION'
IMAGE_SHA='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def require(value,reason):
    if not value:raise ValueError(reason)

def verify_flash_record(record_path,build_manifest=None):
    require(record_path,'4WSC_FRESH_FLASH_READBACK_RECORD_REQUIRED')
    r=json.loads(Path(record_path).read_text())
    require(r.get('identity')=='4WSC' and r.get('schema')=='4wsc-postflash-v1' and r.get('flashed') is True
        and r.get('independent_readback_verified') is True and r.get('image_sha256')==IMAGE_SHA,'exact 4WSC v8 deployment required')
    first=Path(r['backup_pass1_path']).read_bytes();second=Path(r['backup_pass2_path']).read_bytes()
    after=Path(r['full_readback_path']).read_bytes();image=Path(r['archived_programmed_image']).read_bytes()
    require(len(first)==len(second)==len(after)==0x100000 and first==second
        and hashlib.sha256(first).hexdigest()==r['backup_sha256'],'two full fresh backups must agree')
    require(len(image)==53092 and hashlib.sha256(image).hexdigest()==IMAGE_SHA
        and hashlib.sha256(after).hexdigest()==r['full_readback_sha256']
        and after[:len(image)]==image and after[len(image):65536]==b'\xff'*(65536-len(image))
        and after[65536:]==first[65536:],'full deployed image/tail/padding proof failed')
    folder=Path(r['full_readback_path']).parent
    options=[(folder/n).read_bytes() for n in ('option_bytes_before_pass1.bin','option_bytes_before_pass2.bin','option_bytes_after.bin')]
    require(len(options[0])==16 and options[0]==options[1]==options[2]
        and r.get('option_bytes_unchanged') is True and r.get('final_core_halted') is True
        and r.get('two_full_backups_identical') is True,'option-byte/core/double-backup proof failed')
    require(r.get('chip_uid_words')==['00000000','00000000','00000000']
        and r.get('stlink_sn')=='000000000000000000000000','identified physical target required')
    for name,digest in r['log_sha256'].items():
        require(Path(name).name==name and sha(folder/name)==digest,'deployment log changed')
    return r

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
