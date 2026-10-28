"""Post-flash FILE evidence and fresh site approval, before ROS/socket creation.

No device is opened here. A build artifact is never treated as a board readback.
Preparation creates NO flash record and NO site-confirmed runtime profile.
"""
import hashlib
import json
import time
from pathlib import Path

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def verify_flash_record(record_path,build_manifest):
    if not record_path:raise ValueError('POST_FLASH_RECORD_REQUIRED; no hardware opened')
    manifest=json.loads(Path(build_manifest).read_text())
    record=json.loads(Path(record_path).read_text())
    image=Path(manifest['firmware']['path']);readback=Path(record['full_readback_path'])
    if (manifest['firmware']['identity']!='4WS7' or manifest['firmware']['authority_version']!=3 or
        manifest['firmware']['stage']!=0 or manifest['firmware']['pwm_limit']!=350 or
        record.get('flashed') is not True or record.get('independent_readback_verified') is not True or
        sha(image)!=manifest['firmware']['sha256'] or readback.stat().st_size!=1048576 or
        sha(readback)!=record.get('full_readback_sha256') or
        readback.read_bytes()[:image.stat().st_size]!=image.read_bytes()):
        raise ValueError('post-flash evidence does not match bounded stage0 image/full readback')
    return record

def verify_runtime_profile(profile,purpose,now=None):
    if purpose not in ('sign','suspended'):raise ValueError('fixed commissioning purpose required')
    approval=profile.get('stage0_runtime_approval',{})
    now=time.time() if now is None else now
    if (approval.get('purpose')!=purpose or type(approval.get('issued_utc_s')) not in (int,float) or
        not 0<=now-approval['issued_utc_s']<=600 or
        approval.get('clear_and_power_cut_ready_now') is not True or
        (purpose=='suspended' and approval.get('four_wheels_secured_clear_now') is not True)):
        raise ValueError('fresh purpose-specific site approval required')
    verify_flash_record(approval.get('flash_record',''),approval.get('build_manifest',''))
    if (profile.get('mapping_scale')!=1 or profile.get('hardware',{}).get('required_motion_tag')!='4WS7' or
        profile['hardware'].get('output_authority_version')!=3 or
        profile['hardware'].get('diagnostic_payload_bytes')!=184 or
        profile['hardware'].get('pwm_cap_permille')!=350 or
        profile.get('safety',{}).get('max_speed_mps')!=.20):
        raise ValueError('runtime differs from fixed stage0 contract')
    return True
