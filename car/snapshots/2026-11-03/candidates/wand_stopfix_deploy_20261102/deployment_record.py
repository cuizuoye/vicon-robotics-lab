"""File-only STOPFIX receipt and frozen-host checks; never hardware I/O."""
import hashlib
import json
from pathlib import Path

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HOST=ROOT/'docs/wand_stopfix_deploy_2026-11-02/prepared_v1/host'
RECORD=ROOT/'docs/wand_stopfix_2026-11-02/DEPLOYMENT_2026-11-02.json'
RECORD_SHA='4efad26c629ee51d214d2dca865253b6a129acf301e948b18c4fc1893b367e97'
IMAGE_SHA='c02b27867b1c92f3284e00c340775b7a5afe89985393e647cf7c0e34b90f81c0'
OLD_IMAGE_SHA='b471a2a3fe5b24b7b23844a0452a5923bd3b9fc14a78df2a5ffaa9707a7e5769'
BACKUP=ROOT/'stm32/mecanum_f407/backups/pre_stopfix_20261102_u5YBen'
EQ=ROOT/'docs/wand_stopfix_2026-11-02/prepared_v1/MOTION_EQUIVALENCE.json'
EQ_SHA='1f6ded2b8baceb875bb50ace40112bb6976de6351c8fdfb8a04c90241ee66fd9'
MIGRATION=dict(revision='STOPFIX_V1',image_sha256=IMAGE_SHA,record=str(RECORD),record_sha256=RECORD_SHA,
    old_observation_image_sha256=OLD_IMAGE_SHA,normal_motion_equivalence=str(EQ),equivalence_sha256=EQ_SHA,
    inherited_observation_only=True,new_revision_physical_qualified=False)

def require(value,reason):
    if not value:raise ValueError('STOPFIX: '+reason)

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def audit_record(r):
    require(r.get('schema')=='stopfix-deployment-v1' and r.get('identity')=='4WSC'
        and r.get('firmware_revision')=='STOPFIX_V1' and r.get('state')=='FLASH_VERIFIED_CORE_HALTED_MOTION_NOT_AUTHORIZED'
        and r.get('protocol_authority_version')==8 and r.get('diagnostic_payload_bytes')==208
        and r.get('image_sha256')==IMAGE_SHA and r.get('previous_image_sha256')==OLD_IMAGE_SHA,'exact STOPFIX revision required')
    for key in ('flashed','backup_verified','two_full_backups_identical','independent_readback_verified',
                'image_prefix_matches_byte_for_byte','option_bytes_unchanged','final_core_halted'):
        require(r.get(key) is True,'missing deployment evidence '+key)
    require(r.get('chip_uid_words')==['00000000','00000000','00000000'] and r.get('stlink_sn')=='000000000000000000000000','wrong physical target')
    paths=dict(backup_pass1_path='flash_before_pass1_1MiB.bin',backup_pass2_path='flash_before_pass2_1MiB.bin',
        full_readback_path='flash_after_stopfix_1MiB.bin',archived_programmed_image='stopfix_v1_image.bin')
    for key,name in paths.items():require(r.get(key)==str(BACKUP/name),'wrong deployment artifact '+key)
    before=(BACKUP/paths['backup_pass1_path']).read_bytes();second=(BACKUP/paths['backup_pass2_path']).read_bytes()
    after=(BACKUP/paths['full_readback_path']).read_bytes();image=(BACKUP/paths['archived_programmed_image']).read_bytes()
    require(len(before)==len(second)==len(after)==0x100000 and before==second and
        hashlib.sha256(before).hexdigest()==r['backup_sha256'],'fresh full double-backup mismatch')
    require(len(image)==53332 and hashlib.sha256(image).hexdigest()==IMAGE_SHA and
        hashlib.sha256(after).hexdigest()==r['full_readback_sha256'] and after[:len(image)]==image and
        after[len(image):65536]==b'\xff'*(65536-len(image)) and after[65536:]==before[65536:],'full readback/padding/tail mismatch')
    options=[(BACKUP/name).read_bytes() for name in ('option_bytes_before_pass1.bin','option_bytes_before_pass2.bin','option_bytes_after.bin')]
    require(len(options[0])==16 and options[0]==options[1]==options[2] and
        hashlib.sha256(options[0]).hexdigest()==r['option_bytes_sha256'],'option-byte mismatch')
    logs=r.get('log_sha256',{})
    require(set(logs)=={'backup_pass1.log','backup_pass2.log','target_identity_before.log','program_stopfix.log','readback_full.log','final_core_halted.log'},'complete deployment logs required')
    for name,value in logs.items():require(sha(BACKUP/name)==value,'deployment log changed '+name)
    return r

def verify_flash_record(path,build_manifest=None):
    require(path and Path(path).resolve()==RECORD,'old or alternate deployment receipt rejected')
    require(sha(RECORD)==RECORD_SHA,'deployment receipt changed')
    return audit_record(json.loads(RECORD.read_text()))

def verify_migration(profile):
    require(profile.get('firmware_deployment')==MIGRATION,'new revision must acknowledge inherited observation only')
    require(sha(EQ)==EQ_SHA,'normal-motion equivalence changed')
    r=json.loads(EQ.read_text())
    require(r.get('passed') is True and r.get('compared_control_ticks')==15000 and r.get('excluded_fields')==[]
        and r.get('unchanged_stages')==list(range(10)) and r.get('physical_motion') is False,'equivalence is offline, not physical qualification')

def verify_host_snapshot(root=HOST):
    require(Path(root).resolve()==HOST,'exact deployed host snapshot required')
    m=json.loads((HOST/'HOST_MANIFEST.json').read_text())
    require(m.get('host_offline_passed') is True and m.get('hardware_launch_enabled') is True
        and m.get('firmware_revision')=='STOPFIX_V1' and m.get('deployment_record_sha256')==RECORD_SHA,'qualified deployed host required')
    for group in ('protected','source_hashes','artifact_hashes','evidence'):
        require(isinstance(m.get(group),dict) and bool(m[group]),'missing host pins '+group)
        for path,digest in m[group].items():require(sha(path)==digest,'host/source/evidence changed '+path)
    verify_flash_record(RECORD)
    return m
