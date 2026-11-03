import hashlib
import json
import os
from pathlib import Path
import sys
import pytest
ROOT=Path(os.environ['STAGE0_GENERATED'])
for name in ('mecanum_wand_4ws7_control','mecanum_wand_4ws7_hardware'):sys.path.insert(0,str(ROOT/name))
from mecanum_wand_4ws7_hardware.authorization import verify_flash_record,verify_runtime_profile

def test_unapproved_bench_constructor_fails_before_socket_or_lock(monkeypatch,tmp_path):
    import mecanum_wand_4ws7_hardware.suspended_console as ui
    def forbidden(*a,**k):raise AssertionError('must validate before socket/lock')
    monkeypatch.setattr(ui.socket,'socket',forbidden);monkeypatch.setattr(ui.fcntl,'flock',forbidden)
    with pytest.raises(ValueError,match='site approval'):ui.UdpIO({},tmp_path/'not_created.jsonl')
    assert not (tmp_path/'not_created.jsonl').exists()

def proof(tmp_path):
    # These are SYNTHETIC FILES testing the validator, NOT a physical readback.
    image=tmp_path/'SYNTHETIC.bin';image.write_bytes(b'not firmware; unit fixture only')
    readback=tmp_path/'SYNTHETIC_full.bin';readback.write_bytes(image.read_bytes()+bytes(1048576-image.stat().st_size))
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    manifest=tmp_path/'build.json';manifest.write_text(json.dumps(dict(firmware=dict(
        path=str(image),sha256=sha(image),identity='4WS7',authority_version=3,stage=0,pwm_limit=350))))
    record=tmp_path/'flash.json';record.write_text(json.dumps(dict(full_readback_path=str(readback),
        full_readback_sha256=sha(readback),flashed=True,independent_readback_verified=True)))
    return record,manifest

@pytest.mark.parametrize('problem',['unflashed','wrong_prefix','short_readback','hash_drift','wrong_stage','old_version'])
def test_build_or_incorrect_file_evidence_never_stands_in_for_flash(tmp_path,problem):
    record,manifest=proof(tmp_path);r=json.loads(record.read_text());m=json.loads(manifest.read_text())
    if problem=='unflashed':r['flashed']=False
    if problem=='wrong_prefix':
        p=Path(r['full_readback_path']);p.write_bytes(b'X'+p.read_bytes()[1:])
        r['full_readback_sha256']=hashlib.sha256(p.read_bytes()).hexdigest()
    if problem=='short_readback':Path(r['full_readback_path']).write_bytes(b'bad')
    if problem=='hash_drift':r['full_readback_sha256']='0'*64
    if problem=='wrong_stage':m['firmware']['stage']=6
    if problem=='old_version':m['firmware']['authority_version']=2
    record.write_text(json.dumps(r));manifest.write_text(json.dumps(m))
    with pytest.raises(ValueError):verify_flash_record(record,manifest)

def test_purpose_and_current_site_expiry_are_independent_of_file_readback(tmp_path):
    record,manifest=proof(tmp_path)
    assert verify_flash_record(record,manifest)
    profile=dict(mapping_scale=1,safety=dict(max_speed_mps=.20),hardware=dict(required_motion_tag='4WS7',
        output_authority_version=3,diagnostic_payload_bytes=184,pwm_cap_permille=350),
        stage0_runtime_approval=dict(purpose='suspended',issued_utc_s=10.,clear_and_power_cut_ready_now=True,
            four_wheels_secured_clear_now=True,flash_record=str(record),build_manifest=str(manifest)))
    assert verify_runtime_profile(profile,'suspended',now=10.)
    with pytest.raises(ValueError):verify_runtime_profile(profile,'sign',now=10.)
    with pytest.raises(ValueError):verify_runtime_profile(profile,'suspended',now=611.)
