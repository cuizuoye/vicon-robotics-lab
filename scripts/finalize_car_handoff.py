#!/usr/bin/env python3
"""Record export integrity and completed offline results, with no device IO."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--source-car',type=Path,required=True)
    args=ap.parse_args()
    receipt=ROOT/'handoff/CAR_EXPORT_2026-10-28.json'
    value=json.loads(receipt.read_text())
    for row in value['files']:
        original=args.source_car/Path(row['path']).relative_to('car')
        if sha(original)!=row['source_sha256']:
            raise ValueError('Primary source changed since selection: '+str(original))
    value['stage']='Initial mechanical export, before documented public portability/test adaptations'
    value['final_public_hashes']='handoff/CAR_PUBLIC_FILES_2026-10-28.json'
    receipt.write_text(json.dumps(value,indent=2)+'\n')
    ros=json.loads((ROOT/'.local/4wsa-ros-rehearsal.json').read_text())
    if not ros['passed'] or len(ros['cases'])!=6 or any(not c['passed'] or c['unexpected_active_faults'] for c in ros['cases']):
        raise ValueError('Private software ROS cases did not pass')
    (ROOT/'handoff/CAR_EXPORT_ROS_REHEARSAL_2026-10-28.json').write_text(json.dumps(ros,indent=2)+'\n')
    counts={}
    for key,rel in [('car','test-results/car.xml'),('wand_control','test-results/wand-control.xml'),
                    ('drone_unchanged','test-results/drone.xml'),('4wsa_host','4wsa-offline/host.xml')]:
        tree=ET.parse(ROOT/'.local'/rel)
        suites=list(tree.getroot().iter('testsuite'))
        if any(int(s.get('errors','0')) or int(s.get('failures','0')) for s in suites):
            raise ValueError('Offline tests failed: '+key)
        counts[key]=sum(int(s.get('tests','0')) for s in suites)
    binary=ROOT/'.local/4wsa-arm/mecanum_f407_4wsa_staged_NOT_FLASHED.bin'
    validation=dict(date='2026-10-28',commands=[
        'bash scripts/doctor.sh','bash scripts/test_offline.sh','bash scripts/test_car_release.sh',
        'bash scripts/build_car_stm32.sh','python3 scripts/public_scan.py',
        '/usr/bin/python3 -m pytest -q scripts/test_public_scan.py',
        '4WSA ros_rehearsal.py: domain187 LOCALHOST software-only, six cases',
        '4WSA CMake SERVO_ARM=ON build only; no programmer',
        'Vicon capture queue: standalone g++ C++17 test'],
        python_tests=counts,public_scanner_tests=6,
        native_4wsa_asan_ubsan_suites=4,legacy_motion_profiles=12,
        legacy_rectangle_and_wheel_suites_passed=True,private_ros_software_cases=6,
        vicon_capture_queue_native_passed=True,legacy_arm_images_built=3,
        public_4wsa_arm=dict(bytes=binary.stat().st_size,sha256=sha(binary),flashed=False),
        esp_idf_full_build_this_export=False,full_vicon_sdk_build_this_export=False,
        primary_selected_source_hashes_unchanged=len(value['files']),
        document=dict(pages=7,reviewed_all_pages=True,evidence_links=13,
                      math_representation='7 native OMML equations retained',metadata_and_links_sanitized=True),
        validation_environment='Same Linux host, isolated export checkout; not a clean OS or second computer',
        archived_hardware_site_profile_tests='Not treated as current approvals; public onsite confirmations revoked',
        hardware_commands_sent=False,firmware_written=False,network_settings_changed=False,
        drone_source_changed=False,legacy_shared_receiver_changed=False)
    (ROOT/'handoff/CAR_VALIDATION_2026-10-28.json').write_text(json.dumps(validation,indent=2)+'\n')
    names=subprocess.check_output(['git','ls-files','--cached','--others','--exclude-standard','-z'],cwd=ROOT).decode().split('\0')
    files={name:sha(ROOT/name) for name in sorted(set(names)) if name and
           (name.startswith(('car/','scripts/','shared/vicon_receiver_snapshot_20261028/')) or name in ('README.md','AGENTS.md'))
           and (ROOT/name).is_file()}
    (ROOT/'handoff/CAR_PUBLIC_FILES_2026-10-28.json').write_text(json.dumps(dict(
        date='2026-10-28',scope='Final public source hashes, not hardware admission',files=files),indent=2)+'\n')
    print(json.dumps(dict(public_files=len(files),tests=counts,original_source_unchanged=len(value['files']))))


if __name__=='__main__':main()
