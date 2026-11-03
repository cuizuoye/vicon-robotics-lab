"""Independent 4WSB entry. Default is FILE CHECK, never connect or move."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from prepare import ROOT, REPORT, CONTROL, HARDWARE, sha

DEFAULT = REPORT/'prepared_v9'


def check(folder):
    m = json.loads((folder/'HOST_MANIFEST.json').read_text())
    if (m.get('host_offline_passed') is not True or m['identity'] != '4WSB'
            or m['authority_version'] != 7 or m['diagnostic_payload_bytes'] != 208):
        raise ValueError('4WSB host offline validation incomplete')
    for group in ('runtime_hashes','source_hashes','protected','evidence'):
        for p, digest in m[group].items():
            if sha(p) != digest: raise ValueError('source/evidence drift: '+p)
    if sha(m['firmware']['path']) != m['firmware']['sha256']:
        raise ValueError('firmware image changed')
    if sha(ROOT/'tools/start_car_sync_4wsb.sh') != m['file_checker_sha256']:
        raise ValueError('4WSB entry script changed')
    return m


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared', type=Path, default=DEFAULT)
    p.add_argument('--check', action='store_true')
    p.add_argument('--launch', action='store_true')
    p.add_argument('--stage', type=int, choices=range(8), default=0)
    p.add_argument('--purpose', choices=('sign','suspended'), default='sign')
    p.add_argument('--evidence-dir', type=Path)
    p.add_argument('--profile', type=Path)
    p.add_argument('--profile-sha', default='')
    p.add_argument('--consent', default='')
    a = p.parse_args()
    m = check(a.prepared)
    if not a.launch:
        print(json.dumps(dict(passed=True,identity='4WSB',stage=a.stage,
            hardware_admitted=False,firmware_flashed=False,hardware_opened=False,
            launch_requires='fresh flash/readback, stage-specific review and approved site profile')))
        return
    if a.profile is None:
        raise ValueError('NO APPROVED PROFILE: no ROS initialization or hardware socket opened')
    for package in (CONTROL,HARDWARE): sys.path.insert(0,str(Path(m['runtime_src'])/package))
    from mecanum_wand_4wsb_hardware.commissioning import Runtime
    raw = json.loads(a.profile.read_text())
    if type(raw.get('stage')) is not int or raw['stage'] != a.stage:
        raise ValueError('explicit CLI stage must match the reviewed profile')
    if a.purpose == 'suspended':
        from mecanum_wand_4wsb_hardware.authorization import verify_runtime_profile
        if a.profile_sha != sha(a.profile) or a.consent != '4WSB_SUSPENDED_SINGLE5S_CLEAR_POWER_CUT_READY':
            raise ValueError('exact suspended profile approval and explicit consent required')
        verify_runtime_profile(raw,'suspended')
        if a.evidence_dir is None or a.evidence_dir.exists():
            raise ValueError('new evidence directory required before suspended socket construction')
        from mecanum_wand_4wsb_hardware.suspended_console import run
        run(str(a.profile),str(a.evidence_dir))
        return
    # Validation precedes ros2 launch and any Node/transport construction.
    Runtime('hardware',str(a.profile),a.profile_sha,a.consent)
    if a.evidence_dir is not None:
        folder=a.evidence_dir.resolve();folder.mkdir(parents=True,exist_ok=False)
    else:
        base=REPORT/'hardware_sessions';base.mkdir(parents=True,exist_ok=True)
        folder=Path(tempfile.mkdtemp(prefix='stage%d_'%a.stage,dir=base))
    (folder/'RUN_CONTRACT.json').write_text(json.dumps(dict(identity='4WSB',stage=a.stage,
        purpose='sign',profile_sha256=a.profile_sha,firmware_sha256=m['firmware']['sha256'],
        automatic_enable=False,physical_qualified=False),indent=2)+'\n')
    env = dict(os.environ, ROS_DOMAIN_ID='124', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               WAND_AUTOPREP_FLOW='AUTO_PREPARE_SINGLE_SPACE_V1',
               WAND_PILOT_TRACE=str(folder/'observation.jsonl'),ROS_LOG_DIR=str(folder/'ros_logs'))
    subprocess.run(['ros2','launch',HARDWARE,'wand_4wsb_staged.launch.py','mode:=hardware',
        'profile:='+str(a.profile),'approved_sha256:='+a.profile_sha,'hardware_consent:='+a.consent],
        check=True,env=env)


if __name__ == '__main__':
    try: main()
    except (ValueError,OSError,KeyError) as error:
        print('BLOCKED: '+str(error),file=sys.stderr)
        raise SystemExit(2)
