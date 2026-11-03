"""Seal only this browser/software release after protocol and actual UI checks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
DOCS=ROOT/'docs/wand_browser_sim12_2026-10-30'
READY=DOCS/'prepared_v4'


def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def save(p,d):p.write_text(json.dumps(d,ensure_ascii=False,indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--browser-session',type=Path,required=True)
    parser.add_argument('--esc-session',type=Path,required=True)
    args=parser.parse_args();session=args.browser_session.resolve()
    if session.parent!=READY/'sessions':raise ValueError('wrong GUI session root')
    gui=read(session/'result.json');contract=read(session/'CONTRACT.json')
    if (not gui['passed'] or not gui['completed'] or not gui['ros_initialized'] or not gui['authority_ever_acquired']
            or gui['max_client_gap_s']>.1 or gui['hardware_socket_opened'] or gui['physical_motion_started']
            or not gui['software_stop_evidence']['stop_confirmed'] or contract['synthetic'] is not True):
        raise ValueError('actual synthetic browser UI run did not complete safely')
    esc=args.esc_session.resolve();esc_result=read(esc/'result.json')
    if (esc.parent!=READY/'sessions' or esc_result['reason']!='CLIENT_ESC'
            or not esc_result['authority_ever_acquired'] or not esc_result['software_stop_evidence']['stop_confirmed']
            or esc_result['hardware_socket_opened'] or esc_result['physical_motion_started']):
        raise ValueError('actual browser Esc check incomplete')
    protocol=read(DOCS/'PROTOCOL_TESTS_V4.json')
    required={'normal','no_start','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'}
    if not protocol['passed'] or Path(protocol['root'])!=READY or {c['case'] for c in protocol['results']}!=required:
        raise ValueError('missing protocol coverage')
    for case in protocol['results']:
        if not case['passed'] or not case['stop_confirmed'] or case['hardware_output']:
            raise ValueError('failed case: '+case['case'])
    unit=subprocess.run([sys.executable,'-B','-m','unittest','discover','-s',str(HERE),'-p','test_adapter.py','-v'],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (DOCS/'UNIT_TESTS.log').write_text(unit.stdout)
    if unit.returncode:raise ValueError('unit regressions failed')
    manifest=read(READY/'MANIFEST.json')
    visual=read(DOCS/'UI_CHANGE.json');app=READY/'web/app.js'
    for name,digest in manifest['artifact_hashes'].items():
        if Path(name)==app:
            if digest!=visual['before_sha256'] or sha(app)!=visual['after_sha256']:
                raise ValueError('unreviewed final result display change')
        elif sha(Path(name))!=digest:raise ValueError('runtime artifact drift: '+name)
    # Only the outer launcher changed after generation (added single-owner
    # flock); its controller/worker copies must remain exactly as tested.
    for name,digest in manifest['source_hashes'].items():
        if sha(Path(name))!=digest:
            if Path(name).name=='entry.py':pass
            elif Path(name).name=='prepare.py' and sha(Path(name))==visual['prepare_sha256']:pass
            else:raise ValueError('unexpected source drift: '+name)
    for source,target in (('runtime.py','software_runtime.py'),('browser_operator_source.py','browser_operator.py'),('lifecycle.py','lifecycle.py')):
        if sha(HERE/source)!=sha(READY/target):raise ValueError('tested adapter differs from source')
    manifest['source_hashes']={str(p):sha(p) for p in HERE.iterdir() if p.is_file()}
    for tool in ('start_car_browser_local.sh','start_sign_browser_sim_1p2.sh'):
        p=ROOT/'tools'/tool;manifest['source_hashes'][str(p)]=sha(p)
    manifest['qualified']=True;save(READY/'MANIFEST.json',manifest)
    manifest['artifact_hashes'][str(app)]=sha(app);save(READY/'MANIFEST.json',manifest)
    paths=[DOCS/'PROTOCOL_TESTS_V4.json',DOCS/'UNIT_TESTS.log',DOCS/'UI_CHANGE.json',session/'result.json',session/'STOP.json',session/'PROTECTED.json']
    paths.extend(esc/n for n in ('result.json','STOP.json','PROTECTED.json'))
    for case in protocol['results']:
        folder=Path(case['session']);paths.extend(folder/n for n in ('result.json','STOP.json','PROTECTED.json'))
    release=dict(software_qualified=True,hardware_qualified=False,real_sign_human_accepted=False,
        manifest_sha256=sha(READY/'MANIFEST.json'),evidence={str(p):sha(p) for p in paths},
        browser_session=str(session),ui_stall_ms=100,operator_lease_ms=150,session_s=600,
        real_desktop_maximized_freeze_resolved=False,controller_changed=False)
    save(READY/'RELEASE.json',release)
    print(json.dumps(dict(software_qualified=True,hardware_qualified=False,
        evidence_files=len(release['evidence']),manifest_sha256=release['manifest_sha256']),ensure_ascii=False))


if __name__=='__main__':main()
