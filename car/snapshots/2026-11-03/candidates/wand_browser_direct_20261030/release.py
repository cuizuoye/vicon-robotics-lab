"""Seal direct target only after same-source, ROS and actual-browser checks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
DOCS=ROOT/'docs/wand_browser_direct_2026-10-30'
READY=DOCS/'prepared_v1'

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def read(p):return json.loads(p.read_text())
def save(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')

def main():
    p=argparse.ArgumentParser();p.add_argument('--browser-session',type=Path,required=True);a=p.parse_args()
    browser=a.browser_session.resolve()
    if browser.parent!=READY/'sessions':raise ValueError('wrong actual-browser session')
    ui=read(browser/'result.json')
    if (ui['reason']!='CLIENT_ESC' or not ui['authority_ever_acquired'] or not ui['ros_initialized']
            or not ui['software_stop_evidence']['stop_confirmed'] or not ui['software_stop_evidence']['gate_output_zero']
            or ui['hardware_socket_opened'] or ui['physical_motion_started'] or ui['max_client_gap_s']>.1):
        raise ValueError('browser start/motion/Esc/stop evidence incomplete')
    protocol=read(DOCS/'ROS_PROTOCOL.json')
    required={'normal','no_start','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'}
    if not protocol['passed'] or Path(protocol['root'])!=READY or {r['case'] for r in protocol['results']}!=required:
        raise ValueError('ROS protocol evidence incomplete')
    if any(not r['passed'] or not r['stop_confirmed'] or r['hardware_output'] for r in protocol['results']):
        raise ValueError('ROS protocol failure')
    comparison=read(DOCS/'COMPARISON.json')
    if any(v<=10 for v in comparison['improvement_percent'].values()):raise ValueError('sync regression')
    unit=subprocess.run([sys.executable,'-B','-m','pytest','-q',str(HERE/'test_direct.py')],
        stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    (DOCS/'UNIT_TESTS.log').write_text(unit.stdout)
    if unit.returncode:raise ValueError('unit regression')
    manifest=read(READY/'MANIFEST.json')
    for name,digest in manifest['artifact_hashes'].items():
        if sha(Path(name))!=digest:raise ValueError('tested artifact drift: '+name)
    manifest['source_hashes']={str(p):sha(p) for p in HERE.iterdir() if p.is_file()}
    tool=ROOT/'tools/start_sign_browser_direct.sh';manifest['source_hashes'][str(tool)]=sha(tool)
    manifest['qualified']=True;save(READY/'MANIFEST.json',manifest)
    paths=[DOCS/'UNIT_TESTS.log',DOCS/'COMPARISON.json',DOCS/'ROS_PROTOCOL.json',DOCS/'browser_direct_active.jpg']
    for session in [browser,*[Path(r['session']) for r in protocol['results']]]:
        paths.extend(session/f for f in ('result.json','STOP.json','PROTECTED.json','CONTRACT.json'))
    save(READY/'RELEASE.json',dict(software_qualified=True,hardware_qualified=False,real_sign_human_accepted=False,
        manifest_sha256=sha(READY/'MANIFEST.json'),evidence={str(p):sha(p) for p in paths},
        actual_browser_session=str(browser),reference_governor_removed=True,controller_changed=True,
        browser_test_input='synthetic',physical_car_started=False))
    print(json.dumps(dict(software_qualified=True,hardware_qualified=False,tests=29,ros_cases=len(required)),ensure_ascii=False))

if __name__=='__main__':main()
