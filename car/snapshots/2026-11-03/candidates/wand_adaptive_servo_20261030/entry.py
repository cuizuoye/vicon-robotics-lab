"""Independent adaptive browser entry; never a real-car entry."""
import fcntl
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws');HERE=Path(__file__).resolve().parent
DOCS=ROOT/'docs/wand_adaptive_servo_2026-10-30';READY=DOCS/'prepared_v1'
BASE_ENTRY=ROOT/'candidates/wand_browser_direct_20261030/entry.py'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def check():
    baseline=subprocess.run(['/usr/bin/python3','-B',str(BASE_ENTRY),'--check'],capture_output=True,text=True)
    if baseline.returncode:raise ValueError('fixed baseline source/evidence changed: '+baseline.stderr[-1500:])
    m=json.loads((READY/'MANIFEST.json').read_text());r=json.loads((READY/'RELEASE.json').read_text())
    if not (m['qualified'] and m['software_only'] and not m['hardware_entry'] and m['domain']==191
            and m['scope']=='BROWSER_ADAPTIVE_SIGN_1TO1' and r['software_qualified'] and not r['hardware_qualified']
            and r['manifest_sha256']==sha(READY/'MANIFEST.json')):raise ValueError('adaptive software regressions pending')
    for group in ('source_hashes','artifact_hashes'):
        for path,digest in m[group].items():
            if sha(path)!=digest:raise ValueError('candidate drift: '+path)
    for path,digest in r['evidence'].items():
        if sha(path)!=digest:raise ValueError('evidence drift: '+path)
    node=shutil.which('node')
    if not node:raise ValueError('Node missing')
    return dict(passed=True,node=node,software_only=True,hardware_output=False,intermediate_reference=False,
        adaptive_experiment=True,not_promoted_over_fixed_baseline=True,max_session_s=600.,speed_mps=1.2,
        accel_budget_mps2=[1.,4.],yaw_accel_budget_radps2=[1.5,12.],browser_timeout_ms=100)

def main():
    # Reuse tested launcher mechanics (regular-file Node logs and graceful
    # shutdown) in THIS process only. The baseline file is never modified.
    spec=importlib.util.spec_from_file_location('reused_fixed_browser_launcher',BASE_ENTRY)
    parent=importlib.util.module_from_spec(spec);spec.loader.exec_module(parent)
    parent.DOCS=DOCS;parent.READY=READY;parent.check=check
    if '--check' in sys.argv:parent.main();return
    lock=(ROOT/'docs/wand_browser_direct_2026-10-30/RUN.lock').open('a')
    try:
        try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:raise SystemExit('固定基线入口仍运行，请先停止并关闭；不会接管会话。')
        parent.main()
    finally:lock.close()

if __name__=='__main__':main()
