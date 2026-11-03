"""One-command standalone SIM entry. Check never initializes ROS or transport."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import secrets
from profile12 import ROOT,protected,save,contract

HERE=Path(__file__).resolve().parent
REPORT=ROOT/'docs/wand_sim12_2026-10-30'
WRAPPER=ROOT/'tools/start_sign_sim_1p2.sh'

def hashes():
    return {str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted([*HERE.glob('*.py'),HERE/'sign_geometry.json',WRAPPER])}

def check():
    from geometry12 import definition
    definition();manifest=json.loads((REPORT/'PREPARED.json').read_text())
    if manifest.get('passed') is not True or manifest['sources']!=hashes():raise ValueError('SIM12 files/evidence changed; revalidate first')
    digest=hashlib.sha256(json.dumps(protected(),sort_keys=True).encode()).hexdigest()
    if manifest['protected_sha256']!=digest:raise ValueError('original source drift')
    for path,sha in manifest['evidence'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=sha:raise ValueError('evidence drift: '+path)
    return manifest

def main():
    p=argparse.ArgumentParser(description=__doc__);group=p.add_mutually_exclusive_group()
    group.add_argument('--check',action='store_true');group.add_argument('--launch',action='store_true')
    a=p.parse_args();check()
    if a.check:print(json.dumps(dict(passed=True,**contract()),ensure_ascii=False));return
    out=REPORT/('real_sign_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+secrets.token_hex(3))
    save(REPORT/'LATEST_SESSION.json',dict(path=str(out),hardware_output=False))
    print('SIM12_SESSION '+str(out),flush=True)
    from run12 import run
    run(out)

if __name__=='__main__':main()
