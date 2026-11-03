"""Fresh fixed loopback profile for each Node protocol case, no real network."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
from entry import setup_rehearsal

def main(ready,normal_seconds):
    folder=Path(tempfile.mkdtemp(prefix='browser_protocol_',dir=ready.parent));records=[]
    for name in ('normal','no_start','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'):
        case=folder/name;case.mkdir();profile,consent=setup_rehearsal(0,case)
        env=dict(os.environ,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(profile),
            WAND_PROFILE_APPROVAL=hashlib.sha256(profile.read_bytes()).hexdigest(),WAND_HARDWARE_CONSENT=consent,
            WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',PYTHONDONTWRITEBYTECODE='1')
        target=case/'RESULT.json'
        p=subprocess.run(['node',str(Path(__file__).with_name('test_browser.cjs')),str(ready),str(target),name,str(normal_seconds if name=='normal' else 8)],env=env)
        records.append(json.loads(target.read_text()) if target.exists() else dict(case=name,passed=False,error='no case result'))
        passed=all(r['passed'] for r in records)
        (folder/'RESULT.json').write_text(json.dumps(dict(passed=passed,complete=len(records)==9,
            scope='REAL_ROS_REAL_UDP_BRIDGE_EXACT_C_LOOPBACK_PROTOCOL_PEER_NOT_BROWSER',records=records),ensure_ascii=False,indent=2)+'\n')
        if p.returncode:raise SystemExit('failed case preserved: '+str(folder))
    print('BROWSER_PROTOCOL_REPORT '+str(folder/'RESULT.json'),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ready',type=Path,required=True);p.add_argument('--normal-seconds',type=float,default=60.)
    a=p.parse_args();main(a.ready.resolve(),a.normal_seconds)
