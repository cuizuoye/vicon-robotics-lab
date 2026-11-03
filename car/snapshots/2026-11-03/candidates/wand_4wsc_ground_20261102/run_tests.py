"""Serial real ROS/native-C loopback tests, no auto retry and no hardware addresses."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import HERE,OUT,save
from ground_entry import profile,sha


def main(ready):
    folder=Path(tempfile.mkdtemp(prefix='tests_',dir=OUT));records=[]
    def run(name,cmd,env=None):
        p=subprocess.run(cmd,env=env,text=True,capture_output=True,timeout=180)
        (folder/(name+'.stdout.log')).write_text(p.stdout);(folder/(name+'.stderr.log')).write_text(p.stderr)
        row=dict(name=name,command=cmd,exit_code=p.returncode,stdout=str(folder/(name+'.stdout.log')),stderr=str(folder/(name+'.stderr.log')))
        records.append(row);save(folder/'INDEX.json',dict(prepared=str(ready),complete=False,records=records))
        print(json.dumps(row),flush=True)
        if p.returncode:raise SystemExit('Failure retained: '+str(folder))
        return p.stdout
    env=dict(os.environ,GROUND_TEST_READY=str(ready),PYTHONDONTWRITEBYTECODE='1')
    run('unit',[sys.executable,'-B',str(HERE/'test_ground.py')],env)
    base_env=dict(os.environ,PYTHONPATH=str(HERE)+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONDONTWRITEBYTECODE='1')
    native=[]
    for case in ('normal','occlusion','car_loss','feedback','loop_gap'):
        cmd=[sys.executable,'-B',str(ready/'tests/native_trial.py'),'--ready',str(ready),'--stage','0','--case',case,'--seconds','6']
        if case=='normal':cmd+=['--reopen']
        text=run('native_'+case,cmd,base_env)
        native += [str(Path(json.loads(line)['folder'])/'RESULT.json') for line in text.splitlines() if line.startswith('{') and '"folder"' in line]
    browser=[]
    for case in ('normal','no_start','esc','blur','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'):
        d=folder/('browser_'+case);d.mkdir();p,consent=profile(ready,d,True)
        env=dict(base_env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),
            WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        target=d/'RESULT.json'
        run('browser_'+case,['node',str(ready/'tests/browser_peer.cjs'),str(ready),str(target),case,'60' if case=='normal' else '8'],env)
        browser.append(str(target))
    save(folder/'INDEX.json',dict(prepared=str(ready),complete=True,records=records,native=native,browser=browser,
        physical_output=False,real_ROS_bridge_and_native_C_loopback=True))
    print('GROUND_TEST_INDEX '+str(folder/'INDEX.json'),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--ready',type=Path,required=True);main(p.parse_args().ready.resolve())
