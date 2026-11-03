"""Serial loopback qualification of the migrated browser host; no physical IO."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT,HERE,OUT,save
from ground_entry import profile,sha

def main():
    ready=OUT/'host';folder=Path(tempfile.mkdtemp(prefix='tests_',dir=OUT));records=[];native=[];browser=[]
    env=dict(os.environ,PYTHONPATH=str(HERE)+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONDONTWRITEBYTECODE='1')
    def run(name,cmd,environ=env):
        p=subprocess.run(cmd,env=environ,text=True,capture_output=True,timeout=180)
        for suffix,text in (('stdout',p.stdout),('stderr',p.stderr)):(folder/(name+'.'+suffix+'.log')).write_text(text)
        row=dict(name=name,command=cmd,exit_code=p.returncode,stdout=str(folder/(name+'.stdout.log')),stderr=str(folder/(name+'.stderr.log')));records.append(row)
        save(folder/'INDEX.json',dict(prepared=str(ready),complete=False,records=records,physical_output=False))
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError('failure retained '+str(folder)+'\n'+p.stdout[-3000:]+p.stderr[-5000:])
        return p.stdout
    run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')])
    run('focus_client',['node',str(ROOT/'candidates/wand_stopfix_20261102/test_focus.cjs'),str(ready),str(folder/'focus_client')])
    run('disabled_zero',[sys.executable,'-B',str(HERE/'test_idle.py'),str(folder/'disabled_zero')])
    for case in ('normal','occlusion','car_loss','feedback','loop_gap'):
        cmd=[sys.executable,'-B',str(ready/'tests/native_trial.py'),'--ready',str(ready),'--stage','6','--case',case,'--seconds','6']
        if case=='normal':cmd+=['--reopen']
        text=run('native_'+case,cmd)
        native += [str(Path(json.loads(line)['folder'])/'RESULT.json') for line in text.splitlines() if line.startswith('{') and '"folder"' in line]
    for case in ('normal','no_start','esc','blur','proof_timeout','view_disconnect','control_disconnect','second_space','nonce_replay','setpoint_rejected','occlusion'):
        d=folder/('browser_'+case);d.mkdir();p,consent=profile(ready,d,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        target=d/'RESULT.json';run('browser_'+case,['node',str(ready/'tests/browser_peer.cjs'),str(ready),str(target),case,'60' if case=='normal' else '8'],e);browser.append(str(target))
    audit=folder/'audit';audit.mkdir();p,consent=profile(ready,audit,True)
    e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    run('audit_no_authority',['node',str(ROOT/'candidates/wand_stopfix_20261102/test_diagnostic_authority.cjs'),str(ready),str(audit/'RESULT.json'),'proof_timeout','8'],e)
    save(folder/'INDEX.json',dict(prepared=str(ready),complete=True,records=records,native=native,browser=browser,
        disabled_zero=str(folder/'disabled_zero/RESULT.json'),focus_client=str(folder/'focus_client/RESULT.json'),audit=str(audit/'RESULT.json'),physical_output=False,actual_ROS_UDP_C_loopback=True))
    print('DEPLOY_TEST_INDEX '+str(folder/'INDEX.json'),flush=True)
if __name__=='__main__':main()
