"""Qualify the preflight-only fix; optional finite real disabled-zero verification."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT,HERE,BASE,OUT,RECORD,sha,save,hashes
from ground_entry import check,profile

def main(live):
    ready=OUT/'host';m=check(ready,release=False)
    if m['host_offline_passed']:raise ValueError('never overwrite a qualified snapshot')
    folder=Path(tempfile.mkdtemp(prefix='qualification_',dir=OUT));evidence={};records=[]
    env=dict(os.environ,PYTHONPATH=str(HERE)+os.pathsep+os.environ.get('PYTHONPATH',''),PYTHONDONTWRITEBYTECODE='1')
    def run(name,args,environ=env):
        p=subprocess.run(args,env=environ,capture_output=True,text=True,timeout=90)
        for suffix,value in (('stdout',p.stdout),('stderr',p.stderr)):
            path=folder/(name+'.'+suffix+'.log');path.write_text(value);evidence[str(path)]=sha(path)
        records.append(dict(name=name,exit_code=p.returncode));print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout[-2000:]+p.stderr[-5000:])
        return p.stdout
    unit=json.loads(run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')]).splitlines()[-1]);assert unit['passed'] and unit['total_tests']==65
    probe=json.loads(run('native_probe',[sys.executable,'-B',str(HERE/'test_probe.py')]).splitlines()[-1]);assert probe['passed'] and not probe['physical_output']
    assert not probe['cases']['old_false_block']['passed'] and probe['cases']['new_accepts_fresh100ms_source']['passed']
    run('focus_client',['node',str(ROOT/'candidates/wand_stopfix_20261102/test_focus.cjs'),str(ready),str(folder/'focus_client')])
    for case in ('no_start','proof_timeout'):
        d=folder/case;d.mkdir();p,consent=profile(ready,d,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        run('browser_'+case,['node',str(ready/'tests/browser_peer.cjs'),str(ready),str(d/'RESULT.json'),case,'8'],e)
        r=json.loads((d/'RESULT.json').read_text());assert r['passed'] and not r['physical_output'] and r['stop_confirmed']
        for f in Path(r['session']).rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    real=None
    if live:
        sys.path.insert(0,str(ready));import bootstrap
        from idle_probe import run as probe_run,CONSENT
        p=json.loads((ready/'profiles/stage6.LONG600_NOT_APPROVED.json').read_text())
        real=probe_run(p,RECORD,folder/'LIVE_DISABLED_ZERO',CONSENT,30.)
        assert real['passed'] and real['fault_bits']==0 and not real['status']['selected'] and not real['status']['permitted'],real
        assert not any(real['status']['raw_duty']) and real['clock']['ready']
        print('real_disabled_zero30s: PASS',flush=True)
    for root in (folder,Path(probe['folder'])):
        for f in root.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    parent=json.loads((BASE.parent/'QUALIFICATION.json').read_text());assert parent['passed']
    for key in ('worker.py','server.cjs','ground_runtime.py','hardware_runtime.py','web/app.js','web/input_guard.js','web/focus_evidence.js'):
        assert sha(ready/key)==sha(BASE/key),'control/browser runtime changed '+key
    report=dict(passed=True,change_scope='preflight progress consistency and failed-probe evidence only',
        unit_tests=65,native_100ms_cadence_and_batched_feedback_cases=4,DOM_VM_checks=24,browser_protocol_cases_rerun=2,
        inherited_ROS_cases=parent['native_ROS_cases'],inherited_browser_cases=parent['browser_protocol_cases'],
        inherited_normal_active_s=parent['normal_active_s'],parent_qualification=str(BASE.parent/'QUALIFICATION.json'),
        real_zero_only_check=real,real_zero_only_window_s=30. if real else None,physical_motion_started=False,
        firmware_changed=False,network_settings_changed=False,physical_qualified=False,
        probe_progress_timeout_s=1.,unchanged_bridge_progress_timeout_s=1.,feedback_timeout_s=.3,
        continuous_ready_window_s=.6,unchanged_MCU_watchdog_s=.6,select_or_enable_or_clear_sent=False,
        context='User rebooted the car and disconnected drone/C5 together; improvement cannot identify which change caused it. Original300ms trip remains a real freshness event, not reclassified as software-only.',
        additional_notes='Old simulator telemetry20ms did not expose this mismatch; new tests exercise actual100ms cadence and batching. A successful communication check does not qualify motion/braking/current/temperature.')
    m.update(host_offline_passed=True,hardware_launch_enabled=True,evidence=evidence,source_hashes=hashes(HERE))
    wrapper=ROOT/'tools/start_car_stopfix_feedback.sh';m['source_hashes'][str(wrapper)]=sha(wrapper)
    save(ready/'HOST_MANIFEST.json',m)
    try:
        post=json.loads(run('post_qualified',[sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified']).splitlines()[-1]);assert post['passed'] and not post['hardware_opened']
        target=OUT/'QUALIFICATION.json';save(target,report);evidence[str(target)]=sha(target)
        m['evidence']=evidence;m['qualification']=str(target);save(ready/'HOST_MANIFEST.json',m);check(ready)
        save(OUT/'LATEST_PREPARED.json',dict(prepared=str(ready),manifest_sha256=sha(ready/'HOST_MANIFEST.json'),qualification=str(target)))
        print('PREFLIGHT_FIX_QUALIFIED '+str(target),flush=True)
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(ready/'HOST_MANIFEST.json',m);raise
if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--live-zero-only',action='store_true');main(ap.parse_args().live_zero_only)
