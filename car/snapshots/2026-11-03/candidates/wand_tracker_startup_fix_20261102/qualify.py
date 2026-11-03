"""Real-browser and loopback-only qualification. Never touches physical ports."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from build import ROOT,HERE,BASE,OUT,sha,save,hashes,client_source
from ground_entry import check,profile


def main():
    ready=OUT/'host';m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    if m['host_offline_passed']:raise ValueError('never overwrite a qualified snapshot')
    # Refresh generated draft artifacts only, never an already-qualified host.
    shutil.copyfile(HERE/'page_warmup.js',ready/'web/page_warmup.js')
    (ready/'web/app.js').write_text(client_source())
    m.update(source_hashes=hashes(HERE),artifact_hashes=hashes(ready))
    m['artifact_hashes'].pop(str(ready/'HOST_MANIFEST.json'),None)
    save(ready/'HOST_MANIFEST.json',m);check(ready,release=False)
    folder=Path(tempfile.mkdtemp(prefix='qualification_',dir=OUT));evidence={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def run(name,cmd,e=env,timeout=150):
        p=subprocess.run(cmd,env=e,capture_output=True,text=True,timeout=timeout)
        (folder/(name+'.stdout.log')).write_text(p.stdout);(folder/(name+'.stderr.log')).write_text(p.stderr)
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout[-3000:]+p.stderr[-6000:])
        return p.stdout
    for relative,d in m['inherited_parent_runtime_hashes'].items():assert sha(ready/relative)==d,relative
    for name in ('worker.py','ground_runtime.py','hardware_runtime.py','browser_operator.py','control_channel.py','channel.py','web/input_guard.js','web/focus_evidence.js','receiver.py','idle_probe.py'):
        assert sha(ready/name)==sha(BASE/name),name
    admission=json.loads(run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')]).splitlines()[-1]);assert admission['passed']
    run('launcher_unit',[sys.executable,'-B',str(HERE/'test_startup.py')])
    warm=json.loads(run('warmup_unit',['node',str(HERE/'test_warmup.cjs')]).splitlines()[-1]);assert warm['passed']
    run('client_guards',['node',str(HERE/'test_client.cjs'),str(ready),str(folder/'client_guards')])
    dom=json.loads((folder/'client_guards/RESULT.json').read_text());assert dom['checks']==24 and dom['passed']
    real=[]
    for case in ('idle','resize','stall'):
        target=folder/('real_browser_'+case);target.mkdir()
        path,consent=profile(ready,target,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign')
        run('real_browser_'+case,['node',str(HERE/'test_browser.cjs'),str(ready),str(target),case],e)
        r=json.loads((target/'RESULT.json').read_text());assert r['passed'];real.append(r)
        for item in Path(r['session']).rglob('*'):
            if item.is_file() and item.suffix in ('.json','.jsonl','.log'):evidence[str(item)]=sha(item)
    peers=[]
    for case in ('no_start','normal','esc','second_space','blur','proof_timeout','nonce_replay','view_disconnect','setpoint_rejected','occlusion'):
        target=folder/('protocol_'+case);target.mkdir()
        path,consent=profile(ready,target,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign')
        run('protocol_'+case,['node',str(ready/'tests/browser_peer.cjs'),str(ready),str(target/'RESULT.json'),case,'5'],e)
        r=json.loads((target/'RESULT.json').read_text());assert r['passed'];peers.append(r)
        for item in Path(r['session']).rglob('*'):
            if item.is_file() and item.suffix in ('.json','.jsonl','.log'):evidence[str(item)]=sha(item)
    for item in folder.rglob('*'):
        if item.is_file():evidence[str(item)]=sha(item)
    report=dict(passed=True,revision='TRACKER_STARTUP_FIX_V1',unit_admission=admission['total_tests'],launcher_unit_tests=8,warmup_checks=warm['checks'],DOM_VM_checks=24,
                actual_browser_cases=real,real_ROS_native_C_protocol_cases=peers,physical_output=False,firmware_changed=False,network_settings_changed=False,
                controls_and_guards_byte_identical=True,original_frozen_entry_unchanged=True,
                timing_guards_s=dict(browser=.1,MCU_feedback=.3),live_hardware_retest_pending=True,
                spontaneous_original_browser_timeout_reproduced=False,
                scope='pre-connect warmup and cached world grid; persistent status page; explicit fresh-session retry; no automatic resume')
    target=OUT/'QUALIFICATION.json';save(target,report);evidence[str(target)]=sha(target)
    m.update(host_offline_passed=True,hardware_launch_enabled=True,source_hashes=hashes(HERE),evidence=evidence,qualification=str(target))
    wrapper=ROOT/'tools/start_car_tracker_fixed.sh';m['source_hashes'][str(wrapper)]=sha(wrapper)
    save(ready/'HOST_MANIFEST.json',m)
    try:
        post=json.loads(run('post_qualified',[sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified']).splitlines()[-1]);assert post['passed']
        # post-qualified logs were not in evidence at the earlier pass.
        for p in folder.glob('post_qualified.*.log'):m['evidence'][str(p)]=sha(p)
        save(ready/'HOST_MANIFEST.json',m);check(ready)
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(ready/'HOST_MANIFEST.json',m);raise
    print('QUALIFIED '+str(target),flush=True)

if __name__=='__main__':main()
