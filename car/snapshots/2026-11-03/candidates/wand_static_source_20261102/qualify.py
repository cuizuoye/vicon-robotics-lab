"""Qualify only after a real-time 600s raw-ROS/native-C static soak passes."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from build import ROOT,HERE,BASE,PARENT,OUT,PACKAGE,sha,save,hashes
from ground_entry import check,profile


def main():
    ready=OUT/'host';manifest=ready/'HOST_MANIFEST.json'
    m=json.loads(manifest.read_text())
    assert not m['host_offline_passed'],'never mutate a qualified snapshot'
    assert sha(HERE/'source_pipeline.py')==sha(ready/PACKAGE/'source_pipeline.py')
    changed={str(p.relative_to(ready)) for p in ready.rglob('*') if p.is_file()
        and '__pycache__' not in p.parts and 'sessions' not in p.parts and p.name!='HOST_MANIFEST.json'
        and (not (BASE/p.relative_to(ready)).exists() or sha(p)!=sha(BASE/p.relative_to(ready)))}
    parent_manifest=json.loads((BASE/'HOST_MANIFEST.json').read_text())
    m.update(source_hashes=hashes(HERE),artifact_hashes=hashes(ready),direct_changed_runtime_files=sorted(changed),
        changed_runtime_files=sorted(set(m['changed_runtime_files'])|changed),
        inherited_runtime_hashes={p:d for p,d in m['inherited_runtime_hashes'].items() if (ready/p).exists() and sha(ready/p)==d},
        inherited_parent_runtime_hashes={str(Path(p).relative_to(BASE)):d for p,d in parent_manifest['artifact_hashes'].items()
            if Path(p).is_relative_to(BASE) and str(Path(p).relative_to(BASE)) not in changed})
    save(manifest,m);check(ready,release=False)
    folder=Path(tempfile.mkdtemp(prefix='qualification_',dir=OUT));evidence={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def run(name,args,e=env,timeout=120):
        p=subprocess.run(args,env=e,capture_output=True,text=True,timeout=timeout)
        (folder/(name+'.stdout.log')).write_text(p.stdout)
        (folder/(name+'.stderr.log')).write_text(p.stderr)
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout[-2500:]+p.stderr[-5000:])
        return p.stdout
    for relative,digest in m['inherited_parent_runtime_hashes'].items():
        assert sha(ready/relative)==digest,relative
    source=json.loads(run('source_pipeline',[sys.executable,'-B',str(HERE/'test_pipeline.py')]).splitlines()[-1])
    admission=json.loads(run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')]).splitlines()[-1])
    terminal=json.loads(run('terminal_view',['node',str(PARENT/'test_terminal_view.cjs'),str(ready)]).splitlines()[-1])
    cases=[]
    for case in ('no_start','background','renderer_stall','local_stop','second_space','estop','close','occlusion','node_stall','worker_stall','timer'):
        target=folder/case;target.mkdir();path,consent=profile(ready,target,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),
               WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign')
        run('browser_'+case,['node',str(PARENT/'test_browser.cjs'),str(ready),str(target),case,'7' if case=='timer' else '40'],e)
        value=json.loads((target/'RESULT.json').read_text());assert value['passed'] and not value['physical_output']
        cases.append(value)
    fault_cases=[]
    for fault in ('source_drop','geometry'):
        target=folder/('raw_'+fault)
        run('raw_'+fault,[sys.executable,'-B',str(HERE/'soak.py'),'--seconds','10','--fault',fault,'--output',str(target)],timeout=50)
        value=json.loads((target/'RESULT.json').read_text());assert value['passed'] and value['stop_confirmed'] and value['native_output_zero']
        fault_cases.append(value)
    # Fresh final soak is launched only AFTER the browser fault-injection
    # suite has finished. Earlier failed trials remain archived separately.
    run('static600',[sys.executable,'-B',str(HERE/'soak.py'),'--seconds','600',
        '--stall-input-ms','120','--output',str(folder/'static600')],timeout=670)
    soakfile=folder/'static600/RESULT.json';started=time.monotonic();notice=0.
    while not soakfile.exists():
        if time.monotonic()-started>700:raise TimeoutError('600s soak did not finish')
        if time.monotonic()-notice>=30:
            print('WAITING_REAL_TIME_600S_SOAK',flush=True);notice=time.monotonic()
        time.sleep(.5)
    soak=json.loads(soakfile.read_text())
    assert soak['passed'] and soak['requested_active_s']==600. and soak['observed_active_s']>=599.9,soak
    assert soak['actual_raw_ROS_domain']==197 and soak['actual_control_ROS_domain']==196 and not soak['physical_output']
    short=OUT/'short_motion_2/RESULT.json';short_value=json.loads(short.read_text())
    assert short_value['passed'] and short_value['motion_first_s']==8. and short_value['injected_input_stall_ms']==120.
    protected_control=['session_channel.py','session_model.py','session_operator.py','session_prepare.py','worker.py',
        'server.cjs','web/app.js','node.py','ground_runtime.py',
        'runtime/src/mecanum_wand_4wsc_control/mecanum_wand_4wsc_control/safety_core.py',
        'runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/bound_rotation.py']
    for relative in protected_control:assert sha(ready/relative)==sha(BASE/relative),relative
    report=dict(passed=True,revision='BACKGROUND_STATIC_SOURCE_V3',source_unit=source,admission=admission,
        terminal_view=terminal,browser_cases=cases,raw_fault_cases=fault_cases,
        static600=soak,motion_then_static=short_value,unchanged_control_authority_and_gate=protected_control,
        max_active_s=600.,mapping='1:1 XY + relative yaw; Z ignored',
        pose_deadline_s=.2,command_deadline_s=.1,feedback_deadline_s=.3,
        physical_output=False,firmware_changed=False,network_settings_changed=False,
        live_hardware_600s_pending=True,real_Tracker_600s_pending=True,
        caveat='Offline generated raw frames are not proof of uninterrupted real Tracker output. Genuine pose loss still stops.')
    result=OUT/'QUALIFICATION.json';save(result,report)
    for root in (folder,OUT/'short_motion_2'):
        for f in root.rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log','.png'):evidence[str(f)]=sha(f)
    for value in cases:
        for f in Path(value['session']).rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    evidence[str(result)]=sha(result)
    m.update(host_offline_passed=True,hardware_launch_enabled=True,source_hashes=hashes(HERE),
        evidence=evidence,qualification=str(result))
    wrapper=ROOT/'tools/start_car_standalone_stable.sh';m['source_hashes'][str(wrapper)]=sha(wrapper)
    save(manifest,m)
    try:
        run('post_qualified',[sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified'])
        for f in folder.glob('post_qualified.*.log'):m['evidence'][str(f)]=sha(f)
        save(manifest,m);check(ready)
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(manifest,m);raise
    print('QUALIFIED '+str(result),flush=True)

if __name__=='__main__':main()
