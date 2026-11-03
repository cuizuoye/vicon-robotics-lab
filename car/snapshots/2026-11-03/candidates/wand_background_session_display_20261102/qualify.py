"""Freeze bounded browser results and terminal-state ordering; no real hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT,HERE,BASE,OUT,sha,save,hashes
from ground_entry import check,profile

def main():
    ready=OUT/'host';m=json.loads((ready/'HOST_MANIFEST.json').read_text())
    assert not m['host_offline_passed'],'never overwrite a qualified snapshot'
    m['source_hashes']=hashes(HERE);save(ready/'HOST_MANIFEST.json',m);check(ready,release=False)
    assert set(m['direct_changed_runtime_files'])=={'server.cjs','web/app.js','runtime/src/mecanum_wand_4wsc_hardware/mecanum_wand_4wsc_hardware/deployment_record.py'}
    folder=Path(tempfile.mkdtemp(prefix='qualification_',dir=OUT));evidence={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def run(name,args,e=env):
        p=subprocess.run(args,env=e,capture_output=True,text=True,timeout=120)
        (folder/(name+'.stdout.log')).write_text(p.stdout);(folder/(name+'.stderr.log')).write_text(p.stderr)
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout[-2000:]+p.stderr[-5000:])
        return p.stdout
    for p,d in m['inherited_parent_runtime_hashes'].items():assert sha(ready/p)==d,p
    a=json.loads(run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')]).splitlines()[-1]);assert a['passed']
    u=json.loads(run('terminal_view',['node',str(HERE/'test_terminal_view.cjs'),str(ready)]).splitlines()[-1]);assert u['passed']
    cases=[]
    for case in ('no_start','background','renderer_stall','local_stop','second_space','estop','close','occlusion','node_stall','worker_stall','timer'):
        t=folder/case;t.mkdir();p,consent=profile(ready,t,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(p),WAND_PROFILE_APPROVAL=sha(p),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign')
        run('browser_'+case,['node',str(HERE/'test_browser.cjs'),str(ready),str(t),case,'7' if case=='timer' else '40'],e)
        v=json.loads((t/'RESULT.json').read_text());assert v['passed'] and not v['physical_output'];cases.append(v)
        for f in Path(v['session']).rglob('*'):
            if f.is_file() and f.suffix in ('.json','.jsonl','.log'):evidence[str(f)]=sha(f)
    for f in folder.rglob('*'):
        if f.is_file():evidence[str(f)]=sha(f)
    parent=json.loads((BASE.parent/'QUALIFICATION.json').read_text())
    report=dict(passed=True,revision='BACKGROUND_SESSION_DISPLAY_V2',parent_qualification=str(BASE.parent/'QUALIFICATION.json'),
       inherited_session_unit=parent['session_unit'],inherited_client_unit=parent['client_unit'],
       inherited_stop_command_tests=parent['stop_command_tests'],inherited_launcher_tests=parent['launcher_tests'],
       admission=a,terminal_view_unit=u,browser_cases_rerun=cases,physical_output=False,firmware_changed=False,
       network_settings_changed=False,live_hardware_retest_pending=True,max_active_s=600,
       full_new_600s_background_soak_completed=False,control_and_authority_code_byte_identical_to_parent=True,
       UI_terminal_state_verified_in_actual_browser=True,bounded_result_diagnostics_remain_on_disk=True)
    result=OUT/'QUALIFICATION.json';save(result,report);evidence[str(result)]=sha(result)
    m.update(host_offline_passed=True,hardware_launch_enabled=True,evidence=evidence,source_hashes=hashes(HERE),qualification=str(result))
    wrapper=ROOT/'tools/start_car_standalone.sh';m['source_hashes'][str(wrapper)]=sha(wrapper);save(ready/'HOST_MANIFEST.json',m)
    try:
        p=json.loads(run('post_qualified',[sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified']).splitlines()[-1]);assert p['passed']
        for f in folder.glob('post_qualified.*.log'):m['evidence'][str(f)]=sha(f)
        save(ready/'HOST_MANIFEST.json',m);check(ready)
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(ready/'HOST_MANIFEST.json',m);raise
    print('QUALIFIED '+str(result),flush=True)

if __name__=='__main__':main()
