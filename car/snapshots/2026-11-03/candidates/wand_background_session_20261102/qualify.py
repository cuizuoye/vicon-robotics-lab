"""Offline release: real browser background/freeze and real ROS/native-C loopback."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from build import ROOT,HERE,BASE,OUT,sha,save,hashes,refresh_draft
from ground_entry import check,profile


def main():
    ready=OUT/'host';refresh_draft();m=check(ready,release=False)
    folder=Path(tempfile.mkdtemp(prefix='qualification_',dir=OUT));evidence={}
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    def run(name,args,e=env,timeout=120):
        p=subprocess.run(args,env=e,capture_output=True,text=True,timeout=timeout)
        (folder/(name+'.stdout.log')).write_text(p.stdout);(folder/(name+'.stderr.log')).write_text(p.stderr)
        print(name+': '+str(p.returncode),flush=True)
        if p.returncode:raise ValueError(p.stdout[-3000:]+p.stderr[-6000:])
        return p.stdout
    # Same pose processing, servo, safety gate, signed bridge, stop confirmation,
    # firmware image and limits. Only the explicit operator authority mode differs.
    for relative,d in m['inherited_parent_runtime_hashes'].items():assert sha(ready/relative)==d,relative
    for name in ('ground_runtime.py','receiver.py','idle_probe.py','native_peer.py','node.py','view_slot.cjs'):
        assert sha(ready/name)==sha(BASE/name),name
    a=json.loads(run('admission',[sys.executable,'-B',str(HERE/'test_admission.py')]).splitlines()[-1]);assert a['passed']
    u=json.loads(run('session_unit',[sys.executable,'-B',str(HERE/'test_session.py')]).splitlines()[-1]);assert u['passed']
    client=json.loads(run('client_unit',['node',str(HERE/'test_client.cjs')]).splitlines()[-1]);assert client['passed']
    run('stop_command_unit',[sys.executable,'-B',str(HERE/'test_stop.py')])
    run('launcher_unit',[sys.executable,'-B',str(HERE/'test_startup.py')])
    cases=[]
    for case in ('no_start','background','renderer_stall','local_stop','second_space','estop','close','occlusion','node_stall','worker_stall','timer'):
        target=folder/case;target.mkdir();path,consent=profile(ready,target,True)
        e=dict(env,WAND_4WSC_MODE='rehearsal',WAND_4WSC_PROFILE=str(path),WAND_PROFILE_APPROVAL=sha(path),WAND_HARDWARE_CONSENT=consent,WAND_4WSC_PURPOSE='sign')
        run('browser_'+case,['node',str(HERE/'test_browser.cjs'),str(ready),str(target),case,'7' if case=='timer' else '40'],e)
        v=json.loads((target/'RESULT.json').read_text());assert v['passed'] and not v['physical_output'];cases.append(v)
        for p in Path(v['session']).rglob('*'):
            if p.is_file() and p.suffix in ('.json','.jsonl','.log'):evidence[str(p)]=sha(p)
    for p in folder.rglob('*'):
        if p.is_file():evidence[str(p)]=sha(p)
    report=dict(passed=True,revision='BACKGROUND_SESSION_V1',admission=a,session_unit=u,client_unit=client,
                stop_command_tests=4,launcher_tests=8,real_browser_ROS_native_C_cases=cases,
                physical_output=False,firmware_changed=False,network_settings_changed=False,original_frozen_entry_unchanged=True,
                motion_limits_unchanged=True,mapping='1:1 XY + relative yaw; no fixed fence; no intermediate reference',
                max_active_s=600,physical_qualified=False,live_hardware_retest_pending=True,
                authority_mode='WAND_BACKEND_SESSION_V1',browser_focus_visibility_and_rendering_are_not_authority=True,
                browser_disconnect_still_stops=True,backend_control_gap_s=.1,backend_command_channel_timeout_s=.1,
                command_timeout_s=.1,pose_timeout_s=.2,MCU_feedback_timeout_s=.3,
                emergency_confirmation_policy='ESTOP stays latched; native-C zero verified, but unconfirmed stop is NOT relabelled confirmed and restart stays blocked',
                full_new_600s_background_soak_completed=False,
                test_environment_visibility_note='Window minimizes and really loses focus; this X11 environment reports document.hidden=false. That value is recorded, not faked. Hidden-handler logic is unit tested; actual renderer freeze is CDP tested.')
    result=OUT/'QUALIFICATION.json';save(result,report);evidence[str(result)]=sha(result)
    m.update(host_offline_passed=True,hardware_launch_enabled=True,evidence=evidence,source_hashes=hashes(HERE),qualification=str(result))
    wrapper=ROOT/'tools/start_car_background.sh';m['source_hashes'][str(wrapper)]=sha(wrapper);save(ready/'HOST_MANIFEST.json',m)
    try:
        p=json.loads(run('post_qualified',[sys.executable,'-B',str(HERE/'test_admission.py'),'--post-qualified']).splitlines()[-1]);assert p['passed']
        for item in folder.glob('post_qualified.*.log'):m['evidence'][str(item)]=sha(item)
        save(ready/'HOST_MANIFEST.json',m);check(ready)
    except BaseException:
        m.update(host_offline_passed=False,hardware_launch_enabled=False);save(ready/'HOST_MANIFEST.json',m);raise
    print('QUALIFIED '+str(result),flush=True)

if __name__=='__main__':main()
