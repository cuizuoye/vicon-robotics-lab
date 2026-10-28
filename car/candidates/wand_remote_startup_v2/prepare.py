#!/usr/bin/env python3
"""Private remote-input/startup candidate; pin the complete previous runtime."""
import importlib.util
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_ros_bound_history_2026-10-27/prepared'
spec=importlib.util.spec_from_file_location('remote_v2_prepare_helpers',ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
sha,hashes,write,edit,check=(getattr(helpers,n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m=check(OLD/'build_manifest.json')
    if not m.get('ros_bound_history_passed'):raise ValueError('verified bound-history parent required')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):protected.update(m[group])
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    for name in ('async_x11.py','remote_keys.py'):shutil.copyfile(HERE/name,c/name)
    shutil.copyfile(HERE/'startup_candidate.py',h/'startup_candidate.py')
    edit(h/'rotation_nodes.py','def source_main():run_node(RotationSignSource)',
        'from .startup_candidate import StartupCandidateMixin\n'
        'class StartupHistoryGate(StartupCandidateMixin,RotationGateNode):pass\n'
        'RotationGateNode=StartupHistoryGate\n\n'
        'def source_main():run_node(RotationSignSource)')
    for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py','delivery_fixture.py'):
        shutil.copyfile(OLD/name,out/name)
    shutil.copyfile(HERE/'remote_probe.py',out/'remote_probe.py')
    app=out/'oneclick_app.py';edit(app,repr(str(OLD)),repr(str(out)))
    edit(app,"m.get('ros_bound_history_passed')","m.get('remote_startup_v2_passed')")
    app.write_text(app.read_text().replace('SHOW_BOUND_HISTORY_V1','SHOW_REMOTE_STARTUP_V2').replace("b'BOUND_HISTORY_V1'","b'REMOTE_STARTUP_V2'"))
    edit(out/'pose_check.py',str(OLD/'runtime/src'),str(runtime))
    edit(out/'launcher_ui.py','ROS参考帧修复候选 · MCU反馈300ms','远程输入与启动修正版V2 · MCU反馈300ms')
    rehearsal=out/'ros_rehearsal.py'
    edit(rehearsal,"'candidate_burst','reference_loss')):","'candidate_burst','reference_loss','startup_old_zero')):")
    edit(rehearsal,'before_click_nonzero=False)', 'before_click_nonzero=False,startup_old_sent=False)')
    edit(rehearsal,'                            pub.publish(msg)',
        "                            if case=='startup_old_zero' and not state.startup_old_sent and pub.get_subscription_count()>0:\n"
        "                                v=json.loads(msg.data)\n"
        "                                if v['frame']['state']=='STANDBY':\n"
        "                                    v['frame']['issued_steady']-=.25;v['frame']['issued_ros']-=.25\n"
        "                                    pub.publish(String(data=canonical(v)));state.startup_old_sent=True\n"
        "                            pub.publish(msg)")
    edit(rehearsal,"elif case=='used_budget':model.end_trial(now,'SOFTWARE Esc');operator.request('stop')",
        "elif case in ('used_budget','startup_old_zero'):model.end_trial(now,'SOFTWARE Esc');operator.request('stop')")
    edit(rehearsal,"                result=dict(case=case,passed=True,phases=phases,service_and_click_calls=calls,gate_faults=faults,",
        "                if case=='startup_old_zero':\n"
        "                    assert state.startup_old_sent and gate.startup_stale_zero_dropped==1,(state.startup_old_sent,gate.startup_stale_zero_dropped)\n"
        "                    assert 'COMMAND_ENVELOPE_INVALID' not in faults,faults\n"
        "                result=dict(case=case,passed=True,phases=phases,service_and_click_calls=calls,gate_faults=faults,\n"
        "                    startup_stale_zero_dropped=gate.startup_stale_zero_dropped,")
    sources=hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_remote_v2.sh',*(out/n for n in
        ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py','delivery_fixture.py','remote_probe.py'))]:sources[str(p)]=sha(p)
    changes=[]
    for p in runtime.rglob('*'):
        if p.is_file():
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.exists() or sha(p)!=sha(old):changes.append(str(rel))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',changes)
    m.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,remote_startup_v2_passed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json');return m
