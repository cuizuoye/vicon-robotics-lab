#!/usr/bin/env python3
"""300ms comparison plus never-enabled, healthy used-budget startup repair."""
import importlib.util
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_feedback_300ms_2026-10-27/prepared'
spec=importlib.util.spec_from_file_location('ready_helpers',ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
sha,hashes,write,edit,check=(getattr(helpers,n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m=check(OLD/'build_manifest.json')
    if not m.get('feedback_300ms_passed'):raise ValueError('verified 300ms parent required')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    runtime=out/'runtime/src'
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    # This is evidence to request preparation, NOT ready-to-move authority.
    # Existing baseline/clear/select/ACK/prime logic remains the sole reset path.
    edit(h/'nodes.py',"            baseline_current=self.baseline_is_current(now),",
        """            baseline_current=self.baseline_is_current(now),
            startup_reset_ready=bool(self.baseline_is_current(now)
                and not self.user_arm_seen and not self.sock.clock.frozen
                and not self.policy.epoch and p.get('state') == 'STANDBY'
                and self.sock.clock.ready(now) and status['started_ms'] > 0
                and status['watchdog_reset'] == 0 and status['fault_wheel'] == 0
                and status['first_feedback_fault']['reason'] == 0),""")
    edit(c/'auto_prepare.py',"and t.get('fresh') is True and t.get('ready') is True",
        "and t.get('fresh') is True\n"
        "            and (t.get('ready') is True or t.get('startup_reset_ready') is True)")
    # A never-armed zero STANDBY heartbeat cannot move the car. Do not make it
    # depend on delivery of an exact historical sensor packet before requesting
    # preparation. ARMED/ACTIVE still bind the exact raw Sign+car samples.
    edit(h/'rotation_nodes.py',"(f.state=='STANDBY' and (v['sign_digest'] is None or v['car_stamp'] is None))",
         "(f.state=='STANDBY' and not f.epoch and f.command==ZERO\n"
         "                        and not self.core.primed_epoch)")
    edit(h/'rotation_nodes.py',"        except (ValueError,TypeError,KeyError):\n            self.pending.clear();self.core.trip('ROTATION_SELECTED_SOURCE_INVALID');self.publish_output(ZERO)",
         "        except (ValueError,TypeError,KeyError) as error:\n"
         "            if not getattr(self,'selected_error_recorded',False):\n"
         "                self.selected_error_recorded=True\n"
         "                self.get_logger().warn('ROTATION_SELECTED_SOURCE_REJECTED '+str(error))\n"
         "            self.pending.clear();self.core.trip('ROTATION_SELECTED_SOURCE_INVALID');self.publish_output(ZERO)")
    for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'):
        shutil.copyfile(OLD/name,out/name)
    app=out/'oneclick_app.py';edit(app,repr(str(OLD)),repr(str(out)))
    edit(app,"m.get('feedback_300ms_passed')", "m.get('feedback_300ms_startup_passed')")
    app.write_text(app.read_text().replace('SHOW_FEEDBACK_300MS_V1','SHOW_FEEDBACK_300MS_READY_V1')
        .replace("b'FEEDBACK_300MS_V1'","b'FEEDBACK_300MS_READY_V1'"))
    edit(out/'pose_check.py',str(OLD/'runtime/src'),str(runtime))
    edit(out/'launcher_ui.py','MCU反馈300ms对照','MCU反馈300ms · 启动修正版')
    ros=out/'ros_rehearsal.py'
    edit(ros,"'feedback_250ms','feedback_310ms'):", "'feedback_250ms','feedback_310ms','used_budget'):")
    edit(ros,"fresh=True,ready=True,fault='',local_stop=False,",
         "fresh=True,ready=(case!='used_budget' or bool(sup.core.epoch)),\n"
         "                        startup_reset_ready=(case=='used_budget' and not sup.core.epoch),fault='',local_stop=False,")
    edit(ros,"elif case.startswith('feedback_'):pass", "elif case.startswith('feedback_'):pass\n"
         "                        elif case=='used_budget':model.end_trial(now,'SOFTWARE Esc');operator.request('stop')")
    # A 250ms-old source published every 50ms reaches 300ms at the next
    # publication, so normal DDS/scheduler overhead correctly expires it.
    # Exercise a sustained 250ms delay at a declared 20ms fixture cadence,
    # leaving actual source ages below 300ms. The 310ms rejection case stays.
    edit(ros,'source.create_timer(.05,fixture)',
         "source.create_timer(.02 if case=='feedback_250ms' else .05,fixture)")
    changes=[]
    for p in runtime.rglob('*'):
        if p.is_file():
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.is_file() or sha(old)!=sha(p):changes.append(str(rel))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',changes)
    sources=hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_300ms_ready.sh',*(out/n for n in
        ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'))]:sources[str(p)]=sha(p)
    m.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,feedback_300ms_startup_passed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json');return m
