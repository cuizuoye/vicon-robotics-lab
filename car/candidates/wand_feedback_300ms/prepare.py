#!/usr/bin/env python3
"""Independent, explicitly requested MCU feedback freshness comparison."""
import argparse
import importlib.util
import json
from pathlib import Path
import shutil

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
OLD = ROOT/'docs/wand_feedback_guard_2026-10-27/prepared'
spec = importlib.util.spec_from_file_location('timeout_prepare_helpers',
    ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
sha, hashes, write, edit, check = (getattr(helpers, n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m = check(OLD/'build_manifest.json')
    if not m.get('feedback_guard_passed') or m['feedback_protection_ms'] != 200:
        raise ValueError('verified 200ms feedback-order/disable parent required')
    protected = {str(OLD/'build_manifest.json'): sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    out.mkdir(parents=True, exist_ok=False);out.chmod(0o700)
    (out/'COLCON_IGNORE').touch()
    runtime = out/'runtime/src'
    shutil.copytree(m['runtime_src'], runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c = runtime/m['control_package']/m['control_package']
    h = runtime/m['hardware_package']/m['hardware_package']
    # Single source of the ONLY relaxed deadline. All candidate validators and
    # actual transport checks consume the same pinned value, never an env knob.
    (c/'feedback_policy.py').write_text(
        '"""User-requested, private MCU feedback tolerance comparison."""\n'
        'FEEDBACK_TIMEOUT_S = .300\n'
        'POLICY = "MCU_FEEDBACK_300MS_COMPARISON_V1"\n')
    for file, anchor in ((c/'safety_core.py','from dataclasses import asdict, dataclass'),
                         (c/'servo_config.py','from dataclasses import dataclass, replace')):
        edit(file,anchor,anchor+'\nfrom .feedback_policy import FEEDBACK_TIMEOUT_S')
    edit(c/'safety_core.py','feedback_timeout_s: float = .20',
         'feedback_timeout_s: float = FEEDBACK_TIMEOUT_S')
    edit(c/'servo_config.py','feedback_timeout_s=.20,',
         'feedback_timeout_s=FEEDBACK_TIMEOUT_S,')
    edit(c/'servo_config.py','# No timing, current, speed, acceleration or arena scope relaxation.',
         '# Only the explicit private MCU feedback deadline changes; other guards stay pinned.')
    edit(h/'startup_clock.py','import json',
         'import json\nfrom mecanum_wand_4ws8_control.feedback_policy import FEEDBACK_TIMEOUT_S')
    edit(h/'startup_clock.py','if not -.02 <= now-result <= .20:',
         'if not -.02 <= now-result <= FEEDBACK_TIMEOUT_S:')
    edit(h/'startup_clock.py','k in self.accepted_at and 0 <= now - self.accepted_at[k] <= .20',
         'k in self.accepted_at and 0 <= now - self.accepted_at[k] <= FEEDBACK_TIMEOUT_S')
    edit(h/'startup_clock.py',"max_hello_rtt_s=self.MAX_HELLO_RTT_S,",
         'max_hello_rtt_s=self.MAX_HELLO_RTT_S, feedback_timeout_s=FEEDBACK_TIMEOUT_S,')
    # Native sampling continuity (200ms), handshake RTT (40ms), future-time
    # tolerance (20ms) and disabled baseline echo (200ms) remain stricter.
    edit(h/'nodes.py','from .kernel_ingress import KernelIngressSocket',
         'from .kernel_ingress import KernelIngressSocket\n'
         'from mecanum_wand_4ws8_control.feedback_policy import FEEDBACK_TIMEOUT_S')
    edit(h/'nodes.py','v is not None and 0 <= now-v <= .20 for v in',
         'v is not None and 0 <= now-v <= FEEDBACK_TIMEOUT_S for v in')
    edit(h/'nodes.py','# Same 200ms sample deadline as the independent supervisor/gate.',
         '# Same explicit 300ms MCU sample deadline as supervisor/gate.')
    # Pin the lower receipt deadline too; otherwise the inherited sender can
    # disconnect at 200ms before the 300ms source-age policy gets a chance.
    edit(h/'nodes.py',"if (self.esp_ip != self.runtime.ip or self.network_interface != self.runtime.interface",
         'if (self.link_receive_timeout_s != FEEDBACK_TIMEOUT_S\n'
         '                or self.esp_ip != self.runtime.ip or self.network_interface != self.runtime.interface')
    launch = runtime/m['hardware_package']/'launch/wand_4ws8_staged.launch.py'
    edit(launch,'    from mecanum_wand_4ws8_hardware.commissioning import Runtime, DOMAIN, NAMESPACE',
         '    from mecanum_wand_4ws8_hardware.commissioning import Runtime, DOMAIN, NAMESPACE\n'
         '    from mecanum_wand_4ws8_control.feedback_policy import FEEDBACK_TIMEOUT_S')
    edit(launch,"'link_receive_timeout_s': .20,", "'link_receive_timeout_s': FEEDBACK_TIMEOUT_S,")
    # Timing is not a user-overridable profile field. Keep draft profiles byte
    # identical: the private validated dataclass default supplies exactly .300.
    # Runtime's existing rejection of profile timing overrides remains intact.
    for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'):
        shutil.copyfile(OLD/name,out/name)
    app=out/'oneclick_app.py'
    edit(app,repr(str(OLD)),repr(str(out)))
    edit(app,"m.get('feedback_guard_passed')", "m.get('feedback_300ms_passed')")
    app.write_text(app.read_text().replace('SHOW_FEEDBACK_GUARD_V1','SHOW_FEEDBACK_300MS_V1')
        .replace("b'FEEDBACK_GUARD_V1'", "b'FEEDBACK_300MS_V1'"))
    edit(out/'pose_check.py',str(OLD/'runtime/src'),str(runtime))
    edit(out/'launcher_ui.py',"小车全场演示 · 自动自检 · 无需勾选",
         '小车全场演示 · MCU反馈300ms对照 · 未点击空格不运动')
    # Extra cases exercise actual supervisor/gate DDS callbacks with old source
    # stamps. Only this SOFTWARE fixture changes; no fake feedback goes to car.
    ros=out/'ros_rehearsal.py'
    edit(ros,"'source_conflict','wide_motion'):",
         "'source_conflict','wide_motion','feedback_250ms','feedback_310ms'):")
    edit(ros,"send('plant_feedback',dict(sequence=state.seq,issued_steady=now,issued_ros=ros,",
         "age = (.25 if case=='feedback_250ms' else .31) if state.injected is not None and case.startswith('feedback_') else 0.\n"
         "                    send('plant_feedback',dict(sequence=state.seq,issued_steady=now-age,issued_ros=ros-age,")
    edit(ros,"elif case=='actuator_fault':state.fault_bits=0x20",
         "elif case=='actuator_fault':state.fault_bits=0x20\n"
         "                        elif case.startswith('feedback_'):pass")
    edit(ros,"                    if state.injected is not None and state.restored is None and now-state.injected>.45:",
         "                    if case=='feedback_250ms' and state.injected is not None:\n"
         "                        if now-state.injected<1.:\n"
         "                            assert sup.core.state=='ACTIVE' and not gate.core.fault,(sup.core.reason,gate.core.fault)\n"
         "                        elif state.restored is None:\n"
         "                            model.end_trial(now,'SOFTWARE Esc after 250ms tolerance');operator.request('stop')\n"
         "                            state.restored=now\n"
         "                    if case!='feedback_250ms' and state.injected is not None and state.restored is None and now-state.injected>.45:")
    changes=[]
    for p in runtime.rglob('*'):
        if p.is_file():
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.is_file() or sha(p)!=sha(old):changes.append(str(rel))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',changes)
    sources=hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_300ms.sh',*(out/n for n in
            ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'))]:
        sources[str(p)]=sha(p)
    m.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,feedback_300ms_passed=False,
        feedback_protection_ms=300,firmware_changed=False,parent_entry_changed=False,
        feedback_policy_request=str(HERE/'USER_REQUEST.json'),
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json');return m


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    generate(p.parse_args().output.resolve())
