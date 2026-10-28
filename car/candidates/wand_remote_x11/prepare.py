#!/usr/bin/env python3
"""Private remote presentation/input scheduling candidate. No transport edits."""
import importlib.util
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_feedback_300ms_startup_2026-10-27/prepared'
spec=importlib.util.spec_from_file_location('remote_prepare_helpers',ROOT/'candidates/wand_tracking_autostart/prepare.py')
helpers=importlib.util.module_from_spec(spec);spec.loader.exec_module(helpers)
sha,hashes,write,edit,check=(getattr(helpers,n) for n in ('sha','hashes','write','edit','check'))


def generate(out):
    m=check(OLD/'build_manifest.json')
    if not m.get('feedback_300ms_startup_passed'):raise ValueError('verified startup parent required')
    protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):protected.update(m[group])
    out.mkdir(parents=True,exist_ok=False);out.chmod(0o700);(out/'COLCON_IGNORE').touch()
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package']
    for name in ('async_x11.py','remote_keys.py','remote_render.py'):shutil.copyfile(HERE/name,c/name)
    op=c/'operator_console.py'
    edit(op,'from .toggle_keys import ToggleKeysMixin','from .remote_keys import RemoteKeysMixin as ToggleKeysMixin')
    edit(op,'from .x11_key_state import X11KeyState','from .async_x11 import open_key_state as X11KeyState')
    edit(op,'class RotationConsole(TrackingConsoleMixin, LargeTypeConsole):',
        'from .remote_render import RemoteRenderMixin\nclass RotationConsole(RemoteRenderMixin, TrackingConsoleMixin, LargeTypeConsole):')
    for name in ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'):
        shutil.copyfile(OLD/name,out/name)
    app=out/'oneclick_app.py';edit(app,repr(str(OLD)),repr(str(out)))
    edit(app,"m.get('feedback_300ms_startup_passed')", "m.get('remote_x11_passed')")
    app.write_text(app.read_text().replace('SHOW_FEEDBACK_300MS_READY_V1','SHOW_REMOTE_X11_V1')
        .replace("b'FEEDBACK_300MS_READY_V1'","b'REMOTE_X11_V1'"))
    edit(out/'pose_check.py',str(OLD/'runtime/src'),str(runtime))
    edit(out/'launcher_ui.py','MCU反馈300ms · 启动修正版','远程X11候选 · MCU反馈300ms')
    sources=hashes(HERE)
    for p in [ROOT/'tools/start_car_tracking_remote.sh',*(out/n for n in
        ('oneclick_app.py','auto_open.py','launcher_ui.py','pose_check.py','ros_rehearsal.py'))]:sources[str(p)]=sha(p)
    changes=[]
    for p in runtime.rglob('*'):
        if p.is_file():
            rel=p.relative_to(runtime);old=Path(m['runtime_src'])/rel
            if not old.exists() or sha(p)!=sha(old):changes.append(str(rel))
    write(out/'PRIVATE_CHANGE_INVENTORY.json',changes)
    m.update(runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,remote_x11_passed=False,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)])
    write(out/'build_manifest.json',m);check(out/'build_manifest.json');return m
