#!/usr/bin/env python3
"""Generate an isolated host migration. NEVER initialize hardware or flash."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_demo_large_type_2026-10-26/prepared'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
GEOMETRY=ROOT/'docs/wand_rotation_preview_2026-10-26/readonly_uppz8hn2/geometry_confirmed.json'
SIM=ROOT/'candidates/wand_rotation_sim/rotation_core.py'
sys.path.insert(0,str(BASE))
from launch_cli import check


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def hashes(folder):
    return {str(p):sha(p) for p in folder.rglob('*') if p.is_file()
        and not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
def edit(p,old,new):
    s=p.read_text()
    if s.count(old)!=1:raise ValueError('Exact source layout changed: '+str(p)+' / '+old)
    p.write_text(s.replace(old,new))


def generate(out):
    m=check(OLD/'build_manifest.json');out.mkdir(parents=True,exist_ok=False);out.chmod(0o700)
    (out/'COLCON_IGNORE').touch();protected={str(OLD/'build_manifest.json'):sha(OLD/'build_manifest.json')}
    for group in ('runtime_hashes','source_hashes','generated_firmware_hashes','protected_input_hashes'):
        protected.update(m[group])
    extras=[SIM,GEOMETRY,ROOT/'candidates/wand_rotation_preview/preview_core.py',
        ROOT/'candidates/wand_rotation_preview/receiver_guard.py',
        ROOT/'docs/wand_rotation_accel_sim_2026-10-26/A2_FOLLOW_ACCEPTANCE.json',
        ROOT/'tools/start_sign_rotation_accel_sim.sh']
    protected.update({str(p):sha(p) for p in extras})
    for p,v in protected.items():
        if sha(p)!=v:raise ValueError('Protected parent drift: '+p)
    runtime=out/'runtime/src';shutil.copytree(m['runtime_src'],runtime,
        ignore=shutil.ignore_patterns('__pycache__','.pytest_cache','*.egg-info'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    for name in ('rotation_core.py','rotation_source.py','rotation_ui.py'):
        shutil.copyfile(HERE/name,c/name)
    shutil.copyfile(HERE/'rotation_nodes.py',h/'rotation_nodes.py')
    shutil.copyfile(HERE/'rotation_policy.py',c/'demo_policy.py')
    shutil.copyfile(GEOMETRY,c/'rotation_geometry.json')
    shutil.copyfile(ROOT/'candidates/wand_rotation_preview/preview_core.py',c/'rotation_geometry.py')
    guard=c/'rotation_receiver_guard.py';shutil.copyfile(ROOT/'candidates/wand_rotation_preview/receiver_guard.py',guard)
    edit(guard,"if os.environ.get('ROS_DOMAIN_ID','0')!='0':", "if os.environ.get('ROS_DOMAIN_ID','0') not in ('0','120'):")
    # Port the exact mapping/servo/Inputs already tested in simulation. Do not
    # import its bootstrap, software limits, simulation plant or SIM authority.
    s=SIM.read_text();block=s[s.index('class RotationMapping:'):s.index('class RotationSupervisor(')]
    block=block.replace('class RotationMapping:', '@dataclass(init=False)\nclass RotationMapping:\n'
        '    base: RelativeMapping\n    car_origin: Pose\n    wand_origin: Pose\n    scale: float\n    turn: float')
    prefix='''"""Pinned XY+yaw servo port; actual limits supplied by strict hardware config."""
from collections import deque
from dataclasses import dataclass,replace
import math
from .core import Pose,Velocity,RelativeMapping,clamp,wrap_angle
from .continuous_servo import ContinuousSession
from .safety_core import Inputs
from mecanum_vicon_control.mecanum_kinematics import body_to_wheel_angular
WHEEL_RADIUS=.030
HALF_WHEELBASE=.061
HALF_TRACK=.085
MAX_RPM=400.
ANGULAR_LEAD=.50
MAX_ANGULAR_TRACKING_ERROR=.65

'''
    (c/'rotation_servo.py').write_text(prefix+block)
    edit(c/'safety_nodes.py','from .pilot_core import PilotFinalGate as FinalGate, PilotSupervisor as Supervisor',
        'from .rotation_core import RotationFinalGate as FinalGate, RotationSupervisor as Supervisor')
    edit(h/'commissioning.py','from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig as SafetyConfig',
        'from mecanum_wand_4ws8_control.rotation_core import RotationSafetyConfig as SafetyConfig')
    edit(h/'commissioning.py',"candidate='safe_wand_supervisor', cmd_vel='safe_wand_gate', actuator_command='safe_wand_gate')",
        "candidate='safe_wand_supervisor', cmd_vel='safe_wand_gate', actuator_command='safe_wand_gate', rotation_packet='wand_sign_source')")
    edit(h/'nodes.py','class SignSource(Node):','class SignSource(Node):\n    raw_type = RawSource')
    edit(h/'nodes.py','self.raw = RawSource(self.source_context)','self.raw = self.raw_type(self.source_context)')
    kind='BOUNDED_XY_YAW_4WS8_EXISTING_ENVELOPE_NOT_QUALIFICATION'
    edit(h/'authorization.py',"review.get('kind') == 'BOUNDED_SPACE_TOGGLE_120S_NOT_QUALIFICATION'",f"review.get('kind') == {kind!r}")
    auth=h/'pilot_authorization.py'
    auth.write_text(auth.read_text()+f'''

# New scope cannot be approved by the historical translation-only review.
KIND={kind!r}
_verify_existing_pilot = verify_pilot_review
ROTATION_REQUEST=Path({str(HERE/'USER_REQUEST.json')!r})
ROTATION_REQUEST_SHA={sha(HERE/'USER_REQUEST.json')!r}
GEOMETRY_SHA={sha(GEOMETRY)!r}
def verify_pilot_review(review, stage, purpose, firmware_sha):
    if (review.get('rotation_request')!=str(ROTATION_REQUEST)
            or review.get('rotation_request_sha256')!=ROTATION_REQUEST_SHA
            or sha(ROTATION_REQUEST)!=ROTATION_REQUEST_SHA
            or review.get('rotation_geometry_sha256')!=GEOMETRY_SHA
            or review.get('rotation_mapping_scale')!=1.
            or review.get('rotation_physical_qualified') is not False):
        raise ValueError('Independent XY+yaw request, geometry and unqualified pilot scope required')
    return _verify_existing_pilot(review,stage,purpose,firmware_sha)
''')
    setup=runtime/m['hardware_package']/'setup.py'
    for name in ('source','supervisor','gate'):
        edit(setup,'mecanum_wand_4ws8_hardware.nodes:'+name+'_main',
            'mecanum_wand_4ws8_hardware.rotation_nodes:'+name+'_main')
    ui=c/'operator_console.py'
    edit(ui,'Console = LargeTypeConsole','''from .rotation_ui import RotationConsoleMixin
class RotationConsole(RotationConsoleMixin, LargeTypeConsole):
    pass
Console = RotationConsole''')
    app=out/'oneclick_app.py';shutil.copyfile(OLD/'oneclick_app.py',app)
    for name in ('auto_open.py','large_type.py'):shutil.copyfile(OLD/name,out/name)
    edit(app,'READY=Path('+repr(str(OLD))+')','READY=Path('+repr(str(out))+')')
    edit(app,'m=runtime_imports()\n    from mecanum_wand_4ws8_hardware.authorization',
        "m=runtime_imports()\n    if m.get('rotation_migration_passed') is not True:raise ValueError('旋转迁移离线验证尚未完成；不得启动')\n    from mecanum_wand_4ws8_hardware.authorization")
    edit(app,'return dict(kind=KIND,identity=',
        f'return dict(rotation_request={str(HERE/"USER_REQUEST.json")!r}, rotation_request_sha256={sha(HERE/"USER_REQUEST.json")!r}, '
        f'rotation_geometry_sha256={sha(GEOMETRY)!r}, rotation_mapping_scale=1., rotation_physical_qualified=False, kind=KIND,identity=')
    edit(app,'class App(LargeTypeLauncherMixin, AutomaticOpenMixin, ManualApp):\n    pass',
        '''class App(LargeTypeLauncherMixin, AutomaticOpenMixin, ManualApp):
    def __init__(self,*args,**kw):
        super().__init__(*args,**kw)
        from mecanum_wand_4ws8_control.rotation_ui import label_launcher
        label_launcher(self)''')
    edit(app,"ap.add_argument('--preview',action='store_true')", "ap.add_argument('--preview',action='store_true');ap.add_argument('--launch',action='store_true')")
    edit(app,'instance=None if a.preview else single_instance()',
        "if not a.preview and not a.launch:raise ValueError('独立旋转候选尚未实机验收；需明确 --launch 打开未使能窗口，现场确认后才能准备')\n    runtime_imports()\n    instance=None if a.preview else single_instance()")
    # Conservative default is a FILE CHECK, never opening a hardware window.
    sources={**hashes(HERE),str(ROOT/'tools/start_car_xy_yaw_demo.sh'):sha(ROOT/'tools/start_car_xy_yaw_demo.sh'),
        str(app):sha(app),str(out/'large_type.py'):sha(out/'large_type.py'),str(out/'auto_open.py'):sha(out/'auto_open.py')}
    manifest=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),source_hashes=sources,
        protected_input_hashes=protected,host_change='Independent same-source-bound XY+yaw, existing .10/.40 yaw limits',
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        original_entry_changed=False,rotation_physical_qualified=False)
    manifest['rotation_migration_passed']=False
    write(out/'build_manifest.json',manifest)
    oldfiles={str(p.relative_to(Path(m['runtime_src']))):sha(p) for p in Path(m['runtime_src']).rglob('*')
        if p.is_file() and '__pycache__' not in p.parts and not any(k.endswith('.egg-info') for k in p.parts)}
    changed=[str(p.relative_to(runtime)) for p in runtime.rglob('*') if p.is_file()
        and oldfiles.get(str(p.relative_to(runtime)))!=sha(p)]
    write(out/'PRIVATE_CHANGE_INVENTORY.json',sorted(changed));check(out/'build_manifest.json')
    return manifest


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    a=ap.parse_args();m=generate(a.output.resolve())
    print(json.dumps(dict(generated=True,output=str(a.output.resolve()),protected_files=len(m['protected_input_hashes']),
        hardware_started=False,flashed=False,tests_pending=True)))


if __name__=='__main__':main()
