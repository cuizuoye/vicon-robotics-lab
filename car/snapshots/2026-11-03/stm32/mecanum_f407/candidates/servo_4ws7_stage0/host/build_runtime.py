#!/usr/bin/env python3
"""Mechanical private runtime generation; no nodes, socket, flash or old edits."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[4]
BASE=ROOT/'docs/wand_one_to_one_3x3_2026-10-23/verified_runtime/src'
CONTROL='mecanum_wand_4ws7_control';HARDWARE='mecanum_wand_4ws7_hardware'

def replace(path,old,new,count=1):
    text=path.read_text()
    if text.count(old)!=count:raise ValueError('source fragment drift: '+str(path)+' '+old[:80])
    path.write_text(text.replace(old,new))

def generate(output):
    pinned=json.loads((HERE.parent/'upstream.json').read_text())
    if any(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=v for p,v in pinned.items()):
        raise ValueError('protected inputs drifted')
    output.mkdir(parents=True,exist_ok=False)
    pairs=(('mecanum_wand_one_to_one_control',CONTROL),('mecanum_wand_one_to_one_hardware',HARDWARE),
        ('WAND_ONE_TO_ONE_','WAND_4WS7_'),('wand_one_to_one_acceptance','wand_4ws7_stage0'),
        ("97, '/wand_4ws7_stage0'","118, '/wand_4ws7_stage0'"),('domain97','domain118'),('Domain97','Domain118'),
        ('ONE_TO_ONE_3X3_020_CLEAR_AREA_AND_PHYSICAL_POWER_CUT_READY','4WS7_STAGE0_SIGN_CLEAR_POWER_CUT_READY'))
    for old,new in pairs[:2]:shutil.copytree(BASE/old,output/new,ignore=shutil.ignore_patterns('__pycache__','*.egg-info'))
    for p in output.rglob('*'):
        if p.is_file():
            s=p.read_text()
            for a,b in pairs:s=s.replace(a,b)
            p.write_text(s)
    for p in sorted(output.rglob('*'),key=lambda p:len(p.parts),reverse=True):
        name=p.name
        for a,b in pairs:name=name.replace(a,b)
        if name!=p.name:p.rename(p.with_name(name))
    control=output/CONTROL/CONTROL;hardware=output/HARDWARE/HARDWARE
    # Reuse the accepted UI, fixed arena, independent supervisor/gate and XY servo.
    for name in ('continuous_servo.py','servo_config.py'):
        (control/name).write_text((ROOT/'candidates/wand_low_latency_servo'/name).read_text())
    replace(control/'core.py',
        '            tx, ty = limit_vector(\n                feedforward[0]+self.config.kp_position * bx,\n                feedforward[1]+self.config.kp_position * by,\n                self.config.max_speed_mps)',
        '            tx, ty = self.translation_target(bx, by, feedforward)')
    replace(control/'safety_core.py','from .core import Config, Pose, Velocity, WandSession',
        'from .core import Config, Pose, Velocity\nfrom .continuous_servo import ContinuousSession as WandSession')
    replace(control/'safety_core.py',"self.follower.observe(source, value['pose'], value['received'])",
        "self.follower.observe(source, value['pose'], value['received'], source_stamp=value['source_ros'])",2)
    for name in ('stage0_contract.py','stage0_session.py','authorization.py','suspended_core.py','suspended_console.py'):
        shutil.copyfile(HERE/name,hardware/name)
    (hardware/'continuous_status.py').write_text('from .stage0_contract import *\n')
    replace(hardware/'continuous_bridge.py','from .stage020_quantization import FirstStageSession as McLinkSession',
        'from .stage0_session import Stage0Session as McLinkSession')
    commission=hardware/'commissioning.py'
    replace(commission,'from '+CONTROL+'.arena_config import ArenaSafetyConfig as SafetyConfig',
        'from '+CONTROL+'.servo_config import ServoSafetyConfig as SafetyConfig\nfrom .authorization import verify_runtime_profile')
    replace(commission,"        if mode == 'hardware':\n            raw = Path(profile).read_bytes()",
        "        if mode == 'hardware':\n            verify_runtime_profile(json.loads(Path(profile).read_text()), 'sign')\n            raw = Path(profile).read_bytes()")
    text=commission.read_text().replace('4W20','4WS7').replace('156','184').replace('v1 output authority','v3 output authority')
    text=text.replace("hw.get('output_authority_version') != 1","hw.get('output_authority_version') != 3")
    commission.write_text(text)
    nodes=hardware/'nodes.py'
    replace(nodes,'        self.prepared_session = self.enable_requested_at = None\n        self.create_timer',
        '        self.prepared_session = self.enable_requested_at = None\n        self.stage0_selection_session = None\n        self.stage0_selection_at = None\n        self.create_timer')
    replace(nodes,"        if action == 'enable':\n            prepared = (prepared and self.policy.active(now)",
        "        if action == 'enable':\n            prepared = (prepared and self.last_motion_status.get('selected')\n                and self.last_motion_status.get('action_mode') == 1 and self.policy.active(now)")
    replace(nodes,'    def safety_tick(self):\n        now = time.monotonic()',
        '''    def safety_tick(self):
        now = time.monotonic()
        status = self.last_motion_status
        if (self.feedback_is_fresh(now) and self.last_mcu_fault_bits == 0 and status
                and not status['selected'] and not status['started_ms']
                and disabled_motion_status_ready(status) and self.session.connected
                and self.stage0_selection_session != self.session.session_id):
            self.stage0_selection_session = self.session.session_id
            self.stage0_selection_at = now
            if not self.send_immediate(self.session.select_mode(self.monotonic_ms()), 'SELECT_STAGE0_SIGN'):
                self.policy.revoke('STAGE0_SELECT_SEND_FAILED')
        if (self.stage0_selection_at is not None and now-self.stage0_selection_at > 1.
                and (not status or not status['selected'] or status['action_mode'] != 1)):
            self.policy.revoke('STAGE0_SELECT_NOT_CONFIRMED')
            self.stop_wire()
'''.rstrip())
    replace(nodes,"and status['started_ms'] == 0 and self.rearm_started is None",
        "and status['started_ms'] == 0 and status.get('selected')\n                and status.get('action_mode') == 1 and self.rearm_started is None")
    # Explicit first-stage profile; preparation never supplies current site/flash approval.
    config=output/HARDWARE/'config'
    original=config/'one_to_one.3x3_BUFFER050.SITE_CONFIRMATION_REQUIRED.json'
    profile=json.loads(original.read_text())
    for p in config.iterdir():
        if p.is_file():p.unlink()
    profile['hardware'].update(required_motion_tag='4WS7',diagnostic_payload_bytes=184,output_authority_version=3)
    profile['confirmations']['fixed_arena_checked']=False
    profile['stage0_runtime_approval']={}
    profile['evidence']['change_scope']='4WS7 v3 stage0 only; no flashing/hardware action during preparation'
    target=config/'4ws7_stage0.SITE_AND_FLASH_REQUIRED.json'
    target.write_text(json.dumps(profile,indent=2,ensure_ascii=False)+'\n')
    replace(control/'operator_console.py','0.20m/s首档（35%PWM）','4WS7首档0.20m/s（35%PWM）')
    launch=output/HARDWARE/'launch/wand_4ws7_stage0.launch.py'
    oldlaunch=output/HARDWARE/'launch/wand_4ws7_stage0.launch.py'
    # Renaming above already converted the known launch basename.
    if not oldlaunch.exists():raise ValueError('launch basename drift')
    # Full independent gate source must remain identical except package identifiers.
    old=(BASE/'mecanum_wand_one_to_one_control/mecanum_wand_one_to_one_control/safety_core.py').read_text()
    new=(control/'safety_core.py').read_text()
    extract=lambda text:ast.get_source_segment(text,next(n for n in ast.parse(text).body if isinstance(n,ast.ClassDef) and n.name=='FinalGate'))
    if extract(old)!=extract(new):raise ValueError('independent FinalGate changed')
    return dict(control_package=CONTROL,hardware_package=HARDWARE,domain=118,namespace='/wand_4ws7_stage0',
        draft_profile=str(target),independent_final_gate_unchanged=True,hardware_started=False,flashed=False)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    print(json.dumps(generate(p.parse_args().output.resolve()),indent=2))
