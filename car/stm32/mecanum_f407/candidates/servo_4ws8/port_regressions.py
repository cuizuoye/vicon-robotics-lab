"""Mechanical reuse of already accepted offline regressions; never alter originals."""
import ast
from pathlib import Path

def generate(out):
    old=Path('/home/PRIMARY_USER/vicon_mecanum_ws/stm32/mecanum_f407/candidates/servo_4ws7_stage0/host')
    # Real 4WS7 streams are NOT relabelled as 4WS8 measurements.
    text=(old/'test_wire_regression.py').read_text();tree=ast.parse(text)
    excluded={'test_original_response_rule_reproduces_failure_with_real_device_stream',
        'test_replay_all_real_device_frames_without_rewriting_headers_or_payloads',
        'test_stream_cannot_establish_session_or_freshness_before_valid_hello_ack'}
    lines=text.splitlines(True)
    for n in reversed(tree.body):
        if isinstance(n,ast.FunctionDef) and n.name in excluded:
            del lines[n.lineno-1:n.end_lineno]
    text=''.join(lines).replace('from test_stage0 import','from test_staged import').replace('mecanum_wand_4ws7','mecanum_wand_4ws8').replace('4WS7','4WS8')
    text=text.replace("SOURCE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws7_stage0'","SOURCE=GENERATED.parents[1]/'firmware'")
    (out/'test_wire.py').write_text(text)
    text=(old/'test_sign_adapter.py').read_text().replace('STAGE0_GENERATED','STAGED_GENERATED').replace('mecanum_wand_4ws7','mecanum_wand_4ws8').replace('4WS7','4WS8')
    text=text.replace('output_authority_version=3,','output_authority_version=4,stage=0,')
    text=text.replace("runtime=SimpleNamespace(mode='emulator')","runtime=SimpleNamespace(mode='hardware',config=SimpleNamespace(stage=0))")
    text=text.replace("'nonzero','fault'","'nonzero','fault','wrong_stage'")
    text=text.replace("    if defect=='fault':", "    if defect=='wrong_stage':f.last_motion_status['stage']=1\n    if defect=='fault':")
    (out/'test_sign_hooks.py').write_text(text)
    text=(old/'ui_smoke.py').read_text().replace('from test_stage0 import','from test_staged import').replace('mecanum_wand_4ws7','mecanum_wand_4ws8').replace('4WS7','4WS8')
    text=text.replace('SuspendedController(),io','SuspendedController(stage=6),io')
    (out/'ui_smoke.py').write_text(text)
    # Reuse the accepted pure supervisor/gate harness under the new package
    # namespace. It models neither electrical limits nor real stopping distance.
    import shutil
    shutil.copyfile(Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_low_latency_servo/offline_validation.py'),out/'offline_harness.py')
    src=Path('/home/PRIMARY_USER/vicon_mecanum_ws/candidates/wand_low_latency_servo/test_continuous_servo.py').read_text()
    selected={'test_full_supervisor_and_independent_gate_faults_stop_without_auto_resume',
        'test_release_reprepare_never_auto_moves_and_resets_velocity_estimator',
        'test_occlusion_recovery_requires_release_explicit_reprepare_and_new_hold',
        'test_one_to_one_nominal_goals_clamp_and_fifty_cm_buffer'}
    tree=ast.parse(src);parts=[]
    for n in tree.body:
        if isinstance(n,ast.FunctionDef) and n.name in selected:
            # Include pytest parametrize decorators.
            first=min([n.lineno]+[d.lineno for d in n.decorator_list])-1
            parts.append(''.join(src.splitlines(True)[first:n.end_lineno]))
    header='''from dataclasses import replace
import importlib.util
import sys
from pathlib import Path
import math
import pytest
from test_staged import profile,GENERATED
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig
from mecanum_wand_4ws8_control.core import Pose,Velocity,RelativeMapping
from mecanum_wand_4ws8_control.safety_core import ZERO
name='mecanum_wand_4ws8_control.offline_harness'
spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name('offline_harness.py'))
module=importlib.util.module_from_spec(spec);sys.modules[name]=module;spec.loader.exec_module(module)
Harness=module.Harness
@pytest.fixture(params=range(7))
def config(request):return StagedSafetyConfig(**profile(request.param)['safety'])
FAULTS=('wand_loss','car_loss','source_replay','feedback_loss','operator_loss','operator_release',
    'wand_jump','car_jump','lift','tilt','car_boundary','mode_conflict','source_conflict',
    'command_link_loss','supervisor_loss','delayed_command','command_replay','overspeed',
    'acceleration_jump','reference_jump','owner_changed','actuator_fault','estop',
    'session_timeout','clock_backwards','clock_forward','timer_gap')
'''
    text=header+'\n\n'+'\n\n'.join(parts)
    text=text.replace('Velocity(.201)','Velocity(config.max_speed_mps+.001)')
    (out/'test_supervisor_gate.py').write_text(text)
