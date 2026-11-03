"""Virtual-clock helpers; all physical I/O forbidden by each test fixture."""
import importlib.util
import json
import os
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
GENERATED=Path(os.environ['STAGED_GENERATED'])
for name in ('mecanum_wand_4ws8_control','mecanum_wand_4ws8_hardware'):
    sys.path.insert(0,str(GENERATED/name))
from mecanum_wand_4ws8_control.pilot_core import PilotSupervisor,PilotFinalGate,POLICY
from mecanum_wand_4ws8_control.pilot_ui import PilotOperatorModel
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig

spec=importlib.util.spec_from_file_location('mecanum_wand_4ws8_control.signed_test_harness',
    ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests/offline_harness.py')
harness=importlib.util.module_from_spec(spec);spec.loader.exec_module(harness)
harness.Supervisor,harness.FinalGate=PilotSupervisor,PilotFinalGate

def config(stage=4):
    return StagedSafetyConfig(**json.loads((GENERATED/
        f'mecanum_wand_4ws8_hardware/config/stage{stage}.NOT_APPROVED.json').read_text())['safety'])

def model(state='STANDBY',used=False,blocked='',stopped=True):
    m=PilotOperatorModel();m.focused=True
    m.receive_status(dict(state=state,ready_to_start=True,stop_confirmed=stopped,gate_ready=state=='ARMED',
        pilot=dict(used=used,max_active_s=120.,remaining_s=4.2,prepare_blocked=blocked,
            spatial_policy=POLICY['spatial_policy'])),10.)
    m.receive_gate(dict(fault='',hard_fault=False),10.)
    return m
