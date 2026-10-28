"""Load existing pure code only after checking its accepted manifest."""
import importlib.util
import json
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
BASELINE=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_autoprep/ready_v4'
sys.path.insert(0,str(ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'))
from launch_cli import check
MANIFEST=check(BASELINE/'build_manifest.json')
RUNTIME=Path(MANIFEST['runtime_src'])
sys.path[:0]=[str(RUNTIME/MANIFEST['control_package']),str(ROOT/'src/mecanum_vicon_control'),
    str(ROOT/'candidates/wand_rotation_preview')]
from mecanum_wand_4ws8_control.staged_config import StagedSafetyConfig

ACCEPTED=StagedSafetyConfig(**json.loads(Path(MANIFEST['draft_profiles'][4]).read_text())['safety'])
name='mecanum_wand_4ws8_control.rotation_sim_harness'
spec=importlib.util.spec_from_file_location(name,ROOT/'docs/wand_servo_4ws8_2026-10-23/ready/tests/offline_harness.py')
HARNESS=importlib.util.module_from_spec(spec);sys.modules[name]=HARNESS;spec.loader.exec_module(HARNESS)
