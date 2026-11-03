"""Pinned, pure-Python baseline imports. Does not initialize ROS or devices."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
BASE = ROOT/'docs/wand_feedback_300ms_startup_2026-10-27/prepared'
SRC = BASE/'runtime/src'
PROFILE = SRC/'mecanum_wand_4ws8_hardware/config/stage4.NOT_APPROVED.json'
PROFILE_SHA = '47c82aff3a9050b0345f96679aa3fef063286440b7b6f08c3538206d7c60c1f4'
sys.path[:0] = [str(SRC/'mecanum_wand_4ws8_control'), str(ROOT/'src/mecanum_vicon_control')]

from mecanum_wand_4ws8_control.core import Pose, Velocity
from mecanum_wand_4ws8_control.tracking_config import TrackingSafetyConfig
from mecanum_wand_4ws8_control.rotation_servo import RotationSession, wheel_rpm
from mecanum_wand_4ws8_control.rotation_core import RotationSupervisor, RotationFinalGate
from mecanum_wand_4ws8_control.safety_core import ZERO, Lease
from mecanum_wand_4ws8_control.offline_suite import Plant


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1048576),b''): h.update(block)
    return h.hexdigest()


def config():
    if digest(PROFILE)!=PROFILE_SHA: raise ValueError('baseline profile changed')
    return TrackingSafetyConfig(**json.loads(PROFILE.read_text())['safety'])


def protected_hashes():
    paths=list(SRC.rglob('*.py'))+[PROFILE,BASE/'oneclick_app.py']
    paths+=list((ROOT/'tools').glob('start_car*.sh'))
    paths+=list((ROOT/'stm32/mecanum_f407/Src').glob('*'))
    paths+=list((ROOT/'stm32/mecanum_f407/Inc').glob('*'))
    paths+=list((ROOT/'src/mecanum_vicon_control').rglob('*.py'))
    paths+=[Path('/home/PRIMARY_USER/vicon_receiver_ws/src/ros2-vicon-receiver/vicon_receiver/src/communicator.cpp')]
    return {str(p):digest(p) for p in sorted(set(paths)) if p.is_file()}


def fresh_output(path):
    p=Path(path).resolve();p.mkdir(parents=True,exist_ok=False);return p


def save(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
