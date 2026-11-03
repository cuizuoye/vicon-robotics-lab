"""Separate SIM ONLY extension; immutable imports, no device initialization."""
import importlib.util
from pathlib import Path
import sys

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
PREVIOUS = ROOT/'candidates/wand_latency_fast_follow_20261029'
sys.path.insert(0, str(PREVIOUS))
from common import BASE, SRC, config, protected_hashes, save, digest, ZERO, Pose, Velocity
from catchup import FastCatchupSession

sys.path.insert(0, str(SRC/'mecanum_wand_4ws8_hardware'))


def history_imports():
    # Reuse the separately reviewed FIFO/history repair ONLY in this new sim.
    # No original package file or original demo import path is modified.
    path = ROOT/'candidates/wand_ros_bound_history'
    name = 'mecanum_wand_4ws8_control.bound_history'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, path/'bound_history.py')
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    sys.path.insert(0, str(path))
    from bound_rotation import HistoryQosMixin, BoundGateMixin, SourceFaultGate
    return HistoryQosMixin, BoundGateMixin, SourceFaultGate


def input_hashes():
    paths = list(PREVIOUS.glob('*.py'))
    paths += list((SRC/'mecanum_wand_4ws8_control').rglob('*.py'))
    paths += list((SRC/'mecanum_wand_4ws8_hardware').rglob('*.py'))
    paths += [SRC/'mecanum_wand_4ws8_control/mecanum_wand_4ws8_control/rotation_geometry.json']
    paths += [ROOT/'candidates/wand_ros_bound_history'/name for name in ('bound_history.py','bound_rotation.py')]
    return {str(p): digest(p) for p in paths}
