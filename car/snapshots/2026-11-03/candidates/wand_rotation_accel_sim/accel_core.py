"""Independent enumerated SOFTWARE acceleration study. No hardware profile."""
from dataclasses import dataclass, replace
from pathlib import Path
import sys

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
LIVE=ROOT/'candidates/wand_rotation_live_sim'
sys.path.insert(0,str(LIVE))
from engine import Engine, load_definition, preview_model, select_source, Clicks, ZERO
from rotation_core import SimRotationLimits

XY_ACCELS=(.8,1.2,2.)
YAW_ACCELS=(6.,8.,12.)
BASE=SimRotationLimits(2.,6.)


@dataclass(frozen=True)
class AccelerationLimits:
    max_accel_mps2: float
    max_yaw_accel_radps2: float
    def __post_init__(self):
        if (type(self.max_accel_mps2) not in (int,float) or self.max_accel_mps2 not in XY_ACCELS
                or type(self.max_yaw_accel_radps2) not in (int,float)
                or self.max_yaw_accel_radps2 not in YAW_ACCELS):
            raise ValueError('Only enumerated simulation acceleration pairs allowed')
    def __getattr__(self,key):
        if key.startswith('_'): raise AttributeError(key)
        return getattr(BASE,key)
    def follower_config(self):
        return replace(BASE.follower_config(),max_accel_mps2=self.max_accel_mps2,
            max_yaw_accel_radps2=self.max_yaw_accel_radps2)
    @property
    def name(self):return f'SIM_XY_A{self.max_accel_mps2:.2f}_YAW_A{self.max_yaw_accel_radps2:.2f}'


class AccelerationEngine(Engine):
    def __init__(self,definition,now,ros,limits):
        if type(limits) is not AccelerationLimits: raise ValueError('SIM acceleration limits required')
        self.acceleration_limits=limits
        super().__init__(definition,now,ros)
    def _new_cores(self):
        # Base Engine creates geometry before this hook. Geometry and speed
        # are identical; only the explicit software acceleration pair differs.
        self.c=self.acceleration_limits
        super()._new_cores()
