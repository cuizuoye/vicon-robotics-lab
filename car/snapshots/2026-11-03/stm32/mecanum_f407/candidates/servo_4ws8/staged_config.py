"""Fixed-tier dataclass; original low-speed validators/gate remain intact."""
from dataclasses import dataclass,fields
from .servo_config import ServoSafetyConfig
from .stages import STAGES,get_stage

@dataclass(frozen=True)
class StagedSafetyConfig(ServoSafetyConfig):
    def __post_init__(self):
        choices=[i for i,s in enumerate(STAGES) if (s.speed,s.accel)==(self.max_speed_mps,self.max_accel_mps2)]
        if len(choices)!=1:raise ValueError('explicit reviewed stage speed/acceleration pair required')
        base={f.name:getattr(self,f.name) for f in fields(ServoSafetyConfig)}
        base.update(max_speed_mps=.20,max_accel_mps2=.08)
        ServoSafetyConfig(**base) # Validate every other guard using the unchanged accepted class.

    @property
    def stage(self):
        return next(i for i,s in enumerate(STAGES) if (s.speed,s.accel)==(self.max_speed_mps,self.max_accel_mps2))

    @property
    def limits(self):return get_stage(self.stage)
