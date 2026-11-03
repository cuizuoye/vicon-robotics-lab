"""Explicit angular AND XY tiers; every non-motion guard stays pinned."""
from dataclasses import dataclass,fields,replace
from .servo_config import ServoSafetyConfig
from .stages import STAGES,get_stage

@dataclass(frozen=True)
class StagedSafetyConfig(ServoSafetyConfig):
    def __post_init__(self):
        choices=[i for i,s in enumerate(STAGES) if (s.speed,s.accel,s.yaw_rate,s.yaw_accel)==(
            self.max_speed_mps,self.max_accel_mps2,self.max_yaw_rate_radps,self.max_yaw_accel_radps2)]
        if len(choices)!=1:raise ValueError('Exact 4WS9 XY+yaw tier required; no mixed/simulation limits')
        # Use the original strict validator for arena, body, timers, source,
        # tilt, stationary, clock and source-age values. No blanket exemption.
        values={f.name:getattr(self,f.name) for f in fields(ServoSafetyConfig)}
        values.update(max_speed_mps=.20,max_accel_mps2=.08,max_yaw_rate_radps=.10,max_yaw_accel_radps2=.40)
        ServoSafetyConfig(**values)
    @property
    def stage(self):
        return next(i for i,s in enumerate(STAGES) if (s.speed,s.accel,s.yaw_rate,s.yaw_accel)==(
            self.max_speed_mps,self.max_accel_mps2,self.max_yaw_rate_radps,self.max_yaw_accel_radps2))
    @property
    def limits(self):return get_stage(self.stage)
    def follower_config(self):return replace(super().follower_config(),max_wheel_rpm=self.limits.target_rpm)
