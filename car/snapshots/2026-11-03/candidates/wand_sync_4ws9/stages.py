"""Immutable commissioning tiers; capability is NOT physical qualification."""
from dataclasses import dataclass

@dataclass(frozen=True)
class Stage:
    speed:float
    accel:float
    duty:int
    output_slew:float
    yaw_rate:float
    yaw_accel:float
    @property
    def target_rpm(self):return 100 if self.speed==.20 else 180 if self.speed==.35 else 250 if self.speed==.50 else 400
    @property
    def feedback_rpm(self):return 120 if self.speed==.20 else 210 if self.speed==.35 else 280 if self.speed==.50 else 440

STAGES=(Stage(.20,.08,350,1200,.20,.80),Stage(.35,.20,450,1600,.30,1.),
    Stage(.50,.40,600,2000,.60,2.),Stage(.70,.40,700,2000,1.,4.),
    Stage(.70,.80,800,2500,1.5,6.),Stage(.70,1.20,800,3000,2.,8.),
    Stage(.70,2.,800,4000,2.,12.))
SCOPE='HARDWARE_4WS9_V5_XY_YAW_STAGE_'
TIER_MAGIC=0x7900
BENCH_MAGIC=0x7980

def get_stage(stage):
    if type(stage) is not int or not 0<=stage<len(STAGES):raise ValueError('Explicit fixed 4WS9 stage0..6 required')
    return STAGES[stage]
