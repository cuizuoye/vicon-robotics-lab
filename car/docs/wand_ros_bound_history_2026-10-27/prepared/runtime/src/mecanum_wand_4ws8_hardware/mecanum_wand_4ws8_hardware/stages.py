"""Reviewed sequence, not physical acceptance or authority to run."""
from dataclasses import dataclass

@dataclass(frozen=True)
class Stage:
    speed: float
    accel: float
    duty: int
    output_slew: float

    @property
    def target_rpm(self):return 100 if self.speed==.20 else 180 if self.speed==.35 else 250 if self.speed==.50 else 400

    @property
    def feedback_rpm(self):return 120 if self.speed==.20 else 210 if self.speed==.35 else 280 if self.speed==.50 else 440

STAGES=(Stage(.20,.08,350,1200),Stage(.35,.20,450,1600),
        Stage(.50,.40,600,2000),Stage(.70,.40,700,2000),
        Stage(.70,.80,800,2500),Stage(.70,1.20,800,3000),Stage(.70,2.,800,4000))

def get_stage(index):
    if type(index) is not int or not 0<=index<len(STAGES):raise ValueError('explicit stage 0..6 required')
    return STAGES[index]

SCOPE='BOUNDED_SPACE_TOGGLE_120S_4WS8_V4_STAGE_'
