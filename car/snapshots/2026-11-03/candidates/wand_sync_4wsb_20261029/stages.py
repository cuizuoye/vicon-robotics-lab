"""4WSB firmware capabilities, NOT hardware qualifications or live authority."""
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Stage:
    speed: float
    accel: float
    duty: int
    output_slew: float
    yaw_rate: float
    yaw_accel: float
    wheel_slew_guard: float = 26000.

    @property
    def target_rpm(self):
        return 100 if self.speed == .20 else 180 if self.speed == .35 else 250 if self.speed == .50 else 400

    @property
    def feedback_rpm(self):
        return 120 if self.speed == .20 else 210 if self.speed == .35 else 280 if self.speed == .50 else 440


STAGES = (Stage(.20,.08,350,1200,.20,.80), Stage(.35,.20,450,1600,.30,1.),
          Stage(.50,.40,600,2000,.60,2.), Stage(.70,.40,700,2000,1.,4.),
          Stage(.70,.80,800,2500,1.5,6.), Stage(.70,1.20,800,3000,2.,8.),
          Stage(.70,2.,800,4000,2.,12.), Stage(.70,4.,800,4000,.10,.40,33000.))
IDENTITY = '4WSB'
AUTHORITY_VERSION = 7
DIAGNOSTIC_BYTES = 208
TIER_MAGIC = 0x7B00
BENCH_MAGIC = 0x7B80


def get_stage(index):
    if type(index) is not int or not 0 <= index < len(STAGES):
        raise ValueError('explicit fixed 4WSB stage0..7 required')
    return STAGES[index]


def table():
    return [dict(stage=i, **asdict(s), target_rpm=s.target_rpm,
                 feedback_rpm=s.feedback_rpm, physical_qualified=False)
            for i,s in enumerate(STAGES)]
