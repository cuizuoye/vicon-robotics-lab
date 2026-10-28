"""OFFLINE high-speed Sign preparation. No ROS node, socket, serial or flasher.

The existing 4WGC runtime is NOT widened. This module prepares calculations,
mapping and a strict independent decoder, not authority to move real hardware.
"""
from dataclasses import asdict, dataclass
import math
import struct

from mecanum_wand_4wsa_control.core import Config, WandSession
from mecanum_vicon_control.bench_link_status import STATUS
from mecanum_vicon_control.transport_protocol import ProtocolError

SNAPSHOT=struct.Struct('<IIiiiiHHhH')
AUTHORITY=struct.Struct('<4f4bBBH')
SIZE=STATUS.size+SNAPSHOT.size+AUTHORITY.size

CPR = (3179/3, 1049., 3167/3, 1061.)
MOTOR_SIGNS = (1, -1, 1, -1)
TAG = 0x30325734
TARGET_RPM = 100.
FEEDBACK_RPM = 120.
WHEEL_DIAMETER_M = .060
MAX_WHEEL_MPS = math.pi*WHEEL_DIAMETER_M*TARGET_RPM/60
STAGES_MPS = (.20,)


@dataclass(frozen=True)
class HighSpeedPlan:
    first_speed_mps: float = .20
    max_accel_mps2: float = .08
    scale: float = 1.
    target_rpm: float = TARGET_RPM
    feedback_trip_rpm: float = FEEDBACK_RPM
    max_pwm_permille: int = 350
    launch_pwm_permille: int = 300
    wire_timeout_ms: int = 600
    feedback_timeout_s: float = .20
    firmware_budget_s: float = 150.
    session_budget_s: float = 120.

    def __post_init__(self):
        # Fixed reviewed candidate contract, not a way around existing limits.
        expected=(.20,.08,1.,100.,120.,350,300,600,.20,150.,120.)
        if tuple(asdict(self).values()) != expected:
            raise ValueError('candidate constants are pinned; new limits require a new review')

    def follower(self, site_config, *, stage_mps=.20):
        if stage_mps not in STAGES_MPS: raise ValueError('unknown OFFLINE stage')
        config=Config(scale=1.,max_speed_mps=stage_mps,max_accel_mps2=self.max_accel_mps2,
            max_yaw_rate_radps=.10,max_yaw_accel_radps2=.40,input_timeout_s=.20,
            max_goal_radius_m=3.,governed_targets=True,reference_lead_m=.20)
        return WandSession(config,site_config.project_goal)

    def speed_table(self):
        return dict(motor='MG310; operator supplied 500rpm no-load OUTPUT shaft, 20:1',
            cpr=list(CPR),target_rpm=100.,theoretical_straight_mps=MAX_WHEEL_MPS,
            theoretical_diagonal_mps=MAX_WHEEL_MPS/math.sqrt(2),
            first_speed_mps=.20,first_wheel_rpm=.20/(math.pi*.060)*60,
            stages_mps=list(STAGES_MPS),pwm_cap_is_not_rpm_percent=True,
            approximate_full_rpm_ramp_s=max(CPR)*100/60/800,
            full_speed_is_not_guaranteed_under_load=True)


def provisional_stopping_reserve(speed_mps, assumed_decel_mps2):
    """Planning estimate ONLY; zero-output coast deceleration is NOT measured.

    Use the independent 600ms fallback deadline, not only normal ROS latency.
    This cannot approve an arena or certify a physical stopping distance.
    """
    if not all(math.isfinite(v) for v in (speed_mps,assumed_decel_mps2)):
        raise ValueError('nonfinite braking inputs')
    if speed_mps<0 or speed_mps>MAX_WHEEL_MPS or assumed_decel_mps2<=0:
        raise ValueError('invalid braking inputs')
    return .20+speed_mps*.60+speed_mps**2/(2*assumed_decel_mps2)


def require_hardware_admission(*_args, **_kwargs):
    # No profile flag or caller-supplied assertion can turn offline work into
    # a validated transport. A later dedicated implementation is a new stage.
    raise RuntimeError('OFFLINE ONLY: this preview grants no hardware authority; use the independent '
                       'reviewed runtime after physical braking/current/thermal acceptance and explicit migration')


def decode_high_speed_status(payload):
    if len(payload)!=SIZE: raise ProtocolError('4W20 requires exactly 156 bytes')
    v=STATUS.unpack_from(payload)
    if v[0]!=TAG or not 1<=v[3]<=v[4]<=30 or v[5]>1000:
        raise ProtocolError('wrong 4W20 identity or sample deadline')
    cc,duty,target,filtered,body=(list(v[a:b]) for a,b in ((8,16),(16,20),(20,24),(24,28),(28,31)))
    mask,wheel,watchdog,permit=v[31:35]
    a=AUTHORITY.unpack_from(payload,STATUS.size+SNAPSHOT.size)
    effective,directions=list(a[:4]),list(a[4:8])
    reversing,version,reserved=a[8:]
    if (mask>15 or wheel>4 or watchdog or permit not in (0,1) or v[7]>500
            or reversing>15 or version!=1 or reserved):
        raise ProtocolError('invalid high-speed authority state')
    if any(abs(value)>limit for value,limit in zip(body,(200,200,100))):
        raise ProtocolError('high-speed body envelope')
    if math.hypot(body[0],body[1])>201: raise ProtocolError('4W20 vector envelope')
    for i,(requested,effective_cps,g) in enumerate(zip(target,effective,directions)):
        limit=min(1700.,CPR[i]*100/60)
        if (abs(requested)>limit+.501 or not math.isfinite(effective_cps)
                or abs(effective_cps)>limit+.002 or abs(filtered[i])>10000
                or g not in (-1,0,1)):
            raise ProtocolError('high-speed per-wheel RPM/feedback envelope')
    if not permit and any(cc+duty+target+filtered+body+effective+directions+[mask,reversing]):
        raise ProtocolError('high-speed output without permit')
    for i,(d,e,g,sign) in enumerate(zip(duty,effective,directions,MOTOR_SIGNS)):
        if abs(d)>350 or d%10: raise ProtocolError('high-speed PWM cap/resolution')
        expected=([100,100-d//10] if d>0 else [100-abs(d)//10,100] if d<0 else [0,0])
        if cc[2*i:2*i+2]!=expected: raise ProtocolError('high-speed actual CCR mapping')
        if (e and (1 if e>0 else -1)!=g) or (d and (not e or d*e*sign<=0)):
            raise ProtocolError('high-speed output differs from effective granted direction')
        if e*target[i]<0 and not reversing&(1<<i):
            raise ProtocolError('high-speed opposite output without reversal authority')
        if not target[i] and (e or d): raise ProtocolError('high-speed output with zero request')
        if mask&(1<<i) and (not d or abs(d)>300 or reversing&(1<<i)):
            raise ProtocolError('high-speed startup power increased or reversing')
    snapshot=SNAPSHOT.unpack_from(payload,STATUS.size)
    if (snapshot[6] not in range(7) or (snapshot[6]==0 and any(snapshot))
            or (snapshot[6] and not 1<=snapshot[7]<=4) or abs(snapshot[8])>350 or snapshot[8]%10):
        raise ProtocolError('high-speed first-fault snapshot')
    return dict(tag='4W20',ticks=v[1],updates=v[2],dt_min=v[3],dt_max=v[4],irq_us_max=v[5],
        started_ms=v[6],stationary_ms=v[7],watchdog_reset=watchdog,startup_mask=mask,
        permitted=permit,compares=cc,raw_duty=duty,target_cps=target,
        effective_cps=effective,granted_direction=directions,filtered_cps=filtered,body_milli=body,
        reversing_mask=reversing,output_authority_version=version,fault_wheel=wheel,
        first_feedback_fault=dict(zip(('ms','dt_ms','raw_delta','measured_milli_cps','requested_milli_cps',
            'effective_milli_cps','reason','wheel','duty_permille','opposite_counts'),snapshot)))


if __name__=='__main__':
    import json
    print(json.dumps(dict(verification='OFFLINE PREPARATION ONLY; NO HARDWARE ENTRY',
                         limits=asdict(HighSpeedPlan()),calculation=HighSpeedPlan().speed_table()),indent=2))
