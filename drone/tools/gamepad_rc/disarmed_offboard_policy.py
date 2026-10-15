"""Real MAVLink encoding for a future DISARMED/LOCKED Offboard ground check.

No serial adapter or automatic start here. Never permits arm, output-unlock,
takeoff, altitude increments, moving targets, parameter writes or reboot.
"""
from dataclasses import dataclass
import math
from pymavlink.dialects.v20 import common as m
from receive_only_permit import UID

POSITION_YAW_MASK = 2552  # Ignore velocity/acceleration/yaw-rate; retain position+yaw.


def restorable_mode(main, sub):
    return (main in (1,2,3) and sub==0) or (main==4 and sub in (3,6))


def restore_mode_allowed(message, main, sub, *, safe=False, locked=False):
    return bool(safe and locked and restorable_mode(main,sub)
        and message.get_type()=='COMMAND_LONG' and message.command==176
        and (message.target_system,message.target_component,message.confirmation)==(1,1,0)
        and message.param1==1 and message.param2==main and message.param3==sub
        and all(getattr(message,f'param{i}')==0 for i in range(4,8)))


@dataclass(frozen=True)
class GroundObservation:
    t: float
    uid2: str
    firmware: str
    position: tuple
    yaw: float
    reset_counters: tuple  # xy, z, heading counters from the SAME live FC session.
    primary_estimator: int
    armed: bool = False
    lockdown: bool = True
    pose_age_s: float = 0.
    status_age_s: float = 0.
    estimator_ready: bool = False
    gamepad_available: bool = False
    home_latched: bool = False


class DisarmedHoldGate:
    def __init__(self, observation, *, operator_authorized=False):
        self.origin = observation
        self.operator_authorized = operator_authorized
        self.last_time = observation.t
        self.first_sent = self.last_sent = None
        self.sent_count = 0
        self.mode_attempted = False
        self.blocked = False
        if not self.valid(observation):
            raise ValueError('Fresh disarmed locked live FC state and explicit ground-check authorization required')

    def valid(self, o):
        if self.blocked:
            return False
        values = (o.t, *o.position, o.yaw, o.pose_age_s, o.status_age_s)
        valid = (self.operator_authorized and all(math.isfinite(v) for v in values)
            and len(o.position) == 3 and len(o.reset_counters) == 3
            and o.uid2 == UID and o.firmware == '1.14.3' and not o.armed and o.lockdown
            and o.estimator_ready and o.gamepad_available and not o.home_latched
            and 0 <= o.pose_age_s <= .15 and 0 <= o.status_age_s <= .5
            and o.t >= self.last_time and o.reset_counters == self.origin.reset_counters
            and o.primary_estimator == self.origin.primary_estimator
            and math.dist(o.position, self.origin.position) <= .03)
        self.last_time = o.t
        self.blocked = not valid
        return valid

    def hold_message(self, encoder, boot_ms, observation):
        if type(boot_ms) is not int or not 0 <= boot_ms < 2**32 or not self.valid(observation):
            raise ValueError('Ground hold gate blocked')
        return encoder.set_position_target_local_ned_encode(boot_ms, 1, 1, m.MAV_FRAME_LOCAL_NED,
            POSITION_YAW_MASK, *self.origin.position, 0, 0, 0, 0, 0, 0, self.origin.yaw, 0)

    def record_actual_hold_send(self, now):
        if (not math.isfinite(now) or now < self.last_time or self.blocked
                or (self.last_sent is not None and not 0 < now-self.last_sent <= .2)):
            self.blocked = True
            raise ValueError('Actual hold-send continuity lost; no automatic restart')
        if self.first_sent is None:
            self.first_sent = now
        self.last_sent = now
        self.sent_count += 1

    def mode_message(self, encoder, observation):
        if (not self.valid(observation) or self.mode_attempted or self.sent_count < 20
                or self.first_sent is None or observation.t-self.first_sent < 2
                or not 0 <= observation.t-self.last_sent <= .2):
            raise ValueError('Two-second actual hold stream required before ONE Offboard request')
        return encoder.command_long_encode(1, 1, m.MAV_CMD_DO_SET_MODE, 0, 1, 6, 0, 0, 0, 0, 0)

    def record_actual_mode_attempt(self):
        if self.mode_attempted:
            raise ValueError('Offboard request already attempted; no automatic retries')
        self.mode_attempted = True

    def permits(self, message, observation):
        if not self.valid(observation):
            return False
        if message.get_type() == 'SET_POSITION_TARGET_LOCAL_NED':
            return bool((message.target_system, message.target_component, message.coordinate_frame, message.type_mask)
                == (1, 1, m.MAV_FRAME_LOCAL_NED, POSITION_YAW_MASK)
                and all(math.isfinite(v) for v in (message.x,message.y,message.z,message.yaw))
                and all(abs(a-b) <= 1e-6 for a,b in zip((message.x,message.y,message.z), self.origin.position))
                and abs(math.atan2(math.sin(message.yaw-self.origin.yaw), math.cos(message.yaw-self.origin.yaw))) <= 1e-6)
        if message.get_type() == 'COMMAND_LONG':
            try:
                expected = self.mode_message(m.MAVLink(None), observation)
            except ValueError:
                return False
            return message.to_dict() == expected.to_dict()
        return False
