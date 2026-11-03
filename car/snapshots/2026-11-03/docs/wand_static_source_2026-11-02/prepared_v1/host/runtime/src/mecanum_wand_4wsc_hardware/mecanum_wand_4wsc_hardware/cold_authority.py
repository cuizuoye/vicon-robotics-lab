"""Drop authenticated, expired, never-armed disabled discovery traffic only.

Called AFTER complete signature/schema/scope/value validation, BEFORE owner or
sequence mutation. This grants no baseline, epoch, lease, timestamp or output.
"""
from mecanum_wand_4wsc_control.safety_core import ZERO


def virgin_transport(policy):
    p=policy.envelope
    return bool(not policy.epoch and not policy.blocked and not policy.fault
        and not policy.hard_fault and (p is None or
        (p['state']=='STANDBY' and not p['permit'] and not p['epoch'] and not p['estop'])))


def discardable_zero(verifier,p,command,steady,ros):
    return bool(p['state']=='STANDBY' and not p['permit'] and not p['epoch']
        and not p['estop'] and command==ZERO and verifier.owner in (None,p['source'])
        and steady-p['issued_steady']>.10 and ros-p['issued_ros']>.10)
