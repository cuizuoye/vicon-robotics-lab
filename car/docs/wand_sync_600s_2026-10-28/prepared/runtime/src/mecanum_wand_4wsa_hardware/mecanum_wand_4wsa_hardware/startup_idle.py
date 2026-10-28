"""A startup-only reconnect exemption; never clears faults or grants motion."""
import math


def may_retry_unarmed_startup(bridge, reason, now):
    if reason not in ('遥测超时','STM32命令序号停止推进'):
        return False
    if (getattr(bridge,'user_arm_seen',True) or bridge.local_stop_latched
            or bridge.safety_enabled or bridge.motion_gate_requested
            or bridge.pending_control_action not in (None,'disable')
            or bridge.last_mcu_fault_bits != 0):
        return False
    policy=bridge.policy
    p=policy.envelope or {}
    if (policy.epoch or policy.fault or policy.hard_fault
            or p.get('state','STANDBY')!='STANDBY' or p.get('enabled',False)
            or p.get('permit',False) or any(p.get('command',{}).get(k,0.)!=0 for k in ('vx','vy','wz'))):
        return False
    s=bridge.last_motion_status
    if (not s or s.get('tag')!='4WSA' or s.get('output_authority_version')!=4
            or s.get('permitted') is not False or s.get('watchdog_reset')!=0
            or s.get('fault_wheel')!=0
            or s.get('first_feedback_fault',{}).get('reason')!=0):
        return False
    for key,size in (('compares',8),('raw_duty',4),('target_cps',4),
                     ('body_milli',3),('effective_cps',4),('granted_direction',4)):
        values=s.get(key)
        if not isinstance(values,(list,tuple)) or len(values)!=size or any(v!=0 for v in values):
            return False
    rpm=bridge.last_wheel_rpm
    flags=bridge.last_mcu_state_flags
    if (not isinstance(rpm,(list,tuple)) or len(rpm)!=4 or any(v!=0 for v in rpm)
            or not isinstance(flags,int) or not flags & (1<<3) or flags & ((1<<1)|(1<<8))):
        return False
    # Old USED bit is deliberately not forbidden: actual disabled-zero v4
    # diagnostics and no new arm intent are the authority, not historical use.
    for stamp in (bridge.last_motion_status_monotonic,bridge.last_wheel_telemetry_monotonic):
        if stamp is None or not math.isfinite(stamp) or not 0<=now-stamp<=1.1:
            return False
    return True
