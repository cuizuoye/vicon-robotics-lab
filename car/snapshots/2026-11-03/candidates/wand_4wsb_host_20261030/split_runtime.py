"""Software-only contract for deployment-shaped private multi-process tests."""
from types import SimpleNamespace as NS
import os
import re
import math
from bootstrap import config


def stop_acknowledged(status, requested):
    """Never count a pre-stop/ACTIVE readiness flag as a new stop ACK."""
    issued=status.get('issued_steady')
    return bool(type(issued) in (int,float) and math.isfinite(issued)
        and issued>=requested and status.get('state') in ('PAUSED','FAULT')
        and status.get('stop_confirmed') is True)


def hand_gesture(t,stage,recipe='normal'):
    """Continuous synthetic hand gestures, not position teleports."""
    if recipe=='demo':
        if stage not in (6,7):raise ValueError('fast demo recipe requires software stage6/7')
        x=.20*math.sin(1.4*t)+.05*math.sin(4.2*t)
        y=.17*math.sin(1.1*t)+.04*math.sin(3.3*t)
        turn=(.65*math.sin(1.1*t)+.10*math.sin(3.3*t)) if stage==6 else .03*math.sin(.7*t)
        return x,y,turn
    if recipe!='normal':raise ValueError('unknown gesture recipe')
    amp=.035 if stage==0 else .23
    return amp*math.sin(t*(.8 if stage==0 else 1.4)),amp*.7*math.sin(t*1.1),None


def runtime(namespace,stage):
    if (os.environ.get('ROS_DOMAIN_ID')!='191' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST'
            or not re.fullmatch('/host4wsb_[a-f0-9]{10}',namespace)):
        raise ValueError('private domain191/software namespace required before ROS init')
    def guard(node):
        if node.context.get_domain_id()!=191 or node.get_namespace()!=namespace:
            raise ValueError('software-only runtime escaped')
        if node.get_parameter('use_sim_time').value:raise ValueError('real deadline clocks required')
    return NS(namespace=namespace,world='map',config=config(stage),mode='hardware',software_only=True,
        guard=guard,yaw_offset=0.,wand_reference_margin_m=0.,
        expected=dict(car_pose='wand_sign_source',wand_pose='wand_sign_source',rotation_packet='wand_sign_source',
            plant_feedback='safe_native_twin',operator_lease='wand_real_operator',candidate='safe_wand_supervisor',
            cmd_vel='safe_wand_gate',actuator_command='safe_wand_gate'))
