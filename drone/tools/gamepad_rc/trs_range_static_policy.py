"""Explicit stationary-only profile retaining the user's range height reference.

This is not a flight profile. It keeps all legacy message gates and requires
the same barometer fallback prerequisites; it never writes the height reference.
"""
from trs_static_fusion_policy import (COMMANDS, DELAY_MS, READ_PARAMS, allowed,
                                    parameter_message, receive_time_mode, stationary)


def scope_ok(parameters, baseline):
    return bool(
        parameters.get('EKF2_EV_CTRL') in (0, 11)
        and parameters.get('EKF2_HGT_REF') == baseline.get('EKF2_HGT_REF') == 2
        and parameters.get('EKF2_BARO_CTRL') == 1
        and parameters.get('EKF2_AID_MASK') == 0
        and parameters.get('EKF2_EV_QMIN') == 0
        and all(parameters.get(n) == 0 for n in
                ('EKF2_EV_POS_X', 'EKF2_EV_POS_Y', 'EKF2_EV_POS_Z'))
        and parameters.get('EKF2_EV_DELAY') in (0, DELAY_MS)
        and all(n in baseline and parameters.get(n) == baseline[n]
                for n in READ_PARAMS if n not in ('EKF2_EV_CTRL', 'EKF2_EV_DELAY'))
    )
