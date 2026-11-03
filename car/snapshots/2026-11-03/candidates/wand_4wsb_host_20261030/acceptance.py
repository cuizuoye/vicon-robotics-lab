"""Offline plans/blank evidence only. Never grants approval or sends a command."""
import argparse
import json
import math
from pathlib import Path
from bootstrap import ROOT,OUT,config
from prepare import save,HARDWARE
from mecanum_wand_4wsb_hardware.stages import get_stage


def plan(stage):
    s=get_stage(stage);c=config(stage)
    # Conservative planning scenario: host continues renewing during a 300ms
    # feedback outage, then its disable is lost and the last MCU lease takes
    # 600ms to expire. Not a measured reaction latency or braking capability.
    fallback=.30+.60
    footprint=math.hypot(.095,.105)
    rated_torque=.4*9.80665*.01
    mass=1.4
    return dict(identity='4WSB',stage=stage,physical_qualified=False,
        bounds=dict(speed_mps=s.speed,accel_mps2=s.accel,yaw_rate_radps=s.yaw_rate,
            yaw_accel_radps2=s.yaw_accel,pwm_permille=s.duty,target_rpm=s.target_rpm),
        mapping='1:1 XY and relative yaw; Z ignored; no fixed fence or radius',
        session_s=600.,independent_mcu_ms=630000,suspended_single_test_s=5.,
        qualification_order=['fresh authorized full backup/flash/readback, left halted',
            'new image disabled handshake and exact v7/208-byte/stage/clock/fault checks',
            'stage0 secured wheels-up first; each next tier needs actual previous-tier evidence',
            'ground stage0 admission separately reviewed; higher ground tiers do not inherit wheels-up qualification'],
        stop_planning=dict(scenario='300ms feedback detection + lost disable + remaining600ms MCU lease',
            conservative_reaction_s=fallback,reaction_travel_m=s.speed*fallback,
            footprint_circumradius_m=footprint,
            required_clearance_formula='v*T + v*v/(2*a_brake_measured) + footprint_radius + margin',
            measured_braking_deceleration_mps2=None,physical_stop_distance_m=None,
            fifty_cm_automatically_qualified=False),
        tests=['forward/backward, lateral/diagonal and zero crossing',
            'left/right yaw and simultaneous XY+yaw without mixed-tier parameters',
            'Space/Esc/operator loss, Vicon loss, feedback loss, fault latch and no auto resume',
            'measured ground stop at actual tier speed, including communication-loss fallback',
            'loaded current, battery sag, driver/motor/connectors temperature and wiring stability'],
        electrical=dict(driver_model='D157B / AT8236, user supplied label',
            label_continuous_default_A_per_channel=1.32,label_peak_A_per_channel=6.,
            label_max_continuous_4A_not_automatic_qualification=True,
            motor_stall_current_A=None,actual_peak_current_A=None,temperature_C=None,
            ordinary_multimeter_does_not_prove_transient_current_or_temperature=True,
            do_not_parallel_connect_meter_current_range_to_battery=True,
            no_blocked_rotor_or_hand_braking_test=True),
        nominal_torque_sanity_check=dict(source='user reported .4kgf.cm per motor,1.4kg mass; source wheel radius.03m',
            fresh_mass_measured=False,rated_torque_Nm_per_motor=rated_torque,
            ideal_straight_accel_at_all_four_rated_torques_mps2=4*rated_torque/(.03*mass),
            required_ideal_torque_Nm_per_motor_at_selected_accel=mass*s.accel*.03/4,
            rated_torque_is_not_stall_or_peak_torque=True,
            mecanum_losses_traction_and_torque_speed_curve_not_included=True,
            physical_acceleration_qualified=False),
        special_stage7='XY4.00 experiment retains yaw .10/.40; pure yaw may be below inherited100cps start threshold' if stage==7 else None,
        hardware_opened=False,firmware_flashed=False,human_authorized=False)


def blank_observation(stage,purpose):
    return dict(schema=1,identity='4WSB',stage=stage,purpose=purpose,
        image_sha256=json.loads((ROOT/'docs/wand_firmware_4wsb_2026-10-29/ready/DELIVERY_RESULT.json').read_text())['bin']['sha256'],
        target_uid=None,session_log=None,physical_trial=False,
        direction_correct=None,smooth=None,stopped_all_wheels=None,
        measured_speed_mps=None,measured_stop_distance_m=None,measured_stop_time_s=None,
        measured_supply_voltage_V=None,measured_current_A=None,motor_temperature_C=None,
        driver_temperature_C=None,ambient_temperature_C=None,
        observation_notes='',action_and_stop_passed=False,reviewed_by_human=False,
        warning='Blank plan, not permission; SIM or wheels-up evidence does not qualify loaded braking/current/temperature')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=int,choices=range(8),default=0)
    p.add_argument('--purpose',choices=('suspended','sign'),default='suspended')
    p.add_argument('--output',type=Path)
    a=p.parse_args();value=dict(plan=plan(a.stage),observation=blank_observation(a.stage,a.purpose))
    if a.output:
        if a.output.exists():raise ValueError('never overwrite evidence')
        save(a.output,value)
    print(json.dumps(value,ensure_ascii=False,indent=2))
