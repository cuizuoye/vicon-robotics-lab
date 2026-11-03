"""Post-run observations, not a fabricated 600ms fail-safe stopping envelope."""
import json
import math


def analyze(rows,stop_at,confirmed,physical):
    clean=[];last=-math.inf
    for row in rows:
        if (row['source_ros']>last and all(math.isfinite(row[k]) for k in ('at','source_ros','x','y','yaw'))
                and -.005<=row['source_age_s']<=.2):
            clean.append(row);last=row['source_ros']
    peaks=[]
    anchor=0
    for i,b in enumerate(clean):
        # 50ms observation window reduces raw Vicon noise; never filters control.
        while anchor+1<i and b['source_ros']-clean[anchor+1]['source_ros']>=.05:anchor+=1
        prior=clean[anchor] if anchor<i and .05<=b['source_ros']-clean[anchor]['source_ros']<=.2 else None
        if prior:
            dt=b['source_ros']-prior['source_ros']
            peaks.append((math.hypot(b['x']-prior['x'],b['y']-prior['y'])/dt,
                abs(math.atan2(math.sin(b['yaw']-prior['yaw']),math.cos(b['yaw']-prior['yaw'])))/dt))
    before=[r for r in clean if r['at']<=stop_at]
    after=[r for r in clean if r['at']>stop_at]
    stop_rows=(([before[-1]] if before else [])+after)
    gap=max((b['source_ros']-a['source_ros'] for a,b in zip(stop_rows,stop_rows[1:])),default=None)
    path=sum(math.hypot(b['x']-a['x'],b['y']-a['y']) for a,b in zip(stop_rows,stop_rows[1:])) if len(stop_rows)>1 else None
    return dict(kind='OBSERVED_NORMAL_STOP_NOT_FULL_FAILSAFE_ENVELOPE',physical_trial=bool(physical),
        fresh_pose_samples=len(clean),source_timestamps_preserved=True,observation_window_s=.05,
        observed_peak_xy_speed_mps=max((v[0] for v in peaks),default=None),
        observed_peak_abs_yaw_rate_radps=max((v[1] for v in peaks),default=None),
        stop_request_steady=stop_at,stop_confirmed=bool(confirmed),
        observed_path_after_stop_request_m=path,stop_sample_max_gap_s=gap,
        stop_observation_contiguous=bool(confirmed and len(stop_rows)>=2 and gap is not None and gap<=.2),
        physical_stop_envelope_verified=False,includes_600ms_link_loss_test=False,
        battery_current_a=None,driver_temperature_c=None,motor_temperature_c=None,
        electrical_thermal_verified=False,qualified_long_demo=False,
        caveat='Normal-stop pose observation only. Encoder software plant is uncalibrated; no current/temperature sensor. Human review and separate fail-safe measurements remain necessary.')


def read_rows(path):
    with path.open() as f:return [json.loads(line) for line in f if line.strip()]
