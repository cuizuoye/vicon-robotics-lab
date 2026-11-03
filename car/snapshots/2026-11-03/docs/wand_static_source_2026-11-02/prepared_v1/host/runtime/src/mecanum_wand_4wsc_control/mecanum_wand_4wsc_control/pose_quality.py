"""Diagnostic-only car pose quality. Never repairs, filters or accepts a pose.

The original independent lift/tilt guards remain the motion authority. A
near-180 degree step is a suspected tracking flip, not proof of a safe car.
"""
import copy
import math


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


class CarPoseQuality:
    def __init__(self, config):
        self.c = config
        self.latest = self.first_failure = None
        self.samples = self.failures = 0
        self.first_stamp = None
        self.max_source_gap_s = 0.
        self.previous = None

    def observe(self, pose, stamp, steady, ros, roll, pitch):
        if (not pose.valid() or not all(math.isfinite(v) for v in
                (stamp, steady, ros, roll, pitch))):
            return self.reject('POSE_INVALID', steady)
        if not -self.c.future_tolerance_s <= ros-stamp <= self.c.pose_timeout_s:
            return self.reject('POSE_TIMESTAMP', steady)
        if self.previous and stamp <= self.previous['stamp']:
            # Replays never refresh the readiness interval or orientation reference.
            return False
        height_error = pose.z-self.c.ground_z_m
        tilt = max(abs(roll), abs(pitch))
        q = quaternion(roll, pitch, pose.yaw)
        detail = dict(source_ros=stamp, received_steady=steady, height_m=pose.z,
            height_error_m=height_error, roll_deg=math.degrees(roll),
            pitch_deg=math.degrees(pitch), tilt_limit_deg=math.degrees(self.c.tilt_limit_rad),
            lift_tolerance_m=self.c.lift_tolerance_m, orientation_step_deg=None,
            source_step_s=None, classification='HEALTHY', guard_failed=False)
        if self.previous:
            previous = self.previous
            dt = stamp-previous['stamp']
            self.max_source_gap_s = max(self.max_source_gap_s, dt)
            # Absolute dot handles quaternion q/-q equivalence correctly.
            dot = min(1., abs(sum(a*b for a,b in zip(q, previous['q']))))
            detail.update(orientation_step_deg=math.degrees(2*math.acos(dot)), source_step_s=dt)
        if abs(height_error) > self.c.lift_tolerance_m:
            detail.update(classification='HEIGHT_OUT_OF_RANGE', guard_failed=True)
        elif tilt > self.c.tilt_limit_rad:
            detail.update(classification='TILT_OUT_OF_RANGE', guard_failed=True)
            if (self.previous and self.previous['healthy'] and
                    0 < detail['source_step_s'] <= .05 and
                    detail['orientation_step_deg'] >= 150.):
                detail['classification'] = 'ORIENTATION_FLIP_SUSPECT'
        self.samples += 1
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.latest = detail
        if detail['guard_failed']:
            self.failures += 1
            if self.first_failure is None:
                self.first_failure = copy.deepcopy(detail)
        self.previous = dict(stamp=stamp, q=q, healthy=not detail['guard_failed'])
        return not detail['guard_failed']

    def reject(self, reason, steady):
        detail = dict(classification=reason, guard_failed=True, received_steady=steady)
        self.latest = detail
        self.failures += 1
        if self.first_failure is None:
            self.first_failure = copy.deepcopy(detail)
        return False

    def snapshot(self):
        return copy.deepcopy(dict(samples=self.samples, failures=self.failures,
            latest=self.latest, first_failure=self.first_failure,
            source_span_s=(self.previous['stamp']-self.first_stamp
                if self.previous and self.first_stamp is not None else 0.),
            max_source_gap_s=self.max_source_gap_s))

    def preflight_issue(self):
        # A bad sample anywhere in this bounded check cannot be hidden by the
        # final good sample. This adds a preparation check, not a moving filter.
        if self.failures:
            return 'car01姿态检查出现异常：'+failure_text(self.first_failure)+'；请先处理Vicon刚体，不允许准备'
        if self.samples < 20 or self.snapshot()['source_span_s'] < 1.:
            return 'car01连续有效样本不足1秒：不能确认姿态稳定，不允许准备'
        if self.max_source_gap_s > self.c.pose_timeout_s:
            return 'car01连续数据存在超过200毫秒间断：不允许准备'
        return ''


def failure_text(detail):
    detail = detail or {}
    kind = detail.get('classification')
    if kind == 'ORIENTATION_FLIP_SUSPECT':
        step = detail.get('orientation_step_deg', 0.)
        dt = detail.get('source_step_s', 0.)*1000
        return f'车体姿态约{step:.1f}°突变（{dt:.1f}毫秒），疑似Vicon刚体翻转；仍按无效姿态停车'
    if kind == 'HEIGHT_OUT_OF_RANGE':
        return f"车体高度偏差{detail.get('height_error_m', 0.)*100:.1f}厘米超限"
    if kind == 'TILT_OUT_OF_RANGE':
        return f"车体倾斜超限：横滚{detail.get('roll_deg', 0.):.1f}°、俯仰{detail.get('pitch_deg', 0.):.1f}°"
    return '车体位姿无效或时间戳异常'


def stop_hint(status, gate=None):
    """Explain a consumed trial without changing recovery/enable semantics."""
    gate = gate or {}
    reason = status.get('first_stop_reason') or status.get('reason') or gate.get('fault') or ''
    if not reason:
        return ''
    if 'CAR_LIFT_OR_TILT' in reason:
        cause = failure_text(status.get('car_pose_quality', {}).get('first_failure'))
        advice = '先检查Vicon的car01姿态；不会自动恢复，也不会忽略异常帧'
    elif 'PILOT_TIME_LIMIT_600S' in reason or reason == 'SESSION_TIME_LIMIT':
        cause, advice = '600秒单次时限正常结束', '本窗口不能再次准备'
    elif reason in ('operator stop', 'operator pause', 'operator enable released', 'OPERATOR_ENABLE_RELEASED'):
        cause, advice = '操作员停止或松键', '本窗口不能再次准备'
    else:
        cause, advice = reason, '保持松键，检查原因；不会自动恢复'
    phase = '本轮已结束并停稳' if status.get('stop_confirmed') else '本轮已结束：等待停车确认'
    return phase+'；原因：'+cause+'；'+advice+'；本窗口不能再次准备'
