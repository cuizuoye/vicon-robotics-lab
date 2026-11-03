"""Real Sign input, exclusively in-memory simulated actuation. No ROS or IO."""
from dataclasses import dataclass, asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
sys.path.insert(0, str(ROOT/'candidates/wand_rotation_sim'))
from bootstrap import HARNESS
from rotation_core import (SimRotationLimits, RotationSupervisor, RotationFinalGate,
    Pose, ZERO, reference_pose, wheel_rpm)
from mecanum_wand_4ws8_control.safety_core import Lease
from preview_core import IDS, REQUIRED, PreviewModel, TShape, rotation, vector

GEOMETRY = ROOT/'docs/wand_rotation_preview_2026-10-26/readonly_uppz8hn2/geometry_confirmed.json'


def load_definition():
    d = json.loads(GEOMETRY.read_text())
    if d['bar'] != ['Sign1', 'Sign5', 'Sign2'] or d['junction'] != 'Sign5':
        raise ValueError('human-accepted Sign5 definition required')
    return d


def preview_model(definition):
    m = PreviewModel()
    m.definition = TShape(tuple(definition['bar']), tuple(definition['stem']),
        definition['local_points'], tuple(definition['forward_local']),
        definition['width_m'], definition['line_residual_m'])
    return m


@dataclass(frozen=True)
class Raw:
    name: str
    position: tuple
    quaternion: tuple | None
    stamp: float
    received: float


@dataclass(frozen=True)
class Selected:
    """Immutable full same-frame packet, not separately selected latest topics."""
    samples: tuple

    @property
    def digest(self):
        return hashlib.sha256(json.dumps([asdict(s) for s in self.samples],
            sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()

    @property
    def stamp(self): return self.samples[0].stamp
    @property
    def received(self): return min(s.received for s in self.samples)

    def pose(self, definition, now, ros):
        if tuple(s.name for s in self.samples) != REQUIRED:
            raise ValueError('SOURCE_PACKET_NAMES')
        if any(s.stamp != self.stamp or not PreviewModel.fresh(s, now, ros) for s in self.samples):
            raise ValueError('SOURCE_PACKET_AGE_OR_MIXED_FRAME')
        sign = self.samples[0]
        r = rotation(sign.quaternion); origin = vector(sign.position)
        for sample in self.samples[1:]:
            local = r.T @ (vector(sample.position)-origin)
            if math.dist(local, definition['local_points'][sample.name]) > .008:
                raise ValueError('SOURCE_GEOMETRY_OR_ATTITUDE_FLIP')
        return reference_pose(sign.position, sign.quaternion, definition)


def select_source(model, definition, now, ros):
    if not model.source_ok or not model.valid or model.packet is None:
        return None
    result = Selected(tuple(Raw(k, model.packet[k].position, model.packet[k].quaternion,
        model.packet[k].stamp, model.packet[k].received) for k in REQUIRED))
    result.pose(definition, now, ros)
    return result


@dataclass(frozen=True)
class BoundFrame:
    frame: object
    source_digest: str | None
    source_stamp: float | None


class Engine:
    """Two independent pure safety cores, one explicit software plant.

    Packet binding is local-process integrity, NOT DDS authentication or a
    production multi-process command protocol. Neither output leaves memory.
    Physical car01 is deliberately not used as the software car's feedback.
    """
    def __init__(self, definition, now, ros):
        self.definition = definition
        self.c = SimRotationLimits(2., 6.)
        self.origin = Pose(*self.c.geometry().world(-.5, .05), self.c.ground_z_m,
            self.c.arena_heading_rad)
        self.plant = HARNESS.Plant(self.origin)
        self.now, self.ros = now, ros
        self.sequence = self.session = 0
        self.requested = self.prepare_pending = False
        self.output = ZERO; self.selected = None
        self.message = '未使能：先拿稳Sign，再点击准备（仅软件仿真）'
        self.events = []; self.modify_bound = lambda f: f
        self._new_cores()

    def _new_cores(self):
        self.sup, self.gate = RotationSupervisor(self.c), RotationFinalGate(self.c)
        self.session += 1

    def record(self, event):
        self.events.append(dict(steady=self.now, ros=self.ros, session=self.session, event=event))

    @property
    def armed(self):
        return bool(self.sup.state == 'ARMED' and self.gate.primed_epoch == self.sup.epoch
            and not self.gate.fault and not self.gate.hard_fault)

    @property
    def remaining(self):
        return max(0., 120.-(self.now-self.sup.active_started)) if self.sup.active_started is not None else 120.

    def can_prepare(self):
        return bool(not self.requested and not self.prepare_pending and self.selected is not None
            and self.sup.state in ('STANDBY', 'PAUSED', 'FAULT') and self.sup.stop_confirmed
            and not self.sup.inputs.health(self.now, self.ros)
            and not self.sup.hard_fault and not self.gate.hard_fault)

    def prepare(self):
        if not self.can_prepare():
            self.message = '暂不能准备：等待Sign有效、软件车停稳；硬故障须重新打开窗口'
            return False
        if self.sup.state != 'STANDBY':
            # Human acknowledgement creates NEW sim cores, not reuse of an old
            # arm epoch. Preserve measured plant pose AND residual velocity.
            self.record('EXPLICIT_NEW_SIM_SESSION_KEEP_PLANT')
            self._new_cores()
        self.prepare_pending = True
        self.message = '准备中：等待独立静止和稳定窗口；不会运动'
        self.record('PREPARE_REQUEST_NO_ORIGIN_CAPTURE')
        return True

    def start_ready(self):
        return bool(self.armed and self.selected is not None and not self.requested
            and not self.sup.inputs.health(self.now, self.ros))

    def engage(self):
        if not self.start_ready(): return False
        self.requested = True
        self.record('FRESH_SPACE_CLICK_REQUESTED')
        return True

    def stop(self, reason, fault=False):
        self.requested = self.prepare_pending = False
        self.output = ZERO
        if self.sup.state in ('ARMED', 'ACTIVE') or self.sup.used:
            self.sup.request_stop(self.now, reason, fault=fault)
        self.message = reason
        self.record('STOP: '+reason)

    def receive_bound(self, bound, selected):
        try:
            if selected is None: raise ValueError('SOURCE_MISSING')
            expected = selected.pose(self.definition, self.now, self.ros)
            if bound.source_digest != selected.digest or bound.source_stamp != selected.stamp:
                raise ValueError('SELECTED_SOURCE_IDENTITY_MISMATCH')
            stored = self.gate.inputs.poses.get('wand')
            if not stored or stored['pose'] != expected or stored['source_ros'] != min(selected.stamp, self.ros):
                raise ValueError('GATE_SELECTED_POSE_MISMATCH')
            return self.gate.receive(bound.frame, self.now, self.ros)
        except ValueError as exc:
            # Disabled frames still establish the command owner/stop baseline;
            # invalid input cannot arm or enable, and old data is never relabelled.
            if bound.frame.enabled or bound.frame.state == 'ARMED':
                self.gate.trip(str(exc)); return False
            return self.gate.receive(bound.frame, self.now, self.ros)

    def step(self, now, ros, selected):
        dt = now-self.now
        self.now, self.ros = now, ros
        self.sequence += 1
        self.selected = selected
        hand = None
        try:
            if selected is not None: hand = selected.pose(self.definition, now, ros)
        except ValueError:
            self.selected = selected = None
        for inputs in (self.sup.inputs, self.gate.inputs):
            inputs.graph_ok = selected is not None
            car = replace(self.plant.pose, yaw=math.atan2(math.sin(self.plant.pose.yaw), math.cos(self.plant.pose.yaw)))
            inputs.observe_pose('car', car, ros, now, ros)
            if hand is not None:
                inputs.observe_pose('wand', hand, selected.stamp, selected.received, ros)
            inputs.observe_lease(Lease('local-sim-click', self.sequence, now, ros, self.requested), now, ros)
            inputs.observe_feedback(self.sequence, now, ros, now, ros,
                output_zero=self.output == ZERO, enabled=self.output != ZERO,
                speed=math.hypot(self.plant.velocity.vx, self.plant.velocity.vy),
                yaw_rate=abs(self.plant.velocity.wz))
        old_state = self.sup.state
        frame = self.sup.tick(now, ros)
        bound = BoundFrame(frame, selected.digest if selected else None, selected.stamp if selected else None)
        self.receive_bound(self.modify_bound(bound), selected)
        self.output = self.gate.tick(now, ros)
        self.sup.observe_gate(steady=now, ros=ros, epoch=self.gate.primed_epoch,
            fault=self.gate.fault, hard_fault=self.gate.hard_fault)
        if old_state != self.sup.state:
            self.record('STATE: '+old_state+' -> '+self.sup.state+' '+self.sup.reason)
            if self.sup.state == 'ACTIVE':
                self.message = '仅软件车跟随中：松键不停，再次空格或Esc停止'
            elif self.sup.state not in ('STANDBY', 'ARMED'):
                self.message = '已撤销跟随：'+self.sup.reason+'；不会自动恢复'
        if self.sup.state not in ('ARMED', 'ACTIVE'):
            if self.requested:
                self.record('BACKEND_REVOKED: '+(self.sup.reason or self.gate.fault))
            self.requested = False
        if self.prepare_pending and selected is None:
            self.prepare_pending = False
            self.message = '准备已取消：Sign无效，恢复后须手动重新准备'
        if self.prepare_pending and self.sup.ready_since is not None:
            if now-self.sup.ready_since >= self.c.ready_window_s:
                ok, reason = self.sup.request_start(now, ros)
                if ok:
                    self.prepare_pending = False
                    self.message = '准备就绪：轻按松开空格开始；再次空格或Esc停止'
                    self.record('ARMED_NO_MOTION')
                else: self.message = reason
        # Approximate first-order software plant, substeps only for numerical
        # integration; no command extrapolation, no control-loop catch-up.
        # Suspend/resume is a hard timer fault, not hours of replayed control.
        # One second resolves the illustrative coast; the model then rests.
        remaining = min(1., max(0., dt))
        while remaining > 1e-9:
            part = min(.02, remaining); self.plant.step(self.output, part); remaining -= part
        return self.output

    def report(self):
        f = self.sup.follower
        return dict(kind='REAL_SIGN_TO_SOFTWARE_XY_YAW_CAR', session=self.session,
            state=self.sup.state, reason=self.sup.reason, message=self.message,
            gate_fault=self.gate.fault, hard_fault=self.sup.hard_fault or self.gate.hard_fault,
            prepared=self.armed, prepare_pending=self.prepare_pending, requested=self.requested,
            remaining_s=self.remaining, stop_confirmed=self.sup.stop_confirmed,
            car=asdict(self.plant.pose), plant_velocity=asdict(self.plant.velocity),
            command=asdict(self.output), wheel_rpm=wheel_rpm(self.output),
            goal=asdict(f.goal) if f.goal else None,
            mapped_goal=asdict(f.mapped_goal) if f.mapped_goal else None,
            relative_hand_yaw=f.mapping.turn if f.mapping else None,
            relative_car_yaw=self.plant.pose.yaw-f.mapping.car_origin.yaw if f.mapping else None,
            source_stamp=self.selected.stamp if self.selected else None,
            source_digest=self.selected.digest if self.selected else None,
            actuator_outputs=False, physical_car_started=False)


class Clicks:
    """Fresh complete click starts, second press stops. Polling NEVER starts."""
    def __init__(self, engine):
        self.engine = engine
        self.focused = False; self.down = False; self.blocked = True
        self.pending = None

    def focus(self, focused, physically_down=False):
        if self.focused and not focused:
            self.pending = None; self.blocked = True
            self.engine.stop('窗口失焦：已停止，必须手动重新准备')
        self.focused = focused
        if focused and not physically_down and not self.down: self.blocked = False

    def press(self, now):
        if self.down: return False
        self.down = True
        if self.engine.requested:
            self.pending = None; self.engine.stop('再次空格：已停止'); return False
        if self.focused and not self.blocked and self.engine.start_ready(): self.pending = now
        return False

    def release(self, now, commit=True):
        pending, was_down = self.pending, self.down
        self.pending = None; self.down = False
        if self.focused: self.blocked = False
        if (commit and was_down and pending is not None and self.focused
                and 0 <= now-pending <= 1. and self.engine.start_ready()):
            return self.engine.engage()
        return False

    def escape(self):
        self.pending = None; self.engine.stop('Esc/停止按钮：已停止')
