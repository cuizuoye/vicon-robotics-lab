#!/usr/bin/env python3
"""Battery-only single-owner TRS + Vicon read-only gate. Never a flight runner.

No pose injection, parameter write, mode, arming or output-lock changes.
The fixed actuator listener verifies the lock live; an old report is not used
as current state. A pass does not establish fusion or authorize takeoff.
"""
import argparse
from collections import Counter
from contextlib import ExitStack
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import serial
from pymavlink.dialects.v20 import common as m
from estop_only_policy import (FIRMWARE, LISTENER, parse_actuator_snapshot,
                              shell_message, shell_message_allowed)
from mavlink_readonly_probe import Evidence, FC_USB, PassivePowerEvidence, readonly_message
from persistent_shell import reply_complete
from trs_only_estop_bench import state_valid
from trs_uart_readonly_check import UID2

PARAMS = {'EKF2_HGT_REF': 3, 'EKF2_RNG_CTRL': 0, 'EKF2_OF_CTRL': 0,
          'EKF2_BARO_CTRL': 1, 'EKF2_EV_CTRL': 2, 'EKF2_EV_DELAY': 0,
          'COM_RC_IN_MODE': 4, 'COM_OBL_RC_ACT': 4, 'COM_OF_LOSS_T': 1}


def allowed(msg, *, identity_fresh=False):
    if msg.get_type() == 'PARAM_REQUEST_READ':
        name = msg.param_id.decode().rstrip('\0') if isinstance(msg.param_id, bytes) else msg.param_id
        return bool(identity_fresh and name in PARAMS and
                    (msg.target_system, msg.target_component, msg.param_index) == (1, 1, -1))
    if msg.get_type() == 'SERIAL_CONTROL':
        return bool(identity_fresh and shell_message_allowed(msg))
    return readonly_message(msg)


def locked_fresh(state, boot_ms):
    return bool(state_valid(state, boot_ms) and state['lockdown'])


def parameters_match(values):
    return all(values.get(name) == value for name, value in PARAMS.items())


def run(args):
    if not args.guarded_props_installed_readonly_confirmed:
        raise ValueError('Explicit protected, stationary, props-installed read-only confirmation required')
    if args.output.exists() or FC_USB.exists():
        raise ValueError('New evidence file and absent FC USB required')
    if subprocess.run(['fuser', str(Path(args.trs_port).resolve())], capture_output=True).returncode == 0:
        raise ValueError('TRS occupied; no process will be killed')
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros2_ws/src/uav_coord_bridge'))
    from uav_coord_bridge.mavlink_vision import PoseGate
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import PoseStamped

    class Source(Node):
        def __init__(self):
            super().__init__('trs_battery_readonly_source')
            self.gate = PoseGate(max_age_ms=150)
            self.sub = self.create_subscription(PoseStamped, '/vicon/MircoAir/MircoAir', self.receive,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE))

        def receive(self, msg):
            p, q = msg.pose.position, msg.pose.orientation
            self.gate.ingest(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds, time.monotonic_ns(), msg.header.frame_id,
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    rclpy.init(args=[])
    node = Source()
    ev, power = Evidence(250, 201, expected_parameters=tuple(PARAMS)), PassivePowerEvidence()
    enc, dec = m.MAVLink(None, srcSystem=250, srcComponent=201), m.MAVLink(None)
    dec.robust_parsing = True
    tx, texts, snapshots, local_rows, fragments = Counter(), [], [], [], []
    start = time.monotonic()
    observe_start = pending_at = None
    next_hb = next_version = next_params = next_poll = 0
    buffer, error = '', None
    passed = False
    max_source_age_ms = 0

    def elapsed():
        return time.monotonic() - start

    def identity_fresh():
        return bool(not FC_USB.exists() and not power.reboot_observed and not ev.fatal
                    and ev.identity and ev.identity['uid2'] == UID2
                    and ev.identity['firmware'] == FIRMWARE and ev.heartbeats
                    and not ev.heartbeats[-1]['armed']
                    and 0 <= elapsed() - ev.heartbeats[-1]['elapsed_s'] < 2)

    def send(port, msg):
        if not allowed(msg, identity_fresh=identity_fresh()):
            raise RuntimeError('Read-only whitelist rejected ' + msg.get_type())
        packet = msg.pack(enc)
        if port.write(packet) != len(packet):
            raise OSError('Short serial write')
        enc.seq = (enc.seq + 1) % 256
        tx[msg.get_type()] += 1

    try:
        with ExitStack() as stack:
            port = stack.enter_context(serial.Serial(args.trs_port, 115200, timeout=.002,
                                                     write_timeout=.5, exclusive=True))
            port.reset_input_buffer()
            while elapsed() < 30:
                t = elapsed()
                rclpy.spin_once(node, timeout_sec=0)
                if FC_USB.exists():
                    raise RuntimeError('FC USB appeared')
                for msg in dec.parse_buffer(port.read(4096)) or []:
                    ev.ingest(msg, t)
                    power.ingest(msg, t)
                    if (msg.get_srcSystem(), msg.get_srcComponent()) != (1, 1):
                        continue
                    if msg.get_type() == 'STATUSTEXT':
                        texts.append({'elapsed_s': t, 'text': msg.text})
                    elif msg.get_type() == 'LOCAL_POSITION_NED':
                        values = [msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz]
                        if not all(math.isfinite(v) for v in values):
                            raise RuntimeError('Non-finite local telemetry')
                        local_rows.append({'elapsed_s': t, 'boot_ms': msg.time_boot_ms,
                                           'position': values[:3], 'velocity': values[3:]})
                    elif msg.get_type() == 'SERIAL_CONTROL' and pending_at is not None:
                        fragment = bytes(msg.data[:msg.count]).decode('utf8', 'replace')
                        buffer += fragment
                        fragments.append({'elapsed_s': t, 'text': fragment})
                        state = parse_actuator_snapshot(buffer)
                        if state and reply_complete(buffer, LISTENER):
                            boot_ms = power.boot_samples[-1]['time_boot_ms'] if power.boot_samples else None
                            if not locked_fresh(state, boot_ms):
                                raise RuntimeError('Live actuator snapshot is stale or not disarmed/locked')
                            snapshots.append({**state, 'elapsed_s': t, 'requested_at_s': pending_at})
                            buffer, pending_at = '', None
                if ev.fatal or power.reboot_observed:
                    raise RuntimeError(ev.fatal or 'FC reboot; old lock invalid')
                if ev.identity and (ev.identity['uid2'] != UID2 or ev.identity['firmware'] != FIRMWARE):
                    raise RuntimeError('Unexpected vehicle identity')
                if snapshots and not identity_fresh():
                    raise RuntimeError('Fresh disarmed heartbeat lost')
                if t >= next_hb:
                    send(port, enc.heartbeat_encode(m.MAV_TYPE_GCS, m.MAV_AUTOPILOT_INVALID, 0, 0, 0))
                    next_hb = t + 1
                if ev.heartbeats and ev.identity is None and t >= next_version:
                    ev.version_requested_at = t
                    send(port, enc.command_long_encode(1, 1, 512, 0, 148, 0, 0, 0, 0, 0, 0))
                    next_version = t + 3
                if identity_fresh() and not parameters_match(ev.parameters) and t >= next_params:
                    for name in PARAMS:
                        if name not in ev.parameters:
                            ev.requests[name] = t
                            send(port, enc.param_request_read_encode(1, 1, name.encode(), -1))
                    next_params = t + 3
                if any(name in ev.parameters and ev.parameters[name] != val for name, val in PARAMS.items()):
                    raise RuntimeError('Maintenance configuration changed; no automatic correction')
                if identity_fresh() and pending_at is None and (not snapshots or
                        (observe_start is not None and t - observe_start >= 8 and len(snapshots) == 1)):
                    send(port, shell_message(enc))
                    pending_at, buffer = t, ''
                if pending_at is not None:
                    if t - pending_at > 6 or len(buffer) > 16384:
                        raise RuntimeError('Fixed TRS actuator reply incomplete')
                    if t >= next_poll:
                        send(port, shell_message(enc, poll=True))
                        next_poll = t + .5
                source_fresh = node.gate.fresh(node.get_clock().now().nanoseconds, time.monotonic_ns())
                if observe_start is None and snapshots and parameters_match(ev.parameters) and source_fresh:
                    observe_start = t
                    print('READY: 单TRS读取与实时Vicon同时就绪；开始8秒只读观察。', flush=True)
                if observe_start is not None:
                    if not source_fresh:
                        raise RuntimeError('Vicon source stale during joint observation')
                    max_source_age_ms = max(max_source_age_ms,
                        (node.get_clock().now().nanoseconds - node.gate.last.stamp_ns) / 1e6)
                    if len(snapshots) >= 2:
                        passed = bool(local_rows and power.batteries and
                                      power.batteries[-1]['reported_voltage_v'] is not None and
                                      power.batteries[-1]['reported_voltage_v'] > 0)
                        if not passed:
                            raise RuntimeError('Missing battery/local telemetry observations')
                        break
                if observe_start is None and t > 20:
                    raise RuntimeError('Joint readiness timeout; Vicon reason=' + node.gate.reason)
            else:
                raise RuntimeError('Bounded read-only session expired')
    except (Exception, KeyboardInterrupt) as exc:
        error = str(exc) or 'Interrupted'
    finally:
        last = node.gate.last
        source = {'accepted': node.gate.accepted, 'rejected': node.gate.rejected,
                  'reason': node.gate.reason, 'max_observed_source_age_ms': max_source_age_ms,
                  'latest_world_position_m': last.position if last else None,
                  'latest_rigid_quaternion_xyzw': last.quaternion if last else None,
                  'frame': 'map', 'body_origin_and_frame_mapping_verified': False}
        node.destroy_node()
        rclpy.shutdown()
    report = {'captured_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
              'scope': 'Battery-only single-owner TRS and Vicon READ-ONLY preparation; NOT fusion/flight',
              'propeller_state': 'installed', 'physical_conditions_source': 'operator report',
              'fc_usb_present_at_end': FC_USB.exists(), 'duration_s': elapsed(),
              'identity': ev.identity, 'parameters': ev.parameters, 'source': source,
              'snapshots': snapshots, 'local_position_telemetry': local_rows,
              'passive_power': power.result(), 'statustext': texts,
              'shell_fragments': fragments, 'incomplete_shell_text': buffer,
              'tx_messages': dict(tx), 'error': error,
              'readonly_joint_observation_verified': passed and not error,
              'pose_messages_sent': 0, 'parameter_writes_sent': 0, 'motion_or_mode_commands_sent': 0,
              'fusion_verified': False, 'flight_authorized': False,
              'home_monitor_running_after_exit': False, 'serial_port_released': True}
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: report[k] for k in ('readonly_joint_observation_verified', 'source',
                     'parameters', 'tx_messages', 'error', 'flight_authorized')}, ensure_ascii=False, indent=2))
    return 0 if passed and not error else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--trs-port', required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--guarded-props-installed-readonly-confirmed', action='store_true', required=True)
    args = ap.parse_args()
    try:
        return run(args)
    except (ValueError, OSError, RuntimeError) as exc:
        ap.exit(2, str(exc) + '\n')


if __name__ == '__main__':
    raise SystemExit(main())
