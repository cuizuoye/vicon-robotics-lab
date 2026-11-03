#!/usr/bin/env python3
"""单次FL悬空心跳断流试验；不改固件，不拔线，不清故障，不重复启动。"""
import argparse
import json
import signal
import time

from speed_pi_probe import Probe as SpeedProbe, RUN_COMMAND, moving, rates
from speed_readonly_probe import WHEELS

EXPECTED_FAULT = 0x4  # FOUR_SPEED_FAULT_COMMAND，必须是精确这一位。
DRIVE_SECONDS = 1.8
FAULT_REPORT_DEADLINE_S = 0.5
LATCH_OBSERVATION_S = 0.8


def fault_stopped(before, row):
    if (row['fault'] != EXPECTED_FAULT or row['wdg_reset'] or row['en'] or row['sel'] or
            row['used'] != 1 or any(row[f'{p}_{w}'] for w in WHEELS for p in ('d', 'target', 'fcps')) or
            any(row[f'p_{w}{i}'] for w in WHEELS for i in (1, 2)) or
            any(row[f'enc_{w}'] != before[f'enc_{w}'] for w in WHEELS if w != 'fl')):
        raise ValueError('expected isolated COMMAND fault and zero output not proven')


class Probe(SpeedProbe):
    def __init__(self, link, latched=False):
        super().__init__(link, motion=not latched)
        self.latched = latched
        self.hb_allowed = not latched
        self.hb_sent = self.hb_acks = 0
        self.proven = False
        self.cutoff_host_time = None

    def send(self, command):
        if command == 'hb' and not self.hb_allowed:
            raise ValueError('heartbeat forbidden after intentional cutoff')
        super().send(command)
        if command == 'hb':
            self.hb_sent += 1

    def event(self, timeout):
        event = super().event(timeout)
        if event == 'OK,hb_speed':
            self.hb_acks += 1
        return event

    def accepted_final_fault(self):
        return EXPECTED_FAULT if self.proven else 0

    def trial(self, before):
        self.send('hb')
        self.wait(lambda x: x == 'OK,hb_speed', timeout=0.2)
        started = time.monotonic()
        self.send(RUN_COMMAND)
        deadline, next_hb, last_rx = started+DRIVE_SECONDS, started+0.05, started
        acknowledged, active = False, []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now-next_hb > 0.08:
                raise TimeoutError('heartbeat scheduling failed before deliberate loss')
            if now >= next_hb:
                self.send('hb')
                next_hb = now+0.05
            event = self.event(min(0.01, max(0, deadline-time.monotonic())))
            now = time.monotonic()
            if event == 'OK,running_speed':
                acknowledged = True
            if isinstance(event, dict):
                if event['en'] == 0 and not acknowledged and now-started < 0.2:
                    continue
                moving(before, event)
                active.append(event)
                last_rx = now
            if not acknowledged and now-started > 0.3:
                raise TimeoutError('run ACK missing')
            if now-last_rx > 0.3:
                raise TimeoutError('telemetry lost before heartbeat cutoff')
        if not acknowledged or len(active) < 12 or active[-1]['enc_fl']-before['enc_fl'] < 10:
            raise ValueError('no sufficient forward run evidence before cutoff')
        # 最后一个心跳写完后不再发送任何命令，直到独立停机证据完成/失败退出。
        self.send('hb')
        self.cutoff_host_time = time.monotonic()
        self.hb_allowed = False
        cutoff_command_index = len(self.commands)
        deadline = self.cutoff_host_time+FAULT_REPORT_DEADLINE_S
        fault_row, report_delay = None, None
        while time.monotonic() < deadline:
            event = self.event(min(0.01, max(0, deadline-time.monotonic())))
            if isinstance(event, dict):
                if event['fault']:
                    fault_stopped(before, event)
                    fault_row = event
                    report_delay = time.monotonic()-self.cutoff_host_time
                    break
                moving(before, event)
        if fault_row is None:
            raise TimeoutError('no autonomous COMMAND-fault/zero-output report within 0.5s; stop now')
        deadline, latched_rows, last_rx = time.monotonic()+LATCH_OBSERVATION_S, [], time.monotonic()
        while time.monotonic() < deadline:
            event = self.event(min(0.02, max(0, deadline-time.monotonic())))
            if isinstance(event, dict):
                fault_stopped(before, event)
                latched_rows.append(event)
                last_rx = time.monotonic()
            if time.monotonic()-last_rx > 0.3:
                raise TimeoutError('latched-fault telemetry lost')
        if len(latched_rows) < 5 or self.hb_acks != self.hb_sent:
            raise ValueError('insufficient latch samples or heartbeat ACK accounting')
        tail = latched_rows[-5:]
        if any(any(r[f'enc_{w}'] != tail[0][f'enc_{w}'] or
                   any(r[f'{p}_{w}'] for p in ('cps', 'rpm', 'mms')) for w in WHEELS) for r in tail):
            raise ValueError('stable zero-speed samples not observed after fault')
        if len(self.commands) != cutoff_command_index:
            raise ValueError('unexpected command during autonomous fault observation')
        self.proven = True
        return {'drive_host_limit_s': DRIVE_SECONDS, 'firmware_timeout_ms': 150,
                'host_fault_report_deadline_s': FAULT_REPORT_DEADLINE_S,
                'first_fault_host_delay_s': report_delay, 'first_fault_state': fault_row,
                'active_samples': len(active), 'active_cadence': rates(active[0], active[-1]),
                'last_running_state': active[-1], 'latched_samples': len(latched_rows),
                'fault_remained_latched': True, 'hb_sent': self.hb_sent, 'hb_acks': self.hb_acks,
                'no_command_during_fault_observation': True,
                'stop_command_sent_before_fault_evidence': False,
                'timing_scope': 'host serial-write to received 10Hz telemetry; includes buffering/wire delay; '
                                'not MCU exact timeout or physical wheel stop latency'}

    def observe_idle(self, seconds):
        if not self.latched:
            return super().observe_idle(seconds)
        before = self.query()
        fault_stopped(before, before)
        deadline, last_rx, rows = time.monotonic()+5, time.monotonic(), []
        while time.monotonic() < deadline:
            event = self.event(min(0.02, max(0, deadline-time.monotonic())))
            if isinstance(event, dict):
                fault_stopped(before, event)
                if any(event[f'enc_{w}'] != before[f'enc_{w}'] or
                       any(event[f'{p}_{w}'] for p in ('cps', 'rpm', 'mms')) for w in WHEELS):
                    raise ValueError('latched stationary counts/speed changed')
                rows.append(event)
                last_rx = time.monotonic()
            if time.monotonic()-last_rx > 0.3:
                raise TimeoutError('latched telemetry gap')
        if len(rows) < 45:
            raise ValueError('insufficient latched samples')
        self.proven = True
        return before, rows[-1], rates(before, rows[-1])

    def run(self):
        result = super().run()
        result['test'] = 'speed_pi_command_loss_latched_no_power' if self.latched else 'speed_pi_command_loss_once'
        result['expected_fault'] = EXPECTED_FAULT
        result['autonomous_fault_proven'] = self.proven
        result['scope'] = ('USB-TTL UART heartbeat/target timeout only; not actual Wi-Fi/UDP outage, '
                           'physical braking latency, watchdog or four-wheel validation')
        return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--mode', choices=('loss-once', 'latched'), required=True)
    p.add_argument('--confirm-powered-suspended', action='store_true')
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    args = p.parse_args()
    if not ((args.mode == 'loss-once' and args.confirm_powered_suspended and not args.confirm_battery_disconnected) or
            (args.mode == 'latched' and args.confirm_battery_disconnected and not args.confirm_powered_suspended)):
        p.error('mode-specific physical confirmation required; serial unopened')
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.05, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        result = Probe(link, latched=args.mode == 'latched').run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
