#!/usr/bin/env python3
"""forward_pi_v1: 四轮50mm/s单次悬空试转；无动力/结束检查只stop/status。"""
import argparse
import json
import signal
import time

from speed_readonly_probe import FIELDS, Probe as SerialProbe, WHEELS

PI_FIELDS = FIELDS | {'stable_ms', 'used'} | {f'want_{w}' for w in WHEELS}
SIGNS = {'fl': 1, 'fr': -1, 'rl': 1, 'rr': -1}
TARGET_CPS = {'fl': 281, 'fr': 278, 'rl': 280, 'rr': 281}
RUN_COMMAND = 'run F50 S5'
HOST_RUN_SECONDS = 4.7  # 提前stop，独立固件5秒上限仍兜底。


def parse_status(line):
    parts = line.split(',')
    if parts[0] not in ('TELSP', 'STATUSSP'):
        raise ValueError('not forward telemetry')
    raw = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in raw:
            raise ValueError('duplicate field')
        raw[key] = value
    if set(raw) != PI_FIELDS or raw['fw'] != 'forward_pi_v1':
        raise ValueError('exact forward_pi_v1 schema required; no run sent')
    row = {k: int(v, 16 if k == 'fault' else 10) for k, v in raw.items() if k != 'fw'}
    row.update(fw=raw['fw'], kind=parts[0])
    for key in ('ms', 'ticks', 'updates', 'fault'):
        if not 0 <= row[key] <= 0xffffffff:
            raise ValueError('unsigned counter out of range')
    for w in WHEELS:
        for key in ('enc', 'cps', 'fcps', 'target', 'want', 'rpm', 'mms'):
            if not -(1 << 31) <= row[f'{key}_{w}'] < (1 << 31):
                raise ValueError('signed value out of range')
        duty = row[f'd_{w}']
        if not -100 <= duty <= 100 or duty % 10 or duty * SIGNS[w] < 0:
            raise ValueError('wrong forward duty sign/10% limit/quantization')
        expected = (100, 100-duty//10) if duty > 0 else (100+duty//10, 100) if duty < 0 else (0, 0)
        if (row[f'p_{w}1'], row[f'p_{w}2']) != expected:
            raise ValueError('actual CCR does not match slow-decay output')
        if not row['en'] and any(row[f'{p}_{w}'] for p in ('d', 'target', 'fcps', 'want')):
            raise ValueError('disabled state has nonzero control value')
        if row['en'] and (row[f'want_{w}'] != TARGET_CPS[w] or
                          not 0 <= row[f'target_{w}'] <= TARGET_CPS[w]):
            raise ValueError('fixed per-wheel 50mm/s target mismatch')
    if (row['ro'] != 0 or row['gpio_low'] != 0 or row['en'] not in (0, 1) or
            row['wdg_reset'] not in (0, 1) or row['used'] not in (0, 1) or
            not 0 <= row['stable_ms'] <= 500 or
            (row['en'] == 0 and row['sel'] != 0) or
            (row['en'] == 1 and (row['sel'] != 5 or row['used'] != 1 or row['fault'])) or
            not 1 <= row['dt_min'] <= row['dt_max'] <= 30 or
            not 0 <= row['irq_us_max'] <= 1000):
        raise ValueError('bad group mode/one-shot/sampling/fault state')
    return row


def healthy(row):
    if row['fault'] or row['wdg_reset']:
        raise ValueError(f"firmware fault/reset: {row['fault']:#x}/{row['wdg_reset']}")


def stationary(before, row, require_unused=True):
    healthy(row)
    if row['en'] or row['sel'] or (require_unused and row['used']):
        raise ValueError('not idle/unused')
    if any(row[f'enc_{w}'] != before[f'enc_{w}'] or
           any(row[f'{p}_{w}'] for p in ('cps', 'rpm', 'mms')) for w in WHEELS):
        raise ValueError('stationary count/speed changed')


def moving(before, row):
    healthy(row)
    if row['en'] != 1 or row['sel'] != 5 or row['used'] != 1:
        raise ValueError('four-wheel run ended early or wrong selection')
    for w in WHEELS:
        delta = (row[f'enc_{w}']-before[f'enc_{w}']) & 0xffffffff
        if delta >= 0x80000000:
            delta -= 0x100000000
        if (delta * SIGNS[w] < 0 or not 0 <= row[f'cps_{w}'] * SIGNS[w] <= 1000 or
                not 0 <= row[f'fcps_{w}'] <= 1000 or row[f'rpm_{w}'] < 0 or row[f'mms_{w}'] < 0):
            raise ValueError('unexpected per-wheel direction or excessive speed')
    # 动作允许四轮同时有计数，不放宽单轮固件的隔离判断。


def rates(before, after):
    elapsed = (after['ms'] - before['ms']) & 0xffffffff
    if not 500 <= elapsed <= 6500:
        raise ValueError('unexpected sampling interval')
    values = {key: ((after[key] - before[key]) & 0xffffffff)*1000/elapsed for key in ('ticks', 'updates')}
    if any(not 95 <= value <= 105 for value in values.values()):
        raise ValueError('sampling rate not near 100Hz')
    return {'elapsed_ms': elapsed, 'timer_hz': values['ticks'], 'measurement_hz': values['updates']}


class Probe(SerialProbe):
    def __init__(self, link, motion=False, after_run=False, powered_stationary=False):
        super().__init__(link, parser=parse_status)
        if (motion and (after_run or powered_stationary)) or (after_run and powered_stationary):
            raise ValueError('post-run mode cannot send motion')
        self.motion = motion
        self.after_run = after_run
        self.powered_stationary = powered_stationary
        self.run_sent = False
        self.rows = []

    def send(self, command):
        allowed = ('stop', 'status', 'hb', RUN_COMMAND) if self.motion else ('stop', 'status')
        if command not in allowed or (command == RUN_COMMAND and self.run_sent):
            raise ValueError('command allowlist/one-shot violation')
        # 写入失败也不能重试启动：串口对端是否收到无法据此排除。
        if command == RUN_COMMAND:
            self.run_sent = True
        self.commands.append(command)
        data = (command+'\n').encode('ascii')
        if self.link.write(data) != len(data):
            raise ValueError('short serial write')

    def event(self, timeout):
        event = super().event(timeout)
        if isinstance(event, dict):
            self.rows.append(event)
        if isinstance(event, str) and event.startswith('ERR,'):
            raise ValueError('command rejected: '+event)
        return event

    def observe_idle(self, seconds):
        before = self.query()
        stationary(before, before, require_unused=not self.after_run)
        if self.after_run and before['used'] != 1:
            raise ValueError('expected spent one-shot run state')
        deadline, last_rx, latest, count = time.monotonic()+seconds, time.monotonic(), before, 0
        while time.monotonic() < deadline:
            row = self.event(min(0.03, max(0, deadline-time.monotonic())))
            now = time.monotonic()
            if now-last_rx > 0.3:
                raise TimeoutError('idle telemetry gap exceeds 0.3s')
            if isinstance(row, dict):
                stationary(before, row, require_unused=not self.after_run)
                if self.after_run and row['used'] != 1:
                    raise ValueError('one-shot state reset during post-run observation')
                latest, last_rx, count = row, now, count+1
        if count < int(seconds*8) or latest['stable_ms'] < 500:
            raise ValueError('insufficient idle samples/settling')
        return before, latest, rates(before, latest)

    def trial(self, before):
        self.send('hb')
        self.wait(lambda x: x == 'OK,hb_forward', timeout=0.2)
        started = time.monotonic()
        self.send(RUN_COMMAND)
        # 等待运行ACK也必须在同一个有界、继续发心跳的循环里。
        deadline, next_hb, last_rx = started+HOST_RUN_SECONDS, started+0.05, started
        acknowledged, first, last, run_rows = False, None, None, []
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now-next_hb > 0.08:
                raise TimeoutError('host heartbeat scheduling delayed')
            if now >= next_hb:
                self.send('hb')
                next_hb = now+0.05
            event = self.event(min(0.01, max(0, deadline-time.monotonic())))
            now = time.monotonic()
            if event == 'OK,running_forward':
                acknowledged = True
            if isinstance(event, dict):
                # 启动前已在串口中的末帧只读遥测可先到，不能当作运行反馈。
                if event['en'] == 0 and not acknowledged and now-started < 0.2:
                    stationary(before, event)
                    continue
                moving(before, event)
                if first is None:
                    first = event
                last, last_rx = event, now
                run_rows.append(event)
            if not acknowledged and now-started > 0.3:
                raise TimeoutError('run acknowledgement missing')
            if now-last_rx > 0.3:
                raise TimeoutError('running telemetry gap exceeds 0.3s')
        # 主机先停，不等自然5秒截止成为故障；仍保存固件独立5秒兜底。
        self.send('stop')
        self.wait(lambda x: x == 'OK,stopped_forward', timeout=0.5)
        if not acknowledged or len(run_rows) < 30 or last is None:
            raise ValueError('run not sufficiently observed')
        progress = {}
        for w in WHEELS:
            delta = (last[f'enc_{w}']-before[f'enc_{w}']) & 0xffffffff
            if delta >= 0x80000000:
                delta -= 0x100000000
            progress[w] = delta * SIGNS[w]
            if not 10 <= progress[w] <= 100000:
                raise ValueError('insufficient forward progress on '+w)
        sampling = rates(first, last)
        tail = [row for row in run_rows if (row['ms']-first['ms']) & 0xffffffff >= 3000]
        return {'host_run_limit_s': HOST_RUN_SECONDS, 'firmware_run_limit_ms': 5000,
                'run_samples': len(run_rows), 'normalized_forward_progress': progress, 'cadence': sampling,
                'target_wheel_speed_mm_s': 50, 'last_running_state': last,
                'tail_measured_mm_s_mean': {w: sum(r[f'mms_{w}'] for r in tail)/len(tail)
                                           for w in WHEELS} if tail else {},
                'max_abs_duty_permille': max(abs(r[f'd_{w}']) for r in run_rows for w in WHEELS),
                'tracking_acceptance': 'not tuned/loaded driving; suspended integrated observation only'}

    def accepted_final_fault(self):
        return 0

    def run(self):
        errors, before, end, sampling, result = [], None, None, None, None
        try:
            self.wait(lambda x: isinstance(x, dict))
            self.send('stop')
            self.wait(lambda x: x == 'OK,stopped_forward')
            before, settled, sampling = self.observe_idle(0.8 if self.motion else 5)
            if self.motion:
                result = self.trial(settled)
        except (Exception, KeyboardInterrupt) as exc:
            errors.append(type(exc).__name__+': '+str(exc))
        finally:
            try:
                self.send('stop')
                self.wait(lambda x: x == 'OK,stopped_forward', timeout=0.5)
                # 可运行版的stop为高阻滑行，等待实测计数稳定，而非假设立即停轮。
                deadline = time.monotonic()+3
                while time.monotonic() < deadline:
                    event = self.event(0.03)
                    if isinstance(event, dict):
                        end = event
                        if event['en'] == 0 and (event['fault'] or (event['stable_ms'] >= 500 and all(
                                event[f'{p}_{w}'] == 0 for w in WHEELS for p in ('cps', 'rpm', 'mms')))):
                            break
                else:
                    raise TimeoutError('final zero-speed/settling telemetry not observed')
                end = self.query()
                if end['en'] != 0 or end['sel'] != 0:
                    raise ValueError('final output not disabled')
                if end['fault'] != self.accepted_final_fault() or end['wdg_reset']:
                    errors.append(f"final firmware fault/reset: {end['fault']:#x}/{end['wdg_reset']}")
            except (Exception, KeyboardInterrupt) as exc:
                errors.append('final stop/status failed: '+str(exc))
        return {'test': 'forward_pi_forward50_once' if self.motion else
                        'forward_pi_powered_stationary' if self.powered_stationary else
                        'forward_pi_post_run_no_power' if self.after_run else 'forward_pi_no_power',
                'passed': not errors, 'errors': errors, 'firmware': 'forward_pi_v1',
                'power_state': 'powered_suspended_confirmed' if self.motion else
                               'powered_stationary_confirmed' if self.powered_stationary else
                               'battery_disconnected_confirmed',
                'motion_command_sent': self.run_sent, 'command_order': self.commands,
                'start_state': before, 'end_state': end, 'idle_cadence': sampling,
                'trial': result, 'telemetry': self.rows, 'raw_serial_records': self.records,
                'scope': 'protocol/limits/encoder observations only; operator must confirm physical wheel direction/stop; '
                         'no waveform, tuned tracking, loaded driving or Vicon claim'}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    p.add_argument('--mode', choices=('no-power', 'post-run', 'powered-stationary', 'forward50'), required=True)
    p.add_argument('--confirm-battery-disconnected', action='store_true')
    p.add_argument('--confirm-powered-suspended', action='store_true')
    args = p.parse_args()
    if not ((args.mode in ('no-power', 'post-run') and args.confirm_battery_disconnected and not args.confirm_powered_suspended) or
            (args.mode in ('forward50', 'powered-stationary') and args.confirm_powered_suspended and not args.confirm_battery_disconnected)):
        p.error('exact mode-specific physical confirmation required; serial unopened')
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.005, write_timeout=0.05, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        result = Probe(link, motion=args.mode == 'forward50', after_run=args.mode == 'post-run',
                       powered_stationary=args.mode == 'powered-stationary').run()
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())

