#!/usr/bin/env python3
"""speed_ro_v1只读检查：仅stop/status，明确区分断动力与上电静止观察。"""
import argparse
import json
import signal
import time

WHEELS = ('fl', 'fr', 'rl', 'rr')
COMMON = {'fw', 'ms', 'ticks', 'updates', 'dt_min', 'dt_max', 'irq_us_max',
          'ro', 'gpio_low', 'wdg_reset', 'en', 'sel', 'fault'}
FIELDS = COMMON | {f'{p}_{w}' for w in WHEELS
                   for p in ('enc', 'cps', 'fcps', 'target', 'rpm', 'mms', 'd')}
FIELDS |= {f'p_{w}{i}' for w in WHEELS for i in (1, 2)}


def parse_status(line):
    parts = line.split(',')
    if parts[0] not in ('TELSP', 'STATUSSP'):
        raise ValueError('not speed telemetry')
    fields = {}
    for part in parts[1:]:
        key, value = part.split('=', 1)
        if key in fields:
            raise ValueError('duplicate field')
        fields[key] = value
    if set(fields) != FIELDS or fields['fw'] != 'speed_ro_v1':
        raise ValueError('wrong firmware/schema; read-only image required')
    result = {k: int(v, 16 if k == 'fault' else 10) for k, v in fields.items() if k != 'fw'}
    result.update(fw=fields['fw'], kind=parts[0])
    for key in ('ms', 'ticks', 'updates', 'fault'):
        if not 0 <= result[key] <= 0xffffffff:
            raise ValueError('invalid unsigned counter')
    for w in WHEELS:
        for prefix in ('enc', 'cps', 'fcps', 'target', 'rpm', 'mms'):
            if not -(1 << 31) <= result[f'{prefix}_{w}'] < (1 << 31):
                raise ValueError('invalid signed value')
    if (result['ro'] != 1 or result['gpio_low'] != 1 or result['wdg_reset'] != 0 or
            result['en'] != 0 or result['sel'] != 0 or result['fault'] != 0 or
            not 1 <= result['dt_min'] <= result['dt_max'] <= 30 or
            not 0 <= result['irq_us_max'] <= 1000 or
            any(result[f'{p}_{w}'] != 0 for w in WHEELS for p in ('d', 'target', 'fcps')) or
            any(result[f'p_{w}{i}'] != 0 for w in WHEELS for i in (1, 2))):
        raise ValueError('nonzero output, wrong GPIO mode, fault/reset or bad sample timing')
    return result


def cadence(before, after):
    elapsed = (after['ms'] - before['ms']) & 0xffffffff
    ticks = (after['ticks'] - before['ticks']) & 0xffffffff
    updates = (after['updates'] - before['updates']) & 0xffffffff
    if not 4000 <= elapsed <= 6500:
        raise ValueError('unexpected observation duration/reset')
    tick_hz, update_hz = ticks * 1000 / elapsed, updates * 1000 / elapsed
    if not (95 <= tick_hz <= 105 and 95 <= update_hz <= 105):
        raise ValueError('sampling/control-update rate not near 100 Hz')
    return {'elapsed_ms': elapsed, 'timer_ticks': ticks, 'measurement_updates': updates,
            'timer_hz': tick_hz, 'measurement_hz': update_hz}


def stationary(before, after):
    if any(after[f'enc_{w}'] != before[f'enc_{w}'] or
           any(after[f'{p}_{w}'] != 0 for p in ('cps', 'rpm', 'mms'))
           for w in WHEELS):
        raise ValueError('powered stationary encoder count/speed changed')


class Probe:
    def __init__(self, link, parser=parse_status):
        self.link = link
        self.parser = parser
        self.buffer = bytearray()
        self.records = []
        self.commands = []
        self.last_ms = None

    def send(self, command):
        if command not in ('stop', 'status'):
            raise ValueError('read-only allowlist violation')
        self.commands.append(command)
        data = (command + '\n').encode('ascii')
        if self.link.write(data) != len(data):
            raise ValueError('short serial write')

    def event(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if b'\n' in self.buffer:
                raw, _, remaining = self.buffer.partition(b'\n')
                self.buffer = bytearray(remaining)
                line = raw.decode('ascii').strip('\r')
                if not line:
                    continue
                self.records.append(line)
                if line.startswith(('TELSP,', 'STATUSSP,')):
                    row = self.parser(line)
                    if self.last_ms is not None and (row['ms'] - self.last_ms) & 0xffffffff > 0x7fffffff:
                        raise ValueError('MCU time moved backwards/reset')
                    self.last_ms = row['ms']
                    return row
                if line.startswith(('TEL4,', 'STATUS4,', 'TEL,', 'STATUS,')):
                    raise ValueError('old firmware active; do not use speed commands')
                return line
            data = self.link.read(128)
            if data:
                self.buffer.extend(data)
                if len(self.buffer) > 2048:
                    raise ValueError('overlong/noisy serial stream')
        return None

    def wait(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            event = self.event(min(0.4, deadline - time.monotonic()))
            if event is not None and predicate(event):
                return event
        raise TimeoutError('expected response not received')

    def query(self):
        self.send('status')
        return self.wait(lambda x: isinstance(x, dict) and x['kind'] == 'STATUSSP')

    def run(self, powered_stationary=False):
        errors, baseline, final, timing, samples = [], None, None, None, 0
        try:
            self.wait(lambda x: isinstance(x, dict))
            self.send('stop')
            self.wait(lambda x: x == 'OK,stopped_speed')
            baseline = self.query()
            if powered_stationary:
                stationary(baseline, baseline)
            deadline, last_rx = time.monotonic() + 5, time.monotonic()
            while time.monotonic() < deadline:
                event = self.event(min(0.4, deadline - time.monotonic()))
                now = time.monotonic()
                if now - last_rx > 0.5:
                    raise TimeoutError('telemetry/host scheduling gap exceeded 0.5s')
                if isinstance(event, dict):
                    if powered_stationary:
                        stationary(baseline, event)
                    final = event
                    samples += 1
                    last_rx = now
            if samples < 45 or final is None:
                raise ValueError('insufficient observation samples')
            timing = cadence(baseline, final)
        except (Exception, KeyboardInterrupt) as exc:
            errors.append(type(exc).__name__ + ': ' + str(exc))
        finally:
            try:
                self.send('stop')
                self.wait(lambda x: x == 'OK,stopped_speed')
                final = self.query()
                if powered_stationary and baseline is not None:
                    stationary(baseline, final)
            except (Exception, KeyboardInterrupt) as exc:
                errors.append('final stop/status failed: ' + str(exc))
        return {'test': 'speed_readonly_powered_stationary' if powered_stationary else 'speed_readonly_no_power',
                'passed': not errors, 'errors': errors,
                'firmware': 'speed_ro_v1', 'observation_samples': samples, 'cadence': timing,
                'start_state': baseline, 'end_state': final, 'command_order': self.commands,
                'raw_serial_records': self.records, 'motor_commands_sent': False,
                'power_state': 'powered_stationary_confirmed' if powered_stationary else 'battery_disconnected_confirmed',
                'scope': ('powered, stationary read-only encoder/GPIO/CCR/timer and serial checks; '
                          'not manual-turn speed, waveform or wheel speed-loop validation') if powered_stationary else
                         ('battery disconnected; read-only GPIO/CCR/timer and serial checks; '
                          'not powered encoder, waveform, wheel speed-loop or physical stop validation')}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('port')
    confirmation = p.add_mutually_exclusive_group(required=True)
    confirmation.add_argument('--confirm-battery-disconnected', action='store_true')
    confirmation.add_argument('--confirm-powered-readonly', action='store_true',
                              help='现场确认已上电且四轮静止；仍只允许stop/status')
    args = p.parse_args()
    import serial
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    link = serial.Serial(port=None, baudrate=115200, timeout=0.01, write_timeout=0.1, exclusive=True)
    link.dtr = link.rts = False
    link.port = args.port
    link.open()
    try:
        result = Probe(link).run(powered_stationary=args.confirm_powered_readonly)
    finally:
        link.close()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
