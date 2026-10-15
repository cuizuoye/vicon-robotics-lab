#!/usr/bin/env python3
"""手柄急停输入只读验证：HOME按下锁存，松开不恢复；没有飞控通信入口。"""

import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import select
import struct
import time

HOME = 0x13c
EVENT = struct.Struct('llHHi')


class StopLatch:
    def __init__(self):
        self.latched = False
        self.reason = None
        self.home_pressed = False

    def event(self, kind, code, value):
        if kind == 0 and code == 3:  # SYN_DROPPED意味着输入事件队列丢失。
            self.fail('input_queue_overflow')
        elif kind == 1 and code == HOME:
            if value in (1, 2):
                self.home_pressed = True
                self.fail('home_pressed')
            elif value == 0:
                self.home_pressed = False

    def fail(self, reason):
        self.latched = True
        if self.reason is None or reason == 'home_pressed':
            self.reason = reason


def find_pad():
    matches = []
    for entry in Path('/sys/class/input').glob('event*'):
        dev = entry / 'device'
        try:
            vid = (dev / 'id/vendor').read_text().strip()
            pid = (dev / 'id/product').read_text().strip()
            name = (dev / 'name').read_text().strip()
            words = (dev / 'capabilities/key').read_text().split()
            word_bits = struct.calcsize('L') * 8
            key_bits = sum(int(word, 16) << (word_bits * index)
                           for index, word in enumerate(reversed(words)))
        except OSError:
            continue
        if (vid, pid) == ('057e', '2009') and key_bits & (1 << HOME):
            matches.append({'path': f'/dev/input/{entry.name}', 'name': name,
                            'vid': vid, 'pid': pid})
    if len(matches) != 1:
        raise RuntimeError(f'Expected one 057e:2009 HOME-capable gamepad, found {len(matches)}')
    return matches[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seconds', type=float, default=120)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    if args.output.exists() or not 5 <= args.seconds <= 300:
        ap.error('Use a new output file and 5..300 seconds')
    device = find_pad()
    latch = StopLatch()
    counts = Counter()
    events = []
    fd = os.open(device['path'], os.O_RDONLY | os.O_NONBLOCK)
    started = time.monotonic()
    disconnected = False
    try:
        # EVIOCGKEY读当前按键，开程序前已经按住HOME也能识别。
        bitmap = fcntl.ioctl(fd, 0x80000000 | (96 << 16) | (ord('E') << 8) | 0x18,
                             bytes(96))
        if bitmap[HOME // 8] & (1 << (HOME % 8)):
            latch.event(1, HOME, 1)
            events.append({'elapsed_s': 0.0, 'code': HOME, 'value': 1,
                           'origin': 'initial_key_bitmap'})
        print(f"READY {device['name']} {device['path']}; HOME按下并松开一次。只读、不会控制飞控。", flush=True)
        pending = b''
        next_status = 0.0
        while time.monotonic() - started < args.seconds:
            now = time.monotonic() - started
            ready, _, _ = select.select([fd], [], [], 0.1)
            if ready:
                try:
                    chunk = os.read(fd, EVENT.size * 64)
                except BlockingIOError:
                    continue
                if not chunk:
                    raise OSError('Input device disconnected')
                pending += chunk
                while len(pending) >= EVENT.size:
                    _, _, kind, code, value = EVENT.unpack(pending[:EVENT.size])
                    pending = pending[EVENT.size:]
                    latch.event(kind, code, value)
                    if kind == 1:
                        counts[f'0x{code:x}:{value}'] += 1
                        item = {'elapsed_s': round(now, 3), 'code': code, 'value': value}
                        events.append(item)
                        print(f'KEY 0x{code:x} value={value}; latched={latch.latched}', flush=True)
            if now >= next_status:
                print(f'STATUS {now:.1f}s home={latch.home_pressed} latched={latch.latched} reason={latch.reason}', flush=True)
                next_status = now + 10
            home_values = [item['value'] for item in events if item['code'] == HOME]
            if 1 in home_values and home_values[-1:] == [0]:
                # 松开后再读一秒，确认不会解除锁存。
                if now - next(item['elapsed_s'] for item in reversed(events)
                              if item['code'] == HOME and item['value'] == 0) >= 1:
                    break
    except OSError as exc:
        disconnected = True
        latch.fail('device_error')
        print(f'INPUT ERROR {exc}', flush=True)
    except KeyboardInterrupt:
        latch.fail('monitor_interrupted')
    finally:
        os.close(fd)
    values = [item['value'] for item in events if item['code'] == HOME]
    result = {'device': device, 'duration_s': round(time.monotonic() - started, 3),
              'events': events, 'key_counts': dict(counts), 'latched': latch.latched,
              'reason': latch.reason, 'disconnected': disconnected,
              'press_release_verified': 1 in values and values[-1:] == [0]
                  and latch.latched and not disconnected,
              'scope': 'Input-only: no serial, network or flight-controller messages'}
    with args.output.open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result['press_release_verified'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
