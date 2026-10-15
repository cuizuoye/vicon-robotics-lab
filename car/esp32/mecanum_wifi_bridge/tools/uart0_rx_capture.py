#!/usr/bin/env python3
"""ESP32 UART0启动日志：仅接USB-TTL RX/GND，不供电、不发送、不断言启动成功。"""
import argparse
import json
from pathlib import Path
import time

import serial


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--serial', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=300.0)
    parser.add_argument('--confirm-rx-gnd-only', action='store_true', required=True)
    parser.add_argument('--confirm-motors-disconnected', action='store_true', required=True)
    args = parser.parse_args()
    if not 1.0 <= args.seconds <= 600.0:
        parser.error('capture duration must be 1..600 seconds')

    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    # 日志使用独占新文件；不覆盖上一轮证据，不接触板内Flash。
    with target.open('x', encoding='utf-8') as output:
        started = time.monotonic()

        def event(kind, **data):
            output.write(json.dumps(dict(event=kind, elapsed_s=time.monotonic()-started,
                                         **data), ensure_ascii=False)+'\n')
            output.flush()

        port = serial.Serial(port=None, baudrate=115200, timeout=0.1, exclusive=True)
        port.dtr = port.rts = False
        port.port = args.serial
        total = 0
        result = 'timeout'
        try:
            port.open()
            event('listening', serial=args.serial, baud=115200, rx_gnd_only=True,
                  motors_disconnected=True, serial_data_transmitted=False)
            print('LISTENING: UART0 115200, RX/GND only; no serial data transmission', flush=True)
            while time.monotonic()-started < args.seconds:
                data = port.read(min(max(port.in_waiting, 1), 4096))
                if not data:
                    continue
                total += len(data)
                decoded = data.decode('utf-8', errors='replace')
                event('rx', raw_hex=data.hex(), text=decoded)
                print(decoded, end='', flush=True)
                if total >= 1048576:
                    result = 'byte_limit'
                    break
        except KeyboardInterrupt:
            result = 'interrupted'
        except Exception as error:
            result = 'error'
            event('error', message=str(error))
            raise
        finally:
            port.close()
            event('finished', result=result, bytes_received=total,
                  serial_data_transmitted=False)
            print(f'\nCAPTURE END: {result}, {total} bytes; saved {target}', flush=True)


if __name__ == '__main__':
    main()
