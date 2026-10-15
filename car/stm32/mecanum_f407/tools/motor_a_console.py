#!/usr/bin/env python3
"""Interactive Motor_A console with an automatic safety heartbeat."""

import argparse
import sys
import threading
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python3 -m pip install pyserial") from exc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="serial port, for example /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    link = serial.Serial(args.port, args.baud, timeout=0.1)
    stop_event = threading.Event()

    def receiver() -> None:
        while not stop_event.is_set():
            line = link.readline()
            if line:
                print(line.decode(errors="replace").rstrip())

    def heartbeat() -> None:
        while not stop_event.wait(0.1):
            link.write(b"hb\n")

    threading.Thread(target=receiver, daemon=True).start()
    threading.Thread(target=heartbeat, daemon=True).start()
    print("Commands: arm A5 | duty -200..200 | stop | estop | clear 5A | zero | status")

    try:
        for line in sys.stdin:
            command = line.strip()
            if command in {"quit", "exit"}:
                break
            if command:
                link.write(command.encode("ascii") + b"\n")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            link.write(b"stop\n")
            time.sleep(0.05)
        finally:
            stop_event.set()
            link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

