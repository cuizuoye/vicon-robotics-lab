#!/usr/bin/env python3
"""Discover and validate an ESP32 MC-Link bridge without sending motor commands."""

import argparse
from pathlib import Path
import secrets
import socket
import sys
import time


WORKSPACE = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(WORKSPACE / 'src' / 'mecanum_vicon_control'))

from mecanum_vicon_control.transport_protocol import (  # noqa: E402
    FrameFlags,
    MessageType,
    ProtocolError,
    decode_frame,
    encode_frame,
)


COMMAND_PORT = 46000
TELEMETRY_PORT = 46001


def monotonic_ms() -> int:
    return int(time.monotonic() * 1000.0) & 0xFFFFFFFF


def wait_for_ack(sock: socket.socket, session_id: int, sequence: int, deadline: float):
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError
        sock.settimeout(remaining)
        data, sender = sock.recvfrom(512)
        try:
            frame = decode_frame(data)
        except ProtocolError as error:
            print(f'忽略来自 {sender[0]} 的无效帧：{error}', file=sys.stderr)
            continue
        if (frame.message_type == MessageType.HELLO_ACK
                and frame.flags & FrameFlags.RESPONSE
                and frame.session_id == session_id
                and frame.sequence == sequence
                and not frame.payload):
            return sender[0]


def main() -> int:
    parser = argparse.ArgumentParser(
        description='发送 MC-Link HELLO 并验证 ESP32 HELLO_ACK；不会发送速度或使能指令。')
    parser.add_argument('target', nargs='?', default='255.255.255.255',
                        help='ESP32 IPv4；省略时使用局域网广播发现')
    parser.add_argument('--count', type=int, default=5, help='测试次数，默认 5')
    parser.add_argument('--timeout', type=float, default=1.0,
                        help='每次等待应答秒数，默认 1.0')
    args = parser.parse_args()
    if args.count < 1 or args.timeout <= 0.0:
        parser.error('--count 必须大于 0，--timeout 必须为正数')

    session_id = secrets.randbits(32) or 1
    destination = (args.target, COMMAND_PORT)
    results_ms = []

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(('0.0.0.0', TELEMETRY_PORT))

        for index in range(args.count):
            sequence = index + 1
            hello = encode_frame(
                MessageType.HELLO,
                sequence=sequence,
                session_id=session_id,
                sender_time_ms=monotonic_ms(),
                flags=FrameFlags.ACK_REQUIRED,
            )
            started = time.monotonic()
            sock.sendto(hello, destination)
            try:
                responder = wait_for_ack(
                    sock, session_id, sequence, started + args.timeout)
            except TimeoutError:
                print(f'第 {sequence} 次：超时（目标 {args.target}:{COMMAND_PORT}）')
                return 2
            rtt_ms = (time.monotonic() - started) * 1000.0
            results_ms.append(rtt_ms)
            print(f'第 {sequence} 次：{responder} HELLO_ACK，RTT={rtt_ms:.2f} ms')
            time.sleep(0.10)

    print(
        f'链路通过：{len(results_ms)}/{args.count}，'
        f'RTT min/avg/max={min(results_ms):.2f}/'
        f'{sum(results_ms) / len(results_ms):.2f}/{max(results_ms):.2f} ms')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
