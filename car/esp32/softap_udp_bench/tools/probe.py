#!/usr/bin/env python3
"""Diagnostic SAP1/SAR1 only. No MC-Link, UART, network mutations or motor API."""
import argparse
import json
import math
import secrets
import select
import socket
import struct
import time
from pathlib import Path

REQUEST = struct.Struct('<4sBBHIIQQ')
REPLY = struct.Struct('<4sBBHIIQQQIIQQ')


def request(seq, session, host_ns):
    if not 0 < seq <= 0xffffffff or not 0 < session <= 0xffffffff:
        raise ValueError('Nonzero uint32 sequence/session required')
    return REQUEST.pack(b'SAP1', 1, 1, 0, seq, session, host_ns, 0)


def reply(data, session, sent):
    if len(data) != REPLY.size:
        raise ValueError('Reply length')
    magic, ver, kind, reserved, seq, sid, host, rx, tx, boot, count, gap, tail = REPLY.unpack(data)
    if (magic, ver, kind, reserved, sid, tail) != (b'SAR1', 1, 2, 0, session, 0):
        raise ValueError('Reply identity/schema')
    if seq not in sent or sent[seq] != host or tx < rx:
        raise ValueError('Reply echo/timestamp')
    return dict(seq=seq, host_ns=host, esp_rx_us=rx, esp_tx_us=tx,
                boot=boot, accepted=count, esp_max_gap_us=gap)


def stats(values):
    if not values:
        return dict(count=0, max_ms=None, p99_ms=None, over_100ms=0, at_least_150ms=0)
    ordered = sorted(values)
    return dict(count=len(values), max_ms=max(values),
                p99_ms=ordered[max(0, math.ceil(.99 * len(ordered)) - 1)],
                over_100ms=sum(x > 100 for x in values),
                at_least_150ms=sum(x >= 150 for x in values))


def run(args):
    sent, rows, seen, boots = {}, [], set(), set()
    bad = duplicates = reorders = send_errors = skipped = 0
    last_seq = seq = 0
    session = secrets.randbelow(0xffffffff) + 1
    fd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Explicit per-socket interface; never fall back to the internet interface.
    fd.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE, args.interface.encode() + b'\0')
    fd.bind((args.local_ip, 0))
    fd.setblocking(False)
    peer = ('192.168.50.1', 46010)
    start = time.monotonic_ns()
    period = 20_000_000
    due = start
    end = start + int(args.seconds * 1e9)
    interrupted = False
    try:
        while time.monotonic_ns() < end + 1_000_000_000:
            now = time.monotonic_ns()
            if now < end and now >= due:
                missed = (now - due) // period
                skipped += missed
                due += (missed + 1) * period
                seq += 1
                try:
                    fd.sendto(request(seq, session, now), peer)
                    sent[seq] = now
                except OSError:
                    send_errors += 1
            readable, _, _ = select.select([fd], [], [], .002)
            if readable:
                for _ in range(128):
                    try:
                        data, sender = fd.recvfrom(256)
                    except BlockingIOError:
                        break
                    received = time.monotonic_ns()
                    if sender != peer:
                        bad += 1
                        continue
                    try:
                        row = reply(data, session, sent)
                    except ValueError:
                        bad += 1
                        continue
                    if row['seq'] in seen:
                        duplicates += 1
                        continue
                    if row['seq'] < last_seq:
                        reorders += 1
                    last_seq = max(last_seq, row['seq'])
                    seen.add(row['seq']); boots.add(row['boot'])
                    row['host_rx_ns'] = received
                    row['rtt_ms'] = (received - row['host_ns']) / 1e6
                    rows.append(row)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        fd.close()
    by_seq = sorted(rows, key=lambda r: r['seq'])
    rx_gaps = [(b['esp_rx_us'] - a['esp_rx_us']) / 1000
               for a, b in zip(by_seq, by_seq[1:])
               if b['seq'] == a['seq'] + 1 and b['boot'] == a['boot']
               and b['esp_rx_us'] >= a['esp_rx_us']]
    ordered_sent = sorted(sent.items())
    tx_gaps = [(b[1] - a[1]) / 1e6 for a, b in zip(ordered_sent, ordered_sent[1:])]
    summary = dict(scope='Isolated SoftAP diagnostic, no MCU or motor acceptance',
        interface=args.interface, local_ip=args.local_ip, peer=peer, session=session,
        requested_seconds=args.seconds, observed_seconds=(time.monotonic_ns()-start)/1e9,
        interrupted=interrupted, sent=len(sent), unique_replies=len(seen),
        missing_replies=len(sent)-len(seen), invalid=bad, duplicates=duplicates,
        reordered=reorders, send_errors=send_errors, skipped_schedule_slots=skipped,
        boot_ids=sorted(boots), host_send_gap=stats(tx_gaps),
        rtt=stats([r['rtt_ms'] for r in rows]),
        esp_gap_for_consecutive_replied_sequences=stats(rx_gaps),
        esp_reported_max_gap_ms=max((r['esp_max_gap_us']/1000 for r in rows),default=None),
        limitation='RTT is not one-way delay; ESP gaps include host pacing. Missing replies may be forward or reverse loss. No clock synchronization assumed.',
        rows=rows)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, indent=2)
    print(json.dumps({k:v for k,v in summary.items() if k!='rows'}, indent=2))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface', required=True)
    p.add_argument('--local-ip', required=True)
    p.add_argument('--seconds', type=float, default=30)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 600:
        p.error('seconds must be 1..600')
    if args.output.exists():
        p.error('output exists; preserve previous evidence')
    run(args)


if __name__ == '__main__':
    main()
