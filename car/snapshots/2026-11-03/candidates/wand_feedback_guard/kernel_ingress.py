"""Bounded, single-owner Linux receive timing. Never changes sample authority.

Kernel timestamps are diagnostic metadata only. MCU source time and every
existing freshness deadline remain authoritative. No receive thread is added.
"""
import array
import fcntl
import select
import socket
import struct
import time

SO_TIMESTAMPNS = 35  # Linux x86_64, the deployed host ABI.
SO_RXQ_OVFL = 40
FIONREAD = 0x541B
MAX_DRAIN_PACKETS = 32
MAX_DRAIN_S = .003


def ancillary_fields(items):
    stamps, drops = [], []
    for level, kind, value in items:
        if level == socket.SOL_SOCKET and kind == SO_TIMESTAMPNS and len(value) == 16:
            sec, ns = struct.unpack('=qq', value)
            if sec >= 0 and 0 <= ns < 10**9:
                stamps.append(sec * 10**9 + ns)
        elif level == socket.SOL_SOCKET and kind == SO_RXQ_OVFL and len(value) == 4:
            drops.append(struct.unpack('=I', value)[0])
    if len(stamps) != 1:
        raise OSError('missing/invalid/ambiguous kernel receive timestamp')
    return stamps[0], drops[-1] if drops else None


class KernelIngressSocket:
    def __init__(self, raw):
        self.raw = raw
        raw.setsockopt(socket.SOL_SOCKET, SO_TIMESTAMPNS, 1)
        raw.setsockopt(socket.SOL_SOCKET, SO_RXQ_OVFL, 1)
        self.in_drain = False
        self.last_receive = None
        self.error = ''
        self.received_count = self.budget_exhaustions = 0
        self.max_dispatch_ms = 0.

    def __getattr__(self, key):
        return getattr(self.raw, key)

    def begin_drain(self):
        if self.in_drain:
            raise RuntimeError('recursive receive drain')
        self.in_drain = True
        self.drain_count, self.drain_started = 0, time.monotonic()

    def end_drain(self):
        self.in_drain = False

    def recvfrom(self, size):
        if not self.in_drain:
            raise RuntimeError('receive requires a bounded drain')
        if (self.drain_count >= MAX_DRAIN_PACKETS
                or time.monotonic() - self.drain_started >= MAX_DRAIN_S):
            self.budget_exhaustions += 1
            raise BlockingIOError('bounded receive budget exhausted')
        data, ancillary, flags, peer = self.raw.recvmsg(size, 256)
        self.drain_count += 1
        self.received_count += 1
        steady, real = time.monotonic_ns(), time.time_ns()
        try:
            if flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
                raise OSError('truncated receive data or kernel metadata')
            kernel, drops = ancillary_fields(ancillary)
        except OSError as error:
            self.error = str(error)
            # Do not throw away the safety loop along with a bad datagram.
            raise BlockingIOError(self.error) from error
        delay = (real - kernel) / 1e6
        self.max_dispatch_ms = max(self.max_dispatch_ms, delay)
        self.last_receive = dict(kernel_real_ns=kernel, received_real_ns=real,
            received_mono_ns=steady, kernel_to_ros_ms=delay,
            realtime_monotonic_offset_ns=real-steady, rxq_overflow_total=drops,
            msg_flags=flags, peer=peer)
        return data, peer

    def peek(self):
        """Non-consuming evidence at first safety trip, not a recovery path."""
        result = dict(at_steady=time.monotonic(),
                      readable=bool(select.select([self.raw], [], [], 0)[0]))
        size = array.array('i', [0])
        fcntl.ioctl(self.raw.fileno(), FIONREAD, size, True)
        result['first_pending_datagram_bytes'] = size[0]
        try:
            data, items, flags, peer = self.raw.recvmsg(512, 256,
                socket.MSG_PEEK | socket.MSG_DONTWAIT)
            kernel, drops = ancillary_fields(items)
            result.update(available=True, kernel_real_ns=kernel,
                rxq_overflow_total=drops, flags=flags, peer=peer, raw_hex=data.hex())
        except BlockingIOError:
            result['available'] = False
        return result
