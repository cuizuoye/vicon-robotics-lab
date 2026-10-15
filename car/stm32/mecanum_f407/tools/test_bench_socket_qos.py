import socket
import unittest
from bench_socket_qos import configure_low_latency_qos


class FakeSocket:
    def __init__(self):
        self.values = {}
        self.calls = []
    def setsockopt(self, level, option, value):
        self.calls.append((level, option, value))
        self.values[level, option] = value
    def getsockopt(self, level, option):
        return self.values[level, option]


class SocketQoSTests(unittest.TestCase):
    def test_socket_only_and_order(self):
        sock = FakeSocket()
        self.assertEqual(configure_low_latency_qos(sock),
            dict(enabled=True, ip_tos=184, socket_priority=6))
        self.assertEqual(sock.calls, [(socket.IPPROTO_IP, socket.IP_TOS, 184),
            (socket.SOL_SOCKET, socket.SO_PRIORITY, 6)])

    def test_failure_not_suppressed_or_retried(self):
        sock = FakeSocket()
        def fail(*args):
            raise PermissionError('No socket marking permitted')
        sock.setsockopt = fail
        with self.assertRaises(PermissionError):
            configure_low_latency_qos(sock)
        self.assertEqual(sock.calls, [])

    def test_readback_mismatch_rejected(self):
        sock = FakeSocket()
        sock.getsockopt = lambda *args: 0
        with self.assertRaises(RuntimeError):
            configure_low_latency_qos(sock)


if __name__ == '__main__':
    unittest.main()
