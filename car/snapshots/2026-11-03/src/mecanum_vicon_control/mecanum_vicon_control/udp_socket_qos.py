"""Opt-in marking of this process's command socket, not system networking."""
import socket


def configure_low_latency_qos(sock):
    # Set SO_PRIORITY last: Linux may derive a priority when IP_TOS changes.
    # EF DSCP and local priority 6 request low-latency treatment. Actual AP/WMM
    # behavior must be measured; this is not a delivery or safety guarantee.
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xb8)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY, 6)
    values = dict(enabled=True,
        ip_tos=sock.getsockopt(socket.IPPROTO_IP, socket.IP_TOS),
        socket_priority=sock.getsockopt(socket.SOL_SOCKET, socket.SO_PRIORITY))
    if values['ip_tos'] != 0xb8 or values['socket_priority'] != 6:
        raise RuntimeError('Per-socket QoS readback mismatch')
    return values
