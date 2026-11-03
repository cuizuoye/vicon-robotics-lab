"""Offline/local-UNIX only. No robot/Internet socket, ROS init, or driver writes."""
import socket
import struct

import pytest
from trace_eventio_unarmed_20261026 import (
    decode_ancillary,TimestampedZeroSocket,peek_queue,SO_TIMESTAMPNS,SO_RXQ_OVFL,
    ESP,decode_frame,MAX_WINDOW_S,RX_TAIL_S)
from verify_eventio_unarmed_20261026 import offline_check,M,encode_frame

def test_original_zero_only_guard_and_failure_labels():offline_check()

def test_kernel_timestamp_and_drop_counter_exact_decode():
    assert decode_ancillary([(socket.SOL_SOCKET,SO_TIMESTAMPNS,struct.pack('=qq',10,123)),
                            (socket.SOL_SOCKET,SO_RXQ_OVFL,struct.pack('=I',7))])==(10000000123,7)
    with pytest.raises(ValueError):decode_ancillary([])
    with pytest.raises(ValueError):decode_ancillary([(socket.SOL_SOCKET,SO_TIMESTAMPNS,b'x')])

def test_permanent_tx_stop_rejects_even_safe_hello_and_does_not_call_underlying_socket():
    class Base:
        def sendto(self,*a):pytest.fail('stopped socket reached hardware send')
    wire=TimestampedZeroSocket(Base());wire.stop_tx();first=wire.stop_mono_ns;wire.stop_tx()
    assert wire.stop_mono_ns==first
    with pytest.raises(OSError):wire.sendto(encode_frame(M.HELLO,1,123,10),ESP)
    assert not wire.sent

def test_queue_peek_uses_timestamp_without_consuming_local_datagram():
    left,right=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
    try:
        left.setsockopt(socket.SOL_SOCKET,SO_TIMESTAMPNS,1);left.setblocking(False)
        assert not peek_queue(left)['datagram_available']
        raw=encode_frame(M.HELLO_ACK,1,123,10)
        right.send(raw)
        one=peek_queue(left);two=peek_queue(left)
        assert one['datagram_available'] and one['first_pending_datagram_bytes']==len(raw)
        assert one['frame']==two['frame'] and one['kernel_real_ns']==two['kernel_real_ns']
        assert left.recv(1024)==raw
        assert not peek_queue(left)['datagram_available']
    finally:left.close();right.close()

def test_capture_window_is_finite_and_tail_is_receive_only_duration():
    assert MAX_WINDOW_S==60 and RX_TAIL_S==.5
