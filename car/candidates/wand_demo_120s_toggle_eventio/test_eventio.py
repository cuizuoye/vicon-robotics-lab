"""Readiness callbacks and local pipes only; no physical/Internet or ROS I/O."""
import ast
import importlib.util
import json
import os
from pathlib import Path
import threading
import time
from types import SimpleNamespace as NS

import pytest
from support import ROOT,GENERATED
from mecanum_wand_4ws8_hardware.receive_event import ReceiveEventPump
from test_startup_clock import actual_bridge


def test_preprocessor_comparison_preserves_literal_and_token_semantics():
    script=ROOT/'candidates/wand_demo_120s_toggle_eventio/build_esp.py'
    spec=importlib.util.spec_from_file_location('eventio_build_test',script)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    normalise=module.stable_translation_unit
    assert normalise(' f( x,\n  y); ') == normalise('f( x, y);')
    assert normalise('char *s="a  b";') != normalise('char *s="a b";')
    assert normalise('x++ + y;') != normalise('x+ + + y;')
    assert normalise('1 e3') != normalise('1e3')
    assert normalise("int c=' '; ") != normalise("int c='x';")


@pytest.fixture(autouse=True)
def no_hardware(monkeypatch):
    import socket,rclpy
    def forbidden(*a,**k):pytest.fail('eventio test attempted real network/ROS initialization')
    monkeypatch.setattr(socket,'socket',forbidden);monkeypatch.setattr(rclpy,'init',forbidden)


def test_readiness_worker_never_reads_or_retriggers_until_executor_acknowledges():
    read,write=os.pipe();called=threading.Event();calls=[]
    guard=NS(trigger=lambda:(calls.append(threading.get_ident()),called.set()))
    pump=ReceiveEventPump(read,guard)
    try:
        os.write(write,b'x');assert called.wait(.5)
        time.sleep(.015)
        assert len(calls)==1 and calls[0]!=threading.get_ident()
        assert os.read(read,1)==b'x'  # Worker did not consume the byte.
        called.clear();pump.acknowledge();os.write(write,b'y');assert called.wait(.5)
        assert os.read(read,1)==b'y';pump.acknowledge()
    finally:
        assert pump.stop();os.close(read);os.close(write)


def test_receive_worker_stop_is_bounded_even_when_executor_never_dispatches():
    read,write=os.pipe();called=threading.Event()
    pump=ReceiveEventPump(read,NS(trigger=called.set))
    try:
        os.write(write,b'x');assert called.wait(.5)
        before=time.monotonic();assert pump.stop();assert time.monotonic()-before<.25
        assert os.read(read,1)==b'x'
    finally:os.close(read);os.close(write)


def test_receive_worker_bad_descriptor_notifies_main_executor_and_stops():
    called=threading.Event();pump=ReceiveEventPump(-1,NS(trigger=called.set))
    assert called.wait(.5) and pump.error=='ValueError'
    pump.acknowledge();assert pump.stop()


def test_actual_receive_callback_accepts_ack_without_waiting_next_50hz_tick(monkeypatch):
    h=actual_bridge(monkeypatch)
    h.step();assert len(h.socket.sent)==1
    # A 10ms ACK arrives halfway between two normal 20ms sender ticks.
    h.clock.now+=.010;h.socket.advance()
    acknowledgements=[]
    h.b.receive_pump=NS(error='',acknowledge=lambda:acknowledgements.append(True))
    h.b.receive_event()
    assert h.b.session.connected and h.b.sock.clock.qualified
    assert h.b.sock.clock.rtt_s<=.040 and acknowledgements==[True]
    assert len(h.socket.sent)==1 and not h.b.safety_enabled  # RX callback does not send CMD.


def test_actual_receive_pump_error_latches_and_cannot_clear_or_enable(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2);assert h.status()['ready']
    acknowledged=[]
    h.b.receive_pump=NS(error='OSError',acknowledge=lambda:acknowledged.append(True))
    h.b.receive_event();h.run(.2)
    assert h.b.local_stop_latched and h.b.sock.clock.fault=='RECEIVE_EVENT_PUMP_FAILED'
    assert acknowledged and not h.status()['ready']


def test_safety_timer_detects_worker_error_even_if_guard_trigger_itself_is_broken(monkeypatch):
    h=actual_bridge(monkeypatch);h.run(1.2)
    h.b.receive_pump=NS(error='GUARD_TRIGGER_RuntimeError',acknowledge=lambda:None)
    h.b.safety_tick()
    assert h.b.local_stop_latched and h.b.sock.clock.fault=='RECEIVE_EVENT_PUMP_FAILED'


def test_event_io_does_not_modify_any_of_the_original_safety_decisions():
    old=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle_clockfix/ready/runtime/src'
    hardware='mecanum_wand_4ws8_hardware'
    for p in old.rglob('*'):
        if not p.is_file() or any(k=='__pycache__' or k.endswith('.egg-info') for k in p.parts):continue
        relative=p.relative_to(old)
        if str(relative)!=f'{hardware}/{hardware}/nodes.py':
            assert (GENERATED/relative).read_bytes()==p.read_bytes(),str(relative)
    def methods(p):
        cls=next(n for n in ast.parse(p.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='HardwareBridge')
        return {n.name:n for n in cls.body if isinstance(n,ast.FunctionDef)}
    a=methods(old/hardware/hardware/'nodes.py');b=methods(GENERATED/hardware/hardware/'nodes.py')
    for name in a:
        if name not in ('__init__','destroy_node'):assert ast.dump(a[name])==ast.dump(b[name]),name


def test_generated_esp_changes_only_waits_and_not_wire_or_watchdogs():
    out=Path(os.environ['EVENTIO_OUTPUT']);project=out/'esp_project'
    original=ROOT/'esp32/mecanum_wifi_bridge'
    for name in ('mc_protocol.c','mc_protocol.h','mc_uart_packet.h','mc_uart_consumer.h','mc_wifi_ap.h'):
        assert (project/'main'/name).read_bytes()==(original/'main'/name).read_bytes()
    config=(out/'sdkconfig.eventio').read_text()
    for expected in ('CONFIG_MC_EVENT_DRIVEN_FORWARD=y','CONFIG_MC_WIFI_SOFTAP=y',
        'CONFIG_MC_UDP_COMMAND_TIMEOUT_MS=750','CONFIG_MC_UART_BAUD_RATE=921600',
        'CONFIG_MC_UART_TX_GPIO=17','CONFIG_MC_UART_RX_GPIO=18'):
        assert expected in config
    text=(project/'main/app_main.c').read_text();old=(original/'main/app_main.c').read_text()
    # Keep these entire sensitive blocks byte-identical, not merely names.
    for begin,end in [('static void send_timeout_disable(', '\n\nstatic bool same_ipv4_host('),
                      ('static bool same_ipv4_host(', '\n\nstatic bool send_hello_ack(')]:
        assert text[text.index(begin):text.index(end)]==old[old.index(begin):old.index(end)]
    assert 'portMAX_DELAY' in text and 'mc_forward_wait(socket_fd, s_uart_wakeup_fd)' in text
    assert 'chunk, 1u, portMAX_DELAY' in text
