"""Local timing evidence only; no serial writes and no clock-sync assumptions."""
import re
import ipaddress
import threading
import time


def esp_ipv4_from_logs(lines):
    addresses = []
    for line in lines:
        match = re.search(r'Wi-Fi ready, IP=(\d+\.\d+\.\d+\.\d+)', line['text'])
        if match:
            address = ipaddress.IPv4Address(match.group(1))
            if address.is_unspecified or address.is_multicast or address.is_loopback:
                raise ValueError('Invalid ESP station address')
            addresses.append(str(address))
    if not addresses:
        raise ValueError('No ESP station address in COM startup log')
    return addresses[-1]


class SerialLogCapture:
    def __init__(self, port, origin):
        import serial
        self.device = serial.Serial(port=None, baudrate=115200, timeout=0.05,
                                    exclusive=True)
        self.device.dtr = self.device.rts = False
        self.device.port = port
        self.device.open()
        self.origin = origin
        self.lines = []
        self.error = None
        self.stopped = threading.Event()
        self.thread = threading.Thread(target=self._read, daemon=True)
        self.thread.start()

    def _read(self):
        pending = bytearray()
        try:
            while not self.stopped.is_set():
                pending.extend(self.device.read(4096))
                while b'\n' in pending:
                    line, _, pending = pending.partition(b'\n')
                    if len(self.lines) >= 2000:
                        raise RuntimeError('serial log line capacity exceeded')
                    self.lines.append(dict(host_time_s=time.monotonic()-self.origin,
                                           text=line.decode('utf-8', errors='replace')))
                if len(pending) > 8192:
                    raise RuntimeError('serial log line too long')
        except Exception as error:
            self.error = str(error)

    def finish(self):
        # Dump happens only after a fault is forwarded; allow it to finish.
        time.sleep(2)
        self.stopped.set()
        self.thread.join(timeout=0.5)
        self.device.close()
        return dict(lines=self.lines, error=self.error,
                    reader_stopped=not self.thread.is_alive())


def summarize_timing(sent, serial_lines):
    commands = [s for s in sent if s['kind'] == 0x10]
    gaps = [b['start_s']-a['start_s'] for a,b in zip(commands,commands[1:])
            if a['session_id'] == b['session_id']]
    events = []
    fault = None
    pattern = re.compile(r'TRACE_EVENT kind=(\d+) seq=(\d+) at_ms=(\d+) duration_ms=(\d+) detail=(\d+)')
    fault_pattern = re.compile(r'TRACE_FAULT sid=(\d+) esp_ms=(\d+) mcu_ms=(\d+) last_cmd=(\d+) fault=(\d+)')
    for line in serial_lines:
        fault_match = fault_pattern.search(line['text'])
        if fault_match:
            fault = dict(zip(('session_id','esp_ms','mcu_ms','last_command_sequence','fault_bits'),
                             map(int,fault_match.groups())))
        match = pattern.search(line['text'])
        if match:
            events.append(dict(zip(('kind','sequence','at_ms','duration_ms','detail'),
                                   map(int,match.groups()))))
    rx = [e for e in events if e['kind']==1]
    rx_gaps = [((b['at_ms']-a['at_ms']) & 0xffffffff) for a,b in zip(rx,rx[1:])]
    loops = [e for e in events if e['kind']==4]
    loop_gaps = [((b['at_ms']-a['at_ms']) & 0xffffffff) for a,b in zip(loops,loops[1:])]
    # A failed run may end BEFORE the next RX event: include the trailing
    # silence, not only gaps between two successful arrivals.
    trailing_rx = ((fault['esp_ms']-rx[-1]['at_ms']) & 0xffffffff) if fault and rx else None
    return dict(host_command_count=len(commands),
                host_max_command_send_gap_s=max(gaps,default=None),
                host_max_send_call_s=max((s['end_s']-s['start_s'] for s in sent),default=None),
                esp_events=events, esp_max_recent_accepted_rx_gap_ms=max(rx_gaps,default=None),
                esp_max_recent_loop_gap_ms=max(loop_gaps,default=None),
                esp_fault=fault, esp_rx_silence_at_fault_ms=trailing_rx,
                esp_max_recent_uart_write_ms=max((e['duration_ms'] for e in events if e['kind']==2),default=None),
                esp_max_recent_telemetry_send_ms=max((e['duration_ms'] for e in events if e['kind']==3),default=None),
                esp_dump_complete=any('TRACE_END' in line['text'] for line in serial_lines),
                clocks_synchronized=False,
                note='Compare same-device gaps and session/sequence IDs, not absolute cross-device timestamps. UART write duration is not proof of MCU reception.')
