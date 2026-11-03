#!/usr/bin/env python3
"""Recorded renderer/X11 comparison. Never construct a ROS node or car link.

Run with the system Python after sourcing /opt/ros/jazzy/setup.bash (imports
only). All original sources and recordings remain read-only. No live ROS/DDS
traffic is replayed; this isolates drawing/X11 from the original control load.
"""
import argparse
import builtins
from collections import Counter, deque
from datetime import datetime
import faulthandler
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

BASE = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
PREPARED = BASE / 'docs/wand_feedback_300ms_startup_2026-10-27/prepared'
SOURCE = PREPARED / 'runtime/src'
HERE = Path(__file__).resolve().parent
SESSIONS = ('oneclick_session_ejw7_ctw', 'oneclick_session_chibibon')
TOPICS = {'car_pose', 'wand_pose', 'supervisor_status', 'gate_status'}
STATUS_FIELDS = {'state', 'reason', 'gate_ready', 'hard_fault', 'stop_confirmed',
                 'ready_to_start', 'input_health', 'recovery_pending',
                 'mapped_goal', 'goal', 'anchors', 'pilot'}
GATE_FIELDS = {'fault', 'hard_fault', 'output_zero', 'speed_mps'}


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def protected_hashes():
    paths = list(SOURCE.rglob('*.py'))
    paths += [BASE / 'tools/start_car_tracking_300ms_ready.sh',
              PREPARED / 'oneclick_app.py']
    return {str(p): sha(p) for p in sorted(paths) if p.is_file()}


def selected_data(topic, data):
    """Drop packet contents, lease identities and transport authority fields."""
    if topic in ('car_pose', 'wand_pose'):
        return {k: data[k] for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')}
    fields = STATUS_FIELDS if topic == 'supervisor_status' else GATE_FIELDS
    return {k: v for k, v in data.items() if k in fields}


def extract(output):
    events, segments, counts = [], [], Counter()
    offset = 0.
    for session in SESSIONS:
        path = PREPARED / session / 'observation.jsonl'
        first = last = None
        selected = 0
        truncated_tail = []
        with path.open() as f:
            for line_number, line in enumerate(f, 1):
                try:
                    row = json.loads(line)
                except json.JSONDecodeError:
                    # Power-loss files can end in zero-filled unwritten bytes.
                    # Only ignore a terminal NUL/whitespace suffix, never a bad
                    # interior frame or partially plausible JSON silently.
                    if line.strip('\x00\r\n\t '):
                        raise
                    truncated_tail.append(dict(line=line_number, bytes=len(line.encode()),
                                               nul_bytes=line.count('\x00')))
                    continue
                if truncated_tail:
                    raise ValueError('corrupt interior record before valid data')
                stamp = row['received_steady']
                if first is None:
                    first = stamp
                last = stamp
                if row['topic'] not in TOPICS:
                    continue
                data = row['data']
                if isinstance(data, str):
                    data = json.loads(data)
                events.append([offset + stamp - first, row['topic'],
                               selected_data(row['topic'], data)])
                counts[row['topic']] += 1
                selected += 1
        duration = last - first
        segments.append(dict(session=session, source_sha256=sha(path),
                             source_bytes=path.stat().st_size, start_s=offset,
                             duration_s=duration, selected_events=selected,
                             excluded_unwritten_tail=truncated_tail))
        offset += duration + 1.
    manifest = dict(created_at=datetime.now().astimezone().isoformat(),
                    playback_rate=1., duration_s=offset,
                    segments=segments, topic_counts=dict(counts),
                    renderer_only=True, live_ros=False, live_hardware=False)
    with (output / 'renderer_events.jsonl').open('w') as f:
        for event in events:
            f.write(json.dumps(event, separators=(',', ':')) + '\n')
    dump(output / 'dataset.json', manifest)
    return manifest


def deny_live_io():
    """Fail closed even if a future imported UI accidentally calls live code."""
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    attempts = []

    def reject(*args, **kwargs):
        attempts.append('live_io_rejected')
        raise RuntimeError('READONLY REPLAY: ROS/socket/subprocess access prohibited')

    rclpy.init = reject
    Node.__init__ = reject
    SingleThreadedExecutor.__init__ = reject
    socket.socket = reject
    subprocess.Popen = reject
    original_open = builtins.open

    def safe_open(file, *args, **kwargs):
        name = str(file)
        if name.startswith(('/dev/tty', '/dev/serial', '/dev/bus/usb')):
            return reject()
        return original_open(file, *args, **kwargs)

    builtins.open = safe_open
    return attempts


def load_ui():
    for parent in (BASE / 'src', SOURCE):
        for package in parent.iterdir():
            if package.is_dir():
                sys.path.insert(0, str(package))
    from mecanum_wand_4ws8_control import operator_console as original
    return original


def run_gui(args):
    original = load_ui()
    attempts = deny_live_io()
    import tkinter as tk
    from mecanum_wand_4ws8_control.core import Pose

    output = Path(args.output)
    faults = (output / (args.mode + '_python_stacks.log')).open('w', buffering=1)
    faulthandler.enable(file=faults)
    faulthandler.register(signal.SIGUSR1, file=faults, all_threads=True)
    events = [json.loads(line) for line in (output / 'renderer_events.jsonl').open()]
    manifest = json.loads((output / 'dataset.json').read_text())
    metrics = (output / (args.mode + '_ui.jsonl')).open('w', buffering=1)
    model = original.OperatorModel()
    model.notice = '只读历史回放：没有ROS节点、没有通信端口、没有运动使能。Esc结束回放。'
    model.can_start = lambda now: False
    model.can_recover = lambda now: False
    model.instruction = lambda now: '只读回放 · 空格无效 · Esc仅关闭本回放 · 小车保持断电'

    class ReplayNode:
        input_kind = 'hardware'  # Select the accepted real-input renderer only.
        runtime = SimpleNamespace(config=SimpleNamespace(stage=4, max_speed_mps=.7,
                                  max_accel_mps2=.8), wand_reference_margin_m=0.)
        car = raw_wand = None

        def __init__(self):
            self.trace = deque(maxlen=500)
            self.index = self.consumed = self.resets = 0
            self.started = time.monotonic()
            self.segment = 0

        def pump(self):
            now = time.monotonic()
            elapsed = now - self.started
            if (self.segment + 1 < len(manifest['segments']) and
                    elapsed >= manifest['segments'][self.segment + 1]['start_s']):
                self.segment += 1
                self.trace.clear()
                self.car = self.raw_wand = None
                model.status, model.gate = {}, {}
                self.resets += 1
            while self.index < len(events) and events[self.index][0] <= elapsed:
                _, topic, data = events[self.index]
                self.index += 1
                self.consumed += 1
                if topic.endswith('_pose'):
                    qx, qy, qz, qw = (data[k] for k in ('qx', 'qy', 'qz', 'qw'))
                    yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
                    pose = Pose(data['x'], data['y'], data['z'], yaw)
                    if topic == 'car_pose':
                        self.car = pose
                        self.trace.append((pose.x, pose.y))
                    else:
                        self.raw_wand = pose
                elif topic == 'supervisor_status':
                    model.status, model.status_at = data, now
                else:
                    model.gate, model.gate_at = data, now
            # No model.tick(), receive callbacks, leases, services or executors.
            return elapsed

        def input_hint(self):
            return model.instruction(time.monotonic())

        def request(self, name):
            raise RuntimeError('READONLY REPLAY: control request prohibited: ' + name)

    node = ReplayNode()

    class ReadOnlyConsole(original.Console):
        def press(self, _=None): return 'break'
        def release(self, _=None): return 'break'
        def escape(self, _=None):
            self.close()
            return 'break'
        def schedule_focus(self): pass
        def check_focus(self): pass

        def draw(self, now):
            super().draw(now)  # Unmodified accepted map, text and large fonts.
            self.step_text.set(node.input_hint())
            for button in self.buttons.values():
                button.configure(state='disabled')

        def draw_tracking_map(self):
            wall, cpu = time.monotonic(), time.thread_time()
            super().draw_tracking_map()
            self.map_ms = (time.monotonic()-wall)*1000
            self.map_cpu_ms = (time.thread_time()-cpu)*1000

        def loop(self):
            self.loop_job = None
            if self.closing:
                return
            entered = time.monotonic()
            entered_cpu = time.thread_time()
            gap = entered - self.previous_enter if self.previous_enter else None
            cpu_between = (entered_cpu-self.previous_cpu)*1000 if self.previous_enter else None
            self.previous_enter = entered
            self.previous_cpu = entered_cpu
            key_start = time.monotonic()
            self.key_state.down()  # Same synchronous, read-only XQueryKeymap.
            key_end = time.monotonic()
            elapsed = node.pump()
            pump_end = time.monotonic()
            draw_ms = draw_cpu_ms = None
            self.map_ms = self.map_cpu_ms = None
            if pump_end - self.last_draw >= .033:
                cpu_start = time.thread_time()
                self.draw(pump_end)
                draw_cpu_ms = (time.thread_time()-cpu_start)*1000
                draw_ms = (time.monotonic() - pump_end) * 1000
                self.last_draw = pump_end
            row = dict(steady=time.monotonic(), elapsed_s=elapsed,
                       gap_ms=None if gap is None else gap*1000,
                       key_ms=(key_end-key_start)*1000,
                       pump_ms=(pump_end-key_end)*1000, draw_ms=draw_ms,
                       draw_cpu_ms=draw_cpu_ms, map_ms=self.map_ms,
                       map_cpu_ms=self.map_cpu_ms, cpu_between_loops_ms=cpu_between,
                       button_callback_count=sum(len(b._tclCommands or []) for b in self.buttons.values()),
                       trace_points=len(node.trace), items=len(self.map.find_all()),
                       root_size=[self.root.winfo_width(), self.root.winfo_height()],
                       canvas_size=[self.map.winfo_width(), self.map.winfo_height()],
                       events_consumed=node.consumed, control_io_attempts=len(attempts))
            metrics.write(json.dumps(row, separators=(',', ':')) + '\n')
            if elapsed >= manifest['duration_s']:
                self.close(completed=True)
            else:
                self.loop_job = self.root.after(20, self.loop)

        def close(self, completed=False):
            if self.closing:
                return
            self.closing = True
            if self.loop_job:
                self.root.after_cancel(self.loop_job)
            dump(output / (args.mode + '_completion.json'), dict(
                completed=completed, events_consumed=node.consumed,
                total_events=len(events), control_io_attempts=attempts,
                ros_initialized=False, hardware_socket_opened=False,
                motion_started=False, authority_ever_acquired=False))
            self.root.destroy()
            self.key_state.close()

    root = tk.Tk()
    app = ReadOnlyConsole(root, node, model)
    app.previous_enter = None
    root.title('只读诊断 · ' + args.mode + ' · 历史回放 · 无硬件输出')
    top = root.winfo_children()[0].winfo_children()
    top[0].configure(text='只读历史回放 · ' + args.mode + ' · 小车断电')
    top[1].configure(text='复用原绘图；无ROS/通信/使能 · 空格无效 · Esc结束本次回放')
    for button in app.buttons.values():
        button.configure(state='disabled', command=lambda: None)
    if args.mode == 'fullscreen':
        root.attributes('-fullscreen', True)
    # Playback starts after initial WM layout. Equal delay in both conditions.
    node.started = time.monotonic() + 1.
    signal.signal(signal.SIGTERM, lambda *_: app.close())
    root.mainloop()
    metrics.close()
    faults.close()


def run_probe(args):
    for package in SOURCE.iterdir():
        if package.is_dir(): sys.path.insert(0, str(package))
    from mecanum_wand_4ws8_control.x11_key_state import X11KeyState
    key = X11KeyState(os.environ['DISPLAY'])
    path = Path(args.output) / (args.mode + '_x11_probe.jsonl')
    with path.open('w', buffering=1) as f:
        while True:
            start = time.monotonic()
            key.down()
            end = time.monotonic()
            f.write(json.dumps(dict(steady=end, rtt_ms=(end-start)*1000)) + '\n')
            time.sleep(.1)


def proc_sample(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
        name = text[text.index('(')+1:text.rindex(')')]
        fields = text[text.rindex(')')+2:].split()
        status = Path(f'/proc/{pid}/status').read_text().splitlines()
        wanted = ('VmRSS:', 'VmSize:', 'Threads:', 'voluntary_ctxt_switches:',
                  'nonvoluntary_ctxt_switches:')
        stats = {line.split(':')[0]: line.split(':', 1)[1].strip()
                 for line in status if line.startswith(wanted)}
        return dict(pid=pid, name=name, state=fields[0],
                    cpu_ticks=int(fields[11])+int(fields[12]),
                    wchan=Path(f'/proc/{pid}/wchan').read_text().strip(), **stats)
    except (OSError, ValueError):
        return dict(pid=pid, missing=True)


def system_sample(pids):
    mem = {line.split(':')[0]: int(line.split()[1])
           for line in Path('/proc/meminfo').read_text().splitlines()}
    pressure = {}
    for kind in ('cpu', 'memory', 'io'):
        pressure[kind] = Path('/proc/pressure', kind).read_text().strip()
    temps = {}
    for path in Path('/sys/class/hwmon').glob('hwmon*/temp*_input'):
        try: temps[str(path)] = int(path.read_text())/1000
        except (OSError, ValueError): pass
    intel = {}
    for name in ('rps_act_freq_mhz', 'rps_cur_freq_mhz', 'rc6_residency_ms',
                 'throttle_reason_status', 'throttle_reason_thermal'):
        try: intel[name] = int((Path('/sys/class/drm/card1/gt/gt0')/name).read_text())
        except (OSError, ValueError): pass
    return dict(steady=time.monotonic(), mem_available_kib=mem['MemAvailable'],
                swap_used_kib=mem['SwapTotal']-mem['SwapFree'],
                cpu_stat=Path('/proc/stat').read_text().splitlines()[0],
                load=Path('/proc/loadavg').read_text().strip(), pressure=pressure,
                temperatures_c=temps, intel_gt=intel,
                processes={name:proc_sample(pid) for name,pid in pids.items()})


def terminate_owned(proc):
    if proc.poll() is not None:
        return
    # Only this exact child process, never Xorg, shell, receiver or desktop.
    proc.terminate()
    try: proc.wait(timeout=1.)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=1.)


def capture(command):
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=4.)
        # Never dump unrelated application logs, command lines or environments.
        return dict(command=command, returncode=done.returncode,
                    stdout=done.stdout[-40000:], stderr=done.stderr[-2000:])
    except (OSError, subprocess.TimeoutExpired) as error:
        return dict(command=command, error=type(error).__name__)


def named_pids():
    result = {}
    for path in Path('/proc').glob('[0-9]*/comm'):
        try:
            name = path.read_text().strip()
            if name in ('Xorg', 'gnome-shell'):
                result[name] = int(path.parent.name)
        except OSError: pass
    return result


def quantiles(values):
    values = sorted(v for v in values if v is not None)
    if not values: return {}
    return dict(n=len(values), median=values[len(values)//2],
                p95=values[min(len(values)-1, int(len(values)*.95))],
                p99=values[min(len(values)-1, int(len(values)*.99))], max=max(values))


def summarize_phase(output, mode, result):
    def rows(suffix):
        p = output / (mode + suffix)
        return [json.loads(x) for x in p.read_text().splitlines()] if p.exists() else []
    ui, probe, system = rows('_ui.jsonl'), rows('_x11_probe.jsonl'), rows('_system.jsonl')
    result['ui'] = {field: quantiles([row.get(field) for row in ui])
                    for field in ('gap_ms', 'key_ms', 'pump_ms', 'draw_ms')}
    result['x11_rtt_ms'] = quantiles([row['rtt_ms'] for row in probe])
    result['ui_gaps_over_100ms'] = sum((row.get('gap_ms') or 0)>100 for row in ui)
    result['sizes'] = sorted({tuple(row['root_size']) for row in ui})
    result['canvas_sizes'] = sorted({tuple(row['canvas_size']) for row in ui})
    result['max_items'] = max((row['items'] for row in ui), default=0)
    result['max_trace_points'] = max((row['trace_points'] for row in ui), default=0)
    result['rss_mib'] = {}
    result['cpu_percent_one_core'] = {}
    for name in ('ui', 'Xorg', 'gnome-shell'):
        samples = [s['processes'][name] for s in system if name in s['processes']]
        rss = [int(s['VmRSS'].split()[0])/1024 for s in samples if 'VmRSS' in s]
        result['rss_mib'][name] = quantiles(rss)
        if len(samples)>1 and 'cpu_ticks' in samples[-1] and 'cpu_ticks' in samples[0]:
            dt = system[-1]['steady']-system[0]['steady']
            result['cpu_percent_one_core'][name] = (
                (samples[-1]['cpu_ticks']-samples[0]['cpu_ticks']) / os.sysconf('SC_CLK_TCK') / dt * 100)
    result['min_mem_available_gib'] = min((s['mem_available_kib']/1024**2 for s in system), default=None)
    if ui:
        result['elapsed_s'] = ui[-1]['elapsed_s']
        result['frames_drawn'] = result['ui']['draw_ms'].get('n', 0)
        result['control_io_attempts'] = max(row['control_io_attempts'] for row in ui)
    completion = output / (mode + '_completion.json')
    result['completion'] = json.loads(completion.read_text()) if completion.exists() else None
    return result


def run_comparison(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    before = protected_hashes()
    dump(output / 'protected_before.json', before)
    dataset = extract(output)
    started_wall = datetime.now().astimezone().isoformat()
    env = dict(os.environ, DISPLAY=':1', XAUTHORITY='/run/user/1000/gdm/Xauthority',
               PYTHONDONTWRITEBYTECODE='1')
    env.pop('WAND_AUTOPREP_FLOW', None)
    # Current desktop/Xorg diagnostics; no xrandr, power or driver mutation.
    dump(output / 'desktop_before.json', capture(['env', 'DISPLAY=:1',
         'XAUTHORITY=/run/user/1000/gdm/Xauthority', 'xrandr', '--current']))
    dump(output / 'gpu_before.json', capture(['nvidia-smi',
         '--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,memory.used,power.draw', '--format=csv']))
    pids = named_pids()
    xorg_log = Path('/home/PRIMARY_USER/.local/share/xorg/Xorg.1.log')
    xorg_offset = xorg_log.stat().st_size if xorg_log.exists() else 0
    summary = dict(started_at=started_wall, dataset=dataset, phases=[],
                   scope='renderer/X11 replay, no live ROS/DDS/car link',
                   instrumentation_version='v2_thread_cpu_fixed_callback_registration',
                   diagnostic_sha256=sha(Path(__file__).resolve()),
                   watchdog_s=2., protections_unchanged=True)
    for mode in args.modes:
        print('PHASE_START ' + mode, flush=True)
        with (output/(mode+'_child.log')).open('w') as log, \
                (output/(mode+'_system.jsonl')).open('w', buffering=1) as systemlog, \
                (output/(mode+'_watchdog.jsonl')).open('w', buffering=1) as watchlog:
            child = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'gui',
                                      '--output', str(output), '--mode', mode],
                                     stdout=log, stderr=log, env=env, start_new_session=True)
            probe = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), 'probe',
                                      '--output', str(output), '--mode', mode],
                                     stdout=log, stderr=log, env=env, start_new_session=True)
            trial_start = last_system = time.monotonic()
            ui_path = output/(mode+'_ui.jsonl')
            probe_path = output/(mode+'_x11_probe.jsonl')
            stack_sent = False
            result = dict(mode=mode, termination='child_exit')
            try:
                while child.poll() is None:
                    now = time.monotonic()
                    if now-last_system >= .5:
                        systemlog.write(json.dumps(system_sample(dict(pids, ui=child.pid)))+'\n')
                        last_system = now
                    age = time.time()-ui_path.stat().st_mtime if ui_path.exists() else None
                    watchlog.write(json.dumps(dict(steady=now, ui_file_age_s=age,
                                                   probe_file_age_s=(time.time()-probe_path.stat().st_mtime
                                                        if probe_path.exists() else None),
                                                   probe_alive=probe.poll() is None))+'\n')
                    if age is not None and age>.5 and not stack_sent:
                        child.send_signal(signal.SIGUSR1)
                        stack_sent = True
                    if (age is not None and age>2.) or now-trial_start>dataset['duration_s']+20:
                        result['termination'] = 'watchdog_ui_stall_or_deadline'
                        break
                    time.sleep(.1)
            finally:
                terminate_owned(child)
                terminate_owned(probe)
            result['returncode'] = child.returncode
            result['stack_requested'] = stack_sent
        result = summarize_phase(output, mode, result)
        summary['phases'].append(result)
        dump(output/'summary.json', summary)
        print('PHASE_DONE '+json.dumps(result), flush=True)
        # Do not expose the desktop to a second fullscreen stress if one stalled.
        if (result['termination'] != 'child_exit' or child.returncode != 0
                or not result.get('completion',{}).get('completed')):
            break
    after = protected_hashes()
    dump(output/'protected_after.json', after)
    summary['original_sources_unchanged'] = before == after
    summary['protected_files'] = len(before)
    summary['ended_at'] = datetime.now().astimezone().isoformat()
    if xorg_log.exists():
        with xorg_log.open('rb') as f:
            f.seek(xorg_offset)
            (output/'xorg_increment.log').write_bytes(f.read(100000))
    dump(output/'gpu_after.json', capture(['nvidia-smi',
         '--query-gpu=name,driver_version,temperature.gpu,utilization.gpu,memory.used,power.draw', '--format=csv']))
    # Strict unit filters avoid unrelated app contents and secrets in the journal.
    for name, filters in [('kernel', ['-k']), ('gnome', ['_COMM=gnome-shell'])]:
        entry = capture(['journalctl', '-b', '--since', started_wall, '--no-pager',
                         '-o', 'short-iso', *filters])
        entry['stdout'] = '\n'.join(line[:500] for line in entry.get('stdout','').splitlines())
        dump(output/(name+'_journal.json'), entry)
    dump(output/'summary.json', summary)
    print('RESULT '+str(output/'summary.json'), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('compare', 'gui', 'probe'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--mode', choices=('window', 'fullscreen'))
    parser.add_argument('--modes', nargs='+', choices=('window', 'fullscreen'),
                        default=['window', 'fullscreen'])
    args = parser.parse_args()
    {'compare': run_comparison, 'gui': run_gui, 'probe': run_probe}[args.action](args)


if __name__ == '__main__':
    main()
