#!/usr/bin/env python3
"""One <=60s unarmed window plus read-only Linux WiFi/USB evidence.

Uses the verified transport diagnostic unchanged. Only adds a background GET
statistics reader and offline pairing analysis. Never selects/enables/clears,
never changes wireless/network/USB settings and never retries a failed window.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import tempfile
import threading
import time
from wifi_readonly import ReadOnlyWifi,constants,system_snapshot,GETS

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
HERE=Path(__file__).resolve().parent
PARENT=ROOT/'candidates/wand_feedback_guard/diagnose.py'
spec=importlib.util.spec_from_file_location('verified_zero_window',PARENT)
diagnostic=importlib.util.module_from_spec(spec);spec.loader.exec_module(diagnostic)


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def save(path,value):path.write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')


class Sampler:
    """Separate from ROS callbacks. Kernel GETs do not transmit WiFi probes."""
    def __init__(self,constants):
        self.constants=constants;self.samples=[];self.latest=None
        self.stop_event=threading.Event();self.thread=None
    def collect(self,wifi):
        row=dict(at=time.monotonic())
        try:row['wifi']=wifi.snapshot()
        except Exception as error:row['wifi_error']=str(error)
        try:row['system']=system_snapshot()
        except Exception as error:row['system_error']=str(error)
        row['done_at']=time.monotonic();self.samples.append(row);self.latest=row
    def run(self):
        wifi=None
        try:
            wifi=ReadOnlyWifi(self.constants)
            while not self.stop_event.is_set():
                started=time.monotonic();self.collect(wifi)
                self.stop_event.wait(max(.01,.5-(time.monotonic()-started)))
        except Exception as error:self.samples.append(dict(at=time.monotonic(),sampler_error=str(error)))
        finally:
            if wifi:wifi.close()
    def start(self):
        self.thread=threading.Thread(target=self.run,name='readonly_wifi_stats',daemon=True);self.thread.start()
    def stop(self):
        self.stop_event.set()
        if self.thread:self.thread.join(timeout=3)
        if self.thread and self.thread.is_alive():raise RuntimeError('read-only sampler shutdown deadline')


def network_state():
    result={}
    for name,args in (
        ('default_route',['ip','-4','route','show','default']),
        ('tenda_ip',['ip','-4','address','show','dev','wlx000000000000']),
        ('tenda_saved_policy',['nmcli','-g','802-11-wireless.powersave,ipv4.method,ipv4.never-default,ipv4.gateway',
            'connection','show','79614b9e-7213-452a-be72-34f9cdf1e9aa']),
        ('driver',['ethtool','-i','wlx000000000000'])):
        p=subprocess.run(args,capture_output=True,text=True,timeout=3)
        result[name]=p.stdout.strip() if p.returncode==0 else 'UNAVAILABLE: '+p.stderr.strip()
    return result


def analyse(folder,stats):
    """All packet comparisons are same-session, same-native-sample only."""
    rows=json.loads((folder/'kernel_receive.json').read_text())
    result=json.loads((folder/'RESULT.json').read_text())
    native=[json.loads(s) for s in (folder/'native_clock.jsonl').read_text().splitlines()]
    accepted={(r['session'],r['sequence']):r['accepted_sample_steady'] for r in native}
    tx={r['sequence']:r for r in json.loads((folder/'outgoing.json').read_text()) if r['kind']=='CMD_VEL'}
    pairs={};echo=[]
    for r in rows:
        if r.get('kind') not in ('WHEEL_TELEMETRY','LINK_STATUS'):continue
        sample=accepted.get((r['session'],r['sequence']))
        enriched=dict(r)
        if sample is not None:
            kernel_steady=(r['kernel_real_ns']-r['realtime_monotonic_offset_ns'])/1e9
            enriched['source_age_at_kernel_ms']=(kernel_steady-sample)*1000
            enriched['source_age_at_ros_ms']=(r['received_mono_ns']/1e9-sample)*1000
        group=pairs.setdefault((r['session'],r['source_ms']),{})
        # Duplicate frame kinds cannot replace a prior pair member silently.
        if r['kind'] in group:raise ValueError('duplicate same-session native sample in pair analysis')
        group[r['kind']]=enriched
        if r['kind']=='WHEEL_TELEMETRY':
            telemetry=diagnostic.unpack_wheel_telemetry(bytes.fromhex(r['payload_hex']))
            command=tx.get(telemetry.last_command_sequence)
            if command and sample is not None:
                # A latest-command echo, NOT an ESP-internal residence time.
                # HELLO conservatively bounds the MCU-clock anchor uncertainty.
                lo=(sample-command['issued_steady'])*1000
                echo.append(dict(source_ms=r['source_ms'],last_command_sequence=telemetry.last_command_sequence,
                    sample_minus_last_command_issue_earliest_ms=lo,
                    sample_minus_last_command_issue_upper_ms=lo+result['trip']['clock']['hello_rtt_s']*1000,
                    after_tx_stop=r['after_tx_stop']))
    paired=[]
    for (sid,tick),group in pairs.items():
        if len(group)!=2:continue
        w,d=group['WHEEL_TELEMETRY'],group['LINK_STATUS']
        paired.append(dict(session=sid,source_ms=tick,
            diagnostic_minus_wheel_kernel_ms=(d['kernel_real_ns']-w['kernel_real_ns'])/1e6,
            diagnostic_minus_wheel_ros_ms=(d['received_mono_ns']-w['received_mono_ns'])/1e6,
            wheel_sequence=w['sequence'],diagnostic_sequence=d['sequence'],
            wheel_after_tx_stop=w['after_tx_stop'],diagnostic_after_tx_stop=d['after_tx_stop'],
            wheel_source_age_at_kernel_ms=w.get('source_age_at_kernel_ms'),
            diagnostic_source_age_at_kernel_ms=d.get('source_age_at_kernel_ms')))
    paired.sort(key=lambda r:abs(r['diagnostic_minus_wheel_kernel_ms']),reverse=True)
    stations=[s for r in stats for s in r.get('wifi',{}).get('stations',[])]
    changes={}
    if stations:
        for key in ('rx_packets','tx_packets','tx_retries','tx_failed','rx_drop_misc','beacon_loss','beacon_rx'):
            if key in stations[0] and key in stations[-1]:changes[key]=stations[-1][key]-stations[0][key]
    system=[r['system'] for r in stats if 'system' in r]
    usb=[r['usb'] for r in system]
    trip_at=result.get('trip',{}).get('at')
    around=[r for r in stats if trip_at is not None and abs(r['at']-trip_at)<1.]
    out=dict(protocol_window_completed=result.get('completed'),termination=result.get('error') or 'FINITE_WINDOW_COMPLETE',
        native_pair_count=len(paired),worst_same_sample_pairs=paired[:10],
        median_pair_kernel_separation_ms=statistics.median([r['diagnostic_minus_wheel_kernel_ms'] for r in paired]) if paired else None,
        wifi_signal_dbm_range=[min(s['signal'] for s in stations if 'signal' in s),max(s['signal'] for s in stations if 'signal' in s)] if any('signal' in s for s in stations) else None,
        actual_power_save_disabled_samples=sum(r.get('wifi',{}).get('power_save_disabled') is True for r in stats),
        wifi_statistics_samples=len(stats),station_counter_deltas=changes,
        counter_direction_caveat='TX_RETRIES/TX_FAILED refer to PC-to-ESP transmissions, not ESP-to-PC radio retries. Missing/sparse driver counters are not proof of loss-free delivery.',
        usb_statuses=sorted({s['runtime_status'] for s in usb if s.get('runtime_status')}),
        usb_suspended_time_before_after=[usb[0].get('runtime_suspended_time'),usb[-1].get('runtime_suspended_time')] if usb else None,
        stats_around_trip=around,
        last_command_echo_samples=len(echo),
        last_command_echo_upper_max_ms=max((r['sample_minus_last_command_issue_upper_ms'] for r in echo if not r['after_tx_stop']),default=None),
        kernel_to_ros_max_ms=result.get('kernel_to_ros_max_ms'),
        kernel_to_ros_median_ms=result.get('kernel_to_ros_median_ms'),
        queue_at_stop=result.get('trip',{}).get('queue_at_stop'),
        source_ages_at_stop_ms=result.get('trip',{}).get('source_ages_ms'),
        no_direct_esp_uart_enqueue_or_udp_send_timestamp=True,
        physical_component_root_cause_proven=False,protection_ms=200,motion_started=False)
    save(folder/'PAIRS.json',paired);save(folder/'COMMAND_ECHO.json',echo);save(folder/'RETURN_PATH_ANALYSIS.json',out)
    return out


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--confirm-unarmed-zero-only',action='store_true')
    ap.add_argument('--offline-check',action='store_true');args=ap.parse_args()
    if args.offline_check:diagnostic.offline_check();return 0
    if not args.confirm_unarmed_zero_only:ap.error('explicit unarmed-zero-only confirmation required')
    manifest=diagnostic.check(diagnostic.READY/'build_manifest.json')
    if not manifest.get('feedback_guard_passed'):raise ValueError('verified independent transport required')
    os.umask(0o077)
    parent=ROOT/'docs/wand_return_path_trace_2026-10-27';parent.mkdir(exist_ok=True)
    folder=Path(tempfile.mkdtemp(prefix='return_path_',dir=parent))
    source_hashes={str(p):sha(p) for p in (*HERE.glob('*.py'),HERE/'uapi_constants.c',PARENT)}
    save(folder/'AUTHORITY_AND_SCOPE.json',dict(
        user_request='继续定位ESP32转发和Tenda反馈回程；允许未使能零速诊断；不刷固件、不改网络、不影响演示',
        max_windows=1,max_transmit_window_s=60,first_protection_failure_stops_transmission=True,
        no_enable_clear_select_estop_retry=True,feedback_ms=200,source_hashes=source_hashes,
        readonly_kernel_gets=sorted(GETS)))
    before=network_state();save(folder/'NETWORK_BEFORE.json',before)
    c=constants(folder);save(folder/'UAPI_CONSTANTS.json',c)
    sampler=Sampler(c)
    base=diagnostic.KernelIngressSocket
    class ObservedIngress(base):
        def peek(self):
            r=super().peek();latest=sampler.latest
            r['last_readonly_wifi_sample']=latest
            r['wifi_sample_age_s']=None if latest is None else time.monotonic()-latest['done_at']
            return r
    diagnostic.KernelIngressSocket=ObservedIngress
    os.environ.update(ROS_AUTOMATIC_DISCOVERY_RANGE='OFF',ROS_LOG_DIR=str(folder/'ros_logs'))
    print('EVIDENCE='+str(folder),flush=True);sampler.start()
    try:
        result=diagnostic.live(folder)
    finally:
        sampler.stop();save(folder/'WIFI_USB_SAMPLES.json',sampler.samples)
        after=network_state();save(folder/'NETWORK_AFTER.json',after)
        save(folder/'PRESERVATION.json',dict(network_readback_unchanged=before==after,
            source_hashes_unchanged=all(sha(p)==v for p,v in source_hashes.items()),
            accepted_manifest_still_valid=bool(diagnostic.check(diagnostic.READY/'build_manifest.json')),
            firmware_written=False,network_settings_written=False,operator_ui_started=False))
    analysis=analyse(folder,sampler.samples)
    print(json.dumps({k:v for k,v in analysis.items() if k not in ('stats_around_trip','worst_same_sample_pairs')},ensure_ascii=False,indent=2),flush=True)
    return result


if __name__=='__main__':raise SystemExit(main())
