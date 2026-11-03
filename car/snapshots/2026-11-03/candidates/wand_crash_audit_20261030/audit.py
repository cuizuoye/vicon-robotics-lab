"""Offline crash evidence only. No ROS, X11 connection, network, or actuation."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import statistics
import subprocess
from datetime import datetime, timezone, timedelta

ROOT = Path('/home/PRIMARY_USER/vicon_mecanum_ws')
FAILED = ROOT/'docs/wand_sim12_2026-10-30/real_sign_20261030_150437_35d010'
OLD_BOOT = '7a58f799e0404c1bbb12a89a106645de'
CST = timezone(timedelta(hours=8))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')


def load_trace(path):
    raw = path.read_bytes()
    rows, offset, anomalies = [], 0, []
    for number, line in enumerate(raw.splitlines(keepends=True), 1):
        try:
            row = json.loads(line)
        except (ValueError, UnicodeError) as error:
            # Only an all-NUL final crash suffix is acceptable for analysis.
            # Do not strip NULs inside valid-looking data or repair source bytes.
            if offset+len(line) == len(raw) and line and set(line) == {0}:
                anomalies.append(dict(line=number, offset=offset, bytes=len(line),
                                      kind='all_NUL_final_suffix', error=type(error).__name__))
                break
            raise ValueError(f'non-tail corruption at line {number}, byte {offset}') from error
        if not isinstance(row, dict):
            raise ValueError('trace row is not an object')
        rows.append(row)
        offset += len(line)
    if not rows:
        raise ValueError('no valid trace rows')
    return rows, dict(bytes=len(raw), valid_rows=len(rows), anomalies=anomalies,
                      source_sha256=digest(path), source_complete=not anomalies)


def analyze(session):
    rows, integrity = load_trace(session/'TRACE.jsonl')
    heartbeat = json.loads((session/'ui/HEARTBEAT.json').read_text())
    pairs = [r['supervisor'] for r in rows if r.get('supervisor')]
    offsets = [s['issued_ros']-s['issued_steady'] for s in pairs]
    offset = statistics.median(offsets)

    def event(row):
        return dict(steady=row['t'], wall=datetime.fromtimestamp(row['t']+offset, CST).isoformat(timespec='milliseconds'),
                    state=row.get('state'), reason=row.get('reason'),
                    gate_fault=row.get('gate', {}).get('fault'),
                    gate_output_zero=row.get('gate', {}).get('output_zero'),
                    stop_confirmed=row.get('supervisor', {}).get('stop_confirmed'),
                    command=row.get('command'), velocity=row.get('velocity'))

    active = next(r for r in rows if r.get('state') == 'ACTIVE')
    stop = next(r for r in rows if r['t'] > active['t'] and r.get('state') == 'STOPPING')
    confirmed = next(r for r in rows if r['t'] > stop['t'] and
                     r.get('supervisor', {}).get('stop_confirmed') and r.get('state') != 'ACTIVE')
    after = [r for r in rows if r['t'] >= stop['t']]
    hb_event = dict(steady=heartbeat['t'], wall=datetime.fromtimestamp(heartbeat['t']+offset, CST).isoformat(timespec='milliseconds'),
                    note='last persisted heartbeat; file written at >0.2s intervals, not last published operator lease')
    near = [r for r in rows if heartbeat['t']-.2 <= r['t'] <= stop['t']]
    ages = [r['supervisor']['pose_source_ages_s']['wand'] for r in near]
    return dict(
        trace_integrity=integrity, session_files_missing=[n for n in ('RESULT.json','ui/RESULT.json','PROTECTED.json') if not (session/n).exists()],
        wall_conversion=dict(offset_s=offset, max_offset_deviation_s=max(abs(x-offset) for x in offsets)),
        first_trace=event(rows[0]), active=event(active), last_persisted_ui_heartbeat=hb_event,
        first_observed_stop=event(stop), first_observed_fresh_stop_confirmation=event(confirmed), last_trace=event(rows[-1]),
        active_to_last_ui_record_s=heartbeat['t']-active['t'],
        last_ui_record_to_observed_revoke_s=stop['t']-heartbeat['t'],
        observed_revoke_to_stop_confirmation_s=confirmed['t']-stop['t'],
        trace_continuation_after_last_ui_record_s=rows[-1]['t']-heartbeat['t'],
        max_observer_gap_s=max(b['t']-a['t'] for a,b in zip(rows,rows[1:])),
        wand_age_near_incident_s=dict(min=min(ages),max=max(ages)),
        any_hardware_output=any(r.get('hardware_output') for r in rows),
        policy_faults=sorted({r['policy_fault'] for r in rows if r.get('policy_fault')}),
        all_saved_commands_zero_after_revoke=all(all(v == 0 for v in r['command'].values()) for r in after),
        any_saved_active_after_revoke=any(r.get('state') == 'ACTIVE' for r in after),
        car_quality_failures=max(r.get('supervisor',{}).get('car_pose_quality',{}).get('failures',0) for r in rows),
        classification='runtime desktop/UI freeze unresolved; operator-timeout stop observed; not full acceptance',
        timing_limitations=['~20ms trace sampling; status and gate are asynchronous snapshots',
                            'last UI file is not last operator lease; cannot derive 150ms deadline accuracy from file delta',
                            'no incident-time UI/X11 stack, per-process resources, or GPU telemetry captured'],
    )


def collect(output):
    output.mkdir(parents=True, exist_ok=False)
    snapshot = output/'failed_session'
    shutil.copytree(FAILED, snapshot)
    manifest = {}
    for source in sorted(p for p in FAILED.rglob('*') if p.is_file()):
        relative = source.relative_to(FAILED)
        original, copied = digest(source), digest(snapshot/relative)
        if original != copied:
            raise ValueError('session copy mismatch: '+str(relative))
        manifest[str(relative)] = dict(sha256=original, bytes=source.stat().st_size)
    save(output/'SESSION_SHA256.json',manifest)
    xorg = Path('/home/PRIMARY_USER/.local/share/xorg/Xorg.1.log.old')
    if xorg.exists():
        shutil.copy2(xorg,output/'Xorg.previous.log')
        save(output/'XORG_SOURCE.json',dict(path=str(xorg),sha256=digest(xorg),mtime=xorg.stat().st_mtime,
             limitation='previous-boot file, last modified before incident; not a freeze-time stack'))
    commands = {
        'boot_list':['journalctl','--list-boots','--no-pager'],
        'previous_kernel':['journalctl','-b',OLD_BOOT,'-k','--no-pager','-o','short-iso'],
        'gnome_incident':['journalctl','-b',OLD_BOOT,'_COMM=gnome-shell','--since','2026-10-30 15:00:00','--no-pager','-o','short-iso'],
        'xorg_incident':['journalctl','-b',OLD_BOOT,'_COMM=Xorg','--since','2026-10-30 15:00:00','--no-pager','-o','short-iso'],
        'coredump_journal':['journalctl','-b',OLD_BOOT,'SYSLOG_IDENTIFIER=systemd-coredump','--since','2026-10-30 15:00:00','--no-pager','-o','short-iso'],
        'memory_history':['sar','-r','-f','/var/log/sysstat/sa30','-s','14:50:00','-e','15:10:00'],
        'cpu_history':['sar','-u','-f','/var/log/sysstat/sa30','-s','14:50:00','-e','15:10:00'],
        'current_disk':['df','-h',str(ROOT)],
    }
    captures = {}
    for name, command in commands.items():
        try:
            result = subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=20)
            (output/(name+'.txt')).write_bytes(result.stdout+result.stderr)
            captures[name] = dict(command=command,returncode=result.returncode)
        except (OSError,subprocess.TimeoutExpired) as error:
            captures[name] = dict(command=command,error=repr(error))
    save(output/'CAPTURES.json',captures)
    result = analyze(snapshot)
    prepared = json.loads((ROOT/'docs/wand_sim12_2026-10-30/PREPARED.json').read_text())
    host = json.loads((ROOT/'docs/wand_4wsb_host_2026-10-30/prepared_v9/HOST_MANIFEST.json').read_text())
    expected = dict(prepared['sources'])
    for group in ('protected','runtime_hashes','source_hashes'):
        expected.update(host[group])
    expected[host['firmware']['path']] = host['firmware']['sha256']
    expected[str(ROOT/'tools/start_car_sync_4wsb.sh')] = host['file_checker_sha256']
    mismatches = [p for p,h in expected.items() if not Path(p).is_file() or digest(Path(p)) != h]
    result['protected_sources_rechecked'] = dict(count=len(expected),mismatches=mismatches)
    save(output/'ASSESSMENT.json',result)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=collect(args.output)
    print(json.dumps(result,ensure_ascii=False,indent=2))
