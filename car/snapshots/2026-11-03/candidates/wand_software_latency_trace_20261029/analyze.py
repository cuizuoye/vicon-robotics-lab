"""Exact identity joins, never nearest-timestamp/nearest-value matching."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import numpy as np


LABELS = {
    'local_source_to_raw_complete': 'PC源时间戳→六路输入完整',
    'raw_complete_to_relay': '六路完整→私有ROS转发',
    'relay_to_control_tick': '转发→控制计算起点',
    'control_tick_to_publish': '控制计算起点→控制发布',
    'control_publish_to_gate_receive': '控制发布→末级门回调',
    'gate_receive_to_accept': '末级门回调→同源绑定/接收检查完成',
    'gate_accept_to_sign': '接收检查完成→签名开始（含门定时器等待）',
    'sign_to_signed_publish': '签名开始→签名报文发布',
    'signed_publish_to_plant_receive': '签名发布→仿真执行器回调',
    'plant_receive_to_verified': '执行器回调→验签完成',
    'verified_to_model_step': '验签完成→首次模型积分完成',
    'model_step_to_snapshot': '被显示的模型状态→快照完成',
    'snapshot_to_draw': '快照完成→Tk绘图命令提交完成',
    'local_source_to_verified': 'PC源时间戳→验签完成',
    'local_source_to_model_step': 'PC源时间戳→首次模型积分完成',
    'local_source_to_car_draw': 'PC源时间戳→对应蓝车状态绘图提交',
    'local_source_to_goal_draw': 'PC源时间戳→对应橙色目标绘图提交',
    'control_publish_to_verified': '控制发布→验签完成',
    'control_publish_cost': 'ROS控制发布调用耗时',
    'signature_compute_cost': 'HMAC编码调用耗时',
    'signature_verify_cost': 'HMAC及权限/时序校验耗时',
    'draw_cost': 'Tk绘图函数耗时（非屏幕呈现）',
}


def key(value):
    return tuple(value) if value is not None else None


def stats(ns):
    values = np.asarray(ns, dtype=float)/1e6
    if not len(values):
        return None
    return dict(n=len(values), median_ms=float(np.median(values)),
                p95_ms=float(np.quantile(values, .95)), p99_ms=float(np.quantile(values, .99)),
                max_ms=float(max(values)), min_ms=float(min(values)))


def rows(path):
    with Path(path).open() as stream:
        for line in stream:
            if not line.endswith('\n'):
                raise ValueError('partial trace; close the console before analysis')
            yield json.loads(line)


def attribute(events):
    events = list(events)
    groups = defaultdict(list)
    for event in events:
        groups[event['kind']].append(event)
    def indexed(kind, field):
        result = {}
        for event in groups[kind]:
            identity = key(event.get(field))
            if identity is not None:
                if identity in result:
                    raise ValueError('duplicate trace identity: '+kind+' '+str(identity))
                result[identity] = event
        return result
    controls = indexed('control_publish', 'cid')
    receives = indexed('gate_receive', 'cid')
    accepts = indexed('gate_accept', 'cid')
    signatures = indexed('sign_encode', 'oid')
    signed = indexed('signed_publish', 'oid')
    plant_rx = indexed('plant_receive', 'oid')
    verified = indexed('verify_accept', 'oid')
    models = {e['model_seq']: e for e in groups['model_step']}
    snapshots = {e['snapshot_seq']: e for e in groups['snapshot']}
    sources = {e['sign']: e for e in groups['source_relay']}
    active = {cid: c for cid, c in controls.items() if c['state'] == 'ACTIVE'}
    values = defaultdict(list)
    invalid = []
    def add(name, begin, end):
        delta = end-begin
        if delta < 0:
            invalid.append(dict(metric=name, begin_ns=begin, end_ns=end))
        else:
            values[name].append(delta)
    # One controller frame can be signed repeatedly by the 100Hz gate. Count
    # first VERIFIED output, first APPLIED output, first DRAWN state separately.
    first_verified, first_model, first_car_draw, first_goal_draw = {}, {}, {}, {}
    model_by_output = {}
    for model in sorted(models.values(), key=lambda e: e['end_ns']):
        oid = key(model.get('oid'))
        if oid is not None:
            model_by_output.setdefault(oid, model)
    for oid, v in sorted(verified.items(), key=lambda item: item[1]['end_ns']):
        p = signed.get(oid)
        cid = key(p.get('cid')) if p else None
        if not v['applied'] or cid not in active:
            continue
        c = active[cid]
        if not p['permit'] or p['state'] != 'ACTIVE':
            continue
        first_verified.setdefault(cid, (oid, v))
        if oid in model_by_output:
            first_model.setdefault(cid, (oid, model_by_output[oid]))
    for draw in sorted(groups['draw'], key=lambda e: e['end_ns']):
        snapshot = snapshots.get(draw['snapshot_seq'])
        if not snapshot:
            continue
        if (draw['model_seq'], key(draw.get('oid')), key(draw.get('cid'))) != (
                snapshot['model_seq'], key(snapshot.get('oid')), key(snapshot.get('cid'))):
            raise ValueError('display snapshot identity mismatch')
        goal_cid = key(draw.get('cid'))
        if goal_cid in active:
            first_goal_draw.setdefault(goal_cid, (snapshot, draw))
        model = models.get(draw['model_seq'])
        if not model or key(model.get('oid')) != key(draw.get('oid')):
            if draw['model_seq'] is not None:
                raise ValueError('display model identity mismatch')
            continue
        oid = key(model.get('oid'))
        p = signed.get(oid)
        cid = key(p.get('cid')) if p else None
        if cid in active and oid in verified and verified[oid]['applied'] and p['permit']:
            first_car_draw.setdefault(cid, (oid, model, snapshot, draw))
            add('draw_cost', draw['begin_ns'], draw['end_ns'])
    used_sources = set()
    paths = []
    for cid, c in active.items():
        source = sources.get(c['sign'])
        receive, accept = receives.get(cid), accepts.get(cid)
        if not source or not receive or not accept or not accept['accepted']:
            continue
        if c['sign'] not in used_sources:
            used_sources.add(c['sign'])
            add('local_source_to_raw_complete', source['source_local_ns'], source['raw_complete_ns'])
            add('raw_complete_to_relay', source['raw_complete_ns'], source['begin_ns'])
            add('relay_to_control_tick', source['begin_ns'], c['issued_ns'])
        add('control_tick_to_publish', c['issued_ns'], c['begin_ns'])
        add('control_publish_cost', c['begin_ns'], c['end_ns'])
        add('control_publish_to_gate_receive', c['begin_ns'], receive['begin_ns'])
        add('gate_receive_to_accept', receive['begin_ns'], accept['end_ns'])
        if cid in first_verified:
            oid, v = first_verified[cid]
            p, sign, rx = signed[oid], signatures.get(oid), plant_rx.get(oid)
            if sign and key(sign.get('cid')) != cid:
                raise ValueError('signed output/controller identity mismatch')
            if sign:
                add('gate_accept_to_sign', accept['end_ns'], sign['begin_ns'])
                add('sign_to_signed_publish', sign['begin_ns'], p['begin_ns'])
                add('signature_compute_cost', sign['begin_ns'], sign['end_ns'])
            if rx:
                add('signed_publish_to_plant_receive', p['begin_ns'], rx['begin_ns'])
                add('plant_receive_to_verified', rx['begin_ns'], v['end_ns'])
            add('signature_verify_cost', v['begin_ns'], v['end_ns'])
            add('local_source_to_verified', source['source_local_ns'], v['end_ns'])
            add('control_publish_to_verified', c['begin_ns'], v['end_ns'])
        if cid in first_model:
            oid, model = first_model[cid]
            add('verified_to_model_step', verified[oid]['end_ns'], model['end_ns'])
            add('local_source_to_model_step', source['source_local_ns'], model['end_ns'])
        if cid in first_car_draw:
            oid, model, snapshot, draw = first_car_draw[cid]
            add('model_step_to_snapshot', model['end_ns'], snapshot['end_ns'])
            add('snapshot_to_draw', snapshot['end_ns'], draw['end_ns'])
            add('local_source_to_car_draw', source['source_local_ns'], draw['end_ns'])
            # Every point in this path is paired to the exact command used
            # by this displayed MODEL state, not an unrelated nearby timer.
            paths.append(dict(cid=list(cid), sign=c['sign'], oid=list(oid),
                              model_seq=model['model_seq'], snapshot_seq=snapshot['snapshot_seq'],
                              draw_seq=draw['draw_seq'], source_local_ns=source['source_local_ns'],
                              control_publish_ns=c['begin_ns'], verify_ns=verified[oid]['end_ns'],
                              model_ns=model['end_ns'], snapshot_ns=snapshot['end_ns'],
                              draw_ns=draw['end_ns']))
        if cid in first_goal_draw:
            _, draw = first_goal_draw[cid]
            add('local_source_to_goal_draw', source['source_local_ns'], draw['end_ns'])
    active_times = [c['issued_ns'] for c in active.values()]
    epochs = defaultdict(list)
    for cid, control in active.items():
        epochs[(cid[0], cid[2])].append(control)
    kinematic = dict(valid_duration_s=0., xy_speed_cap_time_s=0., xy_accel_cap_time_s=0.,
                     peak_xy_command_mps=0., peak_xy_command_accel_mps2=0.)
    periods = []
    for (owner, epoch), samples in epochs.items():
        samples.sort(key=lambda e:e['issued_ns'])
        periods.append(dict(owner=owner, epoch=epoch, samples=len(samples),
                            span_s=(samples[-1]['issued_ns']-samples[0]['issued_ns'])/1e9))
        if not all('command' in sample for sample in samples):
            continue  # Unit fixture may omit kinematic values.
        for previous, current in zip(samples, samples[1:]):
            dt = (current['issued_ns']-previous['issued_ns'])/1e9
            if not .001 <= dt <= .10:
                continue
            a, b = previous['command'], current['command']
            speed = ((b['vx']**2+b['vy']**2)**.5)
            accel = (((b['vx']-a['vx'])**2+(b['vy']-a['vy'])**2)**.5)/dt
            kinematic['valid_duration_s'] += dt
            kinematic['xy_speed_cap_time_s'] += dt*(speed >= .98*.7)
            kinematic['xy_accel_cap_time_s'] += dt*(accel >= .98*.8)
            kinematic['peak_xy_command_mps'] = max(kinematic['peak_xy_command_mps'], speed)
            kinematic['peak_xy_command_accel_mps2'] = max(kinematic['peak_xy_command_accel_mps2'], accel)
    duration = kinematic['valid_duration_s']
    for name in ('xy_speed_cap', 'xy_accel_cap'):
        kinematic[name+'_time_fraction'] = kinematic[name+'_time_s']/duration if duration else None
    kinematic['target_error_vs_latency_correlation_available'] = False
    kinematic['reason'] = 'This timing sidecar logs commands/model poses, not per-frame raw Sign positions or mapped goals.'
    coverage = dict(active_controls=len(active), selected_unique_sign_frames=len(used_sources),
                    controls_verified=len(first_verified), controls_applied_to_model=len(first_model),
                    controls_in_car_draw=len(first_car_draw), controls_in_goal_draw=len(first_goal_draw),
                    signed_outputs=len(signed), verified_outputs=len(verified),
                    applied_signed_outputs=len(model_by_output),
                    verified_superseded_before_model=sum(oid not in model_by_output for oid in verified),
                    display_subsampling_expected=True)
    return dict(metrics={name: stats(v) for name, v in values.items()}, coverage=coverage,
                paired_draw_paths=paths, negative_intervals=invalid,
                event_counts=dict(Counter(e['kind'] for e in events)),
                active_span_s=(max(active_times)-min(active_times))/1e9 if active_times else 0.,
                active_periods=periods, active_period_span_sum_s=sum(p['span_s'] for p in periods),
                control_kinematics=kinematic,
                clock_pair_span=stats([s['clock_pair_span_ns'] for s in sources.values()]),
                source_is_pc_local_not_exposure=True, endpoint='Tk Canvas draw commands submitted, not monitor photons',
                real_car_started=False, firmware_changed=False)


def run(folder):
    folder = Path(folder)
    logger = json.loads((folder/'TRACE_LOG_RESULT.json').read_text())
    if logger['dropped'] or logger['error'] or not logger['writer_stopped']:
        raise ValueError('trace loss/writer failure makes this sample invalid')
    report = attribute(rows(folder/'trace.jsonl'))
    report['enqueue_cost'] = stats(logger['enqueue_cost_ns'])
    report['trace_sha256'] = hashlib.sha256((folder/'trace.jsonl').read_bytes()).hexdigest()
    report['trace_complete'] = not report['negative_intervals'] and not any(
        report['event_counts'].get(kind, 0) for kind in ('worker_error', 'shutdown_error'))
    report['trace_log_loss'] = logger['dropped']
    report['enqueue_calls'] = len(logger['enqueue_cost_ns'])
    report['written_events'] = logger['written']
    if sum(report['event_counts'].values()) != logger['written']:
        raise ValueError('trace count differs from flushed writer count')
    report['genuine_sign'] = json.loads((folder/'CONTRACT.json').read_text()).get('real_sign', False)
    report['sample_valid'] = bool(report['trace_complete'] and report['coverage']['controls_in_car_draw'] >= 20)
    if (folder/'PROTECTED_BEFORE.json').exists() and (folder/'PROTECTED_AFTER.json').exists():
        report['original_protected_files_unchanged'] = (
            json.loads((folder/'PROTECTED_BEFORE.json').read_text()) ==
            json.loads((folder/'PROTECTED_AFTER.json').read_text()))
        report['sample_valid'] &= report['original_protected_files_unchanged']
    (folder/'LATENCY_RESULT.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    lines = ['# 同序号软件链路延迟', '',
             '只计ACTIVE控制帧；同机monotonic计时。源为PC本地接收后时间戳，不是相机曝光。',
             '蓝车终点是同一签名指令作用下模型状态的Tk绘图提交，不是物理车轮/屏幕呈现。', '',
             '| 测点 | 数量 | 中位数ms | P95 ms | P99 ms | 最大ms |',
             '|---|---:|---:|---:|---:|---:|']
    for name, label in LABELS.items():
        value = report['metrics'].get(name)
        if value:
            lines.append('| %s | %d | %.3f | %.3f | %.3f | %.3f |' %
                         (label, value['n'], value['median_ms'], value['p95_ms'], value['p99_ms'], value['max_ms']))
    lines += ['', '统计来自不同覆盖集合，不能相加P95；端到端使用同序号实测差值。',
              '50Hz控制、100Hz门/模型与约25Hz绘图并非一一刷新；被覆盖和未显示的帧单列，不强行配对。',
              '', '## 覆盖与完整性', '', '```json',
              json.dumps({k: report[k] for k in ('coverage','active_span_s','active_periods','active_period_span_sum_s',
                        'trace_complete','sample_valid','trace_log_loss','genuine_sign',
                        'enqueue_cost','clock_pair_span')}, ensure_ascii=False, indent=2),
              '```', '', '模型本身仍是原0.12秒一阶响应；本报告不把该动态响应当作传输延迟。']
    lines += ['', '## 跟踪误差与软件延迟不能混同', '',
              '本次仅记录时序身份、指令和模型位置，未保存逐帧Sign原始坐标/映射目标，',
              '因此不能用本次记录声称已证明“目标误差越大，通信越慢”。',
              '以下是指令端限幅占用，不是手部速度或目标误差：', '', '```json',
              json.dumps(report['control_kinematics'], ensure_ascii=False, indent=2), '```']
    (folder/'LATENCY_REPORT.md').write_text('\n'.join(lines)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    args = parser.parse_args()
    result = run(args.folder)
    print(json.dumps({k: result[k] for k in ('sample_valid','coverage','active_span_s')}, ensure_ascii=False))
