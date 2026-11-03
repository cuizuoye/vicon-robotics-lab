"""Exact same-ID delay plus tracking-error/limit grouping for new SIM profiles."""
from collections import defaultdict
import json
import math
from pathlib import Path
import sys
import numpy as np
from model import ROOT
sys.path.insert(0,str(ROOT/'candidates/wand_software_latency_trace_20261029'))
from analyze import attribute, rows, key, stats, LABELS


def session_outcome(session, minimum_active_s=40.):
    """Timing coverage never proves that a demonstration stayed healthy."""
    runs = []
    active = None
    for event in session.get('events', []):
        state = event.get('state')
        if not state:
            continue
        if state[0] == 'ACTIVE' and active is None:
            active = event['at']
        elif state[0] != 'ACTIVE' and active is not None:
            reason = state[1] or state[2] or state[0]
            operator_stop = reason.startswith(('操作员', 'operator ', '再次空格停止'))
            runs.append(dict(start_s=active, end_s=event['at'],
                active_s=event['at']-active, first_stop_reason=reason,
                explicit_operator_stop=operator_stop))
            active = None
    snapshot = session.get('snapshot', {})
    interruptions = [r for r in runs if not r['explicit_operator_stop']]
    longest = max((r['active_s'] for r in runs), default=0.)
    terminal_fault = bool(snapshot.get('hard_fault') or snapshot.get('signature_fault')
                          or snapshot.get('state') in ('FAULT', 'STOP_UNCONFIRMED', 'ERROR'))
    return dict(runs=runs, minimum_active_s=minimum_active_s,
        longest_active_s=longest, first_interruption=interruptions[0] if interruptions else None,
        terminal_fault=terminal_fault, active_without_recorded_stop=active is not None,
        passed=bool(runs and not interruptions and not terminal_fault and active is None
                    and longest >= minimum_active_s and snapshot.get('stop_confirmed')))


def analyze(folder):
    folder=Path(folder)
    contract=json.loads((folder/'CONTRACT.json').read_text())
    logger=json.loads((folder/'TRACE_LOG_RESULT.json').read_text())
    events=list(rows(folder/'trace.jsonl'))
    result=attribute(events)
    result['contract']=contract
    result['trace_lost']=logger['dropped']
    result['enqueue_cost']=stats(logger['enqueue_cost_ns'])
    groups=defaultdict(list)
    controls={};sources={};signed={};verified={};tracking={}
    for e in events:
        kind=e['kind']
        if kind=='control_publish' and e['state']=='ACTIVE':controls[key(e['cid'])]=e
        elif kind=='source_relay':sources[e['sign']]=e
        elif kind=='signed_publish':signed[key(e['oid'])]=e
        elif kind=='verify_accept' and e['applied']:verified[key(e['oid'])]=e
        elif kind=='tracking' and e['state']=='ACTIVE':tracking[key(e['cid'])]=e
    first_verified={}
    for oid,v in sorted(verified.items(),key=lambda item:item[1]['end_ns']):
        p=signed.get(oid)
        cid=key(p.get('cid')) if p else None
        if cid in controls and p['permit']:first_verified.setdefault(cid,v)
    errors=[];holds=0;accels=[];moving=0
    old={}
    for cid,c in controls.items():
        track=tracking.get(cid)
        if track and track['error_m'] is not None:
            error=track['error_m'];errors.append(error);holds+=track['reference_limited']
            group=('<5cm' if error<.05 else '5-10cm' if error<.10 else
                   '10-20cm' if error<.20 else '20-40cm' if error<.40 else '>=40cm')
            source=sources.get(c['sign']);v=first_verified.get(cid)
            if source and v:groups[group].append(v['end_ns']-source['source_local_ns'])
        before=old.get((cid[0],cid[2]))
        if before:
            dt=(c['issued_ns']-before['issued_ns'])/1e9
            if .001<=dt<=.1:
                a,b=before['command'],c['command']
                accels.append(math.hypot(b['vx']-a['vx'],b['vy']-a['vy'])/dt)
        old[cid[0],cid[2]]=c
        moving+=math.hypot(c['command']['vx'],c['command']['vy'])>.02
    # Replace attribution helper's historical .70/.80 occupancy assumptions.
    result['control_kinematics']=dict(speed_limit_mps=contract['speed'],
        accel_limit_mps2=contract['accel'],
        accel_cap_sample_fraction=float(np.mean(np.array(accels)>=.98*contract['accel'])) if accels else None,
        moving_command_samples=moving)
    result['tracking']=dict(control_feedback_error_rms_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        control_feedback_error_p95_m=float(np.quantile(errors,.95)) if errors else None,
        max_control_feedback_error_m=max(errors) if errors else None,
        reference_hold_sample_fraction=holds/len(errors) if errors else None,
        error_group_to_source_verified_latency_ms={name:stats(values) for name,values in groups.items()},
        grouping_is_observation_not_causal_proof=True)
    result['original_unchanged']=(json.loads((folder/'PROTECTED_BEFORE.json').read_text())==
                                   json.loads((folder/'PROTECTED_AFTER.json').read_text()))
    result['timing_data_valid']=bool(result['coverage']['controls_in_car_draw']>=20 and not result['negative_intervals']
        and not logger['dropped'] and not logger['error'] and logger['writer_stopped']
        and result['original_unchanged'] and not (folder/'ERROR.json').exists())
    result['stability'] = session_outcome(json.loads((folder/'SESSION_RESULT.json').read_text()))
    result['sample_valid'] = result['timing_data_valid'] and result['stability']['passed']
    (folder/'RESULT.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    lines=['# 真实Sign → 参数匹配/低等待ROS软件车', '',
        '真实Sign输入；控制、签名、模型均在私有域190。无硬件输出；不可当成真机延迟或能力。',
        'PC源时间戳不是相机曝光；绘图提交不是显示器实际呈现；跟踪位置误差不是纯传输延迟。', '',
        '稳定演示验收：%s；计时数据可分析：%s。有效计时片段不能覆盖或抹掉保护中断。'%(
            '通过' if result['stability']['passed'] else '未通过',
            '是' if result['timing_data_valid'] else '否'), '',
        '| 测段 | 中位数ms | P95 ms | 最大ms |', '|---|---:|---:|---:|']
    for name,label in LABELS.items():
        value=result['metrics'].get(name)
        if value:lines.append('|%s|%.3f|%.3f|%.3f|'%(label,value['median_ms'],value['p95_ms'],value['max_ms']))
    lines += ['', '## 参数与跟踪误差分组', '', '```json',json.dumps(dict(contract=contract,
        tracking=result['tracking'],control=result['control_kinematics'],coverage=result['coverage'],
        stability=result['stability'],valid=result['sample_valid']),ensure_ascii=False,indent=2),'```', '',
        '误差是控制器所见目标与反馈的位置差，含源/反馈采样年龄，不是相机同曝光时刻误差。',
        '分组只作相关观察；目标动作/限幅也可能同时变化，不能据此单独断言因果。']
    (folder/'REPORT.md').write_text('\n'.join(lines)+'\n')
    return result
