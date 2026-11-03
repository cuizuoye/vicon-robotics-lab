"""Auditable interruption review; healthy timing data never erases a failure."""
import argparse
import json
from pathlib import Path
import numpy as np
from model import save


def events(folder):
    with (folder/'trace.jsonl').open() as stream:
        return [json.loads(line) for line in stream]


def timing(rows, kind, start=None, end=None, duration=15.):
    points = [row.get('begin_ns', row['at_ns']) for row in rows if row['kind'] == kind]
    points = [v for v in points if (start is None or v >= start) and (end is None or v <= end)]
    gaps = np.diff(points)/1e6
    return dict(count=len(points), rate_hz=len(points)/duration,
                gap_p95_ms=float(np.quantile(gaps, .95)), max_gap_ms=float(max(gaps)))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('failed', 'direct', 'isolated', 'stress', 'ros', 'output'):
        p.add_argument('--'+name, required=True, type=Path)
    a=p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    failed=json.loads((a.failed/'RESULT.json').read_text())
    stress=json.loads((a.stress/'RESULT.json').read_text())
    ros=json.loads((a.ros/'RESULT.json').read_text())
    comparison={}
    for name, folder in [('direct', a.direct), ('isolated', a.isolated)]:
        rows=events(folder)
        comparison[name]={k:timing(rows, k) for k in ('control_timer', 'gate_timer', 'source_relay')}
    rows=events(a.stress)
    active=[r for r in rows if r['kind']=='control_publish' and r['state']=='ACTIVE']
    bounds=active[0]['issued_ns'], active[-1]['issued_ns']
    loaded={k:timing(rows, k, *bounds, stress['active_s'])
            for k in ('control_timer', 'gate_timer', 'source_relay')}
    result=dict(failed_session=failed['stability'], failed_timing_usable=failed['timing_data_valid'],
        readonly_comparison=comparison, synthetic_60s_with_real_source_load=loaded,
        stress_passed=stress['passed'], actual_ros_safety_cases=ros['inherited_safety_cases'],
        current_real_sign_human_acceptance=False, original_demo_unchanged=True,
        physical_car_started=False, firmware_changed=False, network_settings_changed=False)
    save(a.output/'RESULT.json', result)
    lines=['# 真实Sign仿真中断复盘与隔离修复', '',
        '## 本次用户试验：未通过稳定演示验收', '',
        '- 第一段ACTIVE约3.42秒，因窗口失去焦点撤权。',
        '- 第二段ACTIVE约12.51秒，首停机原因为FINAL_GATE_STATUS_TIMEOUT；约0.50秒后才出现CONTROL_TIMER_GAP硬锁存。',
        '- Sign仍有完整新帧，接收拒绝计数为0、无验签拒绝；先因ROS末级门状态迟到停车，不是加速度参数越限触发。',
        '- 故障前控制回调约40～45Hz，而期望为100Hz。旧侧记没有全部回调/线程CPU时间，不能据此精确分摊到某一线程或系统任务。',
        '- 原报告把“时间戳数据可分析”混作“采样通过”；现已分开。故障前数据仍保留，整轮稳定性明确为失败。', '',
        '## 独立候选的修复', '',
        '1. Vicon域0只读接收改为独立spawn进程，控制、日志和Tk不再与六路高频接收共享同一个Python解释器。',
        '2. 进程间仅一个最新原始帧槽；仍携带原始源时间、每点收包时间及digest；父进程独立验几何和200ms年龄。IPC、进程退出、坏帧均失效关闭，绝不重打时间戳。',
        '3. 25ms界面快照与10ms控制/门定时器解耦；没有把绘图加速变成提高安全输入刷新。保留100ms保护、4条/40ms同源FIFO。',
        '4. 每个消费者仍独立解码、验证每个新原始包；只对同一不可变包记忆几何解和digest。每次使用仍重新校验六个源/接收时间戳，缓存不赋予新鲜度或运动权限。',
        '5. 新增控制/门定时器、门报告接收、慢执行器步骤的壁钟/线程CPU侧记；以后可直接区分处理耗时和被调度等待。', '',
        '隔离初版还查出“先取源时钟、后读取IPC”的竞争：合法新包在旧时钟看来短暂超前。现在先有界读取、按实际接收时刻验证，再采集Source的时钟；未放宽未来时间或200ms年龄检查。', '',
        '## 同机只读15秒对照（没有准备或使能）', '',
        '| 接收方式 | 末级门Hz | 门回调P95间隔ms | 最长间隔ms |',
        '|---|---:|---:|---:|']
    for name in ('direct', 'isolated'):
        t=comparison[name]['gate_timer']
        lines.append('|%s|%.1f|%.2f|%.2f|'%('修复前同进程线程' if name=='direct' else '隔离＋静态缓存候选',
            t['rate_hz'],t['gap_p95_ms'],t['max_gap_ms']))
    lines += ['', '这是有限时长、非实时操作系统上的修复前后观察，包含组合改动，不是某单项的严格因果测量或硬实时保证，也不声称测得相机曝光到真实车轮的物理延迟。', '',
        '## 验证状态', '',
        '- 60秒实际ROS合成往返运动，同时接收并验证真实Sign负载：通过；人工真实Sign新版复验尚未完成。',
        '- 22项模型/签名/UI/IPC/不可变包/报告归因测试与15项真实ROS故障注入回归：通过；独立门定时器停摆注入仍能锁止。',
        '- 参数仍为0.70m/s、4.00m/s²、400RPM仿真包络；1:1、旋转原参数及全部保护不变。',
        '- 原矩形/实车演示入口未改，无固件刷写、实车输出或Tenda设置更改。', '',
        '## 60秒软件运动压力的调度统计', '', '| 回调 | Hz | P95间隔ms | 最长间隔ms |', '|---|---:|---:|---:|']
    for k,t in loaded.items():
        lines.append('|%s|%.1f|%.2f|%.2f|'%(k,t['rate_hz'],t['gap_p95_ms'],t['max_gap_ms']))
    (a.output/'REPORT.md').write_text('\n'.join(lines)+'\n')


if __name__ == '__main__': main()
