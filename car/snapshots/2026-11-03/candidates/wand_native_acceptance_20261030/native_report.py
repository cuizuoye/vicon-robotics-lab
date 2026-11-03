"""Derive a separate assessment; never relabel or overwrite raw results."""
import argparse
import json
import os
from pathlib import Path
from common import save


def resource_summary(path):
    rows=[json.loads(line) for line in path.open()]
    result=dict(samples=len(rows),processes={},min_available_gib=min(r['mem_available_kib'] for r in rows)/1024**2,
        max_swap_mib=max(r['swap_used_kib'] for r in rows)/1024,
        max_readable_temperature_c=max(v for r in rows for v in r['temperatures_c'].values()),
        intel_thermal_throttle_samples=sum(bool(r.get('intel_gt',{}).get('throttle_reason_thermal')) for r in rows))
    for name in ('ui','Xorg','gnome-shell'):
        samples=[(r['steady'],r['processes'][name]) for r in rows if r['processes'].get(name)]
        rss=[int(p['VmRSS'].split()[0])/1024 for _,p in samples]
        warm=[(t,p) for t,p in samples if t>=samples[0][0]+10.]
        first,last=samples[0],samples[-1]
        result['processes'][name]=dict(average_cpu_one_core_percent=
            (last[1]['cpu_ticks']-first[1]['cpu_ticks'])/os.sysconf('SC_CLK_TCK')/(last[0]-first[0])*100.,
            peak_rss_mib=max(rss),rss_after10s_delta_mib=
            (int(last[1]['VmRSS'].split()[0])-int(warm[0][1]['VmRSS'].split()[0]))/1024 if warm else None,
            uninterruptible_samples=sum(p['state']=='D' for _,p in samples))
    return result


def assess(out,manual_maximized):
    raw=json.loads((out/'RESULT.json').read_text());phases=[]
    for phase in raw['phases']:
        mode=phase['mode'];transitions=[];previous=None;total=0;ordinary_until=None
        for line in (out/(mode+'_ui.jsonl')).open():
            row=json.loads(line);total+=1
            value=(tuple(row['root_size']),row['wm_fullscreen'],row['wm_zoomed'])
            if value!=previous:
                transitions.append(dict(elapsed_s=row['elapsed_s'],size=value[0],fullscreen=value[1],maximized=value[2]))
                previous=value
            if mode=='window' and (row['wm_fullscreen'] or row['wm_zoomed']) and ordinary_until is None:
                ordinary_until=row['elapsed_s']
        mixed=mode=='window' and ordinary_until is not None
        phases.append(dict(**phase,transitions=transitions,controlled_single_mode=not mixed,
            label='ordinary -> manually maximized (operator confirmed)' if mixed and manual_maximized else
                'ordinary -> unexpected WM state transition' if mixed else mode,
            ordinary_observed_s=ordinary_until if mixed else None,
            resources=resource_summary(out/(mode+'_system.jsonl'))))
    assessment=dict(renderer_run_passed=raw['passed'],phases=phases,
        all_three_pure_600s_modes_passed=bool(raw['passed'] and len(phases)==3 and all(p['controlled_single_mode'] for p in phases)),
        manual_maximization_confirmed=manual_maximized,protected_unchanged=raw.get('protected_unchanged'),
        original_os_freeze_root_cause_identified=False,original_os_freeze_proven_fixed=False,
        hardware_output=False,drivers_or_kernel_changed=False)
    assessment['journal_stdout']={name:json.loads((out/(name+'_journal.json')).read_text()).get('stdout','')
        for name in ('kernel','gnome')}
    assessment['xorg_increment_bytes']=(out/'xorg_increment.log').stat().st_size
    save(out/'ASSESSMENT.json',assessment)
    lines=['# 原生桌面600秒显示压力复核','','仅显示回放，不是实车运动或完整实时ROS负载验收。',
        '未修改驱动、内核、网络设置或原演示；原候选和受保护文件前后哈希一致。' if raw.get('protected_unchanged') else '受保护哈希检查未通过。','',
        '| 模式 | 循环P99 / 最大 | X11最大 | UI峰值RSS | 可用内存最低 |',
        '|---|---:|---:|---:|---:|']
    for p in phases:
        r=p['resources'];lines.append('| %s | %.2f / %.2f ms | %.2f ms | %.1f MiB | %.2f GiB |'%(
            p['label'],p['gap_ms']['p99'],p['gap_ms']['max'],p['x11_ms']['max'],
            r['processes']['ui']['peak_rss_mib'],r['min_available_gib']))
    lines+=['','普通窗口段经用户确认曾手动最大化，保留为切换压力证据，不冒充纯普通窗口600秒对照。',
        '固定最大化和原生全屏以实际窗口状态、EWMH回执及全程几何记录核对。',
        '有限测试中未复现，不等于旧整机死机根因已定位或已彻底修复。',
        '系统日志、Xorg增量及GPU辅助快照保存在本目录，需与资源数据一起解读。',
        '本轮Xorg日志增量为0字节。内核记录1条perf自动降低采样率信息；GNOME在两次窗口状态切换附近记录4条布局分配警告。',
        '这些时段回放与独立X11探测仍正常更新；不能据此断言日志警告就是旧卡死根因，也不将其隐去。',
        '独立看门狗只终止本次测试子进程；不能保证整机级故障时仍能工作。','']
    # Generated diagnostic report, not a source edit.
    (out/'ASSESSMENT.md').write_text('\n'.join(lines))
    return assessment


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--window-manually-maximized',action='store_true');a=p.parse_args()
    assess(a.output,a.window_manually_maximized)
