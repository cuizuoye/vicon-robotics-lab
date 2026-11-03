"""Generate reproducible scientific plots/report, not hardware qualification."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import numpy as np
from model import ROOT,Limits,save,protected_hashes
from sweep import cases,run_case


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep',required=True,type=Path);parser.add_argument('--ros',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    before=protected_hashes()
    sweep=json.loads((a.sweep/'RESULT.json').read_text())
    records=json.loads((a.sweep/'SWEEP.json').read_text())
    ros=json.loads((a.ros/'RESULT.json').read_text())
    by={(r['profile'],r['case']):r for r in records}
    ranks={r['profile']:r for r in sweep['ranked']}
    presets=[Limits(.7,.8,400.),Limits(.7,2.,400.),Limits(.7,4.,400.),
             Limits(1.,3.,400.),Limits(1.,12.,400.)]
    data,provenance=cases();traces={};sensitivity=[]
    for p in presets:
        for name in ('fast_reversal','real_sign_xy_yaw'):
            r,rows=run_case(p,name,data[name],keep=True)
            assert r['passed'];traces[p.name,name]=rows
    for p in (presets[0],presets[2],presets[3]):
        for name in ('normal','fast_reversal','real_sign_xy_yaw'):
            r,_=run_case(p,name,data[name],lag=.20,gains=(.90,.80,.90))
            sensitivity.append(r)
    save(a.output/'SLOW_PLANT_STOP_REVIEW.json',sensitivity)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(13,8),constrained_layout=True)
    speeds=sorted({r['speed_mps'] for r in sweep['ranked']})
    accels=sorted({r['accel_mps2'] for r in sweep['ranked']})
    matrix=np.array([[ranks[Limits(v,acc,400.).name]['primary_mean_moving_rms_m']*100
                      for acc in accels] for v in speeds])
    image=axes[0,0].imshow(matrix,aspect='auto',origin='lower',cmap='viridis_r')
    axes[0,0].set(xticks=range(len(accels)),xticklabels=accels,yticks=range(len(speeds)),
                  yticklabels=speeds,xlabel='Acceleration cap (m/s^2)',ylabel='Speed cap (m/s)',
                  title='400 RPM model | 8 primary cases, moving RMS')
    fig.colorbar(image,ax=axes[0,0],label='cm (lower is better)')
    real=traces[presets[0].name,'real_sign_xy_yaw']
    axes[0,1].plot([r['t'] for r in real],[r['goal_x'] for r in real],'k--',label='same real Sign target')
    for p in presets[:4]:
        rows=traces[p.name,'real_sign_xy_yaw']
        axes[0,1].plot([r['t'] for r in rows],[r['car_x'] for r in rows],label=p.name)
    axes[0,1].set(title='Same real source replay, no time shift',xlabel='Virtual seconds',ylabel='Forward (m)')
    axes[0,1].legend(fontsize=7);axes[0,1].grid(alpha=.25)
    rows=traces[presets[0].name,'fast_reversal']
    t=np.array([r['t'] for r in rows])
    axes[1,0].plot(t,.75*np.sin(2.5*t),'k--',label='hand velocity')
    for p in presets[:4]:
        rows=traces[p.name,'fast_reversal']
        axes[1,0].plot([r['t'] for r in rows],[r['car_vx']*.98 for r in rows],label=p.name)
    axes[1,0].set(xlim=(0,8),title='Reversal: .75 m/s smooth gesture',xlabel='Virtual seconds',ylabel='World forward m/s')
    axes[1,0].legend(fontsize=7);axes[1,0].grid(alpha=.25)
    comparisons=ros['comparisons'];x=np.arange(len(comparisons))
    median=[r['metrics']['control_publish_to_verified']['median_ms'] for r in comparisons]
    p95=[r['metrics']['control_publish_to_verified']['p95_ms'] for r in comparisons]
    axes[1,1].bar(x-.15,median,width=.3,label='median')
    axes[1,1].bar(x+.15,p95,width=.3,label='P95')
    axes[1,1].set(xticks=x,xticklabels=['legacy','optimized','optimized','legacy'],
                  ylabel='ms',title='Actual ROS: same .70/.80 profile, synthetic input')
    axes[1,1].legend();axes[1,1].grid(axis='y',alpha=.25)
    fig.suptitle('SIM ONLY | 1:1 | unchanged guards | approximate dynamics, NOT physical capability')
    fig.savefig(a.output/'comparison.png',dpi=140);plt.close(fig)
    lines=['# 高同步参数与ROS调度对照（独立仿真）','','2026-10-29。无实车输出、无固件或Tenda更改，原演示入口不变。','',
        '## 候选结论','',
        '本次80组参数×11轨迹=880个名义模型对照，660个已完成比较直接复用，新增220个高加速度比较。',
        '速度0.50/0.70/1.00/1.20/1.50m/s；加速度0.80/1.20/2/3/4/6/8/12m/s²；轮速400/800RPM两种模型包络。',
        '400RPM只保留原运动学包络，不能证明现有电机可产生这些加速度；800RPM是假想替换驱动，不是给现有电机提速。','',
        '主指标为8条平顺/真实手势轨迹的“移动期间RMS误差”等权平均。两条瞬间速度起步压力轨迹和微小抖动单列；全部11条仍需通过保护与名义停稳检查。',
        '原400RPM组最低主指标约5.29cm；0.70m/s、4.00m/s²约5.52cm，距最低约4.4%，作为较小速度需求的近平台仿真候选。',
        '这不是全局最优：只对本次动作集合、控制器和一阶模型有效。1.00/12.00位于扫描加速度边界，但8→12的主指标收益已很小。','',
        '| 档位 m/s；m/s² | 主指标cm | 快换向误差RMS cm | 快换向延迟ms | 真实XY+yaw移动RMS cm |',
        '|---|---:|---:|---:|---:|']
    for p in presets:
        r=ranks[p.name];fast=by[p.name,'fast_reversal'];real=by[p.name,'real_sign_xy_yaw']
        lines.append('| %.2f；%.2f | %.2f | %.2f | %.1f | %.2f |'%(p.max_speed_mps,p.max_accel_mps2,
            r['primary_mean_moving_rms_m']*100,fast['moving_rms_m']*100,
            fast['reversal_delay_median_s']*1000,real['moving_rms_m']*100))
    lines += ['', '固定保留1:1平移/相对旋转、Z不映射、原跟踪有效性空间策略、0.20m参考超前约束、0.65m跟踪异常阈值、角速度0.10rad/s和角加速度0.40rad/s²。',
        '因此平移同步改进不代表旋转同步也已调优。真实记录的XY-only版本仅为明确标注的消融，不替代保留yaw的主回放。','',
        '## 稳定性边界，不隐藏失败','',
        '名义一阶模型tau=0.12s、前/侧/转增益0.98/0.88/0.95；并未由真实电机辨识。',
        '当tau=0.20s、增益0.90/0.80/0.90时，部分档位在输出已归零后，模型惯性导致1.50s内未完成原停稳确认。',
        '这些记录明确标为失败，不把后来停稳当成按时通过。不放宽停稳时限；硬件迁移前需辨识/改善执行器制动能力。',
        '慢模型复查见SLOW_PLANT_STOP_REVIEW.json，包含原0.70/0.80档作为对照。','',
        '## ROS改动与同参数调度对照','',
        '- 只在私有域190：源转发5ms、控制计算10ms；末级门独立10ms定时器保留。',
        '- FIFO完整同源绑定、参考/加速度/租约检查通过后及时发布已校验输出，不直接转发候选。事件回调不刷新定时器ClockCheck.previous。',
        '- 删去独立采样中重复的五个日志观察订阅，不删控制/反馈/来源校验订阅；时间戳侧记由有界异步日志记录。',
        '- 删去两个旋转适配器明确忽略的旧wand_pose空回调，保留其发布者与图身份；mode/source_status日常发布20Hz，模式或拒绝计数变化立即发布，原始Sign帧不节流。',
        '- 同一已提交非零控制帧在8ms内不重复签发；新控制序号、所有零速和撤权永不被该去重抑制，门定时器每次仍独立校验并发布健康状态。',
        '- 更快更新下修复READY会话竞争：必须等监督器消费同epoch门确认；源缺失队列仍最多4条/40ms，容量触发保留具体来源故障。',
        '- 新窗口复用Canvas图元、缩短快照等待，绘图仍与ROS执行线程分离，100ms UI保护不变。','',
        '| 重复次序 | 调度 | 控制发布→验签 中位数ms | P95 ms |','|---|---|---:|---:|']
    for i,c in enumerate(comparisons):
        v=c['metrics']['control_publish_to_verified']
        lines.append('| %d | %s | %.3f | %.3f |'%(i+1,'优化' if c['optimized'] else '原周期',v['median_ms'],v['p95_ms']))
    lines += ['', '这里是同机合成输入的ROS因果对照，不与早先真实Sign的31.9ms直接混算；机器负载和输入不同。',
        '依然保留位姿200ms、指令/签名/UI100ms、租约150ms、反馈300ms、120s会话、异常不自动恢复等保护。','',
        '## 验证分类','',
        '- 已通过：名义模型880次；12项纯模型/签名/UI测试；实际ROS同参数4次交替调度对照；15项来源/验签/超时/恢复回归；独立末级门计时器停摆注入。',
        '- 计时器独立性注入为了隔离门自身ClockCheck，显式向监督器注入新鲜门报告，防止监督器自己的报告超时先停车而掩盖被测门保护；这只在合成测试中使用。',
        '- 不作通过声明：慢执行器的部分停稳确认；真实Sign新参数用户验收（需单独会话）；任何真实车轮响应、制动、电流、温升或硬件升级能力。',
        '- car01：新入口只读显示可见性/位置，不把静止反光点当成软件车运动反馈。','',
        '## 新入口','', '```bash',
        'bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_sync_tuning_sim.sh --check',
        'bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_wand_sync_tuning_sim.sh --launch',
        '```','',
        '默认0.70m/s、4.00m/s²、400RPM仿真包络。打开后不自动准备/使能；人手动准备、真正READY后单击空格。再次空格/Esc停，松键不停。',
        '若以后对照1.00/3.00，可显式追加--speed 1.0 --accel 3.0 --rpm 400；参数仅作用于本独立软件车。',
        '每次关闭窗口后自动生成完整时序、目标误差分组、参考约束状态和来源原始帧，避免把目标间距当通信距离。','',
        '![comparison](comparison.png)']
    assert protected_hashes()==before
    (a.output/'REPORT.md').write_text('\n'.join(lines)+'\n')
    save(a.output/'RESULT.json',dict(passed=True,model_sweep=str(a.sweep),ros_evidence=str(a.ros),
        protected_unchanged=True,real_car_started=False,hardware_qualified=False,
        slow_plant_stop_failed_cases=sum(not r['passed'] for r in sensitivity)))
    print(str(a.output/'REPORT.md'),flush=True)


if __name__=='__main__':main()
