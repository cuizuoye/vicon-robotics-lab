"""Aggregate new coverage without rewriting accepted historical evidence."""
import argparse
import json
from pathlib import Path
from common import save


def run(out):
    raw=json.loads((out/'RESULT.json').read_text())
    lines=['# 6米×5米与随机旋转：新增覆盖验收','','## 范围与判据','',
        '保持现有参数、1:1 XY/相对角度映射、无固定围栏语义和原安全门控。实车未连接，未刷写，未更改Tenda或原演示。',
        '运行生产ROS监督器、末级门、同源校验、签名接收和实际C控制器；电机/编码器为未辨识的原有软件模型，不是物理验证。',
        '模拟杆按6米×5米四边和约6米对角线往返，角度随机目标约±30至±115度，并含跨±180度的连续旋转；最大输入角速度1.2rad/s。',
        '同一随机种子20261030用于档6、档7；正常手势最高0.55m/s，超车速压力手势最高1.00m/s。另测从静止开始的纯旋转。',
        '预先设置的旋转筛选线：角误差P95≤10度、最大≤20度、拟合时移≤150ms、保持段P95≤3度。',
        '新增位置筛选线单列为P95≤0.10m、最大≤0.20m；这是演示工程建议，不是已有用户或实车指标。',
        '拟合时移只描述目标与软件车运动曲线，尤其限速/饱和时不能解释为ROS或相机到车轮的物理延迟。','',
        '| 档位/场景 | 运行完整且停稳 | 角误差P95 | 位置误差P95/最大 | 新增筛选 |',
        '|---|---|---:|---:|---|']
    assessment=[]
    for entry in raw['cases']:
        case=entry['case'];metrics=entry['result'];folder=out/case['name']
        stop_path=folder/'normal_stop_RESULT.json'
        stop=json.loads(stop_path.read_text()) if stop_path.exists() else {}
        positional=metrics.get('xy_error_p95_m',float('inf'))<=.10 and metrics.get('xy_error_max_m',float('inf'))<=.20
        angular=metrics.get('synchronization_passed') is True
        complete=metrics.get('stability_passed') is True
        accepted=bool(complete and angular and positional)
        item=dict(case=case,complete_and_stop_confirmed=complete,rotation_passed=angular,
            position_passed=positional,overall_demo_screen_passed=accepted,
            interruption=stop.get('unexpected'),metrics=metrics)
        assessment.append(item)
        if 'angle_p95_deg' in metrics:
            lines.append('| %s | %s | %.2f° | %.3f / %.3f m | %s |'%(
                case['name'],'是' if complete else '否',metrics['angle_p95_deg'],
                metrics['xy_error_p95_m'],metrics['xy_error_max_m'],'通过' if accepted else '未通过'))
        else:lines.append('| %s | 否 | 未完成 | 未完成 | 执行失败 |'%case['name'])
    report=dict(complete=raw['complete'],cases=assessment,protected_unchanged=raw['protected_unchanged'],
        stage7_full_demo_admitted=False,physical_validation=False,hardware_output=False,
        old_sealed_evidence_rewritten=False)
    save(out/'ASSESSMENT.json',report)
    lines+=['','## 已暴露的问题','',
        '- 档6：0.70m/s、2.00m/s²，旋转上限2.00rad/s、12.00rad/s²。正常大范围手势可完成；1.00m/s手势超过车速上限，出现显著追赶误差，不能以“没有停机”冒充同步达标。',
        '- 档7：0.70m/s、4.00m/s²，但旋转仍限0.10rad/s、0.40rad/s²；不等于档6的快速旋转参数。随机大角度目标会严重积压。',
        '- 底层跟踪归零门槛20cps和启动门槛100cps不同；纯0.10rad/s对应每轮约81至82cps，低于启动门槛。平移停下后若需要重新起转，可能没有实际旋转进展，触发ROTATION_NO_PROGRESS。',
        '- 未关闭进展保护、未延长3秒门限、未将不同档位强行混配；档7不能作为大范围平移加随机旋转的完整演示版本放行。',
        '- 失败轮的复用测试器在检测到异常退出ACTIVE后立即结束，未继续取得完整停车确认；因此这些轮次不作为停车验收通过，原始失败记录保留。',
        '', '## 结论边界与后续','',
        '之前的小幅回归和600秒显示回放通过，只覆盖当时的输入，不能证明这次大范围随机旋转也合格。此报告补充新发现，不改写旧记录。',
        '后续应独立修正高速平移档与旋转档的能力组合及微速启动逻辑，再用相同种子、范围、速度和保护完整重测。不能单靠放宽故障保护解决。',
        '真实Sign覆盖、遮挡恢复、物理制动、电流和温升仍是单独验收；本报告不授权给小车上电、刷写或运动。','']
    (out/'ASSESSMENT.md').write_text('\n'.join(lines))
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args().output)
