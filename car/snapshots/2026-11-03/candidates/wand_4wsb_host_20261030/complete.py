"""Finish offline evidence, plans and host manifest; never hardware admission."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile
from prepare import ROOT,HERE,FW,CONTROL,HARDWARE,sha,hashes,save


def validate_stages(out):
    env=dict(os.environ,WAND_4WSB_HOST=str(out),ROS_DOMAIN_ID='191',
             ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',PYTHONDONTWRITEBYTECODE='1')
    results=[]
    for stage in range(8):
        folder=out/f'ros_stage{stage}'
        with (out/f'ros_stage{stage}.log').open('x') as log:
            r=subprocess.run([sys.executable,str(HERE/'split_rehearsal.py'),'--output',str(folder),
                '--stage',str(stage),'--case','normal_stop','--duration','5'],
                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=22.)
        record=json.loads((folder/'RESULT.json').read_text())
        results.append(record)
        print('STAGE%d %s'%(stage,'PASS' if r.returncode==0 and record['passed'] else 'FAIL'),flush=True)
        if r.returncode or not record['passed']:raise ValueError('native ROS stage regression failed: '+str(stage))
    save(out/'STAGE_MATRIX.json',dict(passed=True,actual_c_ros_stages=8,results=results,
        hardware_output=False,physical_qualified=False))


def seal(out):
    manifest=json.loads((out/'HOST_MANIFEST.json').read_text())
    xml=ET.parse(out/'host_tests_final.xml').getroot()
    cases=xml.findall('.//testcase')
    if len(cases)<45 or xml.findall('.//failure') or xml.findall('.//error'):
        raise ValueError('host unit/admission regression incomplete')
    fault=json.loads((out/'ros_faults_final/RESULT.json').read_text())
    stages=json.loads((out/'STAGE_MATRIX.json').read_text())
    ui=json.loads((out/'ui_600s_actual_pixels/RESULT.json').read_text())
    suite=json.loads((out/'SOFTWARE_VERIFICATION.json').read_text())
    if not suite['passed'] or len(suite['jobs'])!=10 or not all(j['passed'] for j in suite['jobs']):
        raise ValueError('serial software qualification/stop phases/rotation demo incomplete')
    if not fault['passed'] or len(fault['cases'])!=14 or not fault['production_source_relay_exercised']:
        raise ValueError('full 14-case source/ROS/HMAC/C fault suite incomplete')
    if not stages['passed'] or stages['actual_c_ros_stages']!=8:
        raise ValueError('stage matrix incomplete')
    if not ui['passed'] or len(ui['runs'])!=2:
        raise ValueError('600s nested UI comparison incomplete')
    evidence=[out/'host_tests_final.xml',out/'ros_faults_final/RESULT.json',out/'STAGE_MATRIX.json',out/'ui_600s_actual_pixels/RESULT.json',out/'SOFTWARE_VERIFICATION.json']
    from software_timing import folder
    timings={}
    for run in ui['runs']:
        r=Path(run['result']);u=r.parent/'ui/RESULT.json'
        v=json.loads(u.read_text())
        if not v['passed'] or v['active_s']<599.8 or v['peak_items']!=7:
            raise ValueError('nested UI did not complete actual600s active period')
        evidence.extend((r,u))
        run_value=json.loads((r.parent/'normal_stop_RESULT.json').read_text())
        if not run_value.get('actual_production_observer') or not (r.parent/'observer/RESULT.json').exists():
            raise ValueError('long UI test missing actual production logger')
        t=folder(r.parent);timings[run['mode']]=t
        if (t['matched']<59000 or t['unmatched'] or t['rejected_matched']
                or t['issued_to_core_return']['max_ms']>=100.):
            raise ValueError('same-sequence active command timing/coverage qualification failed')
        for name in ('supervisor_TIMING.jsonl','gate_TIMING.jsonl','observer/RESULT.json'):
            evidence.append(r.parent/name)
        for n in ('normal_stop_RESULT.json','normal_stop_TRACE.jsonl'):
            evidence.append(r.parent/n)
    for stage in range(8):evidence.append(out/f'ros_stage{stage}/normal_stop_RESULT.json')
    for job in suite['jobs']:
        if job['name'].startswith('stop_phase_') or job['name']=='rotation_demo_60s':
            evidence.append(out/job['name']/'normal_stop_RESULT.json')
    save(out/'COMMAND_TIMING.json',timings);evidence.append(out/'COMMAND_TIMING.json')
    for case in fault['cases']:evidence.append(out/'ros_faults_final'/(case['case']+'_RESULT.json'))
    for p,digest in manifest['protected'].items():
        if sha(p)!=digest:raise ValueError('protected original changed: '+p)
    if hashes(out/'runtime/src')!=manifest['runtime_hashes']:
        raise ValueError('generated runtime changed during validation')
    from acceptance import plan,blank_observation
    for stage in range(8):
        save(out/f'acceptance/STAGE{stage}_PLAN.json',plan(stage))
        for purpose in ('suspended','sign'):
            save(out/f'acceptance/STAGE{stage}_{purpose}_BLANK_NOT_APPROVED.json',blank_observation(stage,purpose))
    blockers=dict(hardware_admitted=False,flashed=False,physical_motion_started=False,
        fresh_two_full_backups=False,new_image_independent_readback=False,
        disabled_handshake_and_clock_verified=False,stage0_suspended_passed=False,
        loaded_braking_current_and_temperature_qualified=False,
        full_desktop_os_freeze_fixed=False,
        instructions='No automatic promotion. Firmware archive is historical NOT_FLASHED delivery; future use requires separate live authorization.')
    save(out/'ACTIVATION_BLOCKERS.json',blockers)
    save(out/'PROTECTED_AFTER.json',{p:sha(p) for p in manifest['protected']})
    manifest.update(host_offline_passed=True,host_software_port_complete=True,stability_qualified=True,
        runtime_hashes=hashes(out/'runtime/src'),source_hashes=hashes(HERE),
        evidence={str(p):sha(p) for p in evidence},hardware_admitted=False,
        physical_qualified=False,flashed=False,full_screen_issue_fixed=False,
        file_checker_entry=str(ROOT/'tools/start_car_sync_4wsb.sh'),
        file_checker_sha256=sha(ROOT/'tools/start_car_sync_4wsb.sh'))
    save(out/'HOST_MANIFEST.json',manifest)
    result=dict(passed=True,scope='OFFLINE host port and software evidence ONLY',identity='4WSB',
        firmware_sha256=manifest['firmware']['sha256'],unit_tests=len(cases),actual_ros_c_fault_cases=14,
        actual_ros_c_stages=8,nested_ui_modes_s=600,extra_stop_phases=6,translation_rotation_demo_s=60,
        actual_production_observer=True,command_timing=timings,
        protected_original_files_unchanged=len(manifest['protected']),
        firmware_flashed=False,hardware_output=False,physical_qualified=False,
        source_injection='synthetic Sign/car; real camera/UDP/MCU interrupt path NOT exercised',
        original_os_freeze_fixed=False,hardware_admission=False,
        pending=['fresh authorized backup/flash/readback','real disabled link/clock admission',
                 'stage0 secured wheels-up first','loaded stop/current/temperature qualification',
                 'native GNOME maximize/fullscreen reproduction, car powered off'])
    save(out/'RESULT.json',result)
    write_report(out,result,fault,ui)
    save(out.parent/'LATEST_SOFTWARE.json',dict(prepared=str(out),report=str(out/'REPORT.md'),
        host_offline_passed=True,hardware_admitted=False,firmware_flashed=False,
        original_desktop_freeze_fixed=False))
    archive=out.parent/'4WSB_HOST_OFFLINE_NOT_ADMITTED.zip'
    if archive.exists():raise ValueError('do not overwrite existing release archive')
    with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
        for base,prefix in ((HERE,Path('candidate_source')),(out/'runtime/src',Path('runtime/src')),
                            (out/'acceptance',Path('acceptance'))):
            for p in sorted(base.rglob('*')):
                if p.is_file() and '__pycache__' not in p.parts:z.write(p,prefix/p.relative_to(base))
        for name in ('HOST_MANIFEST.json','RESULT.json','ACTIVATION_BLOCKERS.json','REPORT.md'):
            z.write(out/name,name)
    save(out/'ARCHIVE.json',dict(path=str(archive),sha256=sha(archive),
        contains='private host source/configs, blank acceptance plans, manifest/report; evidence paths refer to this workstation',
        standalone_portable_ros_install=False,hardware_admitted=False))
    print(json.dumps(result,ensure_ascii=False),flush=True)


def write_report(out,result,fault,ui):
    text=['# 4WSB 启动实车前离线准备结果','',
        '本轮不刷写、不启动实车、不更改ESP32/Tenda/驱动/内核；原矩形和原演示入口保持不变。',
        '这是独立主机覆盖层，不会因文件存在而替换当前4WS8演示。','',
        '## 完成内容','',
        f"- {result['unit_tests']}项文件、配置、报文、同源匹配、时效、权限及启动阻断测试通过。",
        '- 2个独立ROS包构建通过；4WSB/v7/208字节严格对接，固定档0—7，不允许运行中换档。',
        '- 复用既有600秒策略、1:1 XY/相对旋转、Z不驱动和当前无固定围栏语义；不是恢复历史3×3限位。',
        '- 已验证低延迟算法/100Hz控制、同源FIFO与事件提交移植到新包；独立10ms门计时器和100ms保护不放宽。',
        '- 新入口先文件核验；无新镜像读回证明、逐档审查和新鲜现场批准时，在构造ROS/硬件socket前拒绝。',
        '- 实机入口自动检查/准备仍不运动，需新的单击空格；再次空格/Esc/失焦/异常停车，松键不停车。',
        '- 专用悬空入口仍为按住使能、最长5秒；悬空与落地审查分开，不复用为负载资格。',
        '- 新UI保持1280×720大字；绘图改为7个持久图元，不改原UI源码。','',
        '## 本轮软件修复','',
        '- 连续指令深度4与原始参考帧历史32分开；监督器取最新输入，门仍严格验证同序号的原始帧，不跳过中性使能帧。',
        '- 冷启动仅丢弃签名/范围/结构均已验证、从未准备的过期STANDBY失能零速报文；不更新owner、序号、时间或租约。'
        '未来时间、错误签名、来源变化、ARMED/ACTIVE及任何已用epoch仍按原100ms限制拒绝。',
        '- 停稳反馈改为完整0.5秒的绝对运动量上包络，保留独立Vicon窗口、零输出/撤能要求与1.5秒期限；'
        '移除两路已窗口化之后的重复0.5秒等待。正反运动不抵消，持续慢动、脉冲过大、缺帧、旧帧和故障不能通过。',
        '- 测试器与UI使用同一松键观察流程；停车报告只接受停止请求之后、PAUSED/FAULT状态的新确认。'
        '这些测试器修正不是放宽实车准备/启停规则。',
        '- 六个预先固定的不同停车相位和60秒平移＋旋转检查通过；不是失败后自动重试直到通过。','',
        '## 端到端软件链与故障注入','',
        '合成同帧Sign/car输入 → 实际SourceRelay → 实际ROS监督器 → 独立同源门 → HMAC权限验证 → '
        'MC-Link二进制帧 → 与ARM镜像同源的C控制器 → 历史未辨识PWM/编码器模型 → ROS反馈。',
        '没有创建实际HardwareBridge/UDP或串口；真实Vicon子进程、STM32中断/无线链路仍需后续现场验证。',
        '八个档位均执行了实际ROS+C跟随、换向/过零和停止回归。下表时间是测试动作到C控制器撤能且PWM为零，'
        '**不是物理制动距离或车轮完全停转的实测时间**。','',
        '|场景|撤能零输出延迟 ms|结果|','|---|---:|---|']
    for r in fault['cases']:
        text.append('|%s|%.1f|%s|'%(r['case'],r['stop_output_after_injection_s']*1000,'通过' if r['passed'] else '失败'))
    text+=['','遮挡后重新可见、反馈重新到达或被取消的定时器恢复，均未自动恢复C输出；'
        '控制/门定时器停摆保持硬锁存。所有档均检测真实C模型输出与编码器运动，不能只靠零速假通过。','',
        '## 界面长测与原卡死问题','',
        '|独立显示模式|活动时间 s|UI间隔最大 ms|绘制最大 ms|图元上限|预热后RSS范围 MiB|',
        '|---|---:|---:|---:|---:|---:|']
    for run in ui['runs']:
        u=json.loads((Path(run['result']).parent/'ui/RESULT.json').read_text())
        text.append('|%s|%.2f|%.2f|%.2f|%d|%.2f|'%(run['nested_screen'],u['active_s'],
            u['gap']['max_ms'],u['draw']['max_ms'],u['peak_items'],u['rss_range_after10s_bytes']/1024**2))
    text+=['','两组串行运行完整ROS+C链、真实Console和原生产日志观察器；不并行编译或运行第二个测试实例，后台原有程序未停止。'
        '这是应用层正常演示稳定性验收，不是GPU根因的单变量实验。'
        '在隐藏父窗口内的独立Xephyr显示中测试，'
        '不修改桌面分辨率、不全屏化用户GNOME桌面。X11独立往返、系统/Xorg/GNOME资源和看门狗记录一并保留。',
        '**原“鼠标不能移动、需强制重启”的根因仍未定位，不能宣称已修复。** '
        '原生桌面标题栏最大化/全屏的组合路径需要另行确认后复验。','',
        '首轮普通窗口在511.42秒触发控制间隔约114ms，UI/X11仍响应，未通过。'
        '保留在`../prepared/ui_600s`；没有删除或改写为通过。测试器当时在控制进程积累约5万条逐帧字典，'
        '独立读取相同记录后全量GC耗时约47—57ms，提示观测开销，但不足以证明原114ms全部由GC造成。',
        '修订测试器改为512条有界队列、独立进程落盘、仅保留200条内存快照，并记录GC暂停；'
        '主机SourceRelay仅对字节完全相同的不可变包复用几何解，每次仍检查原始时间戳。'
        '这不是禁用GC或放宽100ms保护。','',
        '第二轮测试器在约43秒发生原生oracle中间态取样错误。已通过专用C回归复现：'
        '极小命令经轮速归零后，“处理命令→控制tick”之间状态尚未完整；真实STM32 ISR在控制tick后才缓存遥测。'
        '仅修正软件执行器取样顺序为真实ISR顺序，固件不改；原失败日志保留在`ui_600s`。','',
        '另发现隐藏父窗口初始1×1导致Xephyr实际屏幕裁剪；此前配置值为1280/3072的记录不能证明相应尺寸绘图负载。'
        '最终脚本显式调整自己的隐藏父窗口并用xdpyinfo断言实际屏幕尺寸；正式结果仅采用`ui_600s_actual_pixels`。','',
        '## 同序号软件计时','',
        '|模式|ACTIVE匹配帧|发布调用到回调 p99/ms|发令到门验证完成 p99/ms|发令到门验证完成 最大/ms|',
        '|---|---:|---:|---:|---:|']
    for mode,t in result['command_timing'].items():
        text.append('|%s|%d|%.3f|%.3f|%.3f|'%(mode,t['matched'],t['publish_call_to_callback']['p99_ms'],
            t['issued_to_core_return']['p99_ms'],t['issued_to_core_return']['max_ms']))
    text+=['','上述是本机同序号软件时间戳，发布调用包含序列化，不是网卡收发时刻；'
        '不是Vicon曝光到实车起转的物理延迟。旧版约104.85ms积压及所有未通过目录保留。','',
        '## 分档与硬件资格','',
        '- 档0—6不改变原已离线验证表；档7为0.70m/s、4.00m/s²平移实验档，角速度/角加速度仍为0.10/0.40。',
        '- 档7纯微转可能低于继承的100cps启动门槛；更快旋转档4—6与档7不是同一个参数组合，不能混档。',
        '- 电机、负载、轮地作用与热模型没有经辨识；软件通过不等于现有硬件能达到4.00m/s²。',
        '- 按历史用户数据：1.4kg、每电机0.4kgf·cm、轮半径0.03m，四轮均输出额定转矩时直行理想加速度约3.74m/s²；'
        '未计麦轮损耗、摩擦或转矩—转速特性，也不知道峰值转矩，不能把此粗算当成绝对极限。',
        '- 若按反馈异常300ms后撤能报文丢失、最后600ms租约兜底的保守情景，0.70m/s仅反应段就约0.63m；'
        '还未加入制动和车体尺寸，历史50cm余量不能自动算合格。此为规划情景，不是测得响应时间。',
        '- 普通万用表与悬空平顺只能初筛，不能证明瞬态电流或温升；不做堵转/手挡轮，不将电流档并接电池。','',
        '## 入口和后续','',
        '仅文件检查（不会连接小车）：','',
        '```bash','bash /home/PRIMARY_USER/vicon_mecanum_ws/tools/start_car_sync_4wsb.sh --check --stage 0','```','',
        '当前`--launch`没有可用的批准记录，因此应阻断，不应为了开窗临时跳过准入。'
        '刷写包仍是10月29日的独立4WSB包，SHA不变；其中“firmware-only/host未迁移”是当时交付快照，'
        '本轮主机状态以此目录HOST_MANIFEST为准，不改写旧交付证据。','',
        '恢复硬件后顺序：另行授权 → 两份新鲜完整备份 → 刷写/独立全量读回/保持停止 → '
        '失能握手、时钟和故障位 → 首档悬空5秒 → 按档位与动作审查落地停车、电流、温升。',
        '空白记录在`acceptance/`；所有物理试验和人工审批字段均为false。'
        '现在不需要接ST-Link或给小车上电。','']
    (out/'REPORT.md').write_text('\n'.join(text))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepared',type=Path,required=True)
    p.add_argument('--stages',action='store_true');p.add_argument('--seal',action='store_true')
    a=p.parse_args();out=a.prepared.resolve()
    if a.stages:validate_stages(out)
    if a.seal:seal(out)
