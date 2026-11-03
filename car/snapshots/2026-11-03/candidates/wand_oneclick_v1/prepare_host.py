#!/usr/bin/env python3
"""Independent startup/UI fix, retaining accepted controllers and firmware."""
import argparse,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
OLD=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready'
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
sys.path.insert(0,str(BASE))
from launch_cli import check
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def hashes(d):return {str(p):sha(p) for p in d.rglob('*') if p.is_file() and not any(k in ('__pycache__','.pytest_cache') or k.endswith('.egg-info') for k in p.parts)}
def edit(p,a,b):
    s=p.read_text()
    if s.count(a)!=1:raise ValueError('input drift: '+str(p)+' '+a)
    p.write_text(s.replace(a,b))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True)
    out=ap.parse_args().output.resolve();m=check(OLD/'build_manifest.json')
    out.mkdir(parents=True,exist_ok=False);(out/'COLCON_IGNORE').write_text('')
    runtime=out/'runtime/src';runtime.parent.mkdir()
    shutil.copytree(m['runtime_src'],runtime,ignore=shutil.ignore_patterns('__pycache__','*.egg-info','.pytest_cache'))
    c=runtime/m['control_package']/m['control_package'];h=runtime/m['hardware_package']/m['hardware_package']
    shutil.copyfile(HERE/'startup_idle.py',h/'startup_idle.py');shutil.copyfile(HERE/'novice_ui.py',c/'novice_ui.py')
    edit(h/'nodes.py','        self.feedback_sequence, self.last_feedback_identity = 0, None',
         '        self.user_arm_seen = False\n        self.feedback_sequence, self.last_feedback_identity = 0, None')
    edit(h/'nodes.py','        self.policy.receive(msg.data, time.monotonic(), self.get_clock().now().nanoseconds*1e-9)\n        self.safety_tick()',
         "        self.policy.receive(msg.data, time.monotonic(), self.get_clock().now().nanoseconds*1e-9)\n        if (self.policy.envelope or {}).get('state') in ('ARMED','ACTIVE'):\n            self.user_arm_seen = True\n        self.safety_tick()")
    edit(h/'nodes.py','    def latch_local_stop(self, reason):\n',
         "    def latch_local_stop(self, reason):\n        from .startup_idle import may_retry_unarmed_startup\n        if may_retry_unarmed_startup(self, reason, time.monotonic()):\n            self.force_local_disabled()\n            self.reconnect_after = time.monotonic()+.65\n            self.get_logger().warn('启动期间失能零输出遥测超时：等待旧租约结束后重连；不会使能、清故障或恢复运动')\n            return\n")
    edit(c/'pilot_ui.py','from .pilot_core import POLICY',
         'from .pilot_core import POLICY\nfrom .novice_ui import backend_hint, prepare_connection_ok')
    edit(c/'pilot_ui.py','return (self.pilot_valid() and not self.pilot_used',
         'return (self.pilot_valid() and not self.pilot_used and prepare_connection_ok(self,now)')
    edit(c/'pilot_ui.py',"        state=self.status.get('state')", "        hint=backend_hint(self,now)\n        if hint:return hint\n        state=self.status.get('state')")
    edit(c/'operator_console.py','from .pilot_ui import reason_text, real_instruction',
         'from .pilot_ui import reason_text, real_instruction\nfrom .novice_ui import acknowledge_physical_release')
    edit(c/'operator_console.py','        self.model = model',
         '        self.model = model\n        model.transport_status, model.transport_at = {}, -math.inf\n        self.create_subscription(String, "transport_status", self.on_transport, LATEST)')
    edit(c/'operator_console.py','    def on_status(self, msg):',
         '    def on_transport(self, msg):\n        try:\n            self.model.transport_status, self.model.transport_at = self.validated_status(msg)\n        except (ValueError, TypeError, KeyError):\n            self.model.transport_at = -math.inf\n\n    def on_status(self, msg):')
    edit(c/'operator_console.py',"            button.pack(fill='x', pady=3)",
         "            if name in ('start','stop','estop'): button.pack(fill='x', pady=3)")
    edit(c/'operator_console.py',"('start', '1  准备（不运动）')","('start', '1  准备好后点这里（不会运动）')")
    edit(c/'operator_console.py',"('stop', '停止（等待反馈确认）')","('stop', '正常停止 / 结束本轮')")
    edit(c/'operator_console.py',"text = '软件急停（不能代替物理断电）'","text = '紧急停止（锁存；异常时使用）'")
    edit(c/'operator_console.py','真实Sign输入：准备就绪后持续按住空格并移动杆；松开或切出窗口停车。',
         '操作：点准备 → 等就绪 → 按住空格移动Sign；松开即结束。正常结束不要点急停。')
    edit(c/'operator_console.py',"        self.model.focus(focused, time.monotonic())",
         "        self.model.focus(focused, time.monotonic())\n        if focused and self.key_state is not None:\n            acknowledge_physical_release(self.model,self.server_space_down(),time.monotonic())")
    edit(c/'pilot_ui.py','准备就绪：先拿稳Sign；按住空格时才记录原点并计时，3×3m区内全向跟随；最长20秒、仅一次',
         '准备就绪：现在持续按住空格，平移Sign即可跟随；松开停车；本轮最长20秒')
    tests=out/'tests';shutil.copytree(OLD/'tests',tests,ignore=shutil.ignore_patterns('__pycache__','.pytest_cache'))
    edit(tests/'test_demo_range.py',"('nodes.py','stage0_contract.py','stage0_session.py')","('stage0_contract.py','stage0_session.py')")
    shutil.copyfile(HERE/'test_oneclick.py',tests/'test_oneclick.py')
    env=os.environ.copy();env.update(STAGED_GENERATED=str(runtime),ONECLICK_SOURCE=str(HERE),
        PYTHONPATH=str(BASE)+os.pathsep+env.get('PYTHONPATH',''),DISPLAY=env.get('DISPLAY',':1'),
        XAUTHORITY=env.get('XAUTHORITY','/run/user/1000/gdm/Xauthority'))
    r=subprocess.run([sys.executable,'-m','pytest','-q',str(tests),'--junitxml='+str(out/'host_tests.xml')],
        cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    log=r.stdout+r.stderr;(out/'host_tests.log').write_text(log);print(log,flush=True)
    if r.returncode:raise ValueError('one-click regressions failed; no hardware opened')
    pins=dict(m['protected_input_hashes'])
    for k in ('runtime_hashes','source_hashes','generated_firmware_hashes'):pins.update(m[k])
    pins[str(OLD/'build_manifest.json')]=sha(OLD/'build_manifest.json')
    for p in (OLD/'ground_stage4_single_9vd3kzd9/observation.jsonl',
              OLD/'ground_stage4_single_9vd3kzd9/mcu_receive.jsonl',
              OLD.parent/'accepted_20261026/ACCEPTED_BASELINE.json'):
        pins[str(p)]=sha(p)
    sources={**m['source_hashes'],**hashes(HERE)}
    for p in (ROOT/'tools/start_car_oneclick.sh',Path('/home/PRIMARY_USER/.local/bin/小车'),
              Path('/home/PRIMARY_USER/.local/share/applications/vicon-car-console.desktop')):
        sources[str(p)]=sha(p)
    result=dict(m,runtime_src=str(runtime),runtime_hashes=hashes(runtime),
        source_hashes=sources,protected_input_hashes=pins,
        draft_profiles=[str(runtime/m['hardware_package']/f'config/stage{i}.NOT_APPROVED.json') for i in range(7)],
        host_change='Separate novice UI/startup-only zero reconnect fix; accepted servo, safety core, firmware and3x3/20s scope unchanged')
    (out/'build_manifest.json').write_text(json.dumps(result,indent=2)+'\n');check(out/'build_manifest.json')
    r=subprocess.run(['colcon','--log-base',str(out/'colcon_log'),'build','--base-paths',str(runtime),
        '--build-base',str(out/'runtime/build'),'--install-base',str(out/'runtime/install'),'--symlink-install',
        '--packages-select',m['control_package'],m['hardware_package']],cwd=ROOT,env=env,capture_output=True,text=True,timeout=60)
    (out/'colcon_build.log').write_text(r.stdout+r.stderr);print(r.stdout+r.stderr,flush=True)
    if r.returncode:raise ValueError('private build failed')
    check(out/'build_manifest.json')
    (out/'PREPARATION_RESULT.json').write_text(json.dumps(dict(passed=True,tests=log.strip().splitlines()[-1],
        protected_files_unchanged=len(pins),hardware_started=False,flashed=False,
        accepted_control_algorithms_unchanged=True),indent=2)+'\n')
if __name__=='__main__':main()
