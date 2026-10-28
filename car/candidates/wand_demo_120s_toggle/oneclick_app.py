#!/usr/bin/env python3
"""One command opens a guided UI. GUI confirmation never enables the car."""
import argparse,fcntl,hashlib,json,os,queue,signal,socket,subprocess,sys,tempfile,threading,time
from pathlib import Path
import tkinter as tk
from tkinter import ttk,messagebox

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]
READY=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_120s_toggle/ready'
ACCEPTED=ROOT/'docs/wand_servo_4ws8_2026-10-26/demo_3x3_20s/ready'
PROFILE_UUID='00000000-0000-4000-8000-000000000000'
PROVENANCE=Path('/home/PRIMARY_USER/vicon_receiver_ws/drone_sim/tools/gamepad_rc')
BASE=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8'
FLASH=ROOT/'stm32/mecanum_f407/backups/pre_4ws8_staged_20261023_1q2wgy/POST_FLASH_RECORD.json'
sys.path.insert(0,str(BASE))
from launch_cli import check

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(args,timeout=10):
    p=subprocess.run(args,capture_output=True,text=True,timeout=timeout)
    if p.returncode:raise ValueError((p.stderr or p.stdout or '命令失败').strip())
    return p.stdout.strip()

def runtime_imports():
    m=check(READY/'build_manifest.json')
    for name in (m['control_package'],m['hardware_package']):
        path=str(Path(m['runtime_src'])/name)
        if path not in sys.path:sys.path.insert(0,path)
    return m

def make_review(m):
    from mecanum_wand_4ws8_control.pilot_core import POLICY
    from mecanum_wand_4ws8_hardware.pilot_authorization import KIND
    evidence=ROOT/'docs/wand_servo_4ws8_2026-10-23/requested_suspended_050_070/host_ready/stage4_suspended_l79694pa/bench/SESSION_RESULT.json'
    user=ROOT/'stm32/mecanum_f407/candidates/servo_4ws8_ground_pilot/USER_AUTHORIZATION.json'
    request=HERE/'USER_REQUEST.json'
    return dict(kind=KIND,identity='4WS8',authorized_stage=4,purpose='sign',human_authorized=True,
        image_sha256=m['firmware']['sha256'],pilot_policy=POLICY,
        continuous_ground_qualified=False,electrical_thermal_trial_reviewed=False,
        operator_input_mode='SPACE_TOGGLE_120S_V1',
        suspended_evidence=str(evidence),suspended_evidence_sha256=sha(evidence),
        user_authorization=str(user),user_authorization_sha256=sha(user),
        range_request=str(request),range_request_sha256=sha(request))

def verify_files():
    m=runtime_imports()
    from mecanum_wand_4ws8_hardware.authorization import verify_flash_record,verify_trial_review
    verify_flash_record(FLASH,READY/'build_manifest.json')
    verify_trial_review(make_review(m),4,'sign',m['firmware']['sha256'])
    return m

def check_port():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    try:sock.bind(('0.0.0.0',46001))
    except OSError:raise ValueError('已有小车通信程序占用端口：请先正常关闭旧操作台/矩形程序，不会自动抢占或杀进程')
    finally:sock.close()

def ensure_tenda():
    never=run(['nmcli','-g','ipv4.never-default','connection','show',PROFILE_UUID])
    gateway=run(['nmcli','-g','ipv4.gateway','connection','show',PROFILE_UUID])
    if never!='yes' or gateway not in ('','--'):raise ValueError('Tenda配置不是已保存的独立链路；为保护电脑联网，停止自动连接')
    before=run(['ip','-4','route','show','default'])
    active=run(['nmcli','-t','-f','UUID','connection','show','--active']).splitlines()
    if PROFILE_UUID not in active:
        run(['nmcli','connection','up','uuid',PROFILE_UUID],timeout=20)
    if run(['ip','-4','route','show','default'])!=before:raise ValueError('默认联网线路发生变化：停止启动，请检查网络；不自动重写其他网络设置')
    run(['ping','-n','-c','2','-W','1','-I','wlx000000000000','192.168.50.1'],timeout=5)

def ensure_receiver(folder):
    sys.path.insert(0,str(PROVENANCE))
    from vicon_receiver_provenance import verify_current,child_environment,recheck_before_spawn
    p=verify_current();existing=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():continue
        try:
            if (entry/'comm').read_text().strip()=='vicon_client':existing.append(entry)
        except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if existing:
        if len(existing)!=1:raise ValueError('存在多个Vicon接收器；为避免来源冲突，请先人工检查')
        process=existing[0]
        if str((process/'exe').resolve())!=p['installed']:raise ValueError('现有Vicon程序来源与已核验版本不符；不会替换或停止它')
        env=dict(v.split('=',1) for v in (process/'environ').read_bytes().decode().split('\0') if '=' in v)
        args=(process/'cmdline').read_bytes().decode().split('\0')
        if env.get('ROS_DOMAIN_ID','0')!='0' or 'hostname:=10.10.10.2' not in args:
            raise ValueError('现有Vicon接收域或服务器与基线不同；不会擅自改动')
        return None
    env=child_environment(os.environ,p['policy'])
    logdir=folder/'receiver_ros_logs';logdir.mkdir(mode=0o700)
    env.update(ROS_DOMAIN_ID='0',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logdir))
    params=dict(hostname='10.10.10.2',buffer_size='200',namespace='vicon',world_frame='map',
        vicon_frame='vicon',map_xyz='[0.0,0.0,0.0]',map_rpy='[0.0,0.0,0.0]',
        map_rpy_in_degrees='false',stream_mode='ServerPush',enable_marker_ray_data='false',
        enable_debug_data='false',enable_device_data='false',publish_latency='true',capture_frame_samples='false')
    command=[p['installed'],'--ros-args']
    for key,value in params.items():command.extend(['-p',key+':='+value])
    recheck_before_spawn(p)
    with (folder/'receiver.log').open('xb') as stream:
        return subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)

def check_poses(folder):
    logdir=folder/'pose_ros_logs';logdir.mkdir(mode=0o700,exist_ok=True)
    env=dict(os.environ,ROS_DOMAIN_ID='0',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(logdir))
    p=subprocess.run([sys.executable,str(HERE/'pose_check.py')],env=env,capture_output=True,text=True,timeout=8)
    if p.returncode:raise ValueError('定位检查未完成：'+(p.stderr or p.stdout)[-800:])
    report=json.loads(p.stdout.strip().splitlines()[-1])
    if not report['ready']:raise ValueError('；'.join(report['issues']))
    return report

def create_session(confirmations,preflight_at,now=None):
    now=time.monotonic() if now is None else now
    if len(confirmations)!=3 or not all(v is True for v in confirmations):raise ValueError('请逐项确认现场条件；不会自动替你确认')
    if not 0<=now-preflight_at<=120.:raise ValueError('检查已过期，请点“重新检查连接”')
    check_port();m=verify_files()
    from mecanum_wand_4ws8_hardware.authorization import verify_runtime_profile
    from mecanum_wand_4ws8_hardware.commissioning import Runtime,CONSENT
    profile=json.loads(Path(m['draft_profiles'][4]).read_text());profile['trial_review']=make_review(m)
    profile['staged_runtime_approval']=dict(purpose='sign',issued_utc_s=time.time(),
        clear_and_power_cut_ready_now=True,single_click_mode_acknowledged_now=True,flash_record=str(FLASH),build_manifest=str(READY/'build_manifest.json'))
    profile['confirmations']['fixed_arena_checked']=True
    profile['arena_confirmations']={k:True for k in profile['arena_confirmations']}
    verify_runtime_profile(profile,'sign')
    folder=Path(tempfile.mkdtemp(prefix='oneclick_session_',dir=READY));approved=folder/'site_confirmed.json'
    approved.write_text(json.dumps(profile,ensure_ascii=False,indent=2)+'\n');approved.chmod(0o600)
    Runtime('hardware',str(approved),sha(approved),CONSENT)
    logdir=folder/'ros_logs';logdir.mkdir(mode=0o700)
    env=dict(os.environ,ROS_DOMAIN_ID='120',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
        ROS_LOG_DIR=str(logdir),WAND_PILOT_TRACE=str(folder/'observation.jsonl'),
        WAND_4WS8_RAW_RECEIVE_TRACE=str(folder/'mcu_receive.jsonl'))
    command=['ros2','launch',m['hardware_package'],'wand_4ws8_staged.launch.py','mode:=hardware',
        'gui:=true','profile:='+str(approved),'approved_sha256:='+sha(approved),'hardware_consent:='+CONSENT]
    return folder,command,env

def single_instance():
    parent=Path(os.environ.get('XDG_RUNTIME_DIR',f'/run/user/{os.getuid()}'))
    if not parent.is_dir() or parent.stat().st_uid!=os.getuid():raise ValueError('找不到当前用户的安全运行目录')
    lock=(parent/'mecanum_space_toggle.lock').open('a');path=parent/'mecanum_space_toggle.sock'
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
            s.settimeout(1.);s.connect(str(path));s.sendall(b'SHOW')
        return None
    if path.is_symlink():raise ValueError('操作台通信路径异常')
    if path.exists():
        if path.stat().st_uid!=os.getuid():raise ValueError('操作台通信路径不属于当前用户')
        path.unlink()
    server=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);server.bind(str(path));server.listen(2);server.setblocking(False)
    return lock,server,path

class App:
    def __init__(self,root,instance=None,preview=False):
        self.root,self.instance,self.preview=root,instance,preview
        self.events=queue.Queue();self.busy=False;self.good_at=-1e9
        self.receiver=self.backend=None;self.trial=None;self.trace_at=0;self.partial='';self.status={}
        self.folder=Path(tempfile.mkdtemp(prefix='oneclick_ui_')) if not preview else None
        root.title('小车单击 · 120秒独立演示');root.geometry('900x700');root.minsize(820,650)
        style=ttk.Style(root);style.configure('TLabel',font=('Noto Sans CJK SC',11));style.configure('TButton',font=('Noto Sans CJK SC',12),padding=10)
        frame=ttk.Frame(root,padding=22);frame.pack(fill='both',expand=True)
        ttk.Label(frame,text='小车操作台：检查 → 准备 → 单击空格跟随',font=('Noto Sans CJK SC',19,'bold')).pack(anchor='w')
        ttk.Label(frame,text='1:1 · 2.0×0.9m · 最长120秒 · 0.70m/s / 0.80m/s²上限\n打开窗口不运动；不刷固件、不自动清急停、不改变已保存控制算法。').pack(anchor='w',pady=10)
        self.message=tk.StringVar(value='正在检查连接……' if not preview else '离线界面预览：没有连接设备，不能启动实车。')
        ttk.Label(frame,textvariable=self.message,wraplength=820,foreground='#17466b').pack(fill='x',pady=10)
        self.detail=tk.Text(frame,height=9,wrap='word',font=('Noto Sans CJK SC',10));self.detail.pack(fill='both',expand=True)
        self.append('只有检查通过并由你逐项确认后，才能打开跟随窗口。已运行的旧程序不会被自动杀掉。')
        self.checks=[]
        for text in ('小车已落地、位于绿色区域内，原朝向正确；Sign已拿稳且可见',
                     '原起点前0.5/后1.5/左0.5/右0.4m及四边50cm缓冲空旷，接线已固定',
                     '有人能立即断电；已知松键不停车，再次空格/Esc/停止结束；急停仅用于异常'):
            v=tk.BooleanVar(value=False);self.checks.append(v)
            ttk.Checkbutton(frame,text=text,variable=v,command=self.update_buttons).pack(anchor='w',pady=5)
        buttons=ttk.Frame(frame);buttons.pack(fill='x',pady=10)
        self.check_button=ttk.Button(buttons,text='重新检查连接',command=self.preflight);self.check_button.pack(side='left')
        self.open_button=ttk.Button(buttons,text='打开跟随窗口（不会运动）',command=self.open_console);self.open_button.pack(side='left',padx=12)
        ttk.Button(buttons,text='关闭 / 停止本次程序',command=self.close).pack(side='right')
        ttk.Label(frame,text='跟随窗口中：点“准备” → 等“准备就绪” → 轻按并松开空格一次，然后移动Sign。\n松键不停车；再次空格/Esc、切出窗口、遮挡或120秒到时会停车；一轮结束后不能自动续跑。',wraplength=820).pack(anchor='w',pady=8)
        root.protocol('WM_DELETE_WINDOW',self.close);self.closed=False;self.update_buttons()
        self.poll_job=root.after(100,self.poll)
        if not preview:root.after(200,self.preflight)
    def append(self,text):
        self.detail.insert('end',text+'\n');self.detail.see('end')
    def update_buttons(self):
        allowed=not self.busy and self.backend is None and time.monotonic()-self.good_at<=120 and all(v.get() for v in self.checks) and not self.preview
        self.open_button.configure(state='normal' if allowed else 'disabled')
        self.check_button.configure(state='disabled' if self.busy or self.backend is not None or self.preview else 'normal')
    def preflight(self):
        if self.busy or self.backend is not None or self.preview:return
        self.busy=True;self.good_at=-1e9;self.message.set('正在检查：不会使能或转轮');self.update_buttons()
        def work():
            try:
                self.events.put(('log','1/4 核对已保存基线、候选文件和完整Flash读回记录'));verify_files();check_port()
                self.events.put(('log','2/4 检查Tenda，只恢复已保存的独立链路，不切换互联网'));ensure_tenda()
                self.events.put(('log','3/4 核验Vicon来源；已有接收器复用，没有时才启动只读接收'))
                if self.receiver is None:self.receiver=ensure_receiver(self.folder)
                self.events.put(('log','4/4 检查Sign、car01及绿色区/朝向'));p=check_poses(self.folder)
                self.events.put(('good',p))
            except Exception as e:self.events.put(('error',str(e)))
        threading.Thread(target=work,daemon=True).start()
    def open_console(self):
        if self.preview:return
        try:
            folder,cmd,env=create_session([v.get() for v in self.checks],self.good_at)
            with (folder/'launch.log').open('xb') as stream:
                self.backend=subprocess.Popen(cmd,env=env,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
            self.trial=folder;self.trace_at=0;self.partial='';self.status={}
            self.message.set('跟随窗口已打开：请在新窗口点准备，等待就绪后单击空格；这里只显示运行状态。')
            self.append('本轮日志：'+str(folder));self.update_buttons()
        except Exception as e:self.message.set('未启动：'+str(e));self.good_at=-1e9;self.update_buttons()
    def poll(self):
        if self.closed:return
        if self.instance:
            try:
                connection,_=self.instance[1].accept()
                with connection:
                    if connection.recv(16)==b'SHOW':self.root.deiconify();self.root.lift();self.root.focus_force()
            except BlockingIOError:pass
        try:
            while True:
                kind,value=self.events.get_nowait()
                if kind=='log':self.append(value)
                elif kind=='good':
                    self.busy=False;self.good_at=time.monotonic()
                    self.message.set('连接检查通过。请逐项确认现场条件，再打开跟随窗口；真实MCU反馈将由安全链路继续核验。')
                    self.append('小车区域坐标：'+str(value.get('car_local_m'))+'；朝向误差：'+str(round(value.get('heading_error_deg',0),2))+'°')
                else:
                    self.busy=False;self.good_at=-1e9;self.message.set('检查未通过：'+value);self.append(value)
        except queue.Empty:pass
        if self.backend is not None:
            trace=self.trial/'observation.jsonl'
            if trace.exists():
                with trace.open() as f:f.seek(self.trace_at);data=f.read(262144);self.trace_at=f.tell()
                parts=(self.partial+data).split('\n');self.partial=parts.pop()
                for line in parts:
                    try:r=json.loads(line)
                    except ValueError:continue
                    if r['topic'] in ('supervisor_status','transport_status'):self.status[r['topic']]=r['data']
                s=self.status.get('supervisor_status',{});t=self.status.get('transport_status',{})
                if s.get('hard_fault') or s.get('input_health')=='ACTUATOR_FAULT':
                    self.message.set('小车/安全链路已锁止：请松键，不要反复重开；断电检查再上电后重新检查。不会自动清除急停。')
                elif t.get('local_stop') or not t.get('fresh',True):self.message.set('小车反馈不稳定/通信门锁止：不能使能。请检查Tenda、电源、接线；不要反复按准备。')
                elif s.get('state')=='ARMED':self.message.set('准备中，或已就绪：请以跟随窗口的提示为准；只有轻按并松开空格一次才运动。')
                elif s.get('state')=='ACTIVE':self.message.set('正在跟随：保持跟随窗口焦点；再次空格或Esc结束本轮。')
                elif s.get('pilot',{}).get('used'):
                    from mecanum_wand_4ws8_control.pose_quality import stop_hint
                    self.message.set(stop_hint(s) or '本轮已结束；等待停稳后检查结果，不会自动续跑。')
            if self.backend.poll() is not None:
                code=self.backend.returncode;self.backend=None;self.good_at=-1e9
                for v in self.checks:v.set(False)
                self.message.set(f'跟随窗口已退出（{code}）。下一轮先检查结果，再重新检查连接、确认现场；不会自动启动。')
                self.append('运行日志：'+str(self.trial/'launch.log'))
        if self.backend is None and not self.busy and self.good_at>0 and time.monotonic()-self.good_at>120.:
            self.message.set('检查已过期，请点“重新检查连接”；确认期间不计入120秒运动时间。')
        self.update_buttons();self.poll_job=self.root.after(100,self.poll)
    def close(self):
        if self.closed:return
        if self.busy:
            messagebox.showinfo('检查进行中','请等待本次有界连接检查结束后再关闭。');return
        self.closed=True
        self.root.after_cancel(self.poll_job)
        self.closing_at=time.monotonic()
        self.closing_children=[child for child in (self.backend,self.receiver) if child is not None]
        for child in self.closing_children:
            if child is not None and child.poll() is None:
                try:os.killpg(child.pid,signal.SIGINT)
                except ProcessLookupError:pass
        self.message.set('正在停止本次拥有的程序；保持松键，异常时立即断电。已有的共享Vicon接收器不会被关闭。')
        self.root.after(100,self.finish_close)
    def finish_close(self):
        live=[child for child in self.closing_children if child.poll() is None]
        if not live:self.root.destroy();return
        age=time.monotonic()-self.closing_at
        if age>5.:
            for child in live:
                try:os.killpg(child.pid,signal.SIGTERM if age<8. else signal.SIGKILL)
                except ProcessLookupError:pass
        self.root.after(100,self.finish_close)

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--check',action='store_true');ap.add_argument('--preview',action='store_true')
    a=ap.parse_args()
    if a.check:verify_files();print('PASS: 一键入口文件与固件读回核验通过；未初始化ROS、未连接设备、未启动小车。');return
    instance=None if a.preview else single_instance()
    if not a.preview and instance is None:print('操作台已经打开，已显示原窗口；没有重复启动。');return
    root=tk.Tk();app=App(root,instance,a.preview)
    signal.signal(signal.SIGINT,lambda *_:root.after(0,app.close))
    signal.signal(signal.SIGTERM,lambda *_:root.after(0,app.close))
    try:root.mainloop()
    finally:
        if instance:
            instance[1].close();instance[0].close()
            if instance[2].exists():instance[2].unlink()
if __name__=='__main__':
    try:main()
    except Exception as e:
        print('无法打开操作台：'+str(e),file=sys.stderr)
        try:
            root=tk.Tk();root.withdraw();messagebox.showerror('小车操作台未启动',str(e));root.destroy()
        except tk.TclError:pass
        raise SystemExit(2)
