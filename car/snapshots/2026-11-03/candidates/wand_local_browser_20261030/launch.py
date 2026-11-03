"""Local browser display trial. Reuse sealed READONLY transport, never ROS/car."""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import webbrowser
import tempfile

ROOT=Path('/home/PRIMARY_USER/vicon_mecanum_ws')
READY=ROOT/'docs/wand_browser_fast_2026-10-28/prepared'
DOCS=ROOT/'docs/wand_local_browser_2026-10-30'


def duration(value):
    value=float(value)
    if not math.isfinite(value) or not 1<=value<=120:
        raise argparse.ArgumentTypeError('只读显示测试时长必须在1至120秒之间')
    return value


def check():
    spec=importlib.util.spec_from_file_location('local_browser_readonly_check',READY/'check.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    result=module.check(READY)
    node=shutil.which('node')
    if not node:
        raise RuntimeError('找不到本机Node运行时')
    return dict(result,node=node,scope='LOCAL_BROWSER_READONLY_ONLY',
                ros_initialized=False,hardware_entry=False,original_demo_modified=False)


def main():
    parser=argparse.ArgumentParser(description='本机浏览器只读显示测试；不会连接或启动小车')
    parser.add_argument('--seconds',type=duration,default=120.)
    parser.add_argument('--no-open',action='store_true',help='仅打印网页地址，不调用默认浏览器')
    parser.add_argument('--check',action='store_true')
    args=parser.parse_args()
    verified=check()
    print('LOCAL_BROWSER_CHECK '+json.dumps(verified,ensure_ascii=False),flush=True)
    if args.check:
        return
    DOCS.mkdir(parents=True,exist_ok=True)
    launch_dir=Path(tempfile.mkdtemp(prefix='launch_',dir=DOCS))
    server_log=(launch_dir/'server.log').open('w')
    server_error=(launch_dir/'server_error.log').open('w')
    # Regular files, not a stdout relay pipe: terminal closure cannot EPIPE
    # the Node server while it is saving its final result.
    child=subprocess.Popen([verified['node'],str(READY/'server.cjs'),'--readonly','--port','0',
                            '--seconds',str(args.seconds)],stdout=server_log,stderr=server_error)
    follow=(launch_dir/'server.log').open()
    stopped=False
    def stop(*_):
        nonlocal stopped
        stopped=True
    for sig in (signal.SIGTERM,signal.SIGINT):signal.signal(sig,stop)
    started=time.monotonic();info=None
    try:
        while not stopped and time.monotonic()-started<args.seconds+200:
            line=follow.readline()
            if not line:
                if child.poll() is not None:break
                time.sleep(.05)
                continue
            print(line,end='',flush=True)
            if line.startswith('BROWSER_READY '):
                info=json.loads(line.split(' ',1)[1])
                if info.get('mode')!='readonly' or info.get('hardware_entry') is not False:
                    raise RuntimeError('拒绝非只读入口')
                record=dict(**info,seconds=args.seconds,scope=verified['scope'],
                            open_requested=not args.no_open,software_animation_only=True,launcher_log=str(launch_dir))
                (DOCS/'LATEST.json').write_text(json.dumps(record,ensure_ascii=False,indent=2)+'\n')
                if not args.no_open:
                    opened=webbrowser.open_new_tab(info['url'])
                    print('默认浏览器打开请求：'+str(opened)+'；若未出现窗口，请复制上面的完整地址。',flush=True)
            if line.startswith('BROWSER_RESULT '):
                print('本轮仅验证浏览器显示/心跳，不代表真实Sign或实车验收通过。',flush=True)
        if info is None:
            raise RuntimeError('网页服务未准备就绪')
    except KeyboardInterrupt:
        print('结束本机只读显示测试。',flush=True)
    finally:
        if child.poll() is None:
            child.terminate()
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:child.kill();child.wait(timeout=2)
        follow.close();server_log.close();server_error.close()


if __name__=='__main__':
    main()
