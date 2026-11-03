"""File-only by default; explicitly authorized launch opens a persistent status page."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import sys
import tempfile
import webbrowser
from build import HERE, OUT, save
import ground_entry
from launcher import Controller, make_server


def main():
    if '--launch' not in sys.argv[1:]:
        return ground_entry.main()
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--launch', action='store_true', required=True)
    p.add_argument('--site-ready', action='store_true', required=True)
    p.add_argument('--operator-trial', action='store_true', required=True)
    p.add_argument('--seconds', type=float, default=600.)
    p.add_argument('--no-open', action='store_true')
    a = p.parse_args()
    if not 1 <= a.seconds <= 600:
        raise ValueError('duration must be 1..600 seconds')
    ground_entry.check(ground_entry.current())
    lock = (OUT / 'LAUNCHER.lock').open('a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        info = OUT / 'LAUNCHER.json'
        raise ValueError('已有启动器，请使用原页面：' + (info.read_text() if info.exists() else str(info)))
    folder = Path(tempfile.mkdtemp(prefix='launcher_', dir=OUT))
    folder.chmod(0o700)
    command = ['/usr/bin/python3', '-B', str(HERE / 'ground_entry.py'), '--launch', '--site-ready',
               '--operator-trial', '--seconds', str(a.seconds), '--no-open']
    controller = Controller(command, folder, dict(os.environ), HERE)
    server, url = make_server(controller, HERE)
    controller.env['WAND_LAUNCHER_URL'] = url
    save(OUT / 'LAUNCHER.json', dict(url=url, pid=os.getpid(), folder=str(folder)))
    (OUT / 'LAUNCHER.json').chmod(0o600)
    print('启动检查页面（同一页面支持手动重新检查）：' + url, flush=True)
    print('尚未打开小车通信；页面连接后才开始失能零速检查。', flush=True)
    if not a.no_open and not webbrowser.open_new_tab(url):
        print('浏览器未能自动打开，请复制上方完整地址。', flush=True)
    done = False
    def end(*_):
        nonlocal done
        done = True
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, end)
    server.timeout = .3
    try:
        while not done:
            server.handle_request()
    finally:
        controller.close()
        server.server_close()
        lock.close()


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, KeyError) as error:
        raise SystemExit('BLOCKED: ' + str(error))
