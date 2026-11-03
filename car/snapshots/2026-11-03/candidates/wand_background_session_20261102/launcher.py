"""Persistent localhost start page. Manual retry starts a NEW disabled check.

This module never imports ROS, creates a hardware socket or synthesizes input.
Only the already-authorized ground entry may construct a real runtime.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import secrets
import subprocess
import threading
import time


class Controller:
    def __init__(self, command, folder, env, cwd):
        self.command, self.folder, self.env, self.cwd = command, Path(folder), env, cwd
        self.lock = threading.RLock()
        self.process = self.thread = None
        self.closed = False
        self.value = dict(phase='IDLE', attempt=0, busy=False, retry_allowed=True,
                          message='等待浏览器连接；尚未打开小车通信', runtime_url=None)

    def snapshot(self):
        with self.lock:
            return dict(self.value)

    def start(self):
        with self.lock:
            if self.closed or self.value['busy'] or not self.value['retry_allowed']:
                raise ValueError('上一会话尚未结束或未确认停车，禁止重复启动')
            n = self.value['attempt'] + 1
            target = self.folder / ('attempt_%03d' % n)
            target.mkdir()
            self.value.update(phase='CHECKING', attempt=n, busy=True, retry_allowed=False,
                              message='正在启动失能零速检查，不会自动运动', runtime_url=None,
                              log=str(target), stop_confirmed=None)
            self.thread = threading.Thread(target=self._run, args=(target,), daemon=True)
            self.thread.start()
            return dict(self.value)

    def _set(self, **values):
        with self.lock:
            self.value.update(values)

    def _run(self, target):
        ready = False
        result = None
        code = -1
        error_tail = ''
        try:
            with (target / 'stdout.log').open('x') as output, (target / 'stderr.log').open('x') as error:
                with self.lock:
                    if self.closed:
                        return
                    self.process = subprocess.Popen(self.command, env=self.env, cwd=self.cwd,
                                                    stdout=subprocess.PIPE, stderr=error, text=True, bufsize=1)
                for line in self.process.stdout:
                    output.write(line)
                    output.flush()
                    if line.startswith('STARTUP_PHASE '):
                        phase = json.loads(line.split(' ', 1)[1])
                        self._set(phase=phase['phase'], message=phase['message'])
                    elif line.startswith('BROWSER_READY '):
                        value = json.loads(line.split(' ', 1)[1])
                        # URLs are trusted only from this exact local child, still validate shape.
                        from urllib.parse import urlsplit
                        u = urlsplit(value['url'])
                        if u.scheme != 'http' or u.hostname != '127.0.0.1' or not u.port or not u.fragment:
                            raise ValueError('invalid local runtime URL')
                        ready = True
                        self._set(phase='BROWSER_READY', message='自检通过，正在打开操作台；单击空格前不会运动',
                                  runtime_url=value['url'])
                    elif line.startswith('BROWSER_RESULT '):
                        result = json.loads(line.split(' ', 1)[1])
                        self._set(phase='STOPPING', message='本轮结束：' + str(result['reason']) + '；正在确认停车')
                code = self.process.wait()
            error_tail = (target / 'stderr.log').read_text()[-1600:].strip()
        except Exception as error:
            error_tail = str(error)
        finally:
            with self.lock:
                child = self.process
            if child is not None and child.poll() is None:
                child.terminate()
                try:
                    child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait(timeout=2)
            confirmed = None
            if result is not None:
                try:
                    r = json.loads((Path(result['session']) / 'result.json').read_text())
                    confirmed = bool(r['stop_evidence']['stop_confirmed'] and r['stop_evidence']['gate_output_zero'])
                except (KeyError, OSError, TypeError, ValueError):
                    confirmed = False
            allow = not ready or confirmed is True
            message = ('本轮已停稳：' + result['reason']) if confirmed else (
                '启动检查失败：' + (error_tail or '进程退出码 ' + str(code)))
            if ready and confirmed is not True:
                message = '未取得停车确认；先确认实车停止，必要时断电。此启动器禁止再次启动。' + message
            self._set(phase='STOPPED' if confirmed else 'BLOCKED', busy=False, retry_allowed=allow,
                      runtime_url=None, message=message, stop_confirmed=confirmed, exit_code=code)
            (target / 'RESULT.json').write_text(json.dumps(self.snapshot(), ensure_ascii=False, indent=2) + '\n')

    def close(self):
        with self.lock:
            self.closed = True
            child = self.process
        if child is not None and child.poll() is None:
            child.terminate()
        if self.thread:
            self.thread.join(timeout=13)


def make_server(controller, web):
    token = secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def valid(self, write=False):
            origin = 'http://127.0.0.1:' + str(self.server.server_port)
            return (self.headers.get('Host') == origin[7:]
                    and (not write or self.headers.get('Origin') == origin)
                    and secrets.compare_digest(self.headers.get('X-Launch-Token', ''), token))

        def reply(self, code, data, mime='application/json'):
            body = data.encode() if isinstance(data, str) else json.dumps(data, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', mime + '; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; frame-ancestors 'none'; object-src 'none'")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/status':
                self.reply(200, controller.snapshot()) if self.valid() else self.reply(403, {})
            elif self.path in ('/', '/launcher.js', '/launcher.css') and self.headers.get('Host') == '127.0.0.1:' + str(self.server.server_port):
                file, mime = {'/': ('launcher.html', 'text/html'), '/launcher.js': ('launcher.js', 'text/javascript'),
                              '/launcher.css': ('launcher.css', 'text/css')}[self.path]
                self.reply(200, (Path(web) / file).read_text(), mime)
            else:
                self.reply(404, {})

        def do_POST(self):
            if self.path != '/check' or not self.valid(write=True):
                self.reply(403, {})
                return
            try:
                n = int(self.headers.get('Content-Length', '0'))
                if not 0 < n <= 64 or self.headers.get('Content-Type') != 'application/json':
                    raise ValueError('invalid request')
                if json.loads(self.rfile.read(n)) != {'action': 'check'}:
                    raise ValueError('only a fresh manual check is allowed')
                self.reply(200, controller.start())
            except (ValueError, OSError) as error:
                self.reply(409, dict(error=str(error)))
    server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
    return server, 'http://127.0.0.1:%d/#%s' % (server.server_port, token)
