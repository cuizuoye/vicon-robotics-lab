"""Persistent launcher lifecycle/security tests. All subprocesses are fake software peers."""
import http.client
import json
from pathlib import Path
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from io import StringIO
from launcher import Controller, make_server


class Child:
    def __init__(self, lines='', code=0, **kw):
        self.stdout=StringIO(lines);self.code=code;self.terminated=False
    def wait(self, **_):return self.code
    def poll(self):return self.code
    def terminate(self):self.terminated=True


def until(fn):
    end=time.monotonic()+3
    while not fn():
        if time.monotonic()>end:raise AssertionError('timeout')
        time.sleep(.005)


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.c=Controller(['never-actual-hardware'],self.root,{},self.root)
    def tearDown(self):self.c.close();self.temp.cleanup()

    def test_initial_state_creates_no_child(self):
        with patch('subprocess.Popen',side_effect=AssertionError('no process')):
            self.assertEqual(self.c.snapshot()['phase'],'IDLE')
            self.assertFalse(self.c.snapshot()['busy'])

    def test_failure_stays_visible_and_manual_retry_is_new_attempt(self):
        calls=[]
        def spawn(*args,**kw):
            calls.append(args);kw['stderr'].write('bounded startup HELLO attempts exhausted');kw['stderr'].flush()
            return Child('STARTUP_PHASE {"phase":"DISABLED_ZERO","message":"失能握手"}\n',1)
        with patch('subprocess.Popen',side_effect=spawn):
            self.c.start();until(lambda:not self.c.snapshot()['busy'])
            self.assertEqual(len(calls),1);self.assertIn('HELLO',self.c.snapshot()['message'])
            time.sleep(.1);self.assertEqual(len(calls),1,'must never retry automatically')
            self.c.start();until(lambda:not self.c.snapshot()['busy'])
            self.assertEqual(len(calls),2);self.assertEqual(self.c.snapshot()['attempt'],2)
            self.assertTrue((self.root/'attempt_001/RESULT.json').exists())
            self.assertTrue((self.root/'attempt_002/RESULT.json').exists())

    def test_double_click_cannot_create_two_writers(self):
        with self.c.lock:
            self.c.value.update(busy=True,retry_allowed=False)
        with patch('subprocess.Popen',side_effect=AssertionError('no second writer')):
            with self.assertRaises(ValueError):self.c.start()

    def test_successful_stop_allows_only_manual_new_session(self):
        session=self.root/'session';session.mkdir()
        (session/'result.json').write_text(json.dumps(dict(stop_evidence=dict(stop_confirmed=True,gate_output_zero=True))))
        lines='BROWSER_READY '+json.dumps(dict(url='http://127.0.0.1:1234/#new-epoch'))+'\n'
        lines+='BROWSER_RESULT '+json.dumps(dict(reason='CLIENT_ESC',session=str(session)))+'\n'
        with patch('subprocess.Popen',return_value=Child(lines)) as spawn:
            self.c.start();until(lambda:not self.c.snapshot()['busy'])
            self.assertEqual(self.c.snapshot()['phase'],'STOPPED');self.assertTrue(self.c.snapshot()['retry_allowed'])
            self.assertIsNone(self.c.snapshot()['runtime_url']);self.assertEqual(spawn.call_count,1)

    def test_unconfirmed_stop_blocks_retry(self):
        lines='BROWSER_READY '+json.dumps(dict(url='http://127.0.0.1:1234/#new-epoch'))+'\n'
        with patch('subprocess.Popen',return_value=Child(lines)):
            self.c.start();until(lambda:not self.c.snapshot()['busy'])
            self.assertFalse(self.c.snapshot()['retry_allowed'])
            with self.assertRaises(ValueError):self.c.start()

    def test_foreign_redirect_rejected(self):
        lines='BROWSER_READY '+json.dumps(dict(url='http://evil.example/#anything'))+'\n'
        with patch('subprocess.Popen',return_value=Child(lines)):
            self.c.start();until(lambda:not self.c.snapshot()['busy'])
            self.assertIsNone(self.c.snapshot()['runtime_url']);self.assertIn('invalid local',self.c.snapshot()['message'])

    def test_http_host_origin_token_and_action(self):
        server,url=make_server(self.c,Path(__file__).parent)
        thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        token=url.split('#')[1];origin=url.split('/#')[0];host=origin[7:]
        def request(route,method='GET',headers=None,body=None):
            conn=http.client.HTTPConnection('127.0.0.1',server.server_port,timeout=2)
            h={'Host':host,**(headers or {})};conn.request(method,route,body,h)
            r=conn.getresponse();value=r.read();conn.close();return r.status,value
        try:
            self.assertEqual(request('/')[0],200)
            self.assertEqual(request('/status')[0],403)
            self.assertEqual(request('/status',headers={'X-Launch-Token':token})[0],200)
            self.assertEqual(request('/status',headers={'Host':'evil','X-Launch-Token':token})[0],403)
            headers={'Origin':origin,'X-Launch-Token':token,'Content-Type':'application/json'}
            with patch.object(self.c,'start',return_value={'attempt':1}) as start:
                self.assertEqual(request('/check','POST',dict(headers,Origin='http://evil'),'{}')[0],403)
                self.assertEqual(request('/check','POST',headers,'{"action":"enable"}')[0],409)
                self.assertEqual(request('/check','POST',headers,'x'*65)[0],409)
                self.assertEqual(start.call_count,0)
                self.assertEqual(request('/check','POST',headers,'{"action":"check"}')[0],200)
                self.assertEqual(start.call_count,1)
        finally:server.shutdown();server.server_close();thread.join()

    def test_closed_launcher_never_reopens(self):
        self.c.close()
        with self.assertRaises(ValueError):self.c.start()


if __name__=='__main__':unittest.main()
