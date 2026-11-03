"""Out-of-browser stop must validate its target and await real stop evidence."""
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import stop_command


class Tests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        self.session=self.root/'host/sessions/one';self.session.mkdir(parents=True)
        self.info=dict(url='http://127.0.0.1:1234/#secret',session=str(self.session))
        self.write();self.out=patch.object(stop_command,'OUT',self.root);self.out.start()
    def tearDown(self):self.out.stop();self.temp.cleanup()
    def write(self):(self.root/'LATEST.json').write_text(json.dumps(self.info))
    def evidence(self,confirmed=True):
        (self.session/'result.json').write_text(json.dumps(dict(reason='LOCAL_OPERATOR_STOP',stop_evidence=dict(stop_confirmed=confirmed,gate_output_zero=confirmed))))
    def test_already_stopped_never_sends_a_start_or_new_request(self):
        self.evidence()
        with patch.object(stop_command,'urlopen',side_effect=AssertionError('no IO')),redirect_stdout(StringIO()) as output:stop_command.stop()
        self.assertTrue(json.loads(output.getvalue())['stop_confirmed'])
    def test_foreign_target_and_session_are_rejected(self):
        for change in ({'url':'http://evil:1234/#token'},{'session':'/tmp/foreign'}):
            old=dict(self.info);self.info.update(change);self.write()
            with patch.object(stop_command,'urlopen',side_effect=AssertionError('no IO')):
                with self.assertRaises(ValueError):stop_command.stop()
            self.info=old
    def test_http_acceptance_without_confirmation_is_not_success(self):
        class Response:
            status=202
            def __enter__(self):return self
            def __exit__(self,*_):pass
        with patch.object(stop_command,'urlopen',return_value=Response()),patch.object(stop_command.time,'monotonic',side_effect=[0,6]),redirect_stdout(StringIO()) as output:
            with self.assertRaisesRegex(ValueError,'not confirmed'):stop_command.stop()
        self.assertEqual(output.getvalue(),'')
    def test_actual_stop_body_and_confirmation(self):
        test=self
        class Response:
            status=202
            def __enter__(self):return self
            def __exit__(self,*_):pass
        def request(req,timeout):
            test.assertEqual(req.full_url,'http://127.0.0.1:1234/operator-stop')
            test.assertEqual(json.loads(req.data),dict(type='stop',token='secret',reason='LOCAL_STOP'))
            test.assertEqual(req.get_header('Origin'),'http://127.0.0.1:1234');test.evidence();return Response()
        with patch.object(stop_command,'urlopen',side_effect=request),redirect_stdout(StringIO()) as output:stop_command.stop()
        self.assertTrue(json.loads(output.getvalue())['stop_confirmed'])

if __name__=='__main__':unittest.main()
