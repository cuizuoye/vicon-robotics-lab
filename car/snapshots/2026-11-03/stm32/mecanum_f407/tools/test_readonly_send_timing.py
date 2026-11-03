import unittest
from readonly_send_timing import TimedSocket,summarize_sends
from mecanum_vicon_control.transport_protocol import encode_frame,pack_cmd_vel,MessageType

class SendTimingTests(unittest.TestCase):
    def test_transparent_and_zero(self):
        class Fake:
            def sendto(self,data,peer):self.received=(data,peer);return len(data)
            def close(self):self.closed=True
        clock=iter([10.1,10.102]);sock=Fake();wrapper=TimedSocket(sock,10,lambda:next(clock))
        raw=encode_frame(MessageType.CMD_VEL,2,9,0,pack_cmd_vel(0,0,0,timeout_ms=150))
        self.assertEqual(wrapper.sendto(raw,('127.0.0.1',1)),len(raw))
        self.assertEqual(sock.received[0],raw)
        self.assertTrue(wrapper.records[0]['disabled_zero']);wrapper.close();self.assertTrue(sock.closed)
    def test_error_preserved(self):
        class Fake:
            def sendto(self,*args):raise OSError(12,'test')
        clock=iter([1,1.001]);wrapper=TimedSocket(Fake(),0,lambda:next(clock))
        with self.assertRaises(OSError):wrapper.sendto(b'x',('x',1))
        self.assertEqual(wrapper.records[0]['error_errno'],12)
    def test_cutoff_and_session_boundaries(self):
        def row(at,seq,sid):return dict(kind=16,start_s=at,end_s=at+.001,
            sequence=seq,session_id=sid,disabled_zero=True,timeout_ms=150)
        records=[row(0,1,8),row(.02,2,8),row(2,1,9),row(2.3,2,9)]
        summary=summarize_sends(records,2.1)
        self.assertEqual(summary['command_count'],3)
        self.assertAlmostEqual(summary['max_same_session_send_gap']['ms'],20)
        self.assertTrue(summary['all_commands_disabled_zero'])

if __name__=='__main__':unittest.main()
