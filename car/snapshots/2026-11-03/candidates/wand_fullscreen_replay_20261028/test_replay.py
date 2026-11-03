"""Offline tests; no Tk window, live ROS graph or hardware connection."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import replay_diagnostic as replay


class ReadOnlyTests(unittest.TestCase):
    def test_packet_and_authority_fields_not_exported(self):
        data = dict(state='ACTIVE', epoch='do-not-copy', pid=123,
                    issued_ros=123., raw_packet='do-not-copy', goal={'x':1.,'y':2.,'yaw':0.})
        result = replay.selected_data('supervisor_status', data)
        self.assertEqual(set(result), {'state','goal'})

    def test_gate_export_is_display_only(self):
        result = replay.selected_data('gate_status', dict(fault='', speed_mps=.2,
            output_zero=False, primed_epoch='private', permit='private'))
        self.assertNotIn('permit', result)
        self.assertNotIn('primed_epoch', result)

    def test_pose_drops_header_and_identity(self):
        data = dict(x=1.,y=2.,z=.1,qx=0.,qy=0.,qz=0.,qw=1., stamp=123.,frame='map')
        self.assertEqual(set(replay.selected_data('car_pose',data)),
                         {'x','y','z','qx','qy','qz','qw'})

    def test_recorded_time_and_order_preserved(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            original = dict(x=0.,y=0.,z=0.,qx=0.,qy=0.,qz=0.,qw=1.)
            for name in ('a','b'):
                (root/name).mkdir()
                lines = [dict(received_steady=5.,topic='operator_lease',data={'held':True}),
                         dict(received_steady=5.1,topic='car_pose',data=original),
                         dict(received_steady=7.,topic='gate_status',data={'fault':''})]
                (root/name/'observation.jsonl').write_text('\n'.join(map(json.dumps,lines)))
            output = root/'output'
            output.mkdir()
            with patch.object(replay,'PREPARED',root), patch.object(replay,'SESSIONS',('a','b')):
                manifest = replay.extract(output)
            events = [json.loads(x) for x in (output/'renderer_events.jsonl').read_text().splitlines()]
            self.assertAlmostEqual(events[0][0],.1)
            self.assertAlmostEqual(events[2][0],3.1)
            self.assertEqual(len(events),4)
            self.assertEqual(manifest['duration_s'],6.)
            self.assertNotIn('operator_lease',manifest['topic_counts'])

    def test_percentiles_ignore_non_draw_cycles(self):
        self.assertEqual(replay.quantiles([None,1.,2.,3.])['n'],3)
        self.assertEqual(replay.quantiles([None]),{})

    def test_only_unwritten_terminal_bytes_may_be_skipped(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);(root/'a').mkdir();(root/'out').mkdir()
            row=json.dumps(dict(received_steady=5.,topic='gate_status',data={'fault':''}))+'\n'
            log=root/'a/observation.jsonl'
            log.write_text(row+'\x00'*20)
            with patch.object(replay,'PREPARED',root), patch.object(replay,'SESSIONS',('a',)):
                result=replay.extract(root/'out')
                self.assertEqual(result['segments'][0]['excluded_unwritten_tail'][0]['nul_bytes'],20)
                log.write_text(row+'\x00\n'+row)
                with self.assertRaises(ValueError):replay.extract(root/'out')
                log.write_text(row+'{broken json')
                with self.assertRaises(json.JSONDecodeError):replay.extract(root/'out')

    def test_exact_owned_process_only(self):
        class FakeChild:
            def __init__(self):self.actions=[]
            def poll(self):return None
            def terminate(self):self.actions.append('terminate_self')
            def wait(self,timeout):self.actions.append('wait_self')
        child=FakeChild()
        replay.terminate_owned(child)
        self.assertEqual(child.actions,['terminate_self','wait_self'])

    def test_ros_import_does_not_initialize_ros(self):
        original = replay.load_ui()
        self.assertFalse(original.rclpy.ok())
        self.assertEqual(original.Console.__name__,'RotationConsole')

    def test_live_io_rejected_in_separate_process(self):
        code = '''import replay_diagnostic as r
r.load_ui()
attempts=r.deny_live_io()
import rclpy,socket,subprocess
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
for action in (rclpy.init,lambda:Node('forbidden'),SingleThreadedExecutor,
               socket.socket,lambda:subprocess.Popen(['true']),
               lambda:open('/dev/ttyACM0','rb')):
 try:action()
 except RuntimeError:pass
 else:raise AssertionError('live IO escaped')
assert len(attempts)==6
assert not rclpy.ok()
print('denied all 6 live IO paths')
'''
        result = replay.subprocess.run([replay.sys.executable,'-c',code],
            cwd=replay.HERE, text=True,capture_output=True,timeout=15.)
        self.assertEqual(result.returncode,0,result.stderr)
        self.assertIn('denied all 6',result.stdout)


if __name__=='__main__':unittest.main()
