import copy
import queue
from types import SimpleNamespace as NS
import unittest

from ros_tuning import instrument
from isolated_reader import IsolatedReader, publish_latest


class ReaderTests(unittest.TestCase):
    def fixture(self):
        reader = object.__new__(IsolatedReader)
        reader.packet = reader.last_state = None
        reader.rejected = 0
        reader.raw_rejected = reader.ipc_rejected = 0
        reader.ipc_rejection_history = []
        reader.error = ''
        reader.process = NS(is_alive=lambda: True)
        reader.channel = queue.Queue(maxsize=1)
        packet = instrument.original.synthetic_packet(.1, .2, .1, 10., 100.)
        state = dict(kind='source', sent_steady=10., packet=packet.payload(),
            status=dict(flags={'sign': True}, rejected=0, error='', complete_packets=1,
                domain0_command_publishers=[],
                car01_reference_only=dict(valid=False, pose=None, stamp=None,
                                         used_as_motion_feedback=False)))
        return reader, packet, state

    def test_original_source_stamps_and_age_are_never_renewed(self):
        reader, packet, state = self.fixture()
        reader.ingest(state, 10.05, 100.05)
        self.assertEqual(reader(10.06, 100.06).payload(), packet.payload())
        self.assertEqual(reader.packet.received, 10.)
        self.assertEqual(reader.packet.digest, packet.digest)
        self.assertIsNone(reader(10.201, 100.201))
        # New IPC heartbeat with the SAME expired raw packet cannot refresh it.
        state['sent_steady'] = 10.21
        reader.ingest(state, 10.21, 100.21)
        self.assertIsNone(reader.packet)
        self.assertGreater(reader.rejected, 0)
        # Publication can occur after the old tick clock but before IPC get.
        # Poll uses the actual RECEIPT clock; Source samples AFTER the poll.
        reader, packet, state = self.fixture()
        state['sent_steady']=10.055
        reader.channel.put_nowait(state)
        reader.poll(lambda:(10.06, 100.06))
        self.assertEqual(reader.rejected, 0)
        self.assertEqual(reader(10.061, 100.061).digest, packet.digest)

    def test_stale_ipc_tamper_and_geometry_rejected(self):
        for mode in ('ipc_age', 'digest', 'marker', 'mixed_stamp'):
            reader, packet, state = self.fixture()
            if mode == 'ipc_age': state['sent_steady'] = 9.
            elif mode == 'digest': state['packet']['digest'] = '0'*64
            elif mode == 'marker': state['packet']['samples'][1]['position'] = [99., 99., 99.]
            else: state['packet']['samples'][1]['stamp'] += .001
            reader.ingest(state, 10.05, 100.05)
            with self.subTest(mode=mode):
                self.assertIsNone(reader.packet)
                self.assertIsNone(reader(10.05, 100.05))
                self.assertGreater(reader.rejected, 0)
        # Raw and IPC failures have independent monotonic counters: an IPC
        # rejection must not hide a later raw rejection, or vice versa.
        reader, packet, state = self.fixture()
        state['status']['rejected'] = 5
        reader.ingest(state, 10.05, 100.05)
        reader.reject('fixture IPC failure', 10.06)
        self.assertEqual(reader.rejected, 6)
        state['status']['rejected'] = 6
        reader.ingest(state, 10.07, 100.07)
        self.assertEqual(reader.rejected, 7)

    def test_one_slot_drops_old_input_not_command_reference_fifo(self):
        channel = queue.Queue(maxsize=1)
        for i in range(100): self.assertTrue(publish_latest(channel, i))
        self.assertEqual(channel.qsize(), 1)
        self.assertEqual(channel.get_nowait(), 99)

    def test_child_exit_revokes_cached_packet_immediately(self):
        reader, packet, state = self.fixture()
        reader.ingest(state, 10.05, 100.05)
        self.assertIsNotNone(reader(10.06, 100.06))
        reader.process = NS(is_alive=lambda: False)
        self.assertIsNone(reader(10.06, 100.06))


if __name__ == '__main__': unittest.main(verbosity=2)
