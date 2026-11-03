import copy
from dataclasses import FrozenInstanceError, replace
import unittest
from unittest.mock import patch

from ros_tuning import instrument
from packet_work import CachedPacket, Packet, Raw, decode_packet


class PacketTests(unittest.TestCase):
    def packet(self):
        return instrument.original.synthetic_packet(.1, .2, .1, 10., 100.)

    def test_original_geometry_and_digest_equal_but_freshness_not_memoized(self):
        original = self.packet()
        cached = CachedPacket(original.samples)
        self.assertEqual(cached.digest, original.digest)
        self.assertEqual(cached.payload(), original.payload())
        expected = original.pose(10., 100.)
        self.assertEqual(cached.pose(10., 100.), expected)
        with patch.object(Packet, 'pose', side_effect=AssertionError('static work repeated')):
            self.assertEqual(cached.pose(10.19, 100.19), expected)
            for now, ros in [(10.201, 100.201), (10.1, 100.21), (9.99, 99.99), (10.01, 99.99)]:
                with self.subTest(now=now, ros=ros):
                    with self.assertRaises(ValueError): cached.pose(now, ros)

    def test_each_consumer_decodes_and_checks_independently(self):
        wire = self.packet().payload()
        a, b = decode_packet(wire, 10., 100.), decode_packet(wire, 10., 100.)
        self.assertIsNot(a, b)
        self.assertIsNot(a.samples[0], b.samples[0])
        self.assertEqual(a.digest, b.digest)
        for mode in ('digest', 'flip', 'marker', 'mixed', 'nan', 'names'):
            value = copy.deepcopy(wire)
            if mode == 'digest': value['digest'] = '0'*64
            elif mode == 'flip': value['samples'][0]['quaternion'] = [1., 0., 0., 0.]
            elif mode == 'marker': value['samples'][1]['position'] = [99., 99., 99.]
            elif mode == 'mixed': value['samples'][1]['stamp'] += .001
            elif mode == 'nan': value['samples'][2]['position'] = [float('nan'), 0., 0.]
            else: value['samples'][1]['name'] = 'Other'
            with self.subTest(mode=mode):
                with self.assertRaises(ValueError): decode_packet(value, 10., 100.)
        self.assertIsNotNone(a.pose(10.1, 100.1))

    def test_no_mutable_source_aliases(self):
        sample = self.packet().samples[0]
        with self.assertRaises(ValueError): CachedPacket([sample])
        with self.assertRaises(ValueError): CachedPacket((replace(sample, position=list(sample.position)),))
        cached = CachedPacket(self.packet().samples)
        with self.assertRaises(FrozenInstanceError): cached.samples[0].stamp = 11.
        with self.assertRaises(FrozenInstanceError): cached.samples = ()


if __name__ == '__main__': unittest.main(verbosity=2)
