import struct
import unittest
from wifi_echo_probe import decode_echo, wait_for_hello_ack, TAG, HEALTH_TAG, RADIO_TAG
from mecanum_vicon_control.transport_protocol import Frame, MessageType, FrameFlags
class EchoTests(unittest.TestCase):
    def test_radio_schema(self):
        f=Frame(MessageType.LINK_STATUS,2,7,9,100,
            struct.pack('<11IiI7I',RADIO_TAG,7,20,100,160,21,0,0,220000,130000,0,-39,0,
                        8,99,160,99000,160000,123,2))
        r=decode_echo(f)
        self.assertEqual(r['radio_count'],8)
        self.assertEqual(r['radio_max_hardware_gap_us'],160000)
        self.assertEqual(r['radio_retries'],2)
        bad=Frame(f.message_type,2,7,9,100,struct.pack('<I',HEALTH_TAG)+f.payload[4:])
        with self.assertRaises(ValueError):decode_echo(bad)
    def test_wait_ack(self):
        class Fake:
            def __init__(self,frames):self.frames=frames;self.now=0
            def receive(self):result=self.frames;self.frames=[];return result
            def clock(self):return self.now
            def sleep(self,delay):self.now+=delay
        ack=Frame(MessageType.HELLO_ACK,2,1,9,100,b'')
        good=Fake([ack]);self.assertEqual(wait_for_hello_ack(good,9,1,clock=good.clock,sleep=good.sleep),0)
        missing=Fake([])
        with self.assertRaises(TimeoutError):
            wait_for_hello_ack(missing,9,1,timeout=.003,clock=missing.clock,sleep=missing.sleep)
        for invalid in [Frame(ack.message_type,0,1,9,100,b''),
                        Frame(ack.message_type,2,2,9,100,b''),
                        Frame(ack.message_type,2,1,9,100,b'x')]:
            bad=Fake([invalid])
            with self.assertRaises(ValueError):wait_for_hello_ack(bad,9,1,clock=bad.clock,sleep=bad.sleep)
    def test_wifi_health_schema(self):
        f=Frame(MessageType.LINK_STATUS,2,7,9,100,
            struct.pack('<11IiI',HEALTH_TAG,7,20,100,160,21,25,12,220000,130000,0,-39,0))
        r=decode_echo(f)
        self.assertEqual(r['last_errno'],12)
        self.assertEqual(r['internal_free'],220000)
        self.assertEqual(r['rssi'],-39)
        self.assertEqual(r['ps_type'],0)
    def test_wifi_schema(self):
        f=Frame(MessageType.LINK_STATUS,FrameFlags.RESPONSE,7,9,100,struct.pack('<7I',TAG,7,20,100,160,21,0))
        self.assertEqual(decode_echo(f)['esp_rx_gap_ms'],160)
        for changed in [Frame(MessageType.WHEEL_TELEMETRY,2,7,9,100,f.payload),
                        Frame(f.message_type,0,7,9,100,f.payload),
                        Frame(f.message_type,2,8,9,100,f.payload),
                        Frame(f.message_type,2,7,9,101,f.payload),
                        Frame(f.message_type,2,7,9,100,b'')]:
            with self.assertRaises(ValueError):decode_echo(changed)
    def test_not_mcu_schema(self):
        f=Frame(MessageType.LINK_STATUS,2,7,9,100,struct.pack('<7I',0x314f5234,7,20,100,0,21,0))
        with self.assertRaises(ValueError):decode_echo(f)
if __name__=='__main__':unittest.main()
