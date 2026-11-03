import unittest
from link_timing import summarize_timing, esp_ipv4_from_logs
from mecanum_vicon_control.transport_protocol import MessageType

class TimingTests(unittest.TestCase):
    def test_station_address(self):
        lines=[dict(text='I mc_bridge: Wi-Fi ready, IP=10.236.223.215')]
        self.assertEqual(esp_ipv4_from_logs(lines),'10.236.223.215')
        for text in ['Wi-Fi ready, IP=999.1.1.1','Wi-Fi ready, IP=0.0.0.0',
                     'Wi-Fi ready, IP=127.0.0.1','Wi-Fi ready, IP=224.0.0.1',
                     'not a startup address']:
            with self.assertRaises(ValueError): esp_ipv4_from_logs([dict(text=text)])

    def test_empty(self):
        self.assertIsNone(summarize_timing([],[])['host_max_command_send_gap_s'])

    def test_gaps_and_wrap(self):
        sent=[dict(kind=int(MessageType.CMD_VEL),sequence=i,session_id=1,
                   start_s=t,end_s=t+0.001) for i,t in enumerate([0,0.02,0.04,0.24])]
        lines=[dict(text=t) for t in [
            'TRACE_EVENT kind=1 seq=2 at_ms=4294967280 duration_ms=0 detail=20',
            'TRACE_EVENT kind=2 seq=2 at_ms=4294967280 duration_ms=1 detail=1',
            'TRACE_EVENT kind=1 seq=12 at_ms=184 duration_ms=0 detail=240',
            'TRACE_EVENT kind=3 seq=9 at_ms=190 duration_ms=4 detail=1',
            'TRACE_EVENT kind=4 seq=9 at_ms=190 duration_ms=0 detail=0',
            'TRACE_EVENT kind=4 seq=9 at_ms=210 duration_ms=0 detail=0',
            'TRACE_END']]
        r=summarize_timing(sent,lines)
        self.assertEqual(r['host_command_count'],4)
        self.assertAlmostEqual(r['host_max_command_send_gap_s'],0.2)
        self.assertEqual(r['esp_max_recent_accepted_rx_gap_ms'],200)
        self.assertEqual(r['esp_max_recent_uart_write_ms'],1)
        self.assertEqual(r['esp_max_recent_telemetry_send_ms'],4)
        self.assertEqual(r['esp_max_recent_loop_gap_ms'],20)
        self.assertTrue(r['esp_dump_complete'])

    def test_no_cross_session_gaps(self):
        sent=[dict(kind=int(MessageType.CMD_VEL),session_id=s,start_s=t,end_s=t)
              for s,t in [(1,0),(2,30)]]
        self.assertIsNone(summarize_timing(sent,[])['host_max_command_send_gap_s'])

    def test_trailing_silence_not_hidden(self):
        lines=[dict(text=t) for t in [
            'TRACE_FAULT sid=4 esp_ms=9422 mcu_ms=1690 last_cmd=63 fault=1',
            'TRACE_EVENT kind=1 seq=63 at_ms=9219 duration_ms=0 detail=9552',
            'TRACE_END']]
        r=summarize_timing([],lines)
        self.assertEqual(r['esp_rx_silence_at_fault_ms'],203)
        self.assertEqual(r['esp_fault']['last_command_sequence'],63)
        self.assertIsNone(r['esp_max_recent_accepted_rx_gap_ms'])

if __name__=='__main__': unittest.main()
