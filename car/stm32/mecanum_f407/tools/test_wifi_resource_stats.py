import unittest
from wifi_resource_stats import FIELDS,parse_resource_line,summarize_resources

class ResourceTests(unittest.TestCase):
    def line(self,**values):
        data=dict.fromkeys(FIELDS,0);data.update(sample=100,echo_stack=2000,tcpip_stack=1500,**values)
        return 'I (200) mc_bridge: WIFI_RESOURCES '+' '.join(f'{k}={v}' for k,v in data.items())
    def test_valid_and_unavailable_pool(self):
        r=parse_resource_line(self.line(udp_rx=70))
        self.assertEqual(r['udp_rx'],70)
        s=summarize_resources([r]);self.assertFalse(s['pool_statistics_available'])
        self.assertIsNone(s['max_pool_errors'])
        self.assertEqual(s['min_echo_stack_free_bytes'],2000)
        self.assertIsNone(parse_resource_line('WIFI_HEALTH free=10000'))
        self.assertEqual(parse_resource_line('\x1b[32m'+self.line()+'\x1b[0m')['sample'],100)
    def test_malformed(self):
        for line in [self.line()+' ms=1',self.line().replace('udp_drop=0','udp_drop=-1'),
                     self.line().replace('arp_errors=0',''),self.line(pending=2)]:
            with self.assertRaises(ValueError):parse_resource_line(line)
    def test_pool_available_and_initial_snapshot(self):
        r=parse_resource_line(self.line(pools_available=1,pool_errors=3,callback_delay=7))
        s=summarize_resources([r]);self.assertEqual(s['max_pool_errors'],3)
        self.assertEqual(s['max_callback_delay_ms'],7)
        initial=dict.fromkeys(FIELDS,0)
        self.assertEqual(summarize_resources([initial]),{'valid_snapshots':0})

if __name__=='__main__':unittest.main()
