import unittest

from ros_four_link_ro_observe import parse_args


class RosReadonlyConfirmationTests(unittest.TestCase):
    def test_physical_confirmation_is_explicit_and_exclusive(self):
        base=['--esp-ip','192.168.50.1','--interface','wlan-test',
              '--output','/tmp/ros-four-ro-confirmation-test.json']
        disconnected=parse_args(base+['--confirm-battery-disconnected'])
        self.assertTrue(disconnected.confirm_battery_disconnected)
        self.assertFalse(disconnected.confirm_battery_powered_motors_connected_wheels_suspended)
        powered=parse_args(base+['--confirm-battery-powered-motors-connected-wheels-suspended'])
        self.assertFalse(powered.confirm_battery_disconnected)
        self.assertTrue(powered.confirm_battery_powered_motors_connected_wheels_suspended)
        with self.assertRaises(SystemExit):
            parse_args(base)
        with self.assertRaises(SystemExit):
            parse_args(base+['--confirm-battery-disconnected',
                             '--confirm-battery-powered-motors-connected-wheels-suspended'])


if __name__=='__main__':
    unittest.main()
