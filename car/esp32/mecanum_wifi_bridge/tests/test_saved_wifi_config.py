import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'tools/configure_saved_wifi.py'
spec = importlib.util.spec_from_file_location('saved_wifi_config', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SavedWifiConfigTest(unittest.TestCase):
    source = 'CONFIG_MC_WIFI_SSID="old"\nCONFIG_MC_WIFI_PASSWORD="oldsecret"\nCONFIG_MC_UART_BAUD_RATE=921600\n'

    def test_changes_only_wifi_values(self):
        result = module.configured_text(self.source, 'example', 'example-password')
        self.assertIn('CONFIG_MC_WIFI_SSID="example"', result)
        self.assertIn('CONFIG_MC_UART_BAUD_RATE=921600', result)
        self.assertNotIn('oldsecret', result)

    def test_quotes_and_backslashes_are_escaped(self):
        result = module.configured_text(self.source, 'a"b', 'example\\"password')
        self.assertIn('CONFIG_MC_WIFI_SSID="a\\"b"', result)
        self.assertIn('example\\\\\\"password', result)

    def test_rejects_missing_or_duplicate_fields(self):
        for source in ['', self.source + 'CONFIG_MC_WIFI_SSID="duplicate"\n']:
            with self.assertRaises(ValueError):
                module.configured_text(source, 'example', 'example-password')

    def test_rejects_bad_credentials(self):
        for ssid, psk in [('', 'example-password'), ('x'*32, 'example-password'),
                          ('example', ''), ('example', '<hidden>'), ('example', 'x'*64),
                          ('example', 'a'*64), ('example', '\u5bc6'*30)]:
            with self.assertRaises(ValueError):
                module.configured_text(self.source, ssid, psk)


if __name__ == '__main__':
    unittest.main()
