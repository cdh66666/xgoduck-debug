import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'pc/tools'))
import discover_unoq as discovery


class DiscoveryFallbackTests(unittest.TestCase):
    def test_missing_executable_uses_installed_python_discovery(self):
        with patch.object(discovery, 'MDNS_EXE', Path('missing-mdns-test-tool')), \
             patch.object(discovery, 'python_mdns_candidates', return_value=['http://192.168.1.8:9527']) as browse:
            self.assertEqual(discovery.mdns_candidates(seconds=.1), ['http://192.168.1.8:9527'])
            browse.assert_called_once_with(.1)

    def test_advertisements_are_private_deduplicated_hints_and_resources_close(self):
        state = {}

        class Zeroconf:
            def __init__(self, **kwargs):
                pass

            def get_service_info(self, type_, name, timeout):
                self.assert_type = type_
                return types.SimpleNamespace(parsed_addresses=lambda version:
                    ['192.168.1.8', '192.168.1.8', '8.8.8.8', '::1'])

            def close(self):
                state['closed'] = True

        class Browser:
            def __init__(self, zc, type_, listener):
                state['type'] = type_
                listener.add_service(zc, type_, 'uno')
                listener.update_service(zc, type_, 'uno')

            def cancel(self):
                state['cancelled'] = True

        fake = types.SimpleNamespace(IPVersion=types.SimpleNamespace(V4Only=1),
            Zeroconf=Zeroconf, ServiceBrowser=Browser, ServiceListener=object)
        with patch.dict(sys.modules, zeroconf=fake):
            self.assertEqual(discovery.python_mdns_candidates(.1), ['http://192.168.1.8:9527'])
        self.assertEqual(state, {'type': '_arduino._tcp.local.', 'cancelled': True, 'closed': True})


if __name__ == '__main__':
    unittest.main()
