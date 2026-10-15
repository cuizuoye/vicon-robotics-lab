from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from px4_readonly_backup import wait_px4_heartbeat


def message(system, component, autopilot):
    return SimpleNamespace(get_srcSystem=lambda: system, get_srcComponent=lambda: component,
                           autopilot=autopilot)


class FakeLink:
    def __init__(self, messages):
        self.messages = iter(messages)
        self.sent = []
        self.mav = SimpleNamespace(heartbeat_send=lambda *args: self.sent.append(args))

    def recv_match(self, **kwargs):
        return next(self.messages, None)


def test_forwarded_gcs_and_other_components_do_not_identify_aircraft():
    fc = message(1, 1, 12)
    link = FakeLink([message(250, 190, 8), message(1, 88, 12), fc])
    assert wait_px4_heartbeat(link) is fc
    assert (link.target_system, link.target_component) == (1, 1)
    assert all(args[:3] == (6, 8, 0) for args in link.sent)


def test_wrong_autopilot_on_fc_identity_is_rejected():
    with pytest.raises(RuntimeError):
        wait_px4_heartbeat(FakeLink([message(1, 1, 3)]))
