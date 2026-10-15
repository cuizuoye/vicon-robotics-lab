from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from readonly_parameter_poll import ParameterPoll


def test_no_reads_without_identity_or_during_shell():
    poll = ParameterPoll(('A', 'B'))
    assert poll.due(0, identity_ready=False, shell_pending=False) is None
    assert poll.due(1, identity_ready=True, shell_pending=True) is None
    assert poll.due(1, identity_ready=True, shell_pending=False) == 'A'
    assert poll.due(10, identity_ready=True, shell_pending=True) is None
    assert poll.due(10, identity_ready=True, shell_pending=False) == 'B'


def test_no_burst_after_pause_and_fixed_names_only():
    poll = ParameterPoll(('A', 'B', 'C'))
    assert poll.due(0, identity_ready=True, shell_pending=False) == 'A'
    assert poll.due(.01, identity_ready=True, shell_pending=False) is None
    assert poll.due(10, identity_ready=True, shell_pending=False) == 'B'
    assert poll.due(10, identity_ready=True, shell_pending=False) is None
    assert poll.due(10.1, identity_ready=True, shell_pending=False) == 'C'
    assert poll.due(10.2, identity_ready=True, shell_pending=False) == 'A'


def test_refresh_waits_and_resumes():
    poll = ParameterPoll(('A',))
    assert poll.due(0, identity_ready=True, shell_pending=False) == 'A'
    assert poll.due(1, identity_ready=True, shell_pending=False) is None
    assert poll.due(2, identity_ready=True, shell_pending=False) == 'A'


@pytest.mark.parametrize('names,interval,refresh', [((), .05, 2), (('A',), 0, 2), (('A',), .05, 0)])
def test_invalid_timing(names, interval, refresh):
    with pytest.raises(ValueError):
        ParameterPoll(names, interval, refresh)
