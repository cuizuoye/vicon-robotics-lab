"""保留崩溃证据后禁止再次主动融合；维护/零值回退保持可用，无绕过参数。"""
import json
from pathlib import Path

INCIDENT = Path(__file__).parent / 'known_incidents/micoair743_v1143_shell_crash.json'


def require_active_bench_clear(parser=None):
    if INCIDENT.exists():
        record = json.loads(INCIDENT.read_text())
        if record.get('block_active_benches') is True:
            message = ('Retained mavlink_if3 crash incident: active fusion/motion benches suspended. '
                       'Keep unpowered/props-off; use readonly or zero-restore maintenance. '
                       'Evidence: ' + record.get('evidence','unknown'))
            if parser is not None:
                parser.error(message)
            raise RuntimeError(message)
