import json
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import incident_guard as g


def test_retained_crash_block_fails_closed(tmp_path,monkeypatch):
    path=tmp_path/'incident.json'
    monkeypatch.setattr(g,'INCIDENT',path)
    g.require_active_bench_clear()
    path.write_text(json.dumps({'block_active_benches':True,'evidence':'retained.json'}))
    with pytest.raises(RuntimeError,match='active fusion'):g.require_active_bench_clear()
