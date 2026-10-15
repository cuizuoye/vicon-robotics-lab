from pathlib import Path
import subprocess
import sys


def test_old_full_gamepad_controller_blocked_before_opening_device_or_link():
    script=Path(__file__).resolve().parents[1]/'gamepad_rc.py'
    result=subprocess.run([sys.executable,str(script),'--link','serial:/not-a-device:115200'],
                          capture_output=True,text=True,timeout=5)
    assert result.returncode==2
    assert 'Retained mavlink_if3 crash incident' in result.stderr
    assert '[mav]' not in result.stdout and '[dev]' not in result.stdout
