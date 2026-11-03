"""Conflicting check/action flags must fail at parsing, before admission or IO."""
from contextlib import redirect_stderr
import io
import json
import sys
from unittest.mock import patch
import ground_entry

def main():
    count=0
    for flags in (['--check','--launch','--site-ready','--operator-trial'],
                  ['--check','--idle-only','--confirm-disabled-zero-now'],['--check','--rehearsal'],
                  ['--launch','--idle-only'],['--launch','--rehearsal'],['--idle-only','--rehearsal']):
        with (patch.object(sys,'argv',['entry',*flags]),
              patch.object(ground_entry,'check',side_effect=AssertionError('parsing must reject before admission')),
              patch('socket.socket',side_effect=AssertionError('hardware IO forbidden')),redirect_stderr(io.StringIO())):
            try:ground_entry.main()
            except SystemExit as error:assert error.code==2
            else:raise AssertionError('conflicting modes accepted')
        count+=1
    print(json.dumps(dict(passed=True,conflicting_mode_cases=count,hardware_opened=False,admission_reached=False)))
if __name__=='__main__':main()
