from pathlib import Path
import sys

import pytest
from pymavlink.dialects.v20 import common as m

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from persistent_shell import reply_complete
import usb_vision_policy as usb
import trs_static_fusion_policy as static
import trs_manual_fusion_policy as manual
import trs_uart_readonly_check as uart
import fc_health_readonly as health

COMMAND = b'listener actuator_armed -n 1\n'


@pytest.mark.parametrize('text', [
    'nsh> ',  # 新会话初始提示符不等于命令完成。
    'nsh> listener actuator_armed -n 1\nTOPIC: actuator_armed\n armed: False\n',
    'nsh> listener vehicle_attitude -n 1\nTOPIC: vehicle_attitude\n\nnsh> ',
    'nsh> listener actuator_armed -n 10\n\nnsh> ',
    'nsh> listener actuator_armed -n 1\n\nnsh> queued output',
])
def test_partial_old_or_different_command_replies_rejected(text):
    assert not reply_complete(text, COMMAND)


def test_ansi_crlf_fragmented_reply_waits_for_prompt():
    text='nsh> \x1b[Klistener actuator_armed -n 1\r\nTOPIC: actuator_armed\r\n armed: False\r\n\r\nnsh> \x1b[K'
    assert reply_complete(text,COMMAND)
    assert not reply_complete(text[:-12],COMMAND)
    assert not reply_complete(text,b'')


def test_continuation_erase_line_can_leave_leading_space_before_echo():
    text = ' \x1b[Klistener actuator_armed -n 1\nTOPIC: actuator_armed\n\nnsh> \x1b[K'
    assert reply_complete(text, COMMAND)
    assert not reply_complete(text, b'listener vehicle_attitude -n 1\n')


def test_all_send_policies_reject_explicit_shell_destruction():
    enc=m.MAVLink(None)
    close=enc.serial_control_encode(10,0,0,0,0,bytes(70))
    assert not usb.shell_allowed(close)
    assert not usb.send_allowed(close,safe=True,bench_authorized=True)
    assert not static.allowed(close,route='usb',safe=True)
    assert not manual.allowed(close,route='usb',safe=True)
    assert not uart.allowed(close,safe=True)
    assert not health.allowed(close,safe=True)


@pytest.mark.parametrize('filename', [
    'trs_manual_fusion_bench.py','trs_static_fusion_bench.py',
    'trs_vicon_receive_bench.py','usb_vicon_receive_bench.py',
    'height_readonly_check.py','trs_uart_readonly_check.py','fc_health_readonly.py',
    'gamepad_estop_bench.py',
])
def test_runners_never_request_shell_close(filename):
    source=(Path(__file__).resolve().parents[1]/filename).read_text()
    assert 'close=True' not in source
    if filename!='trs_uart_readonly_check.py':
        assert 'reply_complete(' in source


def test_regression_parameter_reads_only():
    enc=m.MAVLink(None)
    for name in health.PARAMS:
        msg=enc.param_request_read_encode(1,1,name.encode(),-1)
        assert health.allowed(msg,safe=True)
        assert not health.allowed(msg,safe=False)
        msg.target_system=2
        assert not health.allowed(msg,safe=True)
    # 高度参考加入固定只读集合，用于保留用户选择；其它参数仍不可任意读取。
    assert health.allowed(enc.param_request_read_encode(1,1,b'EKF2_HGT_REF',-1),safe=True)
    assert not health.allowed(enc.param_request_read_encode(1,1,b'EKF2_RNG_CTRL',-1),safe=True)
    assert not health.allowed(enc.command_long_encode(1,1,400,0,1,0,0,0,0,0,0),safe=True)
