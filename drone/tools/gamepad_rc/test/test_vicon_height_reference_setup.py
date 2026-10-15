from pathlib import Path
import struct
import sys

import pytest
from pymavlink.dialects.v20 import common as m
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'ros2_ws/src/uav_coord_bridge'))
from vicon_height_reference_setup import allowed, write_message, TARGET
from usb_vision_policy import shell_message


def enc():
    return m.MAVLink(None, srcSystem=250, srcComponent=190)


@pytest.mark.parametrize('name', TARGET)
def test_only_selected_reference_changes_with_all_gates(name):
    msg = write_message(enc(), name)
    gates = dict(identity_safe=True, locked=True, backup_verified=True, write_authorized=True)
    assert allowed(msg, **gates)
    for gate in gates:
        assert not allowed(msg, **{**gates, gate: False})
    assert struct.unpack('<i', struct.pack('<f', msg.param_value))[0] == TARGET[name]
    msg.target_system = 2
    assert not allowed(msg, **gates)


@pytest.mark.parametrize('name,value', [('EKF2_EV_CTRL', 3), ('EKF2_HGT_REF', 2),
                                      ('EKF2_RNG_CTRL', 2), ('COM_ARM_HFLT_CHK', 0)])
def test_no_fusion_activation_restore_or_safety_disable(name, value):
    wire = struct.unpack('<f', struct.pack('<i', value))[0]
    msg = enc().param_set_encode(1, 1, name.encode(), wire, m.MAV_PARAM_TYPE_INT32)
    assert not allowed(msg, identity_safe=True, locked=True, backup_verified=True, write_authorized=True)


@pytest.mark.parametrize('command', [176, 185, 241, 246, 400, 511])
def test_no_reboot_unlock_arm_modes_or_stream_changes(command):
    msg = enc().command_long_encode(1, 1, command, 0, 1, 0, 0, 0, 0, 0, 0)
    assert not allowed(msg, identity_safe=True, locked=True, backup_verified=True, write_authorized=True)


def test_shell_fixed_lock_read_only_and_no_destroy():
    assert allowed(shell_message(enc(), 'actuator_armed'), identity_safe=True)
    assert not allowed(shell_message(enc(), 'timesync_status'), identity_safe=True)
    assert not allowed(shell_message(enc(), close=True), identity_safe=True)


def test_pose_always_rejected():
    from uav_coord_bridge.mavlink_vision import odometry_message
    msg = odometry_message(enc(), 123, [0, 0, 0], [1, 0, 0, 0])
    assert not allowed(msg, identity_safe=True, locked=True, backup_verified=True, write_authorized=True)


def test_firmware_reference_dependency_and_verify_only_write_block():
    from vicon_height_reference_setup import ENDPOINT
    assert ENDPOINT['EKF2_HGT_REF'] == 3
    assert ENDPOINT['EKF2_EV_CTRL'] == 2
    assert ENDPOINT['EKF2_RNG_CTRL'] == 0
    for name in TARGET:
        assert not allowed(write_message(enc(), name), identity_safe=True, locked=True,
                           backup_verified=True, write_authorized=False)
