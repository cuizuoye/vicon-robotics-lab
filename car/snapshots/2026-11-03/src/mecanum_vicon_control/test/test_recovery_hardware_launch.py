from pathlib import Path


def launch_text():
    root = Path(__file__).resolve().parents[1]
    return (root / 'launch/vicon_recovery_line_hardware.launch.py').read_text()


def test_recovery_hardware_launch_is_inert_and_pins_4wgb():
    text = launch_text()
    assert "executable='path_recovery_controller'" in text
    assert "executable='cmd_vel_udp_bridge'" in text
    assert "'required_motion_tag': '4WGB'" in text
    assert "'wire_timeout_ms': 600" in text
    assert "'tracking_cross_speed_mps': ParameterValue(" in text
    assert "'recovery_cross_speed_mps': ParameterValue(" in text
    assert "'invert_body_xy', default_value='false'" in text
    assert "'start_enabled': False" in text
    assert 'rectangle_run_guard' not in text
    assert 'simulated_vicon_car' not in text


def test_recovery_hardware_launch_starts_with_one_bounded_line():
    text = launch_text()
    assert "'shape': 'line'" in text
    assert "DeclareLaunchArgument('line_length_m', default_value='1.0')" in text
    assert "DeclareLaunchArgument('max_linear_mps', default_value='0.15')" in text
    assert "'waypoint_count': 2" in text
    assert "'loop_count': 1" in text
