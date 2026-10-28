from pathlib import Path


def launch_text():
    root = Path(__file__).resolve().parents[1]
    return (root / 'launch/vicon_rectangle_loop_recovery_hardware.launch.py').read_text()


def test_continuous_recovery_launch_is_inert_and_bounded():
    text = launch_text()
    assert "executable='path_recovery_controller'" in text
    assert "executable='continuous_loop_supervisor'" in text
    assert "executable='cmd_vel_udp_bridge'" in text
    assert "'loop_count': 0" in text
    assert "'waypoint_dwell_s': 0.80" in text
    assert "'start_enabled': False" in text
    assert "'required_motion_tag': '4WGB'" in text
    assert "'wire_timeout_ms': 600" in text
    assert "'require_run_lease': True" in text
    assert "'run_lease_timeout_s': 0.30" in text
    assert "'lap_timeout_s': 140.0" in text
    assert "default_value='3.0'" in text
    assert "default_value='2.0'" in text
    assert "default_value='0.15'" in text
    assert "'rectangle_waypoints', default_value='4'" in text
    assert "'max_recoverable_deviation_m', default_value='0.65'" in text
    assert 'rectangle_run_guard' not in text


def test_package_exposes_acknowledged_start_stop_services():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'mecanum_vicon_control/continuous_loop_supervisor.py').read_text()
    assert "create_service(Trigger, '/car/start_loop'" in source
    assert "create_service(Trigger, '/car/stop'" in source
    assert "self.publish(self.safety_pub, False)" in source
    assert "self.publish(self.trajectory_pub, False)" in source
    assert "self.publish(self.control_pub, False)" in source
