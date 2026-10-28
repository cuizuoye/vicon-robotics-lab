from pathlib import Path
import ast


def launch_text():
    root = Path(__file__).resolve().parents[1]
    return (root / 'launch/disturbance_recovery_sil.launch.py').read_text()


def test_sil_launch_is_hardware_isolated():
    text = launch_text()
    assert "executable='simulated_vicon_car'" in text
    assert "executable='path_recovery_controller'" in text
    assert 'cmd_vel_udp_bridge' not in text
    assert 'rectangle_run_guard' not in text


def test_sil_launch_uses_fixed_heading_and_relative_start():
    text = launch_text()
    assert "'heading_mode': 'fixed'" in text
    assert "'relative_to_start': True" in text
    assert "'waypoint_count': 8" in text


def test_live_verifier_cannot_import_physical_transport():
    root = Path(__file__).resolve().parents[3]
    tree = ast.parse((root / 'tools/verify_disturbance_recovery.py').read_text())
    imports = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.extend(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.append(node.module or '')
    assert not any(
        name == 'socket' or name.startswith('serial')
        or 'cmd_vel_udp_bridge' in name for name in imports)


def test_live_verifier_uses_isolated_domains():
    root = Path(__file__).resolve().parents[3]
    text = (root / 'tools/verify_disturbance_recovery.py').read_text()
    assert 'choices=[94, 95, 96]' in text
    assert "domain_id=args.ros_domain_id" in text
