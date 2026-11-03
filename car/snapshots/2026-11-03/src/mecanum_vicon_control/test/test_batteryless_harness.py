"""Guard the live simulation harness against physical transport imports."""
import ast
from pathlib import Path


def harness_ast():
    root = Path(__file__).resolve().parents[3]
    return ast.parse((root / 'tools/verify_batteryless_rectangle.py').read_text())


def test_no_hardware_transport_imports():
    names = []
    for node in ast.walk(harness_ast()):
        if isinstance(node, ast.Import):
            names.extend(a.name for a in node.names)
        if isinstance(node, ast.ImportFrom):
            names.append(node.module or '')
    assert not any(name == 'socket' or name.startswith('serial')
                   or 'cmd_vel_udp_bridge' in name for name in names)


def test_process_local_isolated_ros_domain():
    calls = [n for n in ast.walk(harness_ast()) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'init'
             and isinstance(n.func.value, ast.Name) and n.func.value.id == 'rclpy']
    assert len(calls) == 1
    domain = next(k.value for k in calls[0].keywords if k.arg == 'domain_id')
    assert isinstance(domain, ast.Attribute) and domain.attr == 'ros_domain_id'
    declarations = [n for n in ast.walk(harness_ast()) if isinstance(n, ast.Call)
                    and any(isinstance(v, ast.Constant) and v.value == '--ros-domain-id' for v in n.args)]
    assert len(declarations) == 1
    choices = next(k.value for k in declarations[0].keywords if k.arg == 'choices')
    assert [v.value for v in choices.elts] == [97, 98, 99]


def test_only_display_child_process():
    calls = [n for n in ast.walk(harness_ast()) if isinstance(n, ast.Call)
             and isinstance(n.func, ast.Attribute) and n.func.attr == 'Popen']
    assert len(calls) == 1
    command = calls[0].args[0]
    assert isinstance(command, ast.List)
    assert [a.value for a in command.elts[:4]] == [
        'ros2', 'launch', 'mecanum_description', 'display.launch.py']
