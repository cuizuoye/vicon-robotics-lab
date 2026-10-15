"""Guard new offline/observation tools and commissioning defaults."""
import ast
from pathlib import Path

import yaml

ROOT=Path(__file__).resolve().parents[3]

def test_offline_fault_harness_has_no_hardware_imports_and_isolates_domain():
    tree=ast.parse((ROOT/'tools/verify_control_faults.py').read_text())
    modules=[]
    for n in ast.walk(tree):
        if isinstance(n,ast.Import):modules.extend(a.name for a in n.names)
        if isinstance(n,ast.ImportFrom):modules.append(n.module or '')
    assert not any(m=='socket' or m.startswith('serial') or 'udp_bridge' in m for m in modules)
    init=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
          and isinstance(n.func.value,ast.Name) and n.func.value.id=='rclpy' and n.func.attr=='init']
    assert len(init)==1 and any(k.arg=='domain_id' and k.value.value==98 for k in init[0].keywords)

def test_real_vicon_observer_never_creates_publishers_or_transport():
    tree=ast.parse((ROOT/'tools/observe_car01_readonly.py').read_text())
    attrs=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert 'create_publisher' not in attrs and 'Popen' not in attrs and 'socket' not in attrs

def test_bounded_profile_requires_explicit_target_without_default_destination():
    data=yaml.safe_load((ROOT/'src/mecanum_vicon_control/config/bounded_motion.yaml').read_text())
    bridge=data['cmd_vel_udp_bridge']['ros__parameters']
    assert bridge['bounded_motion_mode'] and bridge['esp_ip']==''
    assert bridge['wire_timeout_ms']==150 and bridge['send_rate_hz']==50
    assert bridge['max_vx_mps']==bridge['max_vy_mps']==.05 and bridge['max_wz_radps']==.2
    assert not data['safety_watchdog']['ros__parameters']['start_enabled']

def test_motion_target_never_enters_default_build():
    text=(ROOT/'stm32/mecanum_f407/CMakeLists.txt').read_text()
    assert 'add_executable(${FOUR_MOTION_TARGET} EXCLUDE_FROM_ALL' in text
