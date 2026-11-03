import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription


def load_square_launch():
    path = Path(__file__).resolve().parents[1] / 'launch/vicon_rectangle_3x3_hardware.launch.py'
    spec = importlib.util.spec_from_file_location('square_rectangle_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_square_rectangle_launch_pins_geometry_speed_and_firmware_identity():
    includes = [
        entity for entity in load_square_launch().entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 1
    arguments = dict(includes[0].launch_arguments)
    assert arguments['rectangle_width_m'] == '3.0'
    assert arguments['rectangle_height_m'] == '3.0'
    assert arguments['rectangle_waypoints'] == '8'
    assert arguments['max_linear_mps'] == '0.20'
    assert arguments['required_motion_tag'] == '4WG9'
    assert arguments['invert_body_xy'] == 'false'
