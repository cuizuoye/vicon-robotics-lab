"""嵌套模型参数不能覆盖纯观测入口的RViz开关。测试不启动任何ROS节点。"""
import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import GroupAction, SetLaunchConfiguration
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.push_launch_configurations import PushLaunchConfigurations


def load_observe_launch():
    path = Path(__file__).parents[1] / 'launch' / 'vicon_observe.launch.py'
    spec = importlib.util.spec_from_file_location('vicon_observe_launch_test', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


@pytest.mark.parametrize('outer_value', ['true', 'false'])
def test_model_include_is_scoped(outer_value):
    groups = [entity for entity in load_observe_launch().entities if isinstance(entity, GroupAction)]
    assert len(groups) == 1
    actions = groups[0].get_sub_entities()
    pushes = [action for action in actions if isinstance(action, PushLaunchConfigurations)]
    pops = [action for action in actions if isinstance(action, PopLaunchConfigurations)]
    assert len(pushes) == len(pops) == 1
    context = LaunchContext()
    SetLaunchConfiguration('start_rviz', outer_value).execute(context)
    pushes[0].execute(context)
    SetLaunchConfiguration('start_rviz', 'false').execute(context)
    assert context.launch_configurations['start_rviz'] == 'false'
    pops[0].execute(context)
    assert context.launch_configurations['start_rviz'] == outer_value
