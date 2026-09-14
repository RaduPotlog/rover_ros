# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Exercise controller composition without launching hardware or ROS nodes."""

import importlib.util
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchService
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnShutdown
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from launch_ros.utilities import evaluate_parameters, normalize_parameters

import pytest

import yaml


@pytest.fixture
def controller_launch():
    path = Path(__file__).resolve().parents[1] / 'launch' / 'rover_controller.launch.py'
    spec = importlib.util.spec_from_file_location('controller_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def resolve(context, value):
    return perform_substitutions(context, normalize_to_list_of_substitutions(value))


@pytest.mark.parametrize('namespace', ['', 'rover_a1'])
@pytest.mark.parametrize('use_sim', ['False', 'True'])
@pytest.mark.parametrize('selection', ['default', 'common', 'explicit'])
def test_configuration_consumers(controller_launch, monkeypatch, tmp_path,
                                 namespace, use_sim, selection):
    nodes = []
    includes = []
    original_node = controller_launch.Node
    original_include = controller_launch.IncludeLaunchDescription

    def capture_node(**kwargs):
        node = original_node(**kwargs)
        nodes.append((node, kwargs))
        return node

    def capture_include(*args, **kwargs):
        includes.append(dict(kwargs['launch_arguments']))
        return original_include(*args, **kwargs)

    monkeypatch.setattr(controller_launch, 'Node', capture_node)
    monkeypatch.setattr(controller_launch, 'IncludeLaunchDescription', capture_include)
    context = LaunchContext()
    context.launch_configurations.update(namespace=namespace, use_sim=use_sim,
                                         robot_model='rover_a1')
    bundled = (Path(get_package_share_directory('rover_controller')) /
               'config/wheel_01_controller.yaml')
    expected = yaml.safe_load(bundled.read_text())
    if selection != 'default':
        common = tmp_path / 'rover_controller/config/wheel_01_controller.yaml'
        common.parent.mkdir(parents=True)
        common.write_text('marker: common\nframe: <namespace>/base_link\n')
        context.launch_configurations['common_dir_path'] = str(tmp_path)
        expected = {
            'marker': 'common',
            'frame': f'{namespace}/base_link' if namespace else 'base_link',
        }
    if selection == 'explicit':
        explicit = tmp_path / 'override.yaml'
        explicit.write_text('marker: explicit\nframe: <namespace>/base_link\n')
        context.launch_configurations['controller_config_path'] = str(explicit)
        expected['marker'] = 'explicit'

    description = controller_launch.generate_launch_description()
    for action in description.entities:
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(context)

    manager = next((node, args) for node, args in nodes
                   if args['executable'] == 'ros2_control_node')
    assert manager[0].condition.evaluate(context) == (use_sim == 'False')
    assert sum(args['executable'] == 'ros2_control_node' for _, args in nodes) == 1
    parameters = evaluate_parameters(context, normalize_parameters(manager[1]['parameters']))
    assert len(parameters) == 1
    paths = [str(parameters[0]), resolve(context, includes[0]['controller_config_path'])]
    for _, args in nodes:
        assert resolve(context, args['namespace']) == namespace
        if args['executable'] == 'spawner':
            index = args['arguments'].index('--param-file')
            paths.append(resolve(context, args['arguments'][index + 1]))
    assert len(paths) == 5
    assert len(set(paths)) == 1
    assert yaml.safe_load(Path(paths[0]).read_text()) == expected


@pytest.mark.parametrize('failed', [None, 'joint_state_broadcaster',
                                    'drive_controller', 'imu_broadcaster'])
def test_spawner_sequence(controller_launch, monkeypatch, tmp_path, failed):
    """Run real process exits through the production launch event handlers."""
    record = tmp_path / 'started.txt'
    controllers = ['joint_state_broadcaster', 'drive_controller', 'imu_broadcaster']

    def fake_node(**kwargs):
        if kwargs['executable'] == 'ros2_control_node':
            return OpaqueFunction(function=lambda context: [])
        name = kwargs['arguments'][0]
        script = (
            'from pathlib import Path; '
            f'p = Path({str(record)!r}); '
            f'p.open("a").write({name!r} + "\\n"); '
            f'raise SystemExit({7 if name == failed else 0})'
        )
        return ExecuteProcess(cmd=[sys.executable, '-c', script])

    monkeypatch.setattr(controller_launch, 'Node', fake_node)
    monkeypatch.setattr(controller_launch, 'IncludeLaunchDescription',
                        lambda *args, **kwargs: OpaqueFunction(function=lambda context: []))
    service = LaunchService()
    service.context.launch_configurations.update(namespace='', robot_model='rover_a1')
    reasons = []
    description = controller_launch.generate_launch_description()
    description.add_action(RegisterEventHandler(
        OnShutdown(on_shutdown=lambda event, context: reasons.append(event.reason))))
    service.include_launch_description(description)
    service.run()
    expected = controllers if failed is None else controllers[:controllers.index(failed) + 1]
    assert record.read_text().splitlines() == expected
    failures = [reason for reason in reasons if 'spawner failed' in reason]
    assert failures == ([] if failed is None else
                        [f'{failed} spawner failed with exit code 7'])
