# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Exercise controller composition without launching hardware or ROS nodes."""

import importlib.util
import os
import socket
import subprocess
import sys
import time
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

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
        expected['/**']['drive_controller']['ros__parameters'].update(
            wheel_radius=0.21, base_frame_id='<namespace>/base_link')
        common.write_text(yaml.safe_dump(expected))
        context.launch_configurations['common_dir_path'] = str(tmp_path)
        expected['/**']['drive_controller']['ros__parameters']['base_frame_id'] = (
            f'{namespace}/base_link' if namespace else 'base_link')
    if selection == 'explicit':
        explicit = tmp_path / 'override.yaml'
        expected['/**']['drive_controller']['ros__parameters'].update(
            wheel_radius=0.23, base_frame_id='<namespace>/base_link')
        explicit.write_text(yaml.safe_dump(expected))
        context.launch_configurations['controller_config_path'] = str(explicit)
        expected['/**']['drive_controller']['ros__parameters']['base_frame_id'] = (
            f'{namespace}/base_link' if namespace else 'base_link')

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


def test_real_controller_accepts_override(controller_launch, monkeypatch, tmp_path):
    """Activate diff_drive on GenericSystem and read back the namespaced override."""
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_zenoh_cpp')
    try:
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            endpoint = f'tcp/127.0.0.1:{probe.getsockname()[1]}'
    except PermissionError:
        pytest.skip('local socket creation is unavailable in this test environment')
    monkeypatch.setenv(
        'ZENOH_CONFIG_OVERRIDE',
        f'mode="client";connect/endpoints=["{endpoint}"];'
        'connect/timeout_ms=10000;listen/endpoints=[];'
        'scouting/multicast/enabled=false',
    )
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.context import Context
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from controller_manager_msgs.srv import ListControllers
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String

    namespace = 'controller_test_' + uuid.uuid4().hex[:8]
    config = {
        '/**': {
            'controller_manager': {'ros__parameters': {
                'update_rate': 50,
                'drive_controller': {'type': 'diff_drive_controller/DiffDriveController'},
            }},
            'drive_controller': {'ros__parameters': {
                'left_wheel_names': ['left_wheel_joint'],
                'right_wheel_names': ['right_wheel_joint'],
                'wheel_separation': 0.6,
                'wheel_radius': 0.23,
                'base_frame_id': '<namespace>/base_link',
                'enable_odom_tf': False,
            }},
        },
    }
    override = tmp_path / 'override.yaml'
    override.write_text(yaml.safe_dump(config))
    captured = []
    original_node = controller_launch.Node

    def capture_node(**kwargs):
        captured.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(controller_launch, 'Node', capture_node)
    launch_context = LaunchContext()
    launch_context.launch_configurations.update(
        namespace=namespace, robot_model='rover_a1', use_sim='False',
        controller_config_path=str(override))
    description = controller_launch.generate_launch_description()
    for action in description.entities:
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(launch_context)
    manager = next(args for args in captured if args['executable'] == 'ros2_control_node')
    spawner = next(args for args in captured if args['executable'] == 'spawner' and
                   args['arguments'][0] == 'drive_controller')
    params = evaluate_parameters(launch_context, normalize_parameters(manager['parameters']))
    manager_file = str(params[0])
    spawner_args = [resolve(launch_context, arg) for arg in spawner['arguments']]
    assert spawner_args[spawner_args.index('--param-file') + 1] == manager_file

    links = '<link name="base_link"/>'
    joints = ''
    interfaces = ''
    for side in ('left', 'right'):
        links += f'<link name="{side}_wheel"/>'
        joints += f"""<joint name="{side}_wheel_joint" type="continuous">
          <parent link="base_link"/><child link="{side}_wheel"/>
          <axis xyz="0 1 0"/></joint>"""
        interfaces += f"""<joint name="{side}_wheel_joint">
          <command_interface name="velocity"/>
          <state_interface name="position"><param name="initial_value">0</param></state_interface>
          <state_interface name="velocity"><param name="initial_value">0</param></state_interface>
          </joint>"""
    urdf = f"""<robot name="test_rover">{links}{joints}
      <ros2_control name="MockWheels" type="system">
        <hardware><plugin>mock_components/GenericSystem</plugin></hardware>
        {interfaces}
      </ros2_control></robot>"""
    executable_dir = Path(get_package_prefix('controller_manager')) / 'lib/controller_manager'
    context = Context()
    node = None
    executor = None
    processes = []
    log_path = tmp_path / 'controller.log'
    router_log = (tmp_path / 'router.log').open('w')
    try:
        router_env = dict(os.environ, ZENOH_CONFIG_OVERRIDE=(
            f'mode="router";connect/endpoints=[];listen/endpoints=["{endpoint}"];'
            'scouting/multicast/enabled=false'))
        router = Path(get_package_prefix('rmw_zenoh_cpp')) / 'lib/rmw_zenoh_cpp/rmw_zenohd'
        processes.append(subprocess.Popen([str(router)], env=router_env,
                                          stdout=router_log, stderr=subprocess.STDOUT))
        rclpy.init(context=context, domain_id=int(os.environ.get('ROS_DOMAIN_ID', '0')))
        node = rclpy.create_node('test_observer', namespace=namespace, context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        publisher = node.create_publisher(
            String, 'robot_description',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(String(data=urdf))
        with log_path.open('w') as log:
            processes.append(subprocess.Popen([
                str(executable_dir / 'ros2_control_node'), '--ros-args',
                '--params-file', manager_file, '-r', f'__ns:=/{namespace}',
                '-r', f'/robot_description:=/{namespace}/robot_description',
            ], stdout=log, stderr=subprocess.STDOUT))
            client = node.create_client(
                ListControllers, f'/{namespace}/controller_manager/list_controllers')
            deadline = time.monotonic() + 15
            while not client.service_is_ready() and time.monotonic() < deadline:
                publisher.publish(String(data=urdf))
                executor.spin_once(timeout_sec=0.1)
            assert client.service_is_ready(), (
                f'manager did not become ready: domain={context.get_domain_id()}, '
                f'node={node.get_fully_qualified_name()}, '
                f'graph={node.get_node_names_and_namespaces()}')
            processes.append(subprocess.Popen([
                str(executable_dir / 'spawner'), *spawner_args,
                '-r', f'__ns:=/{namespace}',
            ], stdout=log, stderr=subprocess.STDOUT))
            assert processes[-1].wait(timeout=25) == 0, 'spawner failed'
            future = client.call_async(ListControllers.Request())
            executor.spin_until_future_complete(future, timeout_sec=10)
            assert future.done(), 'list_controllers timed out'
            assert any(c.name == 'drive_controller' and c.state == 'active'
                       for c in future.result().controller)
            parameters = node.create_client(
                GetParameters, f'/{namespace}/drive_controller/get_parameters')
            assert parameters.wait_for_service(timeout_sec=10)
            future = parameters.call_async(GetParameters.Request(
                names=['wheel_radius', 'base_frame_id']))
            executor.spin_until_future_complete(future, timeout_sec=10)
            assert future.done(), 'parameter query timed out'
            values = future.result().values
            assert values[0].double_value == pytest.approx(0.23)
            assert values[1].string_value == f'{namespace}/base_link'
    except (AssertionError, RuntimeError, subprocess.TimeoutExpired) as error:
        pytest.fail(f'{error}\n{log_path.read_text() if log_path.exists() else ""}')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=5)
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            context.shutdown()
        router_log.close()
        Path(manager_file).unlink(missing_ok=True)
