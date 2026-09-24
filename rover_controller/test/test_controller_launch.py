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

from launch import EventHandler, LaunchContext, LaunchDescription, LaunchService
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction,
    RegisterEventHandler, SetLaunchConfiguration,
)
from launch.event_handlers import OnShutdown
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from launch_ros.utilities import evaluate_parameters, normalize_parameters

from rover_utils.events import ControllersActive

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
    drive_parameters = expected['/**']['rover_drive_controller']['ros__parameters']
    assert drive_parameters['tf_frame_prefix'] == '~'
    # Controllers ignore ros2_control_node's remaps (use_global_arguments=false), so
    # topic remaps must be each controller's node_options_args.
    manager_parameters = expected['/**']['controller_manager']['ros__parameters']
    assert '~/odom:=odometry/wheels' in manager_parameters['rover_drive_controller']['node_options_args']
    assert '~/cmd_vel:=cmd_vel' in manager_parameters['rover_drive_controller']['node_options_args']
    assert '~/imu:=imu/data' in manager_parameters['rover_imu_broadcaster']['node_options_args']
    if selection != 'default':
        common = tmp_path / 'rover_controller/config/wheel_01_controller.yaml'
        common.parent.mkdir(parents=True)
        expected['/**']['rover_drive_controller']['ros__parameters'].update(
            wheel_radius=0.21, base_frame_id='<namespace>/base_link')
        common.write_text(yaml.safe_dump(expected))
        context.launch_configurations['common_dir_path'] = str(tmp_path)
        expected['/**']['rover_drive_controller']['ros__parameters']['base_frame_id'] = (
            f'{namespace}/base_link' if namespace else 'base_link')
    if selection == 'explicit':
        explicit = tmp_path / 'override.yaml'
        expected['/**']['rover_drive_controller']['ros__parameters'].update(
            wheel_radius=0.23, base_frame_id='<namespace>/base_link')
        explicit.write_text(yaml.safe_dump(expected))
        context.launch_configurations['controller_config_path'] = str(explicit)
        expected['/**']['rover_drive_controller']['ros__parameters']['base_frame_id'] = (
            f'{namespace}/base_link' if namespace else 'base_link')
    # The URDF names the IMU sensor `<namespace>/imu`, so the broadcaster must resolve the same prefix.
    imu_parameters = expected['/**']['rover_imu_broadcaster']['ros__parameters']
    assert imu_parameters['sensor_name'] == '<namespace>/imu'
    assert imu_parameters['frame_id'] == '<namespace>/imu_link'
    prefix = f'{namespace}/' if namespace else ''
    imu_parameters.update(sensor_name=f'{prefix}imu', frame_id=f'{prefix}imu_link')

    description = controller_launch.generate_launch_description()
    for action in description.entities:
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration)):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            action.execute(context)

    manager = next((node, args) for node, args in nodes
                   if args['executable'] == 'ros2_control_node')
    assert manager[0].condition.evaluate(context) == (use_sim == 'False')
    # Only the manager-level /diagnostics remap is allowed here: it reaches controller_manager and
    # the in-process rover_hardware_controller node. Controller topic remaps belong in node_options_args.
    assert list(manager[1].get('remappings') or []) == [('/diagnostics', 'diagnostics')], \
        'controller remaps belong in node_options_args'
    assert sum(args['executable'] == 'ros2_control_node' for _, args in nodes) == 1
    # Log calls from the real-time update() must not publish on /rosout (see the launch file).
    assert '--disable-rosout-logs' in manager[1]['arguments']
    parameters =evaluate_parameters(context, normalize_parameters(manager[1]['parameters']))
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


@pytest.mark.parametrize('failed', [None, 'rover_joint_state_broadcaster',
                                    'rover_drive_controller', 'rover_imu_broadcaster'])
def test_spawner_sequence(controller_launch, monkeypatch, tmp_path, failed):
    """Run real process exits through the production launch event handlers."""
    record = tmp_path / 'started.txt'
    controllers = ['rover_joint_state_broadcaster', 'rover_drive_controller', 'rover_imu_broadcaster']

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
    controllers_active = []
    controller_actions = controller_launch.generate_launch_description().entities
    description = LaunchDescription([
        GroupAction(scoped=True, actions=controller_actions),
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=lambda event, context: reasons.append(event.reason)
            ),
        ),
        RegisterEventHandler(
            EventHandler(
                matcher=lambda event: isinstance(event, ControllersActive),
                entities=[OpaqueFunction(
                    function=lambda context: controllers_active.append(1) or [])],
            ),
        ),
    ])
    service.include_launch_description(description)
    service.run()
    expected = controllers if failed is None else controllers[:controllers.index(failed) + 1]
    assert record.read_text().splitlines() == expected
    failures = [reason for reason in reasons if 'spawner failed' in reason]
    assert failures == ([] if failed is None else
                        [f'{failed} spawner failed with exit code 7'])
    # rover_bringup starts the rest of the stack on this: only once every spawner succeeded.
    assert len(controllers_active) == (1 if failed is None else 0)


def test_chained_wheel_controllers(controller_launch, tmp_path):
    bundled = (Path(get_package_share_directory('rover_controller')) /
               'config/wheel_01_controller.yaml')
    assert controller_launch.chained_wheel_controllers(str(bundled)) == [
        'pid_controller_rl_wheel_base_to_rl_wheel_joint',
        'pid_controller_fl_wheel_base_to_fl_wheel_joint',
        'pid_controller_rr_wheel_base_to_rr_wheel_joint',
        'pid_controller_fr_wheel_base_to_fr_wheel_joint',
    ]
    plain = tmp_path / 'plain.yaml'
    plain.write_text(yaml.safe_dump({'/**': {'rover_drive_controller': {'ros__parameters': {
        'left_wheel_names': ['left_wheel_joint'], 'right_wheel_names': ['right_wheel_joint']}}}}))
    assert controller_launch.chained_wheel_controllers(str(plain)) == []


def test_drive_spawner_activates_wheel_pids_as_group(controller_launch, monkeypatch):
    nodes = []
    original_node = controller_launch.Node

    def capture_node(**kwargs):
        nodes.append(kwargs)
        return original_node(**kwargs)

    monkeypatch.setattr(controller_launch, 'Node', capture_node)
    context = LaunchContext()
    context.launch_configurations.update(namespace='', use_sim='False', robot_model='rover_a1')
    for action in controller_launch.generate_launch_description().entities:
        if isinstance(action, (DeclareLaunchArgument, SetLaunchConfiguration, OpaqueFunction)):
            action.execute(context)
    drive = next(args['arguments'] for args in nodes if args['executable'] == 'spawner' and
                 args['arguments'][0] == 'rover_drive_controller')
    assert '--activate-as-group' in drive
    assert [a for a in drive if str(a).startswith('pid_controller_')] == \
        controller_launch.chained_wheel_controllers(
            resolve(context, drive[drive.index('--param-file') + 1]))


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
    bundled = yaml.safe_load((Path(get_package_share_directory('rover_controller')) /
                              'config/wheel_01_controller.yaml').read_text())
    bundled_drive = bundled['/**']['controller_manager']['ros__parameters']['rover_drive_controller']
    config = {
        '/**': {
            'controller_manager': {'ros__parameters': {
                'update_rate': 50,
                'rover_drive_controller': {
                    'type': 'diff_drive_controller/DiffDriveController',
                    'node_options_args': bundled_drive['node_options_args'],
                },
            }},
            'rover_drive_controller': {'ros__parameters': {
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
        elif isinstance(action, OpaqueFunction):
            action.execute(launch_context)
    manager = next(args for args in captured if args['executable'] == 'ros2_control_node')
    spawner = next(args for args in captured if args['executable'] == 'spawner' and
                   args['arguments'][0] == 'rover_drive_controller')
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
            assert any(c.name == 'rover_drive_controller' and c.state == 'active'
                       for c in future.result().controller)
            # The bundled node_options_args must actually rename the controller's topics.
            deadline = time.monotonic() + 10
            while (node.count_publishers(f'/{namespace}/odometry/wheels') == 0 and
                   time.monotonic() < deadline):
                executor.spin_once(timeout_sec=0.1)
            assert node.count_publishers(f'/{namespace}/odometry/wheels') == 1
            assert node.count_subscribers(f'/{namespace}/cmd_vel') == 1
            assert node.count_publishers(f'/{namespace}/rover_drive_controller/odom') == 0
            parameters = node.create_client(
                GetParameters, f'/{namespace}/rover_drive_controller/get_parameters')
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
