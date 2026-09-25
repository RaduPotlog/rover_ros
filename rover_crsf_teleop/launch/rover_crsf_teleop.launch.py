#!/usr/bin/env python3

# Copyright 2025 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
RC teleop bring-up: the serial bridge that owns the UART, and the teleop node that decodes CRSF
from it.

The UART is deliberately not opened by the teleop node. rover_serial_driver's rover_serial_bridge_node already
does it, is lifecycle-managed, and is maintained upstream; the teleop node subscribes to the raw
bytes it publishes. That also keeps asio out of this package entirely.

Both run as components of one single-threaded container, rover_crsf_container, with intra-process
communication: the bridge publishes one rc/raw message per UART read (~250/s), and as separate
processes each of them crossed the Zenoh router. The single thread is also what the teleop node
was written for (see RoverCrsfTeleopNode).

One trade-off: on Ctrl-C a component container exits without running the lifecycle shutdown
transition, so the teleop node's on_shutdown() doesn't publish its final stop command. The rover
still stops - twist_mux drops the RC input after its timeout and diff_drive's cmd_vel_timeout
zeroes the wheels - just as it would if the process crashed. The standalone executables
(rover_crsf_teleop_node, rover_serial_bridge_node) still run the transition.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

import yaml


def _serial_settings(context, config_path):
    """
    Read the serial device and baud rate out of the teleop config.

    They live there, not inline here, so there is a single source of truth: the teleop node
    declares the same two parameters and reports them in its "RC serial link" diagnostic, and an
    operator editing the config does not have to know that a second node also needs the value.
    """
    resolved = config_path.perform(context)
    device, baudrate = '/dev/ttyUSB0', 460800

    try:
        with open(resolved, 'r') as handle:
            params = yaml.safe_load(handle) or {}

        for entry in params.values():
            settings = (entry or {}).get('ros__parameters', {})
            device = settings.get('serial_device', device)
            baudrate = settings.get('serial_baudrate', baudrate)
    except (OSError, yaml.YAMLError):
        # Fall back to the defaults rather than failing the whole bring-up: the teleop node still
        # comes up and reports the problem through diagnostics.
        pass

    return device, int(baudrate)


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace')
    log_level = LaunchConfiguration('log_level')
    use_sim = LaunchConfiguration('use_sim')
    config_path = LaunchConfiguration('rover_crsf_config_path')

    device, baudrate = _serial_settings(context, config_path)

    # The UART owner: opens the port, publishes raw bytes on `serial_read` and accepts writes on
    # `serial_write`. We never write to the receiver, so `serial_write` is remapped out of the way
    # rather than left on a generic name.
    #
    # `serial_read` is remapped to rc/raw because the default name is generic and this rover has
    # other serial devices; the teleop node's `serial_topic` parameter must match.
    #
    # Both nodes are lifecycle nodes brought to active by their own `autostart` parameter:
    # launch_ros' ComposableLifecycleNode autostart misses the namespace and never reaches them.
    serial_bridge_node = ComposableNode(
        package='rover_serial_driver',
        plugin='rover::transport::serial::SerialBridgeNode',
        name='rover_crsf_serial_bridge',
        namespace=namespace,
        parameters=[{
            'device_name': device,
            'baud_rate': baudrate,
            'flow_control': 'none',
            'parity': 'none',
            'stop_bits': '1',
            'autostart': True,
        }],
        remappings=[('serial_read', 'rc/raw'), ('serial_write', 'rc/raw_write')],
        extra_arguments=[{'use_intra_process_comms': True}],
        # No receiver exists in simulation, so starting a node that can only fail to open
        # /dev/ttyUSB0 is pure noise.
        condition=UnlessCondition(use_sim),
    )

    # Lifecycle node, brought straight to active. A supervisor can later deactivate it to take RC
    # teleop off the command path without killing the process.
    rover_crsf_node = ComposableNode(
        package='rover_crsf_teleop',
        plugin='rover_crsf_teleop::RoverCrsfTeleopNode',
        name='rover_crsf_teleop_node',
        namespace=namespace,
        parameters=[config_path, {'autostart': True}],
        remappings=[('/diagnostics', 'diagnostics')],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    # component_container, not _mt: one thread, as the teleop node requires.
    rover_crsf_container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='rover_crsf_container',
        namespace=namespace,
        composable_node_descriptions=[serial_bridge_node, rover_crsf_node],
        arguments=[
            '--ros-args',
            '--log-level',
            log_level,
        ],
        emulate_tty=True,
    )

    return [rover_crsf_container]


def generate_launch_description():

    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        choices=['DEBUG', 'INFO', 'WARNING', 'ERROR', 'FATAL'],
        description='Logging level',
    )

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('ROVER_NAMESPACE', default_value=''),
        description='Add namespace to all launched nodes',
    )

    # rover_bringup has always passed use_sim and common_dir_path; until now this file declared
    # neither, so both were silently discarded.
    declare_use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='False',
        description='Simulation mode: do not start the serial bridge, there is no receiver.',
    )

    common_dir_path = LaunchConfiguration('common_dir_path')
    declare_common_dir_path_arg = DeclareLaunchArgument(
        'common_dir_path',
        default_value='',
        description='Path to the common configuration directory.',
    )

    rover_crsf_common_dir = PythonExpression(
        ["'", common_dir_path, "/rover_crsf_teleop' if '", common_dir_path,
         "' else '", FindPackageShare('rover_crsf_teleop'), "'"]
    )

    declare_rover_crsf_config_path_arg = DeclareLaunchArgument(
        'rover_crsf_config_path',
        default_value=PathJoinSubstitution(
            [rover_crsf_common_dir, 'config', 'rover_crsf_teleop.yaml']
        ),
        description='Specify the path to the rover CRSF teleop configuration file.',
    )

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_common_dir_path_arg,
        declare_rover_crsf_config_path_arg,
        OpaqueFunction(function=_launch_setup),
    ]

    return LaunchDescription(actions)
