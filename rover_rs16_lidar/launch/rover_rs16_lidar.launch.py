#!/usr/bin/env python3

# Copyright 2026 Mechatronics Academy
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    publish_scan = LaunchConfiguration("publish_scan")
    declare_publish_scan_arg = DeclareLaunchArgument(
        "publish_scan",
        default_value="True",
        description=(
            "Flatten the point cloud into a LaserScan on <namespace>/scan, the topic Nav 2"
            " costmaps consume and the Gazebo bridge publishes in simulation."
        ),
        choices=["True", "true", "False", "false"],
    )

    input_type = LaunchConfiguration("input_type")
    declare_input_type_arg = DeclareLaunchArgument(
        "input_type",
        default_value="lidar",
        choices=["lidar", "pcap"],
        description=(
            "Where the packets come from. 'pcap' replays a capture instead of reading the"
            " sensor, which is how the stack is exercised with no lidar on the network."
        ),
    )

    pcap_path = LaunchConfiguration("pcap_path")
    declare_pcap_path_arg = DeclareLaunchArgument(
        "pcap_path",
        default_value="",
        description="Capture to replay. Required when input_type is 'pcap'.",
    )

    rover_rs16_lidar_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_rs16_lidar' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_rs16_lidar"),
            "'",
        ]
    )

    rover_rs16_lidar_config_path = LaunchConfiguration("rover_rs16_lidar_config_path")
    declare_rover_rs16_lidar_config_path_arg = DeclareLaunchArgument(
        "rover_rs16_lidar_config_path",
        default_value=PathJoinSubstitution(
            [rover_rs16_lidar_common_dir, "config", "rover_rs16_lidar.yaml"]
        ),
        description="Specify the path to the rover RS16 lidar configuration file.",
    )

    # TF frames carry the namespace as prefix (robot_state_publisher frame_prefix), and the
    # config file references <namespace>/lidar_link.
    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    resolved_config = LaunchConfiguration("rover_rs16_lidar_resolved_config")
    resolve_config = SetLaunchConfiguration(
        "rover_rs16_lidar_resolved_config",
        ReplaceString(rover_rs16_lidar_config_path, {"<namespace>/": ns}),
    )

    # One process, one config file: the RS16 driver, the LaserScan projection and the health
    # diagnostics all live in this node.
    rover_rs16_lidar_node = Node(
        package="rover_rs16_lidar",
        executable="rover_rs16_lidar_node",
        name="rover_rs16_lidar_node",
        namespace=namespace,
        parameters=[
            resolved_config,
            {
                # publish_scan arrives as the string "True"/"False"; the node declares a bool.
                "publish_scan": ParameterValue(publish_scan, value_type=bool),
                "input_type": input_type,
                "pcap_path": pcap_path,
            },
        ],
        remappings=[("/diagnostics", "diagnostics")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        emulate_tty=True,
    )

    actions = [
        declare_common_dir_path_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_publish_scan_arg,
        declare_input_type_arg,
        declare_pcap_path_arg,
        declare_rover_rs16_lidar_config_path_arg,
        resolve_config,
        rover_rs16_lidar_node,
    ]

    return LaunchDescription(actions)
