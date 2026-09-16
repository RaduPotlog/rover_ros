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
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
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

    rover_lidar_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_lidar' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_lidar"),
            "'",
        ]
    )

    rover_lidar_config_path = LaunchConfiguration("rover_lidar_config_path")
    declare_rover_lidar_config_path_arg = DeclareLaunchArgument(
        "rover_lidar_config_path",
        default_value=PathJoinSubstitution([rover_lidar_common_dir, "config", "rover_lidar.yaml"]),
        description="Specify the path to the rover lidar configuration file.",
    )

    rslidar_config_path = LaunchConfiguration("rslidar_config_path")
    declare_rslidar_config_path_arg = DeclareLaunchArgument(
        "rslidar_config_path",
        default_value=PathJoinSubstitution([rover_lidar_common_dir, "config", "rslidar.yaml"]),
        description="Specify the path to the RoboSense driver configuration file.",
    )

    # TF frames carry the namespace as prefix (robot_state_publisher frame_prefix), and both
    # config files reference <namespace>/lidar_link.
    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    resolved_rslidar_config = LaunchConfiguration("rover_lidar_resolved_rslidar_config")
    resolve_rslidar_config = SetLaunchConfiguration(
        "rover_lidar_resolved_rslidar_config",
        ReplaceString(rslidar_config_path, {"<namespace>/": ns}),
    )

    resolved_rover_lidar_config = LaunchConfiguration("rover_lidar_resolved_config")
    resolve_rover_lidar_config = SetLaunchConfiguration(
        "rover_lidar_resolved_config",
        ReplaceString(rover_lidar_config_path, {"<namespace>/": ns}),
    )

    # No `name=`: rover_rslidar_sdk_node creates its ROS nodes internally with hardcoded names
    # (rover_rslidar_points_destination_0, param_handle), so a __node remap would collapse them
    # onto one name. The namespace is passed as a global ROS arg and does reach all of them.
    rover_rslidar_driver_node = Node(
        package="rover_rslidar_sdk",
        executable="rover_rslidar_sdk_node",
        namespace=namespace,
        parameters=[{"config_path": resolved_rslidar_config}],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        emulate_tty=True,
    )

    rover_pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="rover_pointcloud_to_laserscan",
        namespace=namespace,
        condition=IfCondition(publish_scan),
        parameters=[resolved_rover_lidar_config],
        remappings=[("cloud_in", "rslidar_points"), ("scan", "scan")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        emulate_tty=True,
    )

    rover_lidar_node = Node(
        package="rover_lidar",
        executable="rover_lidar_node",
        name="rover_lidar_node",
        namespace=namespace,
        parameters=[resolved_rover_lidar_config],
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
        declare_rover_lidar_config_path_arg,
        declare_rslidar_config_path_arg,
        resolve_rslidar_config,
        resolve_rover_lidar_config,
        rover_rslidar_driver_node,
        rover_pointcloud_to_laserscan_node,
        rover_lidar_node,
    ]

    return LaunchDescription(actions)
