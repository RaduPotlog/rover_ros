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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
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

    rover_crfs_config_path = LaunchConfiguration("rover_crfs_config_path")
    declare_rover_crfs_config_path_arg = DeclareLaunchArgument(
        "rover_crfs_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rover_crfs_teleop"),
                "config",
                "rover_crfs.yaml",
            ]
        ),
        description="Specify the path to the rover crfs configuration file.",
    )

    rover_crfs_receiver_node = Node(
        package="crsf_receiver",
        executable="crsf_receiver_node",
        name="rover_crfs_receiver_node",
        namespace=namespace,
        parameters=[
            {'device': '/dev/ttyELRS'},
            {'baud_rate': '420800'},
            {'link_stats': True},
            {'receiver_rate': 50},
        ],
        emulate_tty=True,
    )

    rover_crfs_node = Node(
        package="rover_crfs_teleop",
        executable="rover_crfs_teleop_node",
        name="rover_crfs_teleop",
        parameters=[rover_crfs_config_path],
        namespace=namespace,
        remappings=[("/rover_crf_telop_cmd_vel_stamped", "rover_crf_telop_cmd_vel_stamped")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        emulate_tty=True,
    )

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_rover_crfs_config_path_arg,
        rover_crfs_receiver_node,
        rover_crfs_node,
    ]

    return LaunchDescription(actions)
