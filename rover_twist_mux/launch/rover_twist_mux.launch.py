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

    twist_mux_config_path = LaunchConfiguration("twist_mux_config_path")
    declare_twist_mux_config_path_arg = DeclareLaunchArgument(
        "twist_mux_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rover_twist_mux"),
                "config",
                "rover_twist_mux.yaml",
            ]
        ),
        description="Specify the path to the twist mux configuration file.",
    )

    rover_twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name="rover_twist_mux",
        namespace=namespace,
        output='screen',
        parameters=[twist_mux_config_path],
        remappings={('/cmd_vel_out', '/cmd_vel')},
    )

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_twist_mux_config_path_arg,
        rover_twist_mux_node,
    ]

    return LaunchDescription(actions)