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

from rover_utils.logging import limit_log_level_to_info
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_safety_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_safety' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_safety"),
            "'",
        ]
    )

    rover_safety_pkg = FindPackageShare("rover_safety")

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
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    safety_bt_project_path = LaunchConfiguration("safety_bt_project_path")
    declare_safety_bt_project_path_arg = DeclareLaunchArgument(
        "safety_bt_project_path",
        default_value=PathJoinSubstitution(
            [rover_safety_pkg, "behavior_trees", "RoverSafetyBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for safety.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    safety_node = Node(
        package="rover_safety",
        executable="rover_safety_node",
        name="rover_safety_node",
        parameters=[
            PathJoinSubstitution([rover_safety_pkg, "config", "rover_safety.yaml"]),
            {
                "bt_project_path": safety_bt_project_path,
            },
        ],
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
        condition=UnlessCondition(use_sim),
    )

    actions = [
        declare_common_dir_path_arg,
        declare_log_level_arg,
        declare_safety_bt_project_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        safety_node,
    ]

    return LaunchDescription(actions)