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
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare

# MIN_REQUIRED_OS_VERSION = "v2.2.0"

def generate_launch_description():
    
    exit_on_wrong_hw = LaunchConfiguration("exit_on_wrong_hw")
    declare_exit_on_wrong_hw_arg = DeclareLaunchArgument(
        "exit_on_wrong_hw",
        default_value="false",
        description="Exit if hardware configuration is incorrect.",
        choices=["True", "true", "False", "false"],
    )

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )

    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "true", "False", "false"],
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
        description="Add namespace to all launched nodes.",
    )

    robot_model_name = EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1")
    robot_serial_no = EnvironmentVariable(name="ROBOT_SERIAL_NO", default_value="----")
    robot_version = EnvironmentVariable(name="ROBOT_VERSION", default_value="1.0")

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_controller"),
                    "launch", 
                    "controller.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "False",
            "common_dir_path": common_dir_path,
        }.items(),
    )

    system_diag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_diag_manager"),
                    "launch",
                    "system_diag.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "log_level": log_level, 
            "namespace": namespace,
            "use_sim": "False",
            "common_dir_path": common_dir_path,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_localization"), "launch", "rover_localization.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "False",
            "common_dir_path": common_dir_path,
        }.items(),
    )

    rover_twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_twist_mux"), "launch", "rover_twist_mux.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "False",
            "common_dir_path": common_dir_path,
        }.items(),
    )

    rover_bringup_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_bringup' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_bringup"),
            "'",
        ]
    )

    prevent_exit_action = ExecuteProcess(
        cmd=["sleep", "infinity"],
        condition=UnlessCondition(exit_on_wrong_hw),
    )

    delayed_action = TimerAction(
        period=10.0,
        actions=[
            ekf_launch,
            rover_twist_mux_launch,
        ],
    )

    # TODO: Handle hardware configuration
    hw_config_correct = EnvironmentVariable(name="ROBOT_HW_CONFIG_CORRECT", default_value="true")

    driver_actions = GroupAction(
        [
            controller_launch,
            system_diag_launch,
            delayed_action,
        ],
        condition=IfCondition(hw_config_correct),
    )

    actions = [
        declare_exit_on_wrong_hw_arg,
        declare_common_dir_path_arg,
        declare_disable_manager_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        driver_actions,
    ]

    return LaunchDescription(actions)