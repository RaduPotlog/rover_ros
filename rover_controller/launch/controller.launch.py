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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    
    controller_dir_path = LaunchConfiguration("controller_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "controller_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_controller_dir = PythonExpression(
        [
            "'",
            controller_dir_path,
            "/rover_controller' if '",
            controller_dir_path,
            "' else '",
            FindPackageShare("rover_controller"),
            "'",
        ]
    )

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model",
        choices=["rover_a1"],
    )

    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                rover_controller_dir,
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file."
        ),
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="DEBUG",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
        choices=["True", "true", "False", "false"],
    )

    default_wheel_type = {"rover_a1": "wheel_01"}
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value=PythonExpression([f"{default_wheel_type}['", robot_model, "']"]),
        description=(
            "Specify the wheel type."
        ),
        choices=["wheel_01"],
    )

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_description"), 
                    "launch", 
                    "rover_load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": robot_model,
            "log_level": log_level,
        }.items(),
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config_path = ReplaceString(controller_config_path, {"<namespace>/": ns})

    joint_state_broadcaster_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.joint_state_broadcaster' if '",
            namespace,
            "' else 'joint_state_broadcaster'",
        ]
    )
    controller_manager_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.controller_manager' if '",
            namespace,
            "' else 'controller_manager'",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ns_controller_config_path],
        namespace=namespace,
        remappings=[
            ("drive_controller/cmd_vel", "cmd_vel"),
            ("drive_controller/odom", "odometry/wheels"),
            ("drive_controller/transition_event", "_drive_controller/transition_event"),
            ("imu_broadcaster/imu", "imu/data"),
            ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),
            ("joint_state_broadcaster/transition_event", "_joint_state_broadcaster/transition_event"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=UnlessCondition(use_sim),
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    spawner_common_args = [
        "--controller-manager",
        "controller_manager",
        "--controller-manager-timeout",
        "10",
        "--ros-args",
        "--log-level",
        log_level,
    ]

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "drive_controller",
            "imu_broadcaster",
            "--activate-as-group",
            *spawner_common_args,
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,
        declare_wheel_type_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_log_level_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        load_urdf,
        control_node,
        controllers_spawner,
    ]

    return LaunchDescription(actions)
