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
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogError,
    OpaqueFunction, RegisterEventHandler, SetLaunchConfiguration, Shutdown,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
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

def spawner_exit_handler(controller_name, next_action=None):
    """Continue only after successful activation of a mandatory controller."""
    def on_exit(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            reason = f"{controller_name} spawner failed with exit code {event.returncode}"
            return [LogError(msg=reason), Shutdown(reason=reason)]
        return [next_action] if next_action is not None else []

    return on_exit


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_controller_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_controller' if '",
            common_dir_path,
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

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    resolved_config = LaunchConfiguration("rover_controller_resolved_config")
    resolve_config = SetLaunchConfiguration(
        "rover_controller_resolved_config",
        ReplaceString(controller_config_path, {"<namespace>/": ns}),
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
            "use_sim": use_sim,
            "wheel_type": wheel_type,
            "controller_config_path": resolved_config,
        }.items(),
    )

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

    rover_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[resolved_config],
        namespace=namespace,
        # Only the /diagnostics remap belongs here: it reaches controller_manager and the
        # hardware_controller node the hardware interface creates in this process, but never
        # the controllers. Controller topic remaps are `node_options_args` in the controller
        # config file.
        remappings=[("/diagnostics", "diagnostics")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
            "--log-level",
            limit_log_level_to_info("pluginlib.ClassLoader", log_level),
            "--log-level",
            limit_log_level_to_info(joint_state_broadcaster_log_unit, log_level),
            "--log-level",
            limit_log_level_to_info(controller_manager_log_unit, log_level),
        ],
        condition=UnlessCondition(use_sim),
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    def configure_spawners(context):
        # Process-exit callbacks run after an included launch description's
        # scoped configurations have been restored. Resolve values used by
        # delayed spawners now, while this launch's arguments are in scope.
        config_path = resolved_config.perform(context)
        namespace_value = namespace.perform(context)
        log_level_value = log_level.perform(context)
        rcl_log_level = limit_log_level_to_info(
            'rcl', log_level
        ).perform(context)

        def make_spawner(controller_name, include_log_args=False):
            arguments = [
                controller_name,
                '--controller-manager',
                'controller_manager',
                '--controller-manager-timeout',
                '10',
                '--param-file',
                config_path,
            ]
            if include_log_args:
                arguments.extend([
                    '--ros-args',
                    '--log-level',
                    log_level_value,
                    '--log-level',
                    rcl_log_level,
                ])
            return Node(
                package='controller_manager',
                executable='spawner',
                arguments=arguments,
                namespace=namespace_value,
                emulate_tty=True,
            )

        drive_controller_spawner = make_spawner(
            'drive_controller', include_log_args=True
        )
        joint_state_broadcaster_spawner = make_spawner(
            'joint_state_broadcaster'
        )
        imu_broadcaster_spawner = make_spawner(
            'imu_broadcaster', include_log_args=True
        )

        return [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=spawner_exit_handler(
                        'joint_state_broadcaster',
                        drive_controller_spawner,
                    ),
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=drive_controller_spawner,
                    on_exit=spawner_exit_handler(
                        'drive_controller', imu_broadcaster_spawner
                    ),
                ),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=imu_broadcaster_spawner,
                    on_exit=spawner_exit_handler('imu_broadcaster'),
                )
            ),
            joint_state_broadcaster_spawner,
        ]

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,
        declare_wheel_type_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_log_level_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        resolve_config,
        load_urdf,
        rover_control_node,
        OpaqueFunction(function=configure_spawners),
    ]

    return LaunchDescription(actions)
