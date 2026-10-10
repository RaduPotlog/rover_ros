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

from rover_utils.logging import quiet_rmw_zenoh
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# The GPS and Lidar analyzer groups come from their own files, loaded only while that sensor is
# enabled: a disabled sensor publishes no diagnostics, and its group would otherwise stay STALE
# (and show as an error on the drive UI) forever.
OPTIONAL_GROUP_FILES = (
    ("use_gps", "diagnostic_aggregator_gps.yaml"),
    ("use_lidar", "diagnostic_aggregator_lidar.yaml"),
)


def _flag(value):
    return value.strip().lower() in ("true", "1", "yes", "on")


def optional_group_files(flags):
    """File names of the optional analyzer groups enabled by `flags` (launch argument -> value)."""
    return [name for arg, name in OPTIONAL_GROUP_FILES if _flag(flags.get(arg, ""))]


def generate_launch_description():
    
    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARN", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROVER_SYSTEM_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes",
    )

    system_diag_config_path = LaunchConfiguration("system_diag_config_path")
    declare_system_diag_config_path_arg = DeclareLaunchArgument(
        "system_diag_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rover_diag_manager"),
                "config",
                "system_diag.yaml",
            ]
        ),
        description="Specify the path to the diagnostic manager configuration file.",
    )

    rover_diag_manager_node = Node(
        package="rover_diag_manager",
        executable="rover_diag_manager_node",
        name="rover_diag_manager_node",
        parameters=[system_diag_config_path],
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            quiet_rmw_zenoh(log_level),
        ],
        emulate_tty=True,
    )

    diagnostic_aggregator_config_path = LaunchConfiguration("diagnostic_aggregator_config_path")
    declare_diagnostic_aggregator_config_path_arg = DeclareLaunchArgument(
        "diagnostic_aggregator_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rover_diag_manager"),
                "config",
                "diagnostic_aggregator.yaml",
            ]
        ),
        description="Specify the path to the diagnostic aggregator analyzers configuration file.",
    )

    use_gps = LaunchConfiguration("use_gps")
    declare_use_gps_arg = DeclareLaunchArgument(
        "use_gps",
        default_value=EnvironmentVariable("ROVER_SYSTEM_USE_GPS", default_value="false"),
        description="Add the GPS analyzer group (rover_gps_node, rover_gps_heading_node).",
    )

    use_lidar = LaunchConfiguration("use_lidar")
    declare_use_lidar_arg = DeclareLaunchArgument(
        "use_lidar",
        default_value=EnvironmentVariable("ROVER_SYSTEM_USE_LIDAR", default_value="false"),
        description="Add the Lidar analyzer group (rover_rs16_lidar_node).",
    )

    # Aggregates every node's diagnostics into diagnostics_agg, the topic the drive UI
    # diagnostics page subscribes to. Relative remaps keep it inside the namespace.
    def aggregator_setup(context):
        flags = {
            "use_gps": use_gps.perform(context),
            "use_lidar": use_lidar.perform(context),
        }
        parameters = [diagnostic_aggregator_config_path] + [
            PathJoinSubstitution([FindPackageShare("rover_diag_manager"), "config", name])
            for name in optional_group_files(flags)
        ]
        return [
            Node(
                package="diagnostic_aggregator",
                executable="aggregator_node",
                name="rover_diagnostic_aggregator",
                parameters=parameters,
                namespace=namespace,
                remappings=[
                    ("/diagnostics", "diagnostics"),
                    ("/diagnostics_agg", "diagnostics_agg"),
                    ("/diagnostics_toplevel_state", "diagnostics_toplevel_state"),
                ],
                arguments=[
                    "--ros-args",
                    "--log-level",
                    log_level,
                    "--log-level",
                    quiet_rmw_zenoh(log_level),
                ],
                emulate_tty=True,
            )
        ]

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_system_diag_config_path_arg,
        declare_diagnostic_aggregator_config_path_arg,
        declare_use_gps_arg,
        declare_use_lidar_arg,
        rover_diag_manager_node,
        OpaqueFunction(function=aggregator_setup),
    ]

    return LaunchDescription(actions)
