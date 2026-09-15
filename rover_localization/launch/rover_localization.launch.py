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
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Not restricted with `choices`: the value usually comes straight from the EKF_USE_GPS
    # balena variable, so any of true/1/yes/on (any case) enables GPS fusion.
    fuse_gps = LaunchConfiguration("fuse_gps")
    declare_fuse_gps_arg = DeclareLaunchArgument(
        "fuse_gps",
        default_value=EnvironmentVariable("EKF_USE_GPS", default_value="false"),
        description=(
            "Fuse GPS: adds rover_ekf_global_node (map -> odom) and rover_navsat_transform_node "
            "and loads the _with_gps config. The GPS driver itself is started by rover_gps."
        ),
    )
    fuse_gps_bool = PythonExpression(
        ["'", fuse_gps, "'.strip().lower() in ('true', '1', 'yes', 'on')"]
    )

    localization_mode = LaunchConfiguration("localization_mode")
    declare_localization_mode_arg = DeclareLaunchArgument(
        "localization_mode",
        default_value="rel",
        description=(
            "Specifies the localization mode:\n"
            "\t- 'rel' odometry/filtered data is relative to the initial position and orientation.\n"
            "\t- 'enu' odometry/filtered data is relative to initial position and ENU (East North Up) orientation."
        ),
        choices=["rel", "enu"],
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

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="False",
        description="Enable or disable EKF.",
        choices=["True", "true", "False", "false"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used.",
        choices=["True", "true", "False", "false"],
    )

    mode_prefix = PythonExpression(["'", localization_mode, "_'"])
    gps_postfix = PythonExpression(["'_with_gps' if ", fuse_gps_bool, " else ''"])
    localization_config_filename = PythonExpression(
        ["'", mode_prefix, "localization", gps_postfix, ".yaml'"]
    )

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_localization_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_localization' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("rover_localization"),
            "'",
        ]
    )

    localization_config_path = LaunchConfiguration("localization_config_path")
    declare_localization_config_path_arg = DeclareLaunchArgument(
        "localization_config_path",
        default_value=PathJoinSubstitution(
            [rover_localization_common_dir, "config", localization_config_filename]
        ),
        description="Specify the path to the localization configuration file.",
    )

    ekf_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="rover_ekf_node",
        parameters=[localization_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "localization/enable"),
            ("set_pose", "localization/set_pose"),
            ("toggle", "localization/toggle"),
            ('odometry/filtered', 'odom'),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            # robot_localization leaves odom1/imu1/pose0/twist0 declared but unset; every
            # bulk parameter fetch (foxglove_bridge) then makes rclcpp's parameter service
            # log "Failed to get parameters: parameter 'imu1' is not initialized".
            # Harmless noise; the filter's own messages use the node logger.
            "--log-level",
            "rclcpp:=ERROR",
        ],
        condition=IfCondition(use_ekf),
    )

    gps_enabled = PythonExpression(["'", use_ekf, "'.lower() == 'true' and ", fuse_gps_bool])

    # Global filter of the dual EKF: same inputs as rover_ekf_node plus odometry/gps, publishing
    # map -> odom. Do not run AMCL at the same time; it publishes map -> odom too.
    ekf_global_filter_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="rover_ekf_global_node",
        parameters=[localization_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("enable", "localization/global/enable"),
            ("set_pose", "localization/global/set_pose"),
            ("toggle", "localization/global/toggle"),
            ("odometry/filtered", "odometry/global"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            # Same unset-sensor parameter noise as rover_ekf_node.
            "--log-level",
            "rclcpp:=ERROR",
        ],
        condition=IfCondition(gps_enabled),
    )

    # Converts gps/fix into odometry/gps in the map frame. The heading it needs comes from
    # rover_gps_node (gps/heading_imu), published only once aligned from the GNSS course.
    navsat_transform_node = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="rover_navsat_transform_node",
        parameters=[localization_config_path, {"tf_prefix": namespace}],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("imu", "gps/heading_imu"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/filtered", "odometry/global"),
            ("odometry/gps", "odometry/gps"),
            ("datum", "localization/datum"),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
        ],
        condition=IfCondition(gps_enabled),
    )

    actions = [
        declare_common_dir_path_arg,
        declare_fuse_gps_arg,
        declare_localization_mode_arg,
        declare_localization_config_path_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_ekf_arg,
        declare_use_sim_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        ekf_filter_node,
        ekf_global_filter_node,
        navsat_transform_node,
    ]

    return LaunchDescription(actions)
