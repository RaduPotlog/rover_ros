#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def _env_float(name, default):
    """Read a sensor mount value from the environment (e.g. a balenaCloud device variable).

    A malformed value would break xacro and take robot_state_publisher down with it, so it
    is rejected with a warning and the default is used instead.
    """
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError:
        value = float("nan")
    if value != value or value in (float("inf"), float("-inf")):
        print(f"[rover_load_urdf] WARNING: {name}={raw!r} is not a finite number; using {default}")
        return default
    return raw


def generate_launch_description():
    
    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        description=(
            "Path to the controller configuration file, embedded in the URDF for gz_ros2_control. "
            "Owned and supplied by the caller (e.g. rover_controller)."
        ),
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROVER_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model",
        choices=["rover_a1"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used.",
        choices=["True", "true", "False", "false"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("rover_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description=(
            "Path to wheel configuration file. It is located in rover_description/config/{wheel_type}.yaml."
        ),
    )

    default_wheel_type = {"rover_a1": "wheel_01"}
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value=PythonExpression([f"{default_wheel_type}['", robot_model, "']"]),
        description=(
            "Specify the wheel type."
        ),
        choices=["wheel_01", "custom"],
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config_path = ReplaceString(controller_config_path, {"<namespace>/": ns})

    # IMU mount pose relative to body_link (x forward, y left, z up):
    # centerline, 90 mm to the rear, 200 mm up.
    imu_pos_x = _env_float("ROVER_IMU_LOCALIZATION_X", "-0.09")
    imu_pos_y = _env_float("ROVER_IMU_LOCALIZATION_Y", "0.0")
    imu_pos_z = _env_float("ROVER_IMU_LOCALIZATION_Z", "0.2")
    imu_rot_r = _env_float("ROVER_IMU_ORIENTATION_R", "0.0")
    imu_rot_p = _env_float("ROVER_IMU_ORIENTATION_P", "0.0")
    imu_rot_y = _env_float("ROVER_IMU_ORIENTATION_Y", "0.0")

    lidar_pos_x = _env_float("ROVER_LIDAR_LOCALIZATION_X", "0.0")
    lidar_pos_y = _env_float("ROVER_LIDAR_LOCALIZATION_Y", "0.0")
    lidar_pos_z = _env_float("ROVER_LIDAR_LOCALIZATION_Z", "0.0")
    lidar_rot_r = _env_float("ROVER_LIDAR_ORIENTATION_R", "0.0")
    lidar_rot_p = _env_float("ROVER_LIDAR_ORIENTATION_P", "0.0")
    lidar_rot_y = _env_float("ROVER_LIDAR_ORIENTATION_Y", "0.0")

    # GNSS antenna mount pose relative to body_link. Measure it on the rover and set the
    # variables; until then the antenna is assumed at the body origin.
    gps_pos_x = _env_float("ROVER_GPS_LOCALIZATION_X", "0.0")
    gps_pos_y = _env_float("ROVER_GPS_LOCALIZATION_Y", "0.0")
    gps_pos_z = _env_float("ROVER_GPS_LOCALIZATION_Z", "0.0")
    gps_rot_r = _env_float("ROVER_GPS_ORIENTATION_R", "0.0")
    gps_rot_p = _env_float("ROVER_GPS_ORIENTATION_P", "0.0")
    gps_rot_y = _env_float("ROVER_GPS_ORIENTATION_Y", "0.0")

    urdf_file = PythonExpression(["'", robot_model, ".urdf.xacro'"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rover_description"), "urdf", urdf_file]
            ),
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            ns_controller_config_path,
            " imu_xyz:=",
            f"'{imu_pos_x} {imu_pos_y} {imu_pos_z}'",
            " imu_rpy:=",
            f"'{imu_rot_r} {imu_rot_p} {imu_rot_y}'",
            " lidar_xyz:=",
            f"'{lidar_pos_x} {lidar_pos_y} {lidar_pos_z}'",
            " lidar_rpy:=",
            f"'{lidar_rot_r} {lidar_rot_p} {lidar_rot_y}'",
            " gps_xyz:=",
            f"'{gps_pos_x} {gps_pos_y} {gps_pos_z}'",
            " gps_rpy:=",
            f"'{gps_rot_r} {gps_rot_p} {gps_rot_y}'",
            " namespace:=",
            namespace,
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    publish_robot_state = LaunchConfiguration("publish_robot_state")
    declare_publish_robot_state_arg = DeclareLaunchArgument(
        "publish_robot_state",
        default_value="True",
        description=(
            "Whether to launch the robot_state_publisher node."
            "When set to False, users should publish their own robot description."
        ),
        choices=["True", "true", "False", "false"],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="rover_state_publisher_node",
        arguments=["--ros-args", "--disable-stdout-logs"],
        parameters=[robot_description, {"frame_prefix": ns}],
        namespace=namespace,
        condition=IfCondition(publish_robot_state),
    )
 
    actions = [
        declare_robot_model_arg,
        declare_wheel_type_arg,
        declare_publish_robot_state_arg,
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_pub_node,
    ]

    return LaunchDescription(actions)