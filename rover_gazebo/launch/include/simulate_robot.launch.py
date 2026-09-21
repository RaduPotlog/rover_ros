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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():

    add_world_transform = LaunchConfiguration("add_world_transform")
    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running multiple robots)."
        ),
        choices=["True", "true", "False", "false"],
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("rover_gazebo"), "config", "gz_bridge.yaml"]
        ),
        description="Path to the parameter_bridge configuration file.",
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

    # Same variables as the rover (docker-compose), so a simulation run can mirror a deployment.
    use_gps = LaunchConfiguration("use_gps")
    declare_use_gps_arg = DeclareLaunchArgument(
        "use_gps",
        default_value=EnvironmentVariable("ROVER_USE_GPS", default_value="false"),
        description=(
            "Fuse the simulated GNSS (gps/fix): starts rover_gps_heading_node, "
            "rover_navsat_transform_node and rover_ekf_global_node, as on the rover."
        ),
    )

    publish_global_tf = LaunchConfiguration("publish_global_tf")
    declare_publish_global_tf_arg = DeclareLaunchArgument(
        "publish_global_tf",
        default_value=EnvironmentVariable("ROVER_GPS_PUBLISH_MAP_TF", default_value="false"),
        description="Let rover_ekf_global_node broadcast map -> odom (GPS fusion only).",
    )

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model.",
        choices=["rover_a1"],
    )

    # Spawn pose, shared by the spawner and the optional world -> odom transform.
    pose_args = {
        "x": ("0.0", "Initial robot position in the global 'x' axis."),
        "y": ("-2.0", "Initial robot position in the global 'y' axis."),
        "z": ("0.2", "Spawn (drop) height; odom stays on the ground plane."),
        "roll": ("0.0", "Initial robot 'roll' orientation."),
        "pitch": ("0.0", "Initial robot 'pitch' orientation."),
        "yaw": ("0.0", "Initial robot 'yaw' orientation."),
    }
    declare_pose_args = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, (default, description) in pose_args.items()
    ]
    pose = {name: LaunchConfiguration(name) for name in pose_args}

    rover_spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_gazebo"), "launch", "include/spawn_robot.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": robot_model,
            "log_level": log_level,
            **pose,
        }.items(),
    )

    rover_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("rover_controller"),
                    "launch",
                    "rover_controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "publish_robot_state": "True",
            "use_sim": "True",
            "extra_controller_config_path": PathJoinSubstitution(
                [FindPackageShare("rover_gazebo"), "config", "sim_wheel_pid.yaml"]
            ),
        }.items(),
    )

    rover_ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_localization"), "launch", "rover_localization.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "True",
            "use_ekf": "True",
            "fuse_gps": use_gps,
            "publish_global_tf": publish_global_tf,
        }.items(),
    )

    # Command path as on the rover: Nav2 (nav_cmd_vel_stamped) and teleop go through twist_mux
    # to cmd_vel, gated by motion_lock, which the orchestrator's IsMotionLocked BT node needs.
    rover_twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rover_twist_mux"), "launch", "rover_twist_mux.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
        }.items(),
    )

    # Stands in for the hardware interface's safety I/O, which rover_motion_lock_node reads.
    sim_gpio_state = Node(
        package="rover_gazebo",
        executable="sim_gpio_state.py",
        name="sim_gpio_state_publisher",
        namespace=namespace,
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={"<namespace>": namespace, "//": "/"},
    )

    rover_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=namespace,
        emulate_tty=True,
    )

    # scan from the simulated RS16 cloud, like rover_rs16_lidar's scan: the same +/-0.25 m
    # height slice, 360 deg at 0.5 deg, 0.2-20 m, stamped in the lidar frame.
    pointcloud_to_laserscan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="rover_rs16_lidar_scan",
        namespace=namespace,
        parameters=[
            {
                "target_frame": "",
                "transform_tolerance": 0.01,
                "min_height": -0.25,
                "max_height": 0.25,
                "angle_min": -3.141593,
                "angle_max": 3.141593,
                "angle_increment": 0.008727,
                "scan_time": 0.1,
                "range_min": 0.2,
                "range_max": 20.0,
                "use_inf": True,
            }
        ],
        remappings=[("cloud_in", "rslidar_points"), ("scan", "scan")],
        arguments=["--ros-args", "--log-level", log_level],
        emulate_tty=True,
    )

    child_tf = PythonExpression(["'", namespace, "' + '/odom' if '", namespace, "' else 'odom'"])

    rover_world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        arguments=[
            "--x",
            pose["x"],
            "--y",
            pose["y"],
            # odom lies on the ground plane (base_footprint is the root); the spawn z is only
            # the drop height, so it must not lift odom above the world.
            "--z",
            "0.0",
            "--roll",
            pose["roll"],
            "--pitch",
            pose["pitch"],
            "--yaw",
            pose["yaw"],
            "--frame-id",
            "world",
            "--child-frame-id",
            child_tf,
        ],
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(add_world_transform),
    )

    actions = [
        declare_add_world_transform_arg,
        declare_robot_model_arg,
        declare_gz_bridge_config_path_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_gps_arg,
        declare_publish_global_tf_arg,
        *declare_pose_args,
        SetUseSimTime(True),
        rover_spawn_robot_launch,
        rover_controller_launch,
        rover_ekf_launch,
        rover_twist_mux_launch,
        sim_gpio_state,
        rover_gz_bridge,
        pointcloud_to_laserscan,
        rover_world_transform,
    ]

    return LaunchDescription(actions)
