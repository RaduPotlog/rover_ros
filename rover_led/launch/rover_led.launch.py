
from rover_utils.logging import limit_log_level_to_info
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_model = LaunchConfiguration("robot_model")
    rover_led_pkg = FindPackageShare("rover_led")
    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    rover_led_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/rover_led' if '",
            common_dir_path,
            "' else '",
            rover_led_pkg,
            "'",
        ]
    )

    animations_config = PythonExpression(["'", robot_model, "_animations.yaml'"])

    animations_config_path = LaunchConfiguration("animations_config_path")
    declare_animations_config_path_arg = DeclareLaunchArgument(
        "animations_config_path",
        default_value=PathJoinSubstitution(
            [rover_led_common_dir, "config", animations_config]
        ),
        description="Path to a YAML file with a description of led configuration.",
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
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="rover_a1"),
        description="Specify robot model.",
        choices=["rover_a1"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    user_led_animations_path = LaunchConfiguration("user_led_animations_path")
    declare_user_led_animations_path_arg = DeclareLaunchArgument(
        "user_led_animations_path",
        default_value=PathJoinSubstitution(
            [rover_led_common_dir, "config", "user_animations.yaml"]
        ),
        description="Path to a YAML file with a description of the user defined animations.",
    )

    driver_config = PythonExpression(["'", robot_model, "_driver.yaml'"])
    driver_config_path = PathJoinSubstitution([rover_led_pkg, "config", driver_config])
    led_container = ComposableNodeContainer(
        package="rclcpp_components",
        name="led_container",
        namespace=namespace,
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="rover_led",
                plugin="rover_led::LedDriverNode",
                name="led_driver",
                namespace=namespace,
                remappings=[("/diagnostics", "diagnostics")],
                parameters=[driver_config_path],
                extra_arguments=[
                    {"use_intra_process_comms": True},
                ],
                condition=UnlessCondition(use_sim),
            ),
            ComposableNode(
                package="rover_led",
                plugin="rover_led::LedControllerNode",
                name="led_controller",
                namespace=namespace,
                parameters=[
                    {"animations_config_path": animations_config_path},
                    {"user_led_animations_path": user_led_animations_path},
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True},
                ],
            ),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
            "--log-level",
            limit_log_level_to_info("pluginlib.ClassLoader", log_level),
        ],
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,  # robot_model is used by animations_config_path
        declare_animations_config_path_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_user_led_animations_path_arg,
        led_container,
    ]

    return LaunchDescription(actions)
