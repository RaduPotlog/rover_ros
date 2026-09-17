from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    rviz_config=get_package_share_directory('rover_rslidar_sdk')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(
            package='rover_rslidar_sdk',
            node_namespace='rover_rslidar_sdk',
            node_name='rover_rslidar_sdk_node',
            node_executable='rover_rslidar_sdk_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            node_namespace='rviz2',
            node_name='rviz2',
            node_executable='rviz2',
            arguments=['-d',rviz_config]
        )
    ])
