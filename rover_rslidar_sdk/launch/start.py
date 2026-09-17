from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config=get_package_share_directory('rover_rslidar_sdk')+'/rviz/rviz2.rviz'

    config_file = '' # your config file path
    
    return LaunchDescription([
        Node(namespace='rover_rslidar_sdk', package='rover_rslidar_sdk', executable='rover_rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}]),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ])
