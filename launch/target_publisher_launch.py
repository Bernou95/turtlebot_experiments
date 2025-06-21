from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tracking')
    param_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='tracking',
            executable='target_publisher.py',  # your target publishing node
            name='target_publisher',
            output='screen',
            # parameters=[] if needed
        ),
        Node(
            package='tracking',
            executable='turtlebot3_model',
            name='turtlebot3_model',
            output='screen',
            parameters=[param_file],
        ),
    ])
