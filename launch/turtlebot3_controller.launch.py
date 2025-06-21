from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot3_controller',
            executable='turtlebot3_controller_node',
            name='turtlebot3_controller',
            output='screen',
            parameters=['config/params.yaml'],
            namespace='',  # Set to 'turtlebot3' or simulation namespace if needed
        )
    ])
